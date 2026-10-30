#include "pcan_timesync.h"

// Only compile this file if timer modules are available in the HAL.
#ifdef HAL_TIM_MODULE_ENABLED

// ~~~~~~~~~~~~~~~~~~ The P-CAN Timesync Protocol ~~~~~~~~~~~~~~~~~~
//
// P-CAN Timesync is a simple protocol for synchronizing the time
// between nodes on a CAN bus. It works by designating one node as
// the time dom, which then periodically broadcasts the current
// time to all other nodes (time subs).
//
// The time is represented as a 64 bit unsigned integer which
// counts in microseconds. The epoch (time zero) is user defined.
// This is usually the time when the time dom node was started.
//
// If the dom reboots time counting starts again from zero.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// The frequency of the hardware timer used for timesync.
#define PCAN_TIMESYNC_FREQUENCY_HZ (1000000)
// How often the dom sends the timesync message.
#define PCAN_TIMESYNC_SEND_INTERVAL_US (100000)
// How much deviation from the dom time is allowed before we give
// up and perform a hard reset on the local time.
#define PCAN_TIMESYNC_HARD_TOLERANCE_US (100000)
// How much we can adjust the local time at a time.
#define PCAN_TIMESYNC_MAX_CORRECTION_US (1000)
// The width of the hardware timer counter int bits.
#define PCAN_TIMESYNC_HARDWARE_COUNTER_WIDTH (16)
#define PCAN_TIMESYNC_HARDWARE_COUNTER_MASK  ((1 << PCAN_TIMESYNC_HARDWARE_COUNTER_WIDTH) - 1)
// Magic number to validate timesync messages. ("TIME" in ASCII <3)
#define PCAN_TIMESYNC_MAGIC_NUMBER (0x54494d45)

typedef struct {
    uint32_t magic_number;
    uint64_t dom_time_us;
} pcan_timesync_message_t;

///////////////
// Utilities //
///////////////

/**
 * @brief Utility function for getting the bus clock frequency of the APB the timer is connected to.
 */
uint32_t get_bus_clock_frequency(pcan_timesync_apb_t apb) {
    RCC_ClkInitTypeDef clock_config;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clock_config, &latency);
    (void)latency;

    uint32_t apb_prescaler =
        apb == PCAN_TIMESYNC_APB1 ? clock_config.APB1CLKDivider : clock_config.APB2CLKDivider;
    uint32_t base_frequency =
        apb == PCAN_TIMESYNC_APB1 ? HAL_RCC_GetPCLK1Freq() : HAL_RCC_GetPCLK2Freq();

    return apb_prescaler != RCC_HCLK_DIV1 ? base_frequency * 2 : base_frequency;
}

///////////////////////
// Internal function //
///////////////////////

/**
 * @brief Assigns current time to any incoming message.
 */
int timesync_assing_rx_timestamp(pcan_instance_t *pcan, void *plugin_data,
                                 pcan_message_t *message) {
    pcan_timesync_instance_t *timesync = (pcan_timesync_instance_t *)plugin_data;
    message->timestamp_us              = pcan_timesync_now_us(timesync).global;
    return 0;
}

/**
 * @brief Assigns current time to an outgoing timesync message.
 */
int timesync_assing_tx_timestamp(pcan_instance_t *pcan, void *plugin_data,
                                 pcan_message_t *message) {
    if (message->raw_id != PCAN_TIMESYNC_MESSAGE_ID) {
        // Not a timesync message, ignore.
        return 0;
    }

    pcan_timesync_instance_t *timesync        = (pcan_timesync_instance_t *)plugin_data;
    pcan_timesync_message_t *timesync_message = (pcan_timesync_message_t *)message->data;

    timesync_message->dom_time_us = pcan_timesync_now_us(timesync).global;

    return 0;
}

/**
 * @brief This function is called when a timesync message is received.
 */
void timesync_callback(pcan_message_t *message, void *user_data) {
    pcan_timesync_message_t *timesync_message = (pcan_timesync_message_t *)message->data;
    pcan_timesync_instance_t *timesync        = (pcan_timesync_instance_t *)user_data;

    if (!timesync->enabled || timesync->role != PCAN_TIMESYNC_ROLE_SUB) {
        // We are not a time sub, ignore.
        return;
    }

    if (timesync_message->magic_number != PCAN_TIMESYNC_MAGIC_NUMBER) {
        // Not a valid timesync message, ignore.
        return;
    }

    if ((message->timestamp_us == 0) || (timesync_message->dom_time_us == 0)) {
        // Invalid timestamps, ignore.
        return;
    }

    // 1. Calculate the time difference between the received time and the local time.
    int64_t error_us = (int64_t)timesync_message->dom_time_us - (int64_t)message->timestamp_us;

    // 2. If the absolute value of the time difference is larger than
    //    PCAN_TIMESYNC_HARD_TOLERANCE_US, reset the local time to the received time.
    //    Then end the procedure here. Otherwise, continue to step 3.
    if (error_us > PCAN_TIMESYNC_HARD_TOLERANCE_US || error_us < -PCAN_TIMESYNC_HARD_TOLERANCE_US) {
        timesync->offset_us = timesync_message->dom_time_us - pcan_timesync_now_us(timesync).local;
        return;
    }

    // 3. Set the correction to either
    //    - the time difference if its absolute value is smaller than
    //    PCAN_TIMESYNC_MAX_CORRECTION_US
    //    - PCAN_TIMESYNC_MAX_CORRECTION_US if the time difference is positive and larger than
    //    PCAN_TIMESYNC_MAX_CORRECTION_US
    //    - -PCAN_TIMESYNC_MAX_CORRECTION_US if the time difference is negative and smaller than
    //    -PCAN_TIMESYNC_MAX_CORRECTION_US
    int64_t correction_us = 0;
    if (error_us > PCAN_TIMESYNC_MAX_CORRECTION_US) {
        correction_us = PCAN_TIMESYNC_MAX_CORRECTION_US;
    } else if (error_us < -PCAN_TIMESYNC_MAX_CORRECTION_US) {
        correction_us = -PCAN_TIMESYNC_MAX_CORRECTION_US;
    } else {
        correction_us = error_us;
    }

    // 4. Apply the correction by adjusting the local time.
    timesync->offset_us += correction_us;
}

static int timesync_init(pcan_instance_t *pcan, void *plugin_data) {
    pcan_timesync_instance_t *timesync = (pcan_timesync_instance_t *)plugin_data;
    TIM_HandleTypeDef *htim            = timesync->htim;

    if (timesync->role == PCAN_TIMESYNC_ROLE_SUB) {
        // Register with P-CAN instance.
        pcan_subscribe_raw(pcan, PCAN_TIMESYNC_MESSAGE_ID, PCAN_EXTENDED_CAN_ID_MASK,
                           timesync_callback, timesync);
    }

    if (timesync->htim != NULL) {
        // Get clock speed for hardware timer.
        // TODO: Make APB1 vs APB2 configurable.
        uint32_t timer_clock_frequency = get_bus_clock_frequency(PCAN_TIMESYNC_APB1);

        // Stop and deinitalize the hardware timer in case it was running before.
        HAL_TIM_Base_Stop_IT(htim);
        HAL_TIM_Base_DeInit(htim);

        // Initalize hardware timer.
        htim->Init.Prescaler   = (timer_clock_frequency / PCAN_TIMESYNC_FREQUENCY_HZ) - 1;
        htim->Init.CounterMode = TIM_COUNTERMODE_UP;
        htim->Init.Period = (1 << PCAN_TIMESYNC_HARDWARE_COUNTER_WIDTH) - 1; // Counter width = 16
        htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        htim->Init.RepetitionCounter = 0;

        if (HAL_TIM_Base_Init(htim) != HAL_OK) {
            return -1;
        }

        // Reset counter and clear flags.
        __HAL_TIM_SET_COUNTER(htim, 0);
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

        // Start the hardware timer and interrupt.
        HAL_TIM_Base_Start_IT(htim);
    }

    return 0;
}

static int timesync_poll(pcan_instance_t *pcan, void *plugin_data) {
    pcan_timesync_instance_t *timesync = (pcan_timesync_instance_t *)plugin_data;

    if (!timesync->enabled || timesync->role != PCAN_TIMESYNC_ROLE_DOM) {
        // Timesync is not enabled or we are the dom, do nothing.
        return 0;
    }

    uint64_t dom_time_us = pcan_timesync_now_us(timesync).global;

    int64_t time_since_published = dom_time_us - timesync->sync_sent_time_us;
    if (time_since_published < PCAN_TIMESYNC_SEND_INTERVAL_US) {
        return 0;
    }
    timesync->sync_sent_time_us = dom_time_us;

    pcan_timesync_message_t timesync_message = {.magic_number = PCAN_TIMESYNC_MAGIC_NUMBER,
                                                .dom_time_us  = dom_time_us};

    pcan_publish_raw(pcan, PCAN_TIMESYNC_MESSAGE_ID, (uint8_t *)&timesync_message,
                     sizeof(timesync_message));

    return 0;
}

//////////////////
// External API //
//////////////////

int pcan_timesync_create(pcan_timesync_instance_t *timesync, TIM_HandleTypeDef *htim,
                         pcan_timesync_role_t role) {
    // Configue timesync struct.
    timesync->enabled                 = true;
    timesync->role                    = role;
    timesync->offset_us               = 0;
    timesync->local_time_extension_us = 0;
    timesync->previous_local_time_us  = 0;
    timesync->sync_sent_time_us       = 0;
    timesync->get_time_us             = NULL;
    timesync->set_correction_us       = NULL;
    timesync->user_data               = NULL;
    timesync->htim                    = htim;

    return 0;
}

int pcan_timesync_set_external_clock(pcan_timesync_instance_t *timesync,
                                     uint64_t (*get_time_us)(void *),
                                     uint64_t (*set_correction_us)(void *, int64_t),
                                     void *user_data) {
    // If an external clock is set up, we need to be a sub and cannot use a hardware timer.
    if (timesync->role != PCAN_TIMESYNC_ROLE_DOM || timesync->htim != NULL) {
        // Only the dom can be set to use an external clock and
        // no hardware timer can be set up.
        return -1;
    }

    timesync->get_time_us       = get_time_us;
    timesync->set_correction_us = set_correction_us;
    timesync->user_data         = user_data;

    return 0;
}

pcan_plugin_t pcan_timesync_plugin(pcan_timesync_instance_t *timesync) {
    return (pcan_plugin_t){
        .free        = false,
        .init        = timesync_init,
        .poll        = timesync_poll,
        .on_rx       = timesync_assing_rx_timestamp,
        .on_tx       = timesync_assing_tx_timestamp,
        .plugin_data = timesync,
    };
}

int pcan_timesync_register_plugin(pcan_instance_t *pcan, pcan_timesync_instance_t *timesync) {
    return pcan_register_plugin(pcan, pcan_timesync_plugin(timesync));
}

pcan_timesync_time_t pcan_timesync_now_us(pcan_timesync_instance_t *timesync) {
    if (timesync->role == PCAN_TIMESYNC_ROLE_DOM && timesync->get_time_us) {
        uint64_t time_us = timesync->get_time_us(timesync->user_data);
        return (pcan_timesync_time_t){
            .local=time_us,
            .global=time_us, // Our time is the global time!
        };
    }

    uint64_t upper_time_us_1;
    uint64_t upper_time_us_2;
    uint64_t lower_time_us;

    // Read upper and lower time values until they are consistent.
    do {
        upper_time_us_1 = timesync->local_time_extension_us;
        lower_time_us   = __HAL_TIM_GetCounter(timesync->htim);
        upper_time_us_2 = timesync->local_time_extension_us;
    } while (upper_time_us_1 != upper_time_us_2);

    uint64_t time_us = (upper_time_us_1 << PCAN_TIMESYNC_HARDWARE_COUNTER_WIDTH) | lower_time_us;

    return (pcan_timesync_time_t){
        .local = time_us,
        .global = time_us + timesync->offset_us,
    };
}

pcan_timesync_time_t pcan_timesync_now_ms(pcan_timesync_instance_t *timesync) {
    pcan_timesync_time_t time_us = pcan_timesync_now_us(timesync);
    return (pcan_timesync_time_t){
        .local = time_us.local / 1000,
        .global = time_us.global / 1000,
    };
}

void pcan_timesync_on_period_elapsed(pcan_timesync_instance_t *timesync, TIM_HandleTypeDef *htim) {
    if (htim != timesync->htim) {
        // Not our timer, ignore.
        return;
    }

    timesync->local_time_extension_us++;
}

#endif // HAL_TIM_MODULE_ENABLED
