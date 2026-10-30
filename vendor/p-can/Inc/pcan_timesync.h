#ifndef __PCAN_TIMESYNC_H__
#define __PCAN_TIMESYNC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "pcan_core.h"

// Only compile this file if timer modules are available in the HAL.
#ifdef HAL_TIM_MODULE_ENABLED

#ifndef PCAN_TIMESYNC_MESSAGE_ID
#define PCAN_TIMESYNC_MESSAGE_ID (0xA8929A8)
#endif

typedef enum {
    PCAN_TIMESYNC_ROLE_SUB,
    PCAN_TIMESYNC_ROLE_DOM,
} pcan_timesync_role_t;

/**
 * @brief Storage for the P-CAN timesync protocol.
 */
typedef struct {
    /**
     * @brief Whether timesync is enabled or not.
     */
    bool enabled;
    /**
     * @brief Whether this node it a time dom or time sub;
     */
    pcan_timesync_role_t role;
    /**
     * @brief The estimated offset between the local time and the dom time in microseconds.
     */
    uint64_t offset_us;
    /**
     * @brief Software extension of the local timer in microseconds.
     */
    uint64_t local_time_extension_us;
    /**
     * @brief Updated every time time is read. Used to ensure monotonic time.
     */
    uint64_t previous_local_time_us;
    /**
     * @brief Time when the last sync message was sent in microseconds.
     */
    uint64_t sync_sent_time_us;
    /**
     * @brief Callback to get time from an external clocksource.
     */
    uint64_t (*get_time_us)(void *user_data);
    /**
     * @brief Callback which will be called to update the external clocksource.
     */
    uint64_t (*set_correction_us)(void *user_data, int64_t correction_us);
    void *user_data;
    /**
     * @brief Handle to the hardware timer used for timesync.
     */
    TIM_HandleTypeDef *htim;
} pcan_timesync_instance_t;

typedef enum { PCAN_TIMESYNC_APB1, PCAN_TIMESYNC_APB2 } pcan_timesync_apb_t;

typedef struct {
    uint64_t local;
    uint64_t global;
} pcan_timesync_time_t;

/**
 * @brief Initalizes and enables the P-CAN Timesync protocol using the provided timer.
 *
 * @return int Zero on success, negative error code on failure.
 */
int pcan_timesync_create(pcan_timesync_instance_t *timesync, TIM_HandleTypeDef *htim,
                         pcan_timesync_role_t role);

/**
 * @brief Sets up an external clock for the timesync protocol.
 *
 * This is used when the timesync dom wants control over its own timers.
 *
 * @param timesync
 * @param get_time_us
 * @param set_correction_us
 * @param user_data
 * @return int
 */
int pcan_timesync_set_external_clock(pcan_timesync_instance_t *timesync,
                                     uint64_t (*get_time_us)(void *),
                                     uint64_t (*set_correction_us)(void *, int64_t),
                                     void *user_data);

/**
 * @brief Gets a plugin struct for the P-CAN timesync protocol which can be registered to a P-CAN
 * instance.
 *
 * @param timesync The timesync instance to create the plugin for.
 * @return int
 */
pcan_plugin_t pcan_timesync_plugin(pcan_timesync_instance_t *timesync);

/**
 * @brief Registers a timesync instance to a P-CAN instance.
 *
 * @param pcan P-CAN instance to register the timesync protocol to.
 * @param timesync The timesync instance to register.
 * @return int Non-negative on success, negative error code on failure.
 */
int pcan_timesync_register_plugin(pcan_instance_t *pcan, pcan_timesync_instance_t *timesync);

/**
 * @brief Get the local and current estimated global time in microseconds.
 */
pcan_timesync_time_t pcan_timesync_now_us(pcan_timesync_instance_t *timesync);

/**
 * @brief Gets the current local and estimated global time in milliseconds.
 */
pcan_timesync_time_t pcan_timesync_now_ms(pcan_timesync_instance_t *timesync);

/**
 * @brief Handle hardware timer overflow to extend the software timer.
 */
void pcan_timesync_on_period_elapsed(pcan_timesync_instance_t *timesync, TIM_HandleTypeDef *htim);

#endif // HAL_TIM_MODULE_ENABLED

#ifdef __cplusplus
}
#endif

#endif // __PCAN_TIMESYNC_H__
