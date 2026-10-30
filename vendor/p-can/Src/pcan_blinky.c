#include "pcan_blinky.h"

///////////////
// Utilities //
///////////////

static void led_on(pcan_blinky_led_t led) {
    if (led.behavior == PCAN_BLINKY_LED_ACTIVE_HIGH) {
        HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_RESET);
    }
}

static void led_off(pcan_blinky_led_t led) {
    if (led.behavior == PCAN_BLINKY_LED_ACTIVE_HIGH) {
        HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_SET);
    }
}

#define MORSE_UNIT_MS (100)

#define MORSE_DOT_MS        (1 * MORSE_UNIT_MS)
#define MORSE_DASH_MS       (3 * MORSE_UNIT_MS)
#define MORSE_INTRA_GAP_MS  (1 * MORSE_UNIT_MS)
#define MORSE_LETTER_GAP_MS (3 * MORSE_UNIT_MS)
#define MORSE_WORD_GAP_MS   (7 * MORSE_UNIT_MS)

static bool morse_is_ready(pcan_blinky_morse_context_t *context) {
    return context && context->sequence == NULL && (context->state == PCAN_BLINKY_MORSE_STATE_IDLE);
}

static int morse_start(pcan_blinky_morse_context_t *context, const char *sequence) {
    if (!morse_is_ready(context)) {
        return -1;
    }

    context->sequence = sequence;
    context->state    = PCAN_BLINKY_MORSE_STATE_IDLE;
    context->next_ms  = 0;
    context->index    = 0;

    return 0;
}

static int morse_step(pcan_blinky_morse_context_t *context, pcan_blinky_led_t led,
                      uint32_t now_ms) {
    if (!context) {
        return -1;
    }

    if (now_ms < context->next_ms) {
        return 0;
    }

    if (!context->sequence) {
        return 0;
    }

    char symbol = context->sequence[context->index];

    switch (context->state) {
    case PCAN_BLINKY_MORSE_STATE_IDLE:
        if (symbol == '\0') {
            context->sequence = NULL;
            context->state    = PCAN_BLINKY_MORSE_STATE_IDLE;
        } else if (symbol == '.') {
            led_on(led);
            context->next_ms = now_ms + MORSE_DOT_MS;
            context->state   = PCAN_BLINKY_MORSE_STATE_ACTIVE;
        } else if (symbol == '-') {
            led_on(led);
            context->next_ms = now_ms + MORSE_DASH_MS;
            context->state   = PCAN_BLINKY_MORSE_STATE_ACTIVE;
        } else if (symbol == ' ') {
            context->next_ms = now_ms + (MORSE_LETTER_GAP_MS - MORSE_INTRA_GAP_MS);
            context->index++;
            context->state = PCAN_BLINKY_MORSE_STATE_GAP;
        } else if (symbol == '/') {
            context->next_ms = now_ms + (MORSE_WORD_GAP_MS - MORSE_INTRA_GAP_MS);
            context->index++;
            context->state = PCAN_BLINKY_MORSE_STATE_GAP;
        } else {
            context->index++;
        }

        break;

    case PCAN_BLINKY_MORSE_STATE_ACTIVE:
        led_off(led);

        context->next_ms = now_ms + MORSE_INTRA_GAP_MS;
        context->state   = PCAN_BLINKY_MORSE_STATE_GAP;

        break;

    case PCAN_BLINKY_MORSE_STATE_GAP:
        context->index++;
        context->state = PCAN_BLINKY_MORSE_STATE_IDLE;

        break;
    }

    return 0;
}

////////////////////////
// Internal Callbacks //
////////////////////////

int blinky_init(pcan_instance_t *pcan, void *plugin_data) {
    (void)pcan;
    pcan_blinky_instance_t *blinky = plugin_data;

    led_off(blinky->led);

    return 0;
}

int blinky_poll(pcan_instance_t *pcan, void *plugin_data) {
    (void)pcan;
    pcan_blinky_instance_t *blinky = plugin_data;

    uint32_t now_ms = HAL_GetTick();

    if (morse_is_ready(&blinky->morse_context)) {
        bool contention  = blinky->rx_event_occurred && blinky->tx_event_occurred;
        bool rx_priority = blinky->next_event_priority == PCAN_BLINKY_NEXT_EVENT_PRIORITY_RX;
        bool tx_priority = blinky->next_event_priority == PCAN_BLINKY_NEXT_EVENT_PRIORITY_TX;

        if (blinky->rx_event_occurred && (!contention || rx_priority)) {
            blinky->rx_event_occurred   = false;
            blinky->next_event_priority = PCAN_BLINKY_NEXT_EVENT_PRIORITY_TX;
            morse_start(&blinky->morse_context, ".-. "); // RX
        } else if (blinky->tx_event_occurred && (!contention || tx_priority)) {
            blinky->tx_event_occurred   = false;
            blinky->next_event_priority = PCAN_BLINKY_NEXT_EVENT_PRIORITY_RX;
            morse_start(&blinky->morse_context, "- "); // TX
        }
    }

    morse_step(&blinky->morse_context, blinky->led, now_ms);

    return 0;
}

int blinky_on_rx(pcan_instance_t *pcan, void *plugin_data, pcan_message_t *message) {
    (void)pcan;
    pcan_blinky_instance_t *blinky = plugin_data;
    (void)message;

    blinky->rx_event_occurred = true;

    return 0;
}

int blinky_on_tx(pcan_instance_t *pcan, void *plugin_data, pcan_message_t *message) {
    (void)pcan;
    pcan_blinky_instance_t *blinky = plugin_data;
    (void)message;

    blinky->tx_event_occurred = true;

    return 0;
}

//////////////////
// External API //
//////////////////

int pcan_blinky_create(pcan_blinky_instance_t *blinky, pcan_blinky_led_t led) {
    blinky->led                    = led;
    blinky->rx_event_occurred      = false;
    blinky->tx_event_occurred      = false;
    blinky->morse_context.index    = 0;
    blinky->morse_context.next_ms  = 0;
    blinky->morse_context.sequence = NULL;
    blinky->morse_context.state    = PCAN_BLINKY_MORSE_STATE_IDLE;
    blinky->next_event_priority    = PCAN_BLINKY_NEXT_EVENT_PRIORITY_RX;

    return 0;
}

pcan_plugin_t pcan_blinky_plugin(pcan_blinky_instance_t *blinky) {
    return (pcan_plugin_t){
        .free        = false,
        .init        = blinky_init,
        .poll        = blinky_poll,
        .on_rx       = blinky_on_rx,
        .on_tx       = blinky_on_tx,
        .plugin_data = blinky,
    };
}

int pcan_blinky_register_plugin(pcan_instance_t *pcan, pcan_blinky_instance_t *blinky) {
    return pcan_register_plugin(pcan, pcan_blinky_plugin(blinky));
}
