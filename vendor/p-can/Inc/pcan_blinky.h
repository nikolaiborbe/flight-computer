#ifndef __PCAN_BLINKY_H__
#define __PCAN_BLINKY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "pcan_core.h"

typedef enum {
    PCAN_BLINKY_MORSE_STATE_IDLE,
    PCAN_BLINKY_MORSE_STATE_ACTIVE,
    PCAN_BLINKY_MORSE_STATE_GAP,
} pcan_blinky_morse_state_t;

typedef struct {
    const char *sequence;
    uint32_t index;
    uint32_t next_ms;
    pcan_blinky_morse_state_t state;
} pcan_blinky_morse_context_t;

typedef enum {
    PCAN_BLINKY_LED_ACTIVE_HIGH,
    PCAN_BLINKY_LED_ACTIVE_LOW,
} pcan_blinky_led_behavior_t;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    pcan_blinky_led_behavior_t behavior;
} pcan_blinky_led_t;

typedef enum {
    PCAN_BLINKY_NEXT_EVENT_PRIORITY_RX,
    PCAN_BLINKY_NEXT_EVENT_PRIORITY_TX,
} pcan_blinky_next_event_priority_t;

typedef struct {
    volatile bool rx_event_occurred;
    volatile bool tx_event_occurred;
    pcan_blinky_led_t led;
    pcan_blinky_morse_context_t morse_context;
    pcan_blinky_next_event_priority_t next_event_priority;
} pcan_blinky_instance_t;

int pcan_blinky_create(pcan_blinky_instance_t *blinky, pcan_blinky_led_t led);

pcan_plugin_t pcan_blinky_plugin(pcan_blinky_instance_t *blinky);

int pcan_blinky_register_plugin(pcan_instance_t *pcan, pcan_blinky_instance_t *blinky);

#ifdef __cplusplus
}
#endif

#endif // __PCAN_BLINKY_H__
