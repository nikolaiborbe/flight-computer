/**
 * @file pcan_core.h
 * @author Paulius Juzenas (paulius.juzenas@propulsentnu.no)
 * @brief Propulse-CAN (or just P-CAN) is a simple CAN protocol and driver.
 * @version 0.1
 * @date 2025-12-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef __PCAN_CORE_H__
#define __PCAN_CORE_H__

#ifdef __cplusplus
extern "C" {
#endif

// Figure out which MCU familiy we are using.
#if defined(STM32G473xx) || defined(STM32G431xx)
#define PCAN_USE_G4
#elif defined(STM32H753xx) || defined(STM32H723xx)
#define PCAN_USE_H7
#else
// Throw and error if we don't recognize the MCU to be safe. If its
// in the same family as one of the above, it can most likely just
// be added to the check above. However, it must be double checked
// and tested that the FDCAN peripherals work identically. In any
// case, open an issue or PR on GitLab if suppport is needed.
#error "P-CAN is not supported on this MCU."
#endif

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef PCAN_USE_G4
#include "stm32g4xx_hal.h"
#endif

#ifdef PCAN_USE_H7
#include "stm32h7xx_hal.h"
#endif

// Check if the FDCAN module is enabled in CubeMX.
#if defined(HAL_MODULE_ENABLED) && !defined(HAL_FDCAN_MODULE_ENABLED)
#error "FDCAN module must be enabled to use P-CAN!"
#endif

// The ID of a P-CAN message is currently the standard 11 bit CAN ID
// and is divided into sections for priority, type, category, instance and channel.
// Here is a ASCII representation of the ID format:
// +---------+----+----+----+----+----+----+----+----+----+----+----+
// | Bit nr. | 10 | 09 | 08 | 07 | 06 | 05 | 04 | 03 | 02 | 01 | 00 |
// +---------+----+----+----+----+----+----+----+----+----+----+----+
// | Field   | P. | T. | Category     | Instance     | Channel      |
// +---------+----+----+--------------+--------------+--------------+

// The positions of the different sections of the ID.
#define PCAN_ID_POSITION_PRIORITY (10)
#define PCAN_ID_POSITION_TYPE     (9)
#define PCAN_ID_POSITION_CATEGORY (6)
#define PCAN_ID_POSITION_INSTANCE (3)
#define PCAN_ID_POSITION_CHANNEL  (0)

// Widths of the different sections of the ID.
#define PCAN_ID_WIDTH_PRIORITY (1)
#define PCAN_ID_WIDTH_TYPE     (1)
#define PCAN_ID_WIDTH_CATEGORY (3)
#define PCAN_ID_WIDTH_INSTANCE (3)
#define PCAN_ID_WIDTH_CHANNEL  (3)

// Utility to create masks from position and width.
#define PCAN_ID_MASK(p, w) (((1 << (w)) - 1) << (p))

// Masks for the different sections of the ID.
#define PCAN_ID_MASK_PRIORITY (PCAN_ID_MASK(PCAN_ID_POSITION_PRIORITY, PCAN_ID_WIDTH_PRIORITY))
#define PCAN_ID_MASK_TYPE     (PCAN_ID_MASK(PCAN_ID_POSITION_TYPE, PCAN_ID_WIDTH_TYPE))
#define PCAN_ID_MASK_CATEGORY (PCAN_ID_MASK(PCAN_ID_POSITION_CATEGORY, PCAN_ID_WIDTH_CATEGORY))
#define PCAN_ID_MASK_INSTANCE (PCAN_ID_MASK(PCAN_ID_POSITION_INSTANCE, PCAN_ID_WIDTH_INSTANCE))
#define PCAN_ID_MASK_CHANNEL  (PCAN_ID_MASK(PCAN_ID_POSITION_CHANNEL, PCAN_ID_WIDTH_CHANNEL))

// Special values for "any" in category, instance, and channel fields.
#define PCAN_CATEGORY_ANY (0xFF)
#define PCAN_INSTANCE_ANY (0xFF)
#define PCAN_CHANNEL_ANY  (0xFF)

// Masks for the full standard and extended CAN ID.
#define PCAN_STANDARD_CAN_ID_MASK (0x7FF)
#define PCAN_EXTENDED_CAN_ID_MASK (0x1FFFFFFF)

// Byte alignment used in the P-CAN storage.
// This should be set to the largest alignment on the system.
#ifndef PCAN_ALIGNMENT
#define PCAN_ALIGNMENT (4)
#endif

// Utility for aligning up pointers to the system alignment.
#define PCAN_ALIGN_UP(type, size)                                                                  \
    ((type)(((uintptr_t)(size) + ((uintptr_t)PCAN_ALIGNMENT - 1)) &                                \
            ~((uintptr_t)PCAN_ALIGNMENT - 1)))

// Utility for calculating the size of aligned arrays in the P-CAN storage.
#define PCAN_ALIGNED_UP_ARRAY_SIZE(type, count) (PCAN_ALIGN_UP(size_t, sizeof(type) * (count)))

/**
 * @brief Represents the ID of a node in the P-CAN network.
 */
typedef struct {
    uint8_t category_id;
    uint8_t instance_id;
} pcan_node_id_t;

// Utility for creating a node ID.
#define PCAN_NODE_ID(category_id, instance_id) ((pcan_node_id_t){category_id, instance_id})

/**
 * @brief Represents the ID of a node and channel in the P-CAN network.
 */
typedef struct {
    pcan_node_id_t node_id;
    uint8_t channel_id;
} pcan_endpoint_id_t;

// Utility for creating an endpoint ID.
#define PCAN_ENDPOINT_ID(node_id, channel_id) ((pcan_endpoint_id_t){node_id, channel_id})

/**
 * @brief The priority levels for P-CAN messages.
 */
typedef enum {
    PCAN_PRIORITY_HIGH = 0,
    PCAN_PRIORITY_LOW  = 1,
} pcan_priority_t;

/**
 * @brief The possible types of a P-CAN message.
 */
typedef enum {
    PCAN_TYPE_COMMAND   = 0,
    PCAN_TYPE_TELEMETRY = 1,
} pcan_type_t;

/**
 * @brief A received P-CAN message. This is what is passed to subscription callbacks.
 */
typedef struct {
    uint8_t data[64];
    size_t size;
    pcan_node_id_t node_id;
    uint8_t channel_id;
    uint32_t raw_id;
    uint32_t bitrate_switch;
    uint64_t timestamp_us;
} pcan_message_t;

/**
 * @brief The signature for P-CAN message callbacks.
 */
typedef void (*pcan_callback_t)(pcan_message_t *message, void *user_data);

/**
 * @brief A single element in the P-CAN subscription list.
 */
typedef struct {
    bool free;
    uint32_t id;
    uint32_t mask;
    pcan_callback_t callback;
    void *user_data;
} pcan_subscription_t;

typedef struct PCanInstance pcan_instance_t;

/**
 * @brief Generic plugin poll function which will be called in the main poll loop if set.
 */
typedef struct {
    bool free;
    int (*init)(pcan_instance_t *pcan, void *plugin_data);
    int (*poll)(pcan_instance_t *pcan, void *plugin_data);
    int (*on_rx)(pcan_instance_t *pcan, void *plugin_data, pcan_message_t *message);
    int (*on_tx)(pcan_instance_t *pcan, void *plugin_data, pcan_message_t *message);
    void *plugin_data;
} pcan_plugin_t;

// Helper to calculate the required size for P-CAN storage.
#define PCAN_STORAGE_SIZE(max_subscription_count, max_plugin_count, max_rx_message_count,          \
                          max_tx_message_count)                                                    \
    (PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_subscription_t, max_subscription_count) +                     \
     PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_plugin_t, max_plugin_count) +                                 \
     PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_message_t, max_rx_message_count) +                            \
     PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_message_t, max_tx_message_count) +                            \
     (PCAN_ALIGNMENT - 1)) // Compensate for worst case if base pointer is not aligned.

typedef struct {
    pcan_message_t *messages;
    size_t max_message_count;
    size_t head;
    size_t tail;
} pcan_message_ringbuffer_t;

/**
 * @brief Generic set of mutex functions which will be used if present.
 */
typedef struct {
    void (*lock)(void *context);
    void (*unlock)(void *context);
    void *context;
} pcan_mutex_t;

/**
 * @brief Generic set of semaphore functions which will be used if present.
 */
typedef struct {
    void (*get)(void *context);
    void (*put)(void *context);
    void *context;
} pcan_semaphore_t;

/**
 * @brief Speed configuration for CAN peripheral.
 *
 * NOTE: Presets are tuned for a 50 MHz FDCAN kernel clock.
 */
typedef struct {
    uint32_t prescaler;
    uint32_t sync_jump_width;
    uint32_t time_segment_1;
    uint32_t time_segment_2;
} pcan_speed_config_t;

#define PCAN_SPEED_CONFIG_5_MBIT                                                                     \
    ((pcan_speed_config_t){.prescaler = 1, .sync_jump_width = 2, .time_segment_1 = 7,              \
                           .time_segment_2 = 2})
#define PCAN_SPEED_CONFIG_2_MBIT                                                                     \
    ((pcan_speed_config_t){.prescaler = 1, .sync_jump_width = 5, .time_segment_1 = 18,             \
                           .time_segment_2 = 6})
#define PCAN_SPEED_CONFIG_1_MBIT                                                                     \
    ((pcan_speed_config_t){.prescaler = 5, .sync_jump_width = 1, .time_segment_1 = 7,              \
                           .time_segment_2 = 2})

/**
 * @brief Defines resource sizes for a P-CAN instance.
 */
typedef struct {
    size_t max_subscription_count;
    size_t max_plugin_count;
    size_t max_rx_message_count;
    size_t max_tx_message_count;
    pcan_speed_config_t nominal_speed;
    pcan_speed_config_t data_speed;
    bool bit_rate_switching_on;
} pcan_instance_config_t;

/**
 * @brief Returns the size of the required storage in bytes for the given P-CAN config.
 *
 * @param config P-CAN configuration struct.
 * @return size_t Size in bytes.
 */
size_t pcan_storage_size(pcan_instance_config_t config);

/**
 * @brief Structure to hold P-CAN instance data.
 */
typedef struct PCanInstance {
    FDCAN_HandleTypeDef *hfdcan;
    pcan_node_id_t node_id;
    pcan_subscription_t *subscriptions;
    size_t max_subscription_count;
    pcan_plugin_t *plugins;
    size_t max_plugin_count;
    bool bit_rate_switching_on;
    pcan_speed_config_t nominal_speed;
    pcan_speed_config_t data_speed;
    pcan_message_ringbuffer_t rx_buffer;
    pcan_message_ringbuffer_t tx_buffer;
    pcan_mutex_t mutex;
    pcan_semaphore_t semaphore;
} pcan_instance_t;

/**
 * @brief Creates a P-CAN instance and FDCAN peripheral with the specified parameters.
 *
 * @warning This does not start the FDCAN peripheral, so messages will not be sent or received until
 * that is done.
 *
 * @param pcan A P-CAN struct instance to initialize.
 * @param category The category of this node.
 * @param instance The instance number of this node.
 * @param subscriptions Pointer to an array of subscription structs.
 * @param max_subscription_count The length of the subscription array.
 * @param hfdcan Pointer to the FDCAN peripheral to use.
 * @return int Negative on failure, non-negative on success.
 */
int pcan_create(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan, pcan_node_id_t node_id,
                pcan_instance_config_t config, uint8_t *storage, size_t storage_size);

/**
 * @brief Adds a plugin to the P-CAN instance.
 *
 * @param pcan The P-CAN instance to modify.
 * @param plugin The plugin interface to add.
 * @return int Negative on failure, non-negative on success.
 */
int pcan_register_plugin(pcan_instance_t *pcan, pcan_plugin_t plugin);

/**
 * @brief Initializes a P-CAN instance. This should be called after `pcan_create` and before any
 * other P-CAN functions.
 *
 * @param pcan The P-CAN instance to initialize.
 * @return int Negative on failure, non-negative on success.
 */
int pcan_init(pcan_instance_t *pcan);

/**
 * @brief Set up a mutex for a P-CAN instance.
 *
 * @param pcan The P-CAN instance to modify.
 * @param lock Function which locks the mutex.
 * @param unlock Function which unlocks the mutex.
 * @param context Context to pass to lock and unlock functions.
 * @return int Negative on failure, non-negative on success.
 */
int pcan_set_mutex(pcan_instance_t *pcan, void (*lock)(void *), void (*unlock)(void *),
                   void *context);

/**
 * @brief Set up a semaphore for a P-CAN instance.
 *
 * @param pcan The P-CAN instance to modify.
 * @param get Function which gets the mutex.
 * @param put Function which puts the mutex.
 * @param context Context to pass to get and put functions.
 * @return int Negative on failure, non-negative on success.
 */
int pcan_set_semaphore(pcan_instance_t *pcan, void (*get)(void *), void (*put)(void *),
                       void *context);

/**
 * @brief Publish a message with given id. Should only be used in dire circumstances!
 */
int pcan_publish_raw(pcan_instance_t *pcan, uint32_t id, uint8_t *data, size_t size);

/**
 * @brief Publish a command message to the specified endpoint.
 */
int pcan_publish_command(pcan_instance_t *pcan, pcan_priority_t priority,
                         pcan_endpoint_id_t endpoint_id, uint8_t *data, size_t size);

/**
 * @brief Publish a telemetry message from this node on the specified channel.
 */
int pcan_publish_telemetry(pcan_instance_t *pcan, pcan_priority_t priority, uint8_t channel_id,
                           uint8_t *data, size_t size);

/**
 * @brief Subscribe to messages with given id. Should only be used in dire circumstances!
 */
int pcan_subscribe_raw(pcan_instance_t *pcan, uint32_t id, uint32_t mask, pcan_callback_t callback,
                       void *user_data);

/**
 * @brief Subscribe to incomming commands to this node on the specified channel.
 */
int pcan_subscribe_command(pcan_instance_t *pcan, uint8_t channel_id, pcan_callback_t callback,
                           void *user_data);

/**
 * @brief Subscribe to telemetry messages from specified endpoint.
 */
int pcan_subscribe_telemetry(pcan_instance_t *pcan, pcan_endpoint_id_t endpoint_id,
                             pcan_callback_t callback, void *user_data);

/**
 * @brief Unsubscribe subscription with given id.
 */
int pcan_unsubscribe(pcan_instance_t *pcan, int subscription_id, void **out_user_data);

/**
 * @brief Cycles through received messages and calls relevant subscription callbacks.
 *
 * @param pcan The P-CAN instance to poll.
 * @return int If this is ever non-zero, something has seriously gone wrong. x_x
 */
int pcan_poll(pcan_instance_t *pcan);

/**
 * @brief Function to be called when a message is received in the FDCAN RX FIFO 0.
 *
 * When using HAL, this should be called from within `HAL_FDCAN_RxFifo0Callback`.
 *
 * @param pcan Any P-CAN instance. Function will check if it owns the peripheral before doing
 * anything.
 * @param hfdcan The FDCAN peripheral which received the message.
 * @param RxFifo0ITs The interrupt flags for the RX FIFO 0 interrupt.
 */
void pcan_on_rx_fifo0_receive(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan,
                              uint32_t rx_fifo0_its);

/**
 * @brief Function to be called when a message the FDCAN TX FIFO becomes empty.
 *
 * When using HAL, this should be called from within `HAL_FDCAN_TxFifoEmptyCallback`.
 *
 * @param pcan Any P-CAN instance. Function will check if it owns the peripheral before doing
 * anything.
 * @param hfdcan The FDCAN peripheral which triggered the interrupt.
 */
void pcan_on_tx_fifo_empty(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief Function to be called when the FDCAN peripheral switches state.
 *
 * When using HAL, this should be called from within `HAL_FDCAN_ErrorStatusCallback`.
 *
 * @param pcan Any P-CAN instance. Function will check if it owns the peripheral before doing
 * anything.
 * @param hfdcan The FDCAN peripheral which triggered the interrupt.
 * @param error_status_its The interrupt flags for the error status interrupt.
 */
void pcan_on_error_status(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan,
                          uint32_t error_status_its);

#define PCAN_DEFINE_CALLBACKS_C(...)                                                               \
    static pcan_instance_t *const __pcan_instances[] = {__VA_ARGS__};                              \
    static const size_t __pcan_instance_count =                                                    \
        sizeof(__pcan_instances) / sizeof(__pcan_instances[0]);                                    \
                                                                                                   \
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its) {           \
        for (size_t i = 0; i < __pcan_instance_count; i++) {                                       \
            pcan_on_rx_fifo0_receive(__pcan_instances[i], hfdcan, rx_fifo0_its);                   \
        }                                                                                          \
    }                                                                                              \
                                                                                                   \
    void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {                              \
        for (size_t i = 0; i < __pcan_instance_count; i++) {                                       \
            pcan_on_tx_fifo_empty(__pcan_instances[i], hfdcan);                                    \
        }                                                                                          \
    }                                                                                              \
                                                                                                   \
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its) {   \
        for (size_t i = 0; i < __pcan_instance_count; i++) {                                       \
            pcan_on_error_status(__pcan_instances[i], hfdcan, error_status_its);                   \
        }                                                                                          \
    }

#ifdef __cplusplus
}
#endif

#endif // __PCAN_CORE_H__
