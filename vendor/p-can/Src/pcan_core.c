/**
 * @file pcan.c
 * @author Paulius Juzenas (paulius.juzenas@propulsentnu.no)
 * @brief Implementation of the Propulse-CAN (P-CAN) protocol and driver.
 * @version 0.1
 * @date 2025-12-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "pcan_core.h"

///////////////
// Utilities //
///////////////

#ifdef PCAN_USE_H7
/**
 * @brief Utility function to calculate the message RAM offset for a given FDCAN peripheral on the
 * H7.
 *
 * The H7 has a shared message RAM for all FDCAN peripherals. For simplicity P-CAN divides this RAM
 * into three equal parts and assigns one part to each peripheral. This function calculates the
 * offset for a given peripheral by checking its base address and returning the corresponding
 * offset.
 */
static inline uint32_t calculate_message_ram_offset(FDCAN_HandleTypeDef *hfdcan) {
    // The size of the shared message ram in words.
    // Value taken from 61.4.2 in the datasheet.
    const uint32_t message_ram_size    = 2560;
    const uint32_t peripheral_ram_size = message_ram_size / 3;

    switch ((uintptr_t)hfdcan->Instance) {
    case FDCAN1_BASE: return 0 * peripheral_ram_size;
    case FDCAN2_BASE: return 1 * peripheral_ram_size;
#if defined(STM32H723xx)
    case FDCAN3_BASE: return 2 * peripheral_ram_size;
#endif
    default:
        // This should never happen if the FDCAN peripheral is valid.
        return 0xFFFFFFFF;
    }
}
#endif

/**
 * @brief Utility function for merging P-CAN ID fields into a single 11-bit ID.
 */
static inline uint16_t make_pcan_id(pcan_priority_t priority, pcan_type_t type,
                                    pcan_endpoint_id_t endpoint_id) {
    return (
        ((priority << PCAN_ID_POSITION_PRIORITY) & PCAN_ID_MASK_PRIORITY) |
        ((type << PCAN_ID_POSITION_TYPE) & PCAN_ID_MASK_TYPE) |
        ((endpoint_id.node_id.category_id << PCAN_ID_POSITION_CATEGORY) & PCAN_ID_MASK_CATEGORY) |
        ((endpoint_id.node_id.instance_id << PCAN_ID_POSITION_INSTANCE) & PCAN_ID_MASK_INSTANCE) |
        ((endpoint_id.channel_id << PCAN_ID_POSITION_CHANNEL) & PCAN_ID_MASK_CHANNEL));
}

/**
 * @brief Utility function for creating a mask for P-CAN ID fields.
 *
 * NOTE: The priority bit is always masked out since its only used for arbitration.
 */
static inline uint16_t make_pcan_mask(pcan_endpoint_id_t endpoint_id) {
    return ((PCAN_ID_MASK_TYPE) |
            ((endpoint_id.node_id.category_id == PCAN_CATEGORY_ANY) ? 0 : PCAN_ID_MASK_CATEGORY) |
            ((endpoint_id.node_id.instance_id == PCAN_INSTANCE_ANY) ? 0 : PCAN_ID_MASK_INSTANCE) |
            ((endpoint_id.channel_id == PCAN_CHANNEL_ANY) ? 0 : PCAN_ID_MASK_CHANNEL));
}

/**
 * @brief Utility function for extracting the category and instance field from a P-CAN ID.
 */
static inline pcan_node_id_t extract_pcan_node_id(uint16_t id) {
    return (pcan_node_id_t){
        .category_id = (id & PCAN_ID_MASK_CATEGORY) >> PCAN_ID_POSITION_CATEGORY,
        .instance_id = (id & PCAN_ID_MASK_INSTANCE) >> PCAN_ID_POSITION_INSTANCE};
}

/**
 * @brief Utility function for extracting the channel field from a P-CAN ID.
 */
static inline uint8_t extract_pcan_channel_id(uint16_t id) {
    return (id & PCAN_ID_MASK_CHANNEL) >> PCAN_ID_POSITION_CHANNEL;
}

/**
 * @brief Utility function for converting a data length to a CAN FD Data Length Code (DLC).
 *
 * @param length The length of the data in bytes. Cannot be greater than 64.
 * @return uint32_t The corresponding CAN FD DLC value.
 */
static uint32_t length_to_can_fd_dlc(size_t length) {
    // For classic CAN, data length values up to eight are one-to-one with DLC values.
    // clang-format off
    if (length <= 8) return length;
    // clang-format on

    // For CAN FD, data length values above eight are mapped to specific DLC values.
    // clang-format off
    if (length <= 12) return FDCAN_DLC_BYTES_12;
    if (length <= 16) return FDCAN_DLC_BYTES_16;
    if (length <= 20) return FDCAN_DLC_BYTES_20;
    if (length <= 24) return FDCAN_DLC_BYTES_24;
    if (length <= 32) return FDCAN_DLC_BYTES_32;
    if (length <= 48) return FDCAN_DLC_BYTES_48;
    if (length <= 64) return FDCAN_DLC_BYTES_64;
    // clang-format on

    // User error if length is greater than 64. :P
    return FDCAN_DLC_BYTES_64;
}

/**
 * @brief Convert a CAN FD Data Length Code (DLC) to the corresponding data length in bytes.
 *
 * @param dlc The DLC value.
 * @return size_t The corresponding data length in bytes.
 */
static size_t can_fd_dlc_to_length(uint32_t dlc) {
    // Classic CAN has a one-to-one mapping between DLC and length.
    // clang-format off
    if (dlc <= FDCAN_DLC_BYTES_8) return dlc;
    // clang-format on

    // CAN FD follows a non-linear mapping.
    // clang-format off
    if (dlc <= FDCAN_DLC_BYTES_12) return 12;
    if (dlc <= FDCAN_DLC_BYTES_16) return 16;
    if (dlc <= FDCAN_DLC_BYTES_20) return 20;
    if (dlc <= FDCAN_DLC_BYTES_24) return 24;
    if (dlc <= FDCAN_DLC_BYTES_32) return 32;
    if (dlc <= FDCAN_DLC_BYTES_48) return 48;
    // clang-format on

    // The maximum length for CAN FD is 64 bytes.
    return 64;
}

/**
 * @brief Utility for getting number of full slots in a ringbuffer.
 *
 * @param ringbuffer The ringbuffer to check.
 * @return size_t The number of full slots.
 */
static size_t ringbuffer_full_count(pcan_message_ringbuffer_t *ringbuffer) {
    return (ringbuffer->max_message_count + ringbuffer->head - ringbuffer->tail) %
           ringbuffer->max_message_count;
}

/**
 * @brief Utility for getting number of empty slots in a ringbuffer.
 *
 * @param ringbuffer The ringbuffer to check.
 * @return size_t The number of empty slots.
 */
static size_t ringbuffer_empty_count(pcan_message_ringbuffer_t *ringbuffer) {
    return ringbuffer->max_message_count - ringbuffer_full_count(ringbuffer) - 1;
}

/**
 * @brief Writes a message onto the ringbuffer.
 *
 * @param ringbuffer The ringbuffer to write the message onto.
 * @param message The message to write.
 * @return int Number of empty slots left.
 */
static int ringbuffer_push(pcan_message_ringbuffer_t *ringbuffer, pcan_message_t *message) {
    size_t next_head = (ringbuffer->head + 1) % ringbuffer->max_message_count;

    // Reject if ringbuffer is full.
    if (next_head == ringbuffer->tail) {
        return -1;
    }

    ringbuffer->messages[ringbuffer->head] = *message;
    __DMB(); // Ensure that the message data is written before updating the head index.
    ringbuffer->head = next_head;
    return 0;
}

/**
 * @brief Reads a message from the ringbuffer.
 *
 * @param ringbuffer The ringbuffer to read the message from.
 * @param out_message The read message.
 * @return int New number of full slots.
 */
static int ringbuffer_pop(pcan_message_ringbuffer_t *ringbuffer, pcan_message_t *out_message) {
    // Reject if ringbuffer is empty.
    if (ringbuffer->head == ringbuffer->tail) {
        return -1;
    }

    *out_message = ringbuffer->messages[ringbuffer->tail];
    __DMB(); // Ensure that the message data is read before updating the tail index.
    ringbuffer->tail = (ringbuffer->tail + 1) % ringbuffer->max_message_count;
    return 0;
}

/**
 * @brief Utility function for converting a P-CAN message to an FDCAN TxHeader struct.
 *
 * @param message The P-CAN message.
 * @return FDCAN_TxHeaderTypeDef The corresponding FDCAN TxHeader struct.
 */
static inline FDCAN_TxHeaderTypeDef pcan_message_to_fdcan_tx(pcan_message_t *message) {
    return (FDCAN_TxHeaderTypeDef){
        .Identifier          = message->raw_id,
        .IdType              = message->raw_id > 0x7FF ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
        .TxFrameType         = FDCAN_DATA_FRAME,
        .DataLength          = length_to_can_fd_dlc(message->size),
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch       = message->bitrate_switch,
        .FDFormat            = FDCAN_FD_CAN,
        .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
        .MessageMarker       = 0,
    };
}

/**
 * @brief Utility function for converting an FDCAN RxHeader struct and data to a P-CAN message.
 *
 * @param rx_header The FDCAN RxHeader struct.
 * @param rx_data The received data.
 * @return pcan_message_t The corresponding P-CAN message.
 */
static inline pcan_message_t fdcan_rx_to_pcan_message(FDCAN_RxHeaderTypeDef *rx_header,
                                                      uint8_t (*rx_data)[64]) {
    pcan_message_t message = {
        .data           = {0},
        .size           = can_fd_dlc_to_length(rx_header->DataLength),
        .node_id        = extract_pcan_node_id(rx_header->Identifier),
        .channel_id     = extract_pcan_channel_id(rx_header->Identifier),
        .raw_id         = rx_header->Identifier,
        .bitrate_switch = rx_header->BitRateSwitch,
    };
    memcpy(message.data, *rx_data, message.size);
    return message;
}

/**
 * @brief Internal utility function to lock the mutex if set up.
 *
 * @param pcan The P-CAN instance to lock.
 */
static inline void lock_mutex(pcan_instance_t *pcan) {
    if (pcan->mutex.lock) {
        pcan->mutex.lock(pcan->mutex.context);
    }
}

/**
 * @brief Internal utility function to unlock the mutex if set up.
 *
 * @param pcan The P-CAN instance to unlock.
 */
static inline void unlock_mutex(pcan_instance_t *pcan) {
    if (pcan->mutex.unlock) {
        pcan->mutex.unlock(pcan->mutex.context);
    }
}

/**
 * @brief Internal utility function to get the semaphore if set up.
 *
 * @param pcan The P-CAN instance with semaphore to put.
 */
static inline void get_semaphore(pcan_instance_t *pcan) {
    if (pcan->semaphore.get) {
        pcan->semaphore.get(pcan->semaphore.context);
    }
}

/**
 * @brief Internal utility function to put the semaphore if set up.
 *
 * @param pcan The P-CAN instance with semaphore to put.
 */
static inline void put_semaphore(pcan_instance_t *pcan) {
    if (pcan->semaphore.put) {
        pcan->semaphore.put(pcan->semaphore.context);
    }
}

/**
 * @brief Moves as many messages as possible from the FDCAN RX FIFO to the P-CAN RX ringbuffer.
 *
 * @param pcan The P-CAN instance to move the messages to.
 * @param hfdcan The FDCAN peripheral to read the messages from.
 */
static void flush_rx_fifo(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan) {
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0 &&
           ringbuffer_empty_count(&pcan->rx_buffer) > 0) {
        // Try to retrieve a message from message queue.
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[64];
        if (HAL_FDCAN_GetRxMessage(pcan->hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            // This should never happen.
            break;
        }

        pcan_message_t message = fdcan_rx_to_pcan_message(&rx_header, &rx_data);

        for (size_t i = 0; i < pcan->max_plugin_count; i++) {
            pcan_plugin_t *plugin = &pcan->plugins[i];

            if (plugin->free || plugin->poll == NULL)
                continue;

            plugin->on_rx(pcan, plugin->plugin_data, &message);
        }

        // Try to push to ringbuffer.
        // We have already checked if there is enough space so it should not fail.
        ringbuffer_push(&pcan->rx_buffer, &message);
    }

    // Notify the semaphore if there is at least one pending message.
    if (ringbuffer_full_count(&pcan->rx_buffer) > 0) {
        put_semaphore(pcan);
    }
}

/**
 * @brief Moves as many messages as possible from the P-CAN TX ringbuffer to the FDCAN TX FIFO.
 *
 * @param pcan The P-CAN instance to move the messages from.
 * @param hfdcan The FDCAN peripheral to add the messages to.
 */
static void flush_tx_ringbuffer(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan) {
    // Fill up the FDCAN TX FIFO with any pending messages until either the FIFO is full or there
    // are no more pending messages.
    pcan_message_t message;
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0 &&
           ringbuffer_full_count(&pcan->tx_buffer) > 0) {
        if (ringbuffer_pop(&pcan->tx_buffer, &message) < 0) {
            // This should never happen as we checked that there is pending messages.
            break;
        }

        for (size_t i = 0; i < pcan->max_plugin_count; i++) {
            pcan_plugin_t *plugin = &pcan->plugins[i];

            if (plugin->free || plugin->poll == NULL)
                continue;

            plugin->on_tx(pcan, plugin->plugin_data, &message);
        }

        FDCAN_TxHeaderTypeDef tx_header = pcan_message_to_fdcan_tx(&message);
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, message.data) != HAL_OK) {
            // Adding to the queue should not fail as we checked that there is space.
            break;
        }
    }
}

/**
 * @brief Gets the first active subscribtion if it exists.
 *
 * @param pcan The P-CAN instance to search.
 * @return pcan_subscription_t* Pointer to the subscription if found, NULL otherwise.
 */
static pcan_subscription_t *get_first_subscription(pcan_instance_t *pcan) {
    pcan_subscription_t *subscription = NULL;

    for (int i = 0; i < pcan->max_subscription_count; i++) {
        if (!pcan->subscriptions[i].free) {
            subscription = &pcan->subscriptions[i];
            break;
        }
    }

    return subscription;
}

/**
 * @brief Computes a mask and id for a single CAN filter for all active subscriptions.
 *
 * @param pcan The P-CAN instance to compute a filter for.
 * @param out_filter_id Output variable to store resulting filter id in.
 * @param out_filter_mask Output variable to store resulting filter mask in.
 * @return int Negative on failure, non-negative otherwise.
 */
static int compute_single_filter(pcan_instance_t *pcan, uint32_t *out_filter_id,
                                 uint32_t *out_filter_mask) {
    pcan_subscription_t *first_subscription = get_first_subscription(pcan);

    // Edge-case where no active subscriptions exist.
    if (first_subscription == NULL) {
        // Set filter to a combination which will never accept any messages.
        *out_filter_id   = 0xFFFFFFFF;
        *out_filter_mask = 0xFFFFFFFF;
        return 0;
    }

    uint32_t filter_id   = first_subscription->id;
    uint32_t filter_mask = first_subscription->mask;

    for (int i = 0; i < pcan->max_subscription_count; i++) {
        if (pcan->subscriptions[i].free) {
            continue;
        }

        // Mask which is 1 for same bits and 0 for different bits.
        uint32_t overlap = ~(filter_id ^ pcan->subscriptions[i].id);

        // If any bits are different, we can't filter against them.
        filter_mask &= overlap;

        // If this subscription doesn't care about some bits, we can't filter againt them too.
        filter_mask &= pcan->subscriptions[i].mask;

        // Set bits in the filter id which are different to zero.
        filter_id &= overlap;
    }

    *out_filter_id   = filter_id;
    *out_filter_mask = filter_mask;

    return 0;
}
/**
 * @brief Updates the filters on the FDCAN peripheral to reflect the active subscriptions.
 *
 * @param pcan The P-CAN instance to use.
 * @return int Negative on failure, non-negative otherwise.
 */
static int update_filters(pcan_instance_t *pcan) {
    uint32_t filter_id;
    uint32_t filter_mask;

    if (compute_single_filter(pcan, &filter_id, &filter_mask) < 0) {
        return -1;
    }

    FDCAN_FilterTypeDef filter_config = {
        .IdType       = FDCAN_EXTENDED_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = filter_id & PCAN_EXTENDED_CAN_ID_MASK,
        .FilterID2    = filter_mask & PCAN_EXTENDED_CAN_ID_MASK,
#ifndef PCAN_USE_G4
        .RxBufferIndex    = FDCAN_RX_FIFO0,
        .IsCalibrationMsg = 0,
#endif
    };

    if (HAL_FDCAN_ConfigFilter(pcan->hfdcan, &filter_config) != HAL_OK) {
        return -1;
    }

    filter_config.IdType    = FDCAN_STANDARD_ID;
    filter_config.FilterID1 = filter_id & PCAN_STANDARD_CAN_ID_MASK;
    filter_config.FilterID2 = filter_mask & PCAN_STANDARD_CAN_ID_MASK;

    if (HAL_FDCAN_ConfigFilter(pcan->hfdcan, &filter_config) != HAL_OK) {
        return -1;
    }

    return 0;
}

////////////////////
// Initialization //
////////////////////

size_t pcan_storage_size(pcan_instance_config_t config) {
    return PCAN_STORAGE_SIZE(config.max_subscription_count, config.max_plugin_count,
                             config.max_rx_message_count, config.max_tx_message_count);
}

int pcan_create(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan, pcan_node_id_t node_id,
                pcan_instance_config_t config, uint8_t *storage, size_t storage_size) {
    if (pcan == NULL || hfdcan == NULL || storage == NULL) {
        return -1;
    }

    // Initalize instance fields.
    pcan->hfdcan                      = hfdcan;
    pcan->node_id                     = node_id;
    pcan->max_subscription_count      = config.max_subscription_count;
    pcan->max_plugin_count            = config.max_plugin_count;
    pcan->bit_rate_switching_on       = config.bit_rate_switching_on;
    pcan->nominal_speed               = config.nominal_speed;
    pcan->data_speed                  = config.data_speed;
    pcan->rx_buffer.max_message_count = config.max_rx_message_count;
    pcan->rx_buffer.head              = 0;
    pcan->rx_buffer.tail              = 0;
    pcan->tx_buffer.max_message_count = config.max_tx_message_count;
    pcan->tx_buffer.head              = 0;
    pcan->tx_buffer.tail              = 0;
    pcan->mutex                       = (pcan_mutex_t){0};
    pcan->semaphore                   = (pcan_semaphore_t){0};

    // Initalize storage pointsers.
    uint8_t *base = PCAN_ALIGN_UP(uint8_t *, storage);

    pcan->subscriptions = (pcan_subscription_t *)base;
    base += PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_subscription_t, pcan->max_subscription_count);

    pcan->plugins = (pcan_plugin_t *)base;
    base += PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_plugin_t, pcan->max_plugin_count);

    pcan->rx_buffer.messages = (pcan_message_t *)base;
    base += PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_message_t, pcan->rx_buffer.max_message_count);

    pcan->tx_buffer.messages = (pcan_message_t *)base;
    base += PCAN_ALIGNED_UP_ARRAY_SIZE(pcan_message_t, pcan->tx_buffer.max_message_count);

    // Check if enough storage was provided.
    if ((size_t)(base - storage) >= storage_size) {
        return -1;
    }

    // Mark all subscription slots as free.
    for (size_t i = 0; i < pcan->max_subscription_count; i++) {
        pcan->subscriptions[i].free = true;
    }

    // Mark all plugin slots as free.
    for (size_t i = 0; i < pcan->max_plugin_count; i++) {
        pcan->plugins[i].free = true;
    }

    return 0;
}

int pcan_register_plugin(pcan_instance_t *pcan, pcan_plugin_t plugin) {
    // Find the first free plugin slot and add the plugin there.
    for (size_t i = 0; i < pcan->max_plugin_count; i++) {
        if (!pcan->plugins[i].free) {
            continue;
        }

        pcan->plugins[i] = plugin;

        return 0;
    }

    return -1;
}

int pcan_init(pcan_instance_t *pcan) {
    FDCAN_HandleTypeDef *hfdcan = pcan->hfdcan;

    // Stop CAN peripheral in case it has for some reason already been started.
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK && hfdcan->ErrorCode != HAL_FDCAN_ERROR_NOT_STARTED) {
        return -1;
    };

    // TODO: Currently this driver assumes that the FDCAN peripheral
    // is clocked at 50 MHz. It would be better if it could check
    // the clock speed and adjust the timing parameters accordingly.

    // Configure CAN peripheral independently of CubeMX.
    // The first fields are supported by both the H7 and G4.

    // Always accept BRS messages.
    hfdcan->Init.FrameFormat          = FDCAN_FRAME_FD_BRS;
    hfdcan->Init.Mode                 = FDCAN_MODE_NORMAL;
    hfdcan->Init.AutoRetransmission   = ENABLE;
    hfdcan->Init.TransmitPause        = ENABLE;
    hfdcan->Init.ProtocolException    = ENABLE;
    hfdcan->Init.NominalPrescaler     = pcan->nominal_speed.prescaler;
    hfdcan->Init.NominalSyncJumpWidth = pcan->nominal_speed.sync_jump_width;
    hfdcan->Init.NominalTimeSeg1      = pcan->nominal_speed.time_segment_1;
    hfdcan->Init.NominalTimeSeg2      = pcan->nominal_speed.time_segment_2;
    hfdcan->Init.DataPrescaler        = pcan->data_speed.prescaler;
    hfdcan->Init.DataSyncJumpWidth    = pcan->data_speed.sync_jump_width;
    hfdcan->Init.DataTimeSeg1         = pcan->data_speed.time_segment_1;
    hfdcan->Init.DataTimeSeg2         = pcan->data_speed.time_segment_2;
    hfdcan->Init.StdFiltersNbr        = 1;
    hfdcan->Init.ExtFiltersNbr        = 1;
    hfdcan->Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

#ifndef PCAN_USE_G4
    // The G4 does not support these parameters.
    hfdcan->Init.MessageRAMOffset    = calculate_message_ram_offset(hfdcan);
    hfdcan->Init.RxFifo0ElmtsNbr     = 8;
    hfdcan->Init.RxFifo0ElmtSize     = FDCAN_DATA_BYTES_64;
    hfdcan->Init.RxFifo1ElmtsNbr     = 0;
    hfdcan->Init.RxFifo1ElmtSize     = FDCAN_DATA_BYTES_8;
    hfdcan->Init.RxBuffersNbr        = 0;
    hfdcan->Init.RxBufferSize        = FDCAN_DATA_BYTES_8;
    hfdcan->Init.TxEventsNbr         = 0;
    hfdcan->Init.TxBuffersNbr        = 0;
    hfdcan->Init.TxFifoQueueElmtsNbr = 8;
    hfdcan->Init.TxElmtSize          = FDCAN_DATA_BYTES_64;
#endif

    // Initialize the peripheral with the above configuration.
    if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
        return -1;
    }

    // Reject messages which dont match any filters.
    if (HAL_FDCAN_ConfigGlobalFilter(pcan->hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) < 0) {
        return -1;
    }

    if (update_filters(pcan) < 0) {
        return -1;
    }

    // Set up interrupts.
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return -1;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK) {
        return -1;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        return -1;
    }

    // Initalize plugins.
    for (size_t i = 0; i < pcan->max_plugin_count; i++) {
        pcan_plugin_t *plugin = &pcan->plugins[i];

        if (plugin->free || !plugin->init) {
            continue;
        }

        if (plugin->init(pcan, plugin->plugin_data) < 0) {
            return -1;
        }
    }

    // Start the CAN peripheral.
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return -1;
    }

    return 0;
}

int pcan_set_mutex(pcan_instance_t *pcan, void (*lock)(void *), void (*unlock)(void *),
                   void *context) {
    // Lock and unlock need to be valid pointers.
    if ((lock == NULL) || (unlock == NULL)) {
        return -1;
    }

    // Mutex can only be set up once.
    if (pcan->mutex.lock || pcan->mutex.unlock || pcan->mutex.context) {
        return -1;
    }

    pcan->mutex.lock    = lock;
    pcan->mutex.unlock  = unlock;
    pcan->mutex.context = context;

    return 0;
}

int pcan_set_semaphore(pcan_instance_t *pcan, void (*get)(void *), void (*put)(void *),
                       void *context) {
    // get and put need to be valid pointers.
    if ((get == NULL) || (put == NULL)) {
        return -1;
    }

    // Mutex can only be set up once.
    if (pcan->semaphore.get || pcan->semaphore.put || pcan->semaphore.context) {
        return -1;
    }

    pcan->semaphore.get     = get;
    pcan->semaphore.put     = put;
    pcan->semaphore.context = context;

    return 0;
}

////////////////
// Publishing //
////////////////

int pcan_publish_raw(pcan_instance_t *pcan, uint32_t id, uint8_t *data, size_t size) {
    // Data size cannot be greater than 64 bytes for CAN FD.
    if (size > 64) {
        return -1;
    }

    // Data pointer must be provided if data expected.
    if (size > 0 && !data) {
        return -1;
    }

    pcan_message_t message = {
        .raw_id         = id,
        .data           = {0},
        .size           = size,
        .node_id        = extract_pcan_node_id(id),
        .channel_id     = extract_pcan_channel_id(id),
        .bitrate_switch = pcan->bit_rate_switching_on ? FDCAN_BRS_ON : FDCAN_BRS_OFF,
    };

    // No copy required if we have no data to send.
    if (size > 0) {
        memcpy(message.data, data, size);
    }

    lock_mutex(pcan);

    int result = ringbuffer_push(&pcan->tx_buffer, &message);

    // We need to disable the TX FIFO empty interrupt as it also calls `flush_tx_buffer`.
    __HAL_FDCAN_DISABLE_IT(pcan->hfdcan, FDCAN_IT_TX_FIFO_EMPTY);

    // Fill up the FDCAN TX FIFO with as many pending messages as possible.
    if (HAL_FDCAN_GetTxFifoFreeLevel(pcan->hfdcan) > 0) {
        flush_tx_ringbuffer(pcan, pcan->hfdcan);
    }

    __HAL_FDCAN_ENABLE_IT(pcan->hfdcan, FDCAN_IT_TX_FIFO_EMPTY);

    unlock_mutex(pcan);

    return result;
}

int pcan_publish_command(pcan_instance_t *pcan, pcan_priority_t priority,
                         pcan_endpoint_id_t endpoint_id, uint8_t *data, size_t size) {
    uint32_t id = make_pcan_id(priority, PCAN_TYPE_COMMAND, endpoint_id);
    return pcan_publish_raw(pcan, id, data, size);
}

int pcan_publish_telemetry(pcan_instance_t *pcan, pcan_priority_t priority, uint8_t channel_id,
                           uint8_t *data, size_t size) {
    pcan_endpoint_id_t endpoint_id = {
        .channel_id = channel_id,
        .node_id    = pcan->node_id,
    };
    uint32_t id = make_pcan_id(priority, PCAN_TYPE_TELEMETRY, endpoint_id);
    return pcan_publish_raw(pcan, id, data, size);
}

/////////////////
// Subscribing //
/////////////////

int pcan_subscribe_raw(pcan_instance_t *pcan, uint32_t id, uint32_t mask, pcan_callback_t callback,
                       void *user_data) {
    lock_mutex(pcan);

    // Find a free subscription slot.
    for (size_t i = 0; i < pcan->max_subscription_count; i++) {
        pcan_subscription_t *subscription = &pcan->subscriptions[i];

        // Skip in use subscription slots.
        if (!subscription->free)
            continue;

        subscription->id        = id;
        subscription->mask      = mask;
        subscription->callback  = callback;
        subscription->user_data = user_data;

        // Mark as in use.
        subscription->free = false;

        // Reflect changes in filters.
        update_filters(pcan);

        // Return index as subscription ID.
        unlock_mutex(pcan);
        return i;
    }

    // If we reach this point no free subscription slot was found. :(
    unlock_mutex(pcan);
    return -1;
}

int pcan_subscribe_command(pcan_instance_t *pcan, uint8_t channel_id, pcan_callback_t callback,
                           void *user_data) {
    pcan_endpoint_id_t endpoint_id = {
        .channel_id = channel_id,
        .node_id    = pcan->node_id,
    };
    uint32_t id   = make_pcan_id(0, PCAN_TYPE_COMMAND, endpoint_id);
    uint32_t mask = make_pcan_mask(endpoint_id);
    return pcan_subscribe_raw(pcan, id, mask, callback, user_data);
}

int pcan_subscribe_telemetry(pcan_instance_t *pcan, pcan_endpoint_id_t endpoint_id,
                             pcan_callback_t callback, void *user_data) {
    uint32_t id   = make_pcan_id(0, PCAN_TYPE_TELEMETRY, endpoint_id);
    uint32_t mask = make_pcan_mask(endpoint_id);
    return pcan_subscribe_raw(pcan, id, mask, callback, user_data);
}

///////////////////
// Unsubscribing //
///////////////////

int pcan_unsubscribe(pcan_instance_t *pcan, int subscription_id, void **out_user_data) {
    // Guard against bad input.
    if (subscription_id >= pcan->max_subscription_count || subscription_id < 0)
        return -1;

    lock_mutex(pcan);

    // Don't do anything if the subscription is not in use.
    if (pcan->subscriptions[subscription_id].free) {
        unlock_mutex(pcan);
        return -1;
    }

    pcan_subscription_t *subscription = &pcan->subscriptions[subscription_id];

    // Return user data if requested.
    if (out_user_data) {
        *out_user_data = subscription->user_data;
    }

    // Clear all subscription fields and mark as free.
    *subscription      = (pcan_subscription_t){0};
    subscription->free = true;

    // Reflect changes in filters.
    update_filters(pcan);

    unlock_mutex(pcan);
    return 0;
}

/////////////
// Polling //
/////////////

int pcan_poll(pcan_instance_t *pcan) {
    get_semaphore(pcan);
    lock_mutex(pcan);

    // Check if the peripheral is in bus-off.
    // Taken from:
    // https://community.st.com/t5/stm32-mcus/how-to-recover-from-bus-off-state-with-fdcan-on-stm32-mcus/ta-p/851596
    FDCAN_ProtocolStatusTypeDef protocol_status;
    HAL_FDCAN_GetProtocolStatus(pcan->hfdcan, &protocol_status);
    if (protocol_status.BusOff != 0) {
        // Clear INIT bit to recover from bus-off.
        CLEAR_BIT(pcan->hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }

    // Go through all received messages.
    pcan_message_t message;
    while (ringbuffer_pop(&pcan->rx_buffer, &message) >= 0) {
        // Find relevant subscriptions for this message.
        for (size_t i = 0; i < pcan->max_subscription_count; i++) {
            pcan_subscription_t subscription = pcan->subscriptions[i];

            // Skip subscription if its not used.
            if (subscription.free)
                continue;

            // Skip subscription if wrong id.
            if ((subscription.id & subscription.mask) != (message.raw_id & subscription.mask))
                continue;

            unlock_mutex(pcan);
            subscription.callback(&message, subscription.user_data);
            lock_mutex(pcan);
        }
    }

    // We need to disable the RX FIFO empty interrupt as it also calls `flush_tx_buffer`.
    __HAL_FDCAN_DISABLE_IT(pcan->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

    // Empty the RX buffer in case some messages were received while the ringbuffer was full.
    if (HAL_FDCAN_GetRxFifoFillLevel(pcan->hfdcan, FDCAN_RX_FIFO0) > 0) {
        flush_rx_fifo(pcan, pcan->hfdcan);
    }

    __HAL_FDCAN_ENABLE_IT(pcan->hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

    unlock_mutex(pcan);

    for (size_t i = 0; i < pcan->max_plugin_count; i++) {
        pcan_plugin_t *plugin = &pcan->plugins[i];

        if (plugin->free || plugin->poll == NULL)
            continue;

        plugin->poll(pcan, plugin->plugin_data);
    }

    return 0;
}

////////////////
// Interrupts //
////////////////

void pcan_on_rx_fifo0_receive(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan,
                              uint32_t rx_fifo0_its) {
    // If this P-CAN instance doesn't own the peripheral which triggered the interrupt, do nothing.
    if (pcan == NULL || pcan->hfdcan != hfdcan) {
        return;
    }

    flush_rx_fifo(pcan, hfdcan);
}

void pcan_on_tx_fifo_empty(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan) {
    // If this P-CAN instance doesn't own the peripheral which triggered the interrupt, do nothing.
    if (!pcan || pcan->hfdcan != hfdcan) {
        return;
    }

    flush_tx_ringbuffer(pcan, hfdcan);
}

void pcan_on_error_status(pcan_instance_t *pcan, FDCAN_HandleTypeDef *hfdcan,
                          uint32_t error_status_its) {
    // The FDCAN peripheral can enter a "bus-off" state if it encounters too many errors on the bus.
    // Recovery from this state is not automatic on all STM32 MCUs so we need to do it manually.
    // The recovery process is described in this guide:
    // https://community.st.com/t5/stm32-mcus/how-to-recover-from-bus-off-state-with-fdcan-on-stm32-mcus/ta-p/851596

    // If this P-CAN instance doesn't own the peripheral which triggered the interrupt, do nothing.
    if (!pcan || pcan->hfdcan != hfdcan) {
        return;
    }

    // Check if the peripheral is in bus-off.
    if ((error_status_its & FDCAN_IT_BUS_OFF) != 0) {
        // Re-enable the peripheral by clearing the INIT bit.
        // (Configuration is kept the same.)
        hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    }
}
