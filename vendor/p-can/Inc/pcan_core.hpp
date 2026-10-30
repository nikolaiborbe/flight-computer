/**
 * @file pcan.hpp
 * @author Paulius Juzenas (paulius.juzenas@propulsentnu.no)
 * @brief A simple C++ wrapper for P-CAN.
 * @version 0.1
 * @date 2025-12-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "pcan_core.h"
#include <array>
#include <span>

using PCanCallbackFunction                     = void (*)(pcan_message_t *message);
template <typename T> using PCanCallbackMethod = void (T::*)(pcan_message_t *message);

using CallbackStorage        = std::aligned_storage_t<32, alignof(std::max_align_t)>;
using PCanCallbackTrampoline = void (*)(void *, const CallbackStorage &, pcan_message_t *);

struct PCanCallbackEntry {
    bool free                               = true;
    void *object                            = nullptr;
    CallbackStorage user_callback           = {};
    PCanCallbackTrampoline callback_wrapper = nullptr;
};

class PCan;

class PCanPlugin {
  public:
    virtual int register_plugin(PCan &pcan) = 0;
};

class PCan {
  protected:
    pcan_instance_t pcan;

  private:
    std::span<PCanCallbackEntry> callbacks;

    /**
     * @brief Trampoline function for handling incoming CAN messages.
     */
    static void callback_trampoline(pcan_message_t *message, void *user_data) {
        // The user data is only ever set by us, so it must be a pointer to a PCanCallbackEntry.
        PCanCallbackEntry *entry = static_cast<PCanCallbackEntry *>(user_data);

        // The user data and callback entry should never be null, but we check just in case.
        if (entry && entry->callback_wrapper) {
            entry->callback_wrapper(entry->object, entry->user_callback, message);
        }
    }

    /**
     * @brief Stores a `std::function` callback in an internal array to be passed to the C
     * implementation of P-Can.
     */
    PCanCallbackEntry *add_callback(PCanCallbackFunction function) {
        // Go through all callback entries and find a free slot.
        for (PCanCallbackEntry &entry : this->callbacks) {
            if (!entry.free)
                continue;

            entry.free   = false;
            entry.object = nullptr;

            static_assert(sizeof(function) <= sizeof(entry.user_callback));
            memcpy(&entry.user_callback, &function, sizeof(function));

            entry.callback_wrapper = [](void *obj, const CallbackStorage &user_callback,
                                        pcan_message_t *msg) {
                PCanCallbackFunction function;
                memcpy(&function, &user_callback, sizeof(function));
                function(msg);
            };

            return &entry;
        }

        // No free slot found. :(
        return nullptr;
    }

    template <typename T>
    PCanCallbackEntry *add_callback(T *object, void (T::*method)(pcan_message_t *)) {
        for (PCanCallbackEntry &entry : callbacks) {
            if (!entry.free)
                continue;

            entry.free   = false;
            entry.object = object;

            static_assert(sizeof(method) <= sizeof(entry.user_callback));
            memcpy(&entry.user_callback, &method, sizeof(method));

            entry.callback_wrapper = [](void *object, const CallbackStorage &user_callback,
                                        pcan_message_t *msg) {
                PCanCallbackMethod<T> method;
                memcpy(&method, &user_callback, sizeof(method));
                (static_cast<T *>(object)->*method)(msg);
            };

            return &entry;
        }
        return nullptr;
    }

    /**
     * @brief Removes a `std::function` callback entry from the internal callback array.
     */
    void remove_callback(PCanCallbackEntry *entry) {
        *entry = {
            .free             = true,
            .object           = nullptr,
            .user_callback    = {},
            .callback_wrapper = nullptr,
        };
    }

  public:
    template <size_t N, size_t K>
    PCan(FDCAN_HandleTypeDef *hfdcan, pcan_node_id_t node_id, pcan_instance_config_t config,
         uint8_t (&storage)[N], std::array<PCanCallbackEntry, K> &callbacks)
        : callbacks(callbacks) {
        pcan_create(&this->pcan, hfdcan, node_id, config, storage, N);
    }

    /**
     * @brief Initializes the P-CAN instance. Must be called before any other P-CAN functions.
     */
    int init() { return pcan_init(&this->pcan); }

    /**
     * @brief Publishes a raw CAN message. Should only be used when absolutely necessary!
     */
    int publish_raw(uint32_t id, uint8_t *data, size_t size) {
        return pcan_publish_raw(&this->pcan, id, data, size);
    }

    /**
     * @brief Publish a command message to the specified node on the specified channel.
     */
    int publish_command(pcan_priority_t priority, pcan_endpoint_id_t endpoint_id, uint8_t *data,
                        size_t size) {
        return pcan_publish_command(&this->pcan, priority, endpoint_id, data, size);
    }

    /**
     * @brief Publish a telemetry message from this node on the specified channel.
     */
    int publish_telemetry(pcan_priority_t priority, uint8_t channel_id, uint8_t *data,
                          size_t size) {
        return pcan_publish_telemetry(&this->pcan, priority, channel_id, data, size);
    }

    /**
     * @brief Subscribes to a raw CAN message. Should only be used when absolutely necessary!
     */
    int subscribe_raw(uint32_t id, uint32_t mask, PCanCallbackFunction callback) {
        PCanCallbackEntry *entry = this->add_callback(callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_raw(&this->pcan, id, mask, this->callback_trampoline, entry);
    }

    /**
     * @brief Subscribes to a raw CAN message. Should only be used when absolutely necessary!
     */
    template <typename T>
    int subscribe_raw(uint32_t id, uint32_t mask, T *object, PCanCallbackMethod<T> callback) {
        PCanCallbackEntry *entry = this->add_callback(object, callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_raw(&this->pcan, id, mask, this->callback_trampoline, entry);
    }

    /**
     * @brief Subscribe to incomming commands to this node on the specified channel.
     */
    int subscribe_command(uint8_t channel_id, PCanCallbackFunction callback) {
        PCanCallbackEntry *entry = this->add_callback(callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_command(&this->pcan, channel_id, this->callback_trampoline, entry);
    }

    /**
     * @brief Subscribe to incomming commands to this node on the specified channel.
     */
    template <typename T>
    int subscribe_command(uint8_t channel_id, T *object, PCanCallbackMethod<T> callback) {
        PCanCallbackEntry *entry = this->add_callback(object, callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_command(&this->pcan, channel_id, this->callback_trampoline, entry);
    }

    /**
     * @brief Subscribe to telemetry messages from specified node and channel.
     */
    int subscribe_telemetry(pcan_endpoint_id_t endpoint_id, PCanCallbackFunction callback) {
        PCanCallbackEntry *entry = this->add_callback(callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_telemetry(&this->pcan, endpoint_id, this->callback_trampoline, entry);
    }

    /**
     * @brief Subscribe to telemetry messages from specified node and channel.
     */
    template <typename T>
    int subscribe_telemetry(pcan_endpoint_id_t endpoint_id, T *object,
                            PCanCallbackMethod<T> callback) {
        PCanCallbackEntry *entry = this->add_callback(object, callback);
        if (entry == nullptr)
            return -1;
        return pcan_subscribe_telemetry(&this->pcan, endpoint_id, this->callback_trampoline, entry);
    }

    /**
     * @brief Unsubscribe subscription with given id.
     */
    int unsubscribe(int subscription_id) {
        // Unsubscribe given subscription and retrieve user data pointer.
        void *out_user_data;
        int result = pcan_unsubscribe(&this->pcan, subscription_id, &out_user_data);

        // If the subscription exited, mark the callback entry as free.
        if (result >= 0) {
            PCanCallbackEntry *callback_entry = static_cast<PCanCallbackEntry *>(out_user_data);
            callback_entry->callback_wrapper  = nullptr;
            callback_entry->user_callback     = {};
            callback_entry->object            = nullptr;
            callback_entry->free              = true;
        }

        return result;
    }

    /**
     * @brief Goes through the P-CAN message queue and call relevant subscriptions.
     */
    int poll() { return pcan_poll(&this->pcan); }

    /**
     * @brief Returns a pointer to the underlying P-CAN instance.
     *
     * @warning This should only be used when absolutely necessary.
     */
    pcan_instance_t *get_instance() { return &this->pcan; }

    /**
     * @brief Registers a plugin with this P-CAN instance.
     */
    int register_plugin(PCanPlugin &plugin) { return plugin.register_plugin(*this); }
};

struct PCanInstanceConfig {
    size_t max_subscription_count = 32;
    size_t max_plugin_count       = 8;
    size_t max_rx_message_count   = 64;
    size_t max_tx_message_count   = 64;
    pcan_speed_config_t nominal_speed = PCAN_SPEED_CONFIG_1_MBIT;
    pcan_speed_config_t data_speed    = PCAN_SPEED_CONFIG_5_MBIT;
    bool bit_rate_switching_on        = false;

    operator pcan_instance_config_t() const {
        return {
            .max_subscription_count = this->max_subscription_count,
            .max_plugin_count       = this->max_plugin_count,
            .max_rx_message_count   = this->max_rx_message_count,
            .max_tx_message_count   = this->max_tx_message_count,
            .nominal_speed          = this->nominal_speed,
            .data_speed             = this->data_speed,
            .bit_rate_switching_on  = this->bit_rate_switching_on,
        };
    }
};

template <PCanInstanceConfig C = {}> class PCanBareMetal : public PCan {
  private:
    uint8_t storage[PCAN_STORAGE_SIZE(C.max_subscription_count, C.max_plugin_count,
                                      C.max_rx_message_count, C.max_tx_message_count)];
    std::array<PCanCallbackEntry, C.max_subscription_count> callbacks;

  public:
    PCanBareMetal(FDCAN_HandleTypeDef *hfdcan, pcan_node_id_t node_id)
        : PCan(hfdcan, node_id, C, this->storage, this->callbacks) {};
};

#define PCAN_DEFINE_CALLBACKS_CPP(...)                                                             \
    auto &__get_pcan_instances() {                                                                 \
        static PCan *const __pcan_instances[] = {__VA_ARGS__};                                     \
        return __pcan_instances;                                                                   \
    }                                                                                              \
                                                                                                   \
    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its) {           \
        for (PCan * pcan : __get_pcan_instances()) {                                               \
            pcan_instance_t *instance = nullptr;                                                   \
            if (pcan != nullptr)                                                                   \
                instance = pcan->get_instance();                                                   \
            pcan_on_rx_fifo0_receive(instance, hfdcan, rx_fifo0_its);                              \
        }                                                                                          \
    }                                                                                              \
                                                                                                   \
    void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {                              \
        for (PCan * pcan : __get_pcan_instances()) {                                               \
            pcan_instance_t *instance = nullptr;                                                   \
            if (pcan != nullptr)                                                                   \
                instance = pcan->get_instance();                                                   \
            pcan_on_tx_fifo_empty(instance, hfdcan);                                               \
        }                                                                                          \
    }                                                                                              \
                                                                                                   \
    void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t error_status_its) {   \
        for (PCan * pcan : __get_pcan_instances()) {                                               \
            pcan_instance_t *instance = nullptr;                                                   \
            if (pcan != nullptr)                                                                   \
                instance = pcan->get_instance();                                                   \
            pcan_on_error_status(instance, hfdcan, error_status_its);                              \
        }                                                                                          \
    }
