#ifndef RECOVERY_TESTING
#pragma once

// stm
#include "tx_api.h"

// App
#include "Threads/Entry/FSM.hpp"
#include "Threads/Entry/Kalman.hpp"
#include "Threads/Entry/Network.hpp"
#include "Threads/Entry/Sensor.hpp"

// std
#include <cstdint>

namespace Thread {
struct Type {
    const char* name;
    TX_THREAD* thread;
    void (*entry)(ULONG);
    ULONG input;

    void* stack_ptr;
    ULONG stack_size;

    UINT priority;
    UINT preempt_threshold;
    ULONG time_slice;
    UINT auto_start;
};

extern const Type Threads[7];

namespace Config {
namespace Tx_thread {
extern TX_THREAD pcan;
extern TX_THREAD sensor;
extern TX_THREAD logging;
extern TX_THREAD storage;
extern TX_THREAD network;
extern TX_THREAD fsm;
extern TX_THREAD kalman;
} // namespace Tx_thread

namespace Priority {
constexpr UINT PCAN    = 4;
constexpr UINT NETWORK = 15;
constexpr UINT FSM     = 4;
constexpr UINT SENSOR  = 4;
constexpr UINT KALMAN  = 4;
constexpr UINT LOGGING = 20;
constexpr UINT STORAGE = 20;
} // namespace Priority

namespace Stack {
extern uint8_t pcan[4 * 1024];
extern uint8_t network[4 * 1024];
extern uint8_t fsm[4 * 1024];
extern uint8_t sensor[4 * 1024];
extern uint8_t kalman[4 * 1024];
extern uint8_t logging[4 * 1024];
extern uint8_t storage[4 * 1024];
} // namespace Stack
} // namespace Config
} // namespace Thread

#endif
