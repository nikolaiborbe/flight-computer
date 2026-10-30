#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"

#define NX_UDP_SEND_MAC_ADDRESS 0xffffffffffff
#define NX_UDP_SEND_IP_ADDRESS IP_ADDRESS(10, 19, 255, 255)
#define NX_UDP_SEND_PORT_TO 0x1F90
#define NX_UDP_SEND_PORT_FROM 0x88
#define NX_HARDCODE_MAC_ADDRESS 1

#define RTOS_MS_TO_TICKS(ms) ceil(((ms) / 1000.0f) * TX_TIMER_TICKS_PER_SECOND)
#define RTOS_TICKS_TO_MS(ticks) ceil(1000.0f * (ticks) / TX_TIMER_TICKS_PER_SECOND)


#define RTOS_HZ_TO_TICKS(hz) ((TX_TIMER_TICKS_PER_SECOND + (hz) - 1u) / (hz))

#define FSM_THREAD_HZ 200
#define NETWORK_THREAD_HZ 100
#define SENSOR_THREAD_HZ 200
#define KALMAN_THREAD_HZ 100
#define PCAN_THREAD_HZ 500

#define KALMAN_THREAD_SLEEP_TIMER RTOS_HZ_TO_TICKS(KALMAN_THREAD_HZ)
#define FSM_THREAD_SLEEP_TIMER RTOS_HZ_TO_TICKS(FSM_THREAD_HZ)
#define NETWORK_THREAD_SLEEP_TIMER RTOS_HZ_TO_TICKS(NETWORK_THREAD_HZ)
#define SENSOR_THREAD_SLEEP_TIMER RTOS_HZ_TO_TICKS(SENSOR_THREAD_HZ)
#define PCAN_THREAD_SLEEP_TIMER RTOS_HZ_TO_TICKS(PCAN_THREAD_HZ)

#ifdef __cplusplus
#include <tx_api.h>
#include <cstdint>

inline ULONG ms_to_ticks(uint32_t ms)
{
    return static_cast<ULONG>(
        (static_cast<uint64_t>(ms) * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u
    );
}

inline uint32_t ticks_to_ms(ULONG ticks)
{
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(ticks) * 1000u + TX_TIMER_TICKS_PER_SECOND - 1u) / TX_TIMER_TICKS_PER_SECOND
    );
}
#endif

#endif
