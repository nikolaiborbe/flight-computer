#ifndef __PCAN_THREADX_H__
#define __PCAN_THREADX_H__

#ifdef __cplusplus
extern "C" {
#endif

// This messy piece of preprocessor directives simply
// checks if the header "tx_api.h" is available.
#if defined(__has_include)
#if __has_include("tx_api.h")
#include "tx_api.h"
#define PCAN_THREADX_AVAILABLE
#endif // defined(__has_include)
#endif // __has_include("tx_api.h")

#ifdef PCAN_THREADX_AVAILABLE

#include "pcan_core.h"
#include "tx_api.h"

int pcan_threadx_create_mutex(pcan_instance_t *pcan, TX_MUTEX *tx_mutex);

int pcan_threadx_create_semaphore(pcan_instance_t *pcan, TX_SEMAPHORE *tx_semaphore);

#endif // PCAN_THREADX_AVAILABLE

#ifdef __cplusplus
}
#endif

#endif // __PCAN_THREADX_H__
