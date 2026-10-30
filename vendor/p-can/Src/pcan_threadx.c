#include "pcan_threadx.h"

#ifdef PCAN_THREADX_AVAILABLE

void lock_tx_mutex(void *context) {
    TX_MUTEX *tx_mutex = context;
    tx_mutex_get(tx_mutex, TX_WAIT_FOREVER);
}

void unlock_tx_mutex(void *context) {
    TX_MUTEX *tx_mutex = context;
    tx_mutex_put(tx_mutex);
}

int pcan_threadx_create_mutex(pcan_instance_t *pcan, TX_MUTEX *tx_mutex) {
    UINT result = tx_mutex_create(tx_mutex, "P-CAN Mutex", TX_INHERIT);

    if (result != TX_SUCCESS)
        return -1;

    if (pcan_set_mutex(pcan, lock_tx_mutex, unlock_tx_mutex, tx_mutex) < 0)
        return -1;

    return 0;
}

void get_tx_semaphore(void *context) {
    TX_SEMAPHORE *tx_semaphore = context;
    ULONG wait                 = (1 * TX_TIMER_TICKS_PER_SECOND) / 1000;
    tx_semaphore_get(tx_semaphore, wait > 0 ? wait : 1);
}

void put_tx_semaphore(void *context) {
    TX_SEMAPHORE *tx_semaphore = context;
    tx_semaphore_ceiling_put(tx_semaphore, 1);
}

int pcan_threadx_create_semaphore(pcan_instance_t *pcan, TX_SEMAPHORE *tx_semaphore) {
    UINT result = tx_semaphore_create(tx_semaphore, "P-CAN Semaphore", 0);

    if (result != TX_SUCCESS)
        return -1;

    if (pcan_set_semaphore(pcan, get_tx_semaphore, put_tx_semaphore, tx_semaphore) < 0)
        return -1;

    return 0;
}

#endif // PCAN_THREADX_AVAILABLE
