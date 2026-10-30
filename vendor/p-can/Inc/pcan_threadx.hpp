#pragma once

#include "pcan_core.hpp"
#include "pcan_threadx.h"

#ifdef PCAN_THREADX_AVAILABLE

template <PCanInstanceConfig C = {}> class PCanThreadX : public PCanBareMetal<C> {
  private:
    TX_MUTEX tx_mutex;
    TX_SEMAPHORE tx_semaphore;

  public:
    PCanThreadX(FDCAN_HandleTypeDef *hfdcan, pcan_node_id_t node_id)
        : PCanBareMetal<C>(hfdcan, node_id) {
        pcan_threadx_create_mutex(&this->pcan, &this->tx_mutex);
        pcan_threadx_create_semaphore(&this->pcan, &this->tx_semaphore);
    }
};

#endif // PCAN_THREADX_AVAILABLE
