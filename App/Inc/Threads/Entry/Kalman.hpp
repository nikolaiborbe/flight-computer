#ifndef ENTRY_KALMAN_HPP
#define ENTRY_KALMAN_HPP

// HAL
#include "tx_api.h"

// App
#include "Drivers/Kalmanfilter/kalmanfilter.h"
#include "Drivers/Kalmanfilter/kalmanfilter_terminate.h"
#include "Utilities/config.h"

namespace Thread::Entry {
void Kalman(unsigned long thread_input);
}

#endif