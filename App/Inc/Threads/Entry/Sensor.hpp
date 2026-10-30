#ifndef ENTRY_SENSOR_HPP
#define ENTRY_SENSOR_HPP

// Hal
#include "main.h"
#include "spi.h"

// App
#include "Context.hpp"
#include "Datastructures/Seqlock.hpp"
#include "IIM42652.h"
#include "Utilities/config.h"

namespace Thread::Entry {
void Sensor(unsigned long thread_input);
}

#endif
