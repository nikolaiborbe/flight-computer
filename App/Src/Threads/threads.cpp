#include "Threads/threads.hpp"
#include "Data/Writers.hpp"
#include "Entry/Kalman.hpp"
#include "Entry/Logging.hpp"
#include "Threads/Entry/PCAN.hpp"
#include "Threads/Entry/SdCard.hpp"
#include "tx_api.h"

namespace Thread
{
   // Each producer thread gets the unique write handles for its telemetry
   // cells through thread_input. Under FC_PYTHON_TESTING the sensor/pcan
   // thread bodies are compiled out and their writers are routed to the
   // test feed instead, so those inputs stay 0.
   namespace
   {
#ifndef FC_PYTHON_TESTING
      const ULONG pcan_input   = reinterpret_cast<ULONG>(&Telemetry::Writers::pcan);
      const ULONG sensor_input = reinterpret_cast<ULONG>(&Telemetry::Writers::sensor);
#else
      const ULONG pcan_input   = 0;
      const ULONG sensor_input = 0;
#endif
      const ULONG kalman_input = reinterpret_cast<ULONG>(&Telemetry::Writers::kalman);
      const ULONG fsm_input    = reinterpret_cast<ULONG>(&Telemetry::Writers::fsm);
      const ULONG network_input = reinterpret_cast<ULONG>(&Telemetry::Writers::networking);
   }

   namespace Config
   {
      namespace Tx_thread
      {
         TX_THREAD pcan;
         TX_THREAD sensor;
         TX_THREAD kalman;
         TX_THREAD network;
         TX_THREAD fsm;
         TX_THREAD logging;
         TX_THREAD storage;
      }
      namespace Stack
      {
         alignas(8) uint8_t pcan[4 * 1024];
         alignas(8) uint8_t network[4 * 1024];
         alignas(8) uint8_t fsm[4 * 1024];
         alignas(8) uint8_t sensor[4 * 1024];
         alignas(8) uint8_t kalman[4 * 1024];
         alignas(8) uint8_t logging[4 * 1024];
         alignas(8) uint8_t storage[4 * 1024];
      }
   }

   const Type Threads[7] = {
      { 
         "PCAN",
         &Config::Tx_thread::pcan, Entry::Pcan,
         pcan_input, Config::Stack::pcan, sizeof(Config::Stack::pcan),
         Config::Priority::PCAN, Config::Priority::PCAN,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "Network",
         &Config::Tx_thread::network, Entry::Network,
         network_input, Config::Stack::network, sizeof(Config::Stack::network),
         Config::Priority::NETWORK, Config::Priority::NETWORK,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "Sensor",
         &Config::Tx_thread::sensor, Entry::Sensor,
         sensor_input, Config::Stack::sensor, sizeof(Config::Stack::sensor),
         Config::Priority::SENSOR, Config::Priority::SENSOR,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "FSM",
         &Config::Tx_thread::fsm, Entry::FSM,
         fsm_input, Config::Stack::fsm, sizeof(Config::Stack::fsm),
         Config::Priority::FSM, Config::Priority::FSM,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "Kalman",
         &Config::Tx_thread::kalman, Entry::Kalman,
         kalman_input, Config::Stack::kalman, sizeof(Config::Stack::kalman),
         Config::Priority::KALMAN, Config::Priority::KALMAN,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "SD Card",
         &Config::Tx_thread::storage, Entry::SdCard,
         0, Config::Stack::storage, sizeof(Config::Stack::storage),
         Config::Priority::STORAGE, Config::Priority::STORAGE,
         TX_NO_TIME_SLICE, TX_AUTO_START
      },
      {
         "Logging",
         &Config::Tx_thread::logging, Entry::Logging,
         0, Config::Stack::logging, sizeof(Config::Stack::logging),
         Config::Priority::LOGGING, Config::Priority::LOGGING,
         TX_NO_TIME_SLICE, TX_AUTO_START
      }
   };
}
