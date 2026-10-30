# Fully Working Examples
Bellow are two fully working examples for how P-CAN is envisioned to be used.
Since we are using C++ for P26 it is recommended to stick to the C++ example.
However, some parts may still be C (like P-CAN itself) so the bellow example might be usefull too.

## C++
```cpp
#include "fdcan.h" // Required to access "fdcan" handle
#include "pcan.hpp" // Only this header is needed for P-CAN in C++

// This may be a sensor, fsm, or whatever your heart desires.
class ExampleClass {
  private:
    PCan& pcan;

  public:
    // The constructor can take in a pcan instance and store it for later use.
    ExampleClass(PCan& pcan) : pcan(pcan) {}

    // Class should subscribe to desiered messages uring its initialization.
    int init() {
        constexpr uint8_t commandChannel = 0;
        if (this->pcan.subscribe_command(commandChannel, this, &ExampleClass::callback) < 0)
            return -1;

        return 0;
    }

    // The callback must be a void function which accepts a pointer to a pcan message struct.
    void callback(pcan_message_t *message) {
        // Do stuff with received message here.
    }

    // Using the stored P-CAN instance the class may send whatever data whenever it desires.
    void poll() {
        // Gather data or something from you sensor
        typedef struct { 
            int16_t temperature; 
        } sensor_data_t;
        sensor_data_t sensorData = { 1234 };
    
        // Publish said data
        constexpr uint8_t sensorChannel = 0;
        pcan.publish_telemetry(
            PCAN_PRIORITY_LOW, 
            sensorChannel, 
            reinterpret_cast<uint8_t *>(&sensorData), 
            sizeof(sensorData)
        );
    }
};


extern "C" void pcan_example_entry() {
    // Set up P-CAN driver
    constexpr pcan_node_id_t thisNodesId = { 1, 2 };
    // The template argument can be omitted to use default values.
    PCanThreadX<{ .max_subscription_count = 16 }> pcan(&hfdcan1, thisNodesId);
    if (pcan.init() < 0) {
        return;
    }
    
    // Set up your own stuff.
    // This would be ideally done in another thread.
    ExampleClass myClass(pcan);
    if (myClass.init() < 0) {
        return;
    }

    // Main loop
    while (true) {
        pcan.poll();
        myClass.poll(); // Ideally this class would be polled from another thread.
    }
}
```

## C
```c
#include "fdcan.h"
#include "pcan.h"

#define THIS_NODES_CATEGORY (0)
#define THIS_NODES_INSTANCE (1)

#define MAX_SUBSCRIPTION_COUNT (16)
#define MAX_PLUGIN_COUNT       (8)
#define MAX_RX_MESSAGE_COUNT   (64)
#define MAX_TX_MESSAGE_COUNT   (64)

pcan_instance_t pcan;
pcan_instance_config_t config = {
    MAX_SUBSCRIPTION_COUNT,
    MAX_PLUGIN_COUNT,
    MAX_RX_MESSAGE_COUNT,
    MAX_TX_MESSAGE_COUNT,
};

uint8_t pcan_storage[PCAN_STORAGE_SIZE(
    MAX_SUBSCRIPTION_COUNT,
    MAX_PLUGIN_COUNT,
    MAX_RX_MESSAGE_COUNT,
    MAX_TX_MESSAGE_COUNT
)];

void message_callback(pcan_message_t *message, void *user_data) {
    // The argument "user_data" will be the pointer we provided when we crated the subscription.
    // This is (as the name implies) user defined and is usefull for resuing the same function
    // in different contexts. For example for trampolines. In most cases however one can simply ignore it:
    (void) user_data;

    // Do stuff with the message here.
}

void example_pcan_entry() {
    // Initalizing P-CAN
    pcan_create(
        &pcan, // The P-CAN instance struct to initalize.
        &hfdcan1, // Which FD CAN peripheral to use.
        (pcan_node_id_t){THIS_NODES_CATEGORY, THIS_NODES_INSTANCE}, // The id of this node.
        config,
        pcan_storage,
        sizeof(pcan_storage)
    );
    pcan_init(&pcan);

    pcan_node_id_t remote_node_id = { 0, 1 }; // Remote node's id: instance 1 of category 0.
    uint8_t remote_channel_id = 2; // Channel to listen to from remote node.

    // Creating subscriptions
    pcan_subscribe_telemetry(
        &pcan, // Handle to P-CAN instance.
        (pcan_endpoint_id_t){remote_node_id, remote_channel_id}, // Which node and channel to listen to.
        message_callback, // Function to call with received messages.
        NULL // User defined data which will be passed to the function along with the message.
    );
    
    // Super loop
    while (1) {
        pcan_poll(&pcan);

        // Read sensors or something here
        typedef struct { uint16_t value; } sensor_data_t;
        sensor_data_t sensor_data = { .value = 123 };

        // Publishing data
        pcan_publish_telemetry(
            &pcan, // Handle to P-CAN.
            PCAN_PRIORITY_LOW, // Either high or low.
            0, // Which channel to publish data on. Between 0 and 7 inclusive.
            (uint8_t *)&sensor_data, // Pointer to the data.
            sizeof(sensor_data_t) // Size of the data. Must not be greater than 64 bytes!
        );
    }
}
```
