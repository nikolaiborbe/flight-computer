# API

This document goes through all the available functionality in P-CAN in detail.

## Set Up

Setting up P-CAN is straight forward.
Initialize the instance struct in C, or create the instance object in C++.
P-CAN should only be initialized once per physical CAN peripheral.
If multiple functions/threads/tasks/whatever need to use P-CAN they should share the same instance.
The built-in speed presets assume a 50 MHz FDCAN kernel clock.

C:
```c
#define PCAN_MAX_SUBSCRIPTION_COUNT (67)
#define PCAN_MAX_PLUGIN_COUNT (8)
#define PCAN_MAX_RX_MESSAGE_COUNT (64)
#define PCAN_MAX_TX_MESSAGE_COUNT (64)

pcan_instance_t pcan;
pcan_instance_config_t config = {
  .max_subscription_count = PCAN_MAX_SUBSCRIPTION_COUNT,
  .max_plugin_count = PCAN_MAX_PLUGIN_COUNT,
  .max_rx_message_count = PCAN_MAX_RX_MESSAGE_COUNT,
  .max_tx_message_count = PCAN_MAX_TX_MESSAGE_COUNT,
  .bit_rate_switching_on = true,
  .nominal_speed = PCAN_SPEED_CONFIG_1_MBIT,
  .data_speed = PCAN_SPEED_CONFIG_5_MBIT,
};

uint8_t pcan_storage[PCAN_STORAGE_SIZE(
    PCAN_MAX_SUBSCRIPTION_COUNT,
    PCAN_MAX_PLUGIN_COUNT,
    PCAN_MAX_RX_MESSAGE_COUNT,
    PCAN_MAX_TX_MESSAGE_COUNT
)];

int main() {
  pcan_node_id_t this_node = {CATEGORY, INSTANCE};

  pcan_create(&pcan, &hfdcan1, this_node, config, pcan_storage, sizeof(pcan_storage));
  pcan_init(&pcan);
}
```

C++:
```c++
int main() {
  // Use PCanBareMetal when running without a RTOS.
  // Use PCanThreadX when running in ThreadX.
  // Defaults: 1 Mbit nominal, 5 Mbit data, BRS disabled.
  PCanBareMetal<{ .max_subscription_count = 32 }> pcan(&hfdcan1, {CATEGORY, INSTANCE});
  pcan.init();
}
```

If you want to override the default speeds in C++, pass a custom `PCanInstanceConfig`:

```c++
PCanBareMetal<{
  .max_subscription_count = 32,
  .bit_rate_switching_on = true,
  .nominal_speed = PCAN_SPEED_CONFIG_2_MBIT,
  .data_speed = PCAN_SPEED_CONFIG_5_MBIT,
}> pcan(&hfdcan1, {CATEGORY, INSTANCE});
```

`PCan` is now an interface/base class. In most application modules you can use `PCan *` or `PCan &`
without exposing template arguments.

Then make sure to call `poll` to route incoming messages to the correct callbacks.

C:
```c
int main() {
  // ...

  while (1) {
    // ...

    pcan_poll(&pcan);
  }

  // ...
}
```

C++:
```c++
int main() {
  // ...

  while (1) {
    // ...

    pcan.poll();
  }

  // ...
}
```

### ThreadX

If you are using ThreadX you can use the included ThreadX helpers.

In C you can call `pcan_threadx_create_mutex(...)` and `pcan_threadx_create_semaphore(...)` to set up ThreadX synchronization:
```c
pcan_instance_t pcan;
TX_MUTEX pcan_tx_mutex;
TX_SEMAPHORE pcan_tx_semaphore;

#define PCAN_MAX_SUBSCRIPTION_COUNT (67)
#define PCAN_MAX_PLUGIN_COUNT (8)
#define PCAN_MAX_RX_MESSAGE_COUNT (64)
#define PCAN_MAX_TX_MESSAGE_COUNT (64)

pcan_instance_config_t config = {
  .max_subscription_count = PCAN_MAX_SUBSCRIPTION_COUNT,
  .max_plugin_count = PCAN_MAX_PLUGIN_COUNT,
  .max_rx_message_count = PCAN_MAX_RX_MESSAGE_COUNT,
  .max_tx_message_count = PCAN_MAX_TX_MESSAGE_COUNT,
  .bit_rate_switching_on = true,
  .nominal_speed = PCAN_SPEED_CONFIG_1_MBIT,
  .data_speed = PCAN_SPEED_CONFIG_5_MBIT,
};

uint8_t pcan_storage[PCAN_STORAGE_SIZE(
    PCAN_MAX_SUBSCRIPTION_COUNT,
    PCAN_MAX_PLUGIN_COUNT,
    PCAN_MAX_RX_MESSAGE_COUNT,
    PCAN_MAX_TX_MESSAGE_COUNT
)];

int main() {
  pcan_node_id_t this_node = {CATEGORY, INSTANCE};

  pcan_create(&pcan, &hfdcan1, this_node, config, pcan_storage, sizeof(pcan_storage));
  pcan_threadx_create_mutex(&pcan, &pcan_tx_mutex);
  pcan_threadx_create_semaphore(&pcan, &pcan_tx_semaphore);
  pcan_init(&pcan);
}
```

In C++ you can simply replace `PCanBareMetal` with `PCanThreadX`:
```c++
int main() {
  PCanThreadX<{ .max_subscription_count = 32 }> pcan(&hfdcan1, {CATEGORY, INSTANCE});
  pcan.init();
}
```

Everything else with P-CAN from here on out will work the same.

## Publishing

Publishing messages is just a function call.
Different functions are available for commands and telemetry.

### Commands

To publish a command you specify the target endpoint (`pcan_endpoint_id_t`).

C:
```c
void some_function() {
  pcan_endpoint_id_t target = {{CATEGORY, INSTANCE}, CHANNEL};

  if (pcan_publish_command(&pcan, PCAN_PRIORITY_HIGH, target, (uint8_t *)&data, size) < 0) {
    // Handle error
  }
}
```

C++:
```c++
void someFunction() {
  pcan_endpoint_id_t target = {{CATEGORY, INSTANCE}, CHANNEL};
  pcan.publish_command(PCAN_PRIORITY_HIGH, target, (uint8_t *)&data, size);
}
```

### Telemetry

To publish telemetry you only specify `channel_id`.
The node id is implicitly set to the sender node configured during setup.

C:
```c
void some_function() {
  some_data_t some_data = get_some_data();

  if (pcan_publish_telemetry(&pcan, PCAN_PRIORITY_LOW, CHANNEL, (uint8_t *)&some_data, sizeof(some_data)) < 0) {
    // Handle error
  }
}
```

C++:
```c++
void some_function() {
  some_data_t some_data = get_some_data();

  pcan.publish_telemetry(PCAN_PRIORITY_LOW, CHANNEL, (uint8_t *)&some_data, sizeof(some_data));
}
```

## Subscribing

Subscriptions are callback based.
To subscribe to a message you provide a callback function.
These callbacks should be treated as interrupts (although they technically are not) and should be kept short.

A callback accepts a `pcan_message_t *` argument.
In C there is also a second `void *user_data` argument.
This pointer is user defined and passed to each callback call.

Example callbacks:

C:
```c
void callback(pcan_message_t *message, void *user_data) {
  (void)user_data; // In most cases you will simply ignore this.

  // do stuff with message...
}
```

C++:
```c++
void callback(pcan_message_t *message) {
  // do stuff with message...
}
```

### Commands

Subscribing to commands only requires `channel_id`.
The node id is implicitly the local node.

C:
```c
int main() {
  if (pcan_subscribe_command(&pcan, CHANNEL, callback, NULL) < 0) {
    // Handle error
  }
}
```

C++:
```c++
int main() {
  // ...

  pcan.subscribe_command(CHANNEL, callback);

  // ...
}
```

### Telemetry

Subscribing to telemetry requires specifying endpoint id (`pcan_endpoint_id_t`).

C:
```c
int main() {
  // ...

  pcan_endpoint_id_t source = {{CATEGORY, INSTANCE}, CHANNEL};
  if (pcan_subscribe_telemetry(&pcan, source, callback, NULL) < 0) {
    // Handle error
  }

  // ...
}
```

C++:
```c++
int main() {
  // ...

  pcan_endpoint_id_t source = {{CATEGORY, INSTANCE}, CHANNEL};
  pcan.subscribe_telemetry(source, callback);

  // ...
}
```

## Time Synchronization

Time synchronization is implemented and available in both C and C++.

C:
```c
pcan_timesync_instance_t timesync;

void some_init(void) {
  pcan_timesync_create(&timesync, &htim1, PCAN_TIMESYNC_ROLE_SUB);
  pcan_timesync_register_plugin(&pcan, &timesync);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // ...
  pcan_timesync_on_period_elapsed(&timesync, htim);
  // ...
}

void some_loop(void) {
  // ...
  uint64_t now_us = pcan_timesync_get_time_us(&timesync);
  // ...
}
```

C++:
```c++
PCanThreadX<{ .max_subscription_count = 32 }> pcan(&hfdcan1, {CATEGORY, INSTANCE});
PCanTimesync timesync(&htim1, PCAN_TIMESYNC_ROLE_SUB);

void some_init() {
  pcan.register_plugin(timesync);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  timesync.on_period_elapsed(htim);
}

void some_loop() {
  // ...
  uint64_t now_us = timesync.get_time_us();
  // ...
}
```

## Status LEDs (Blinky)

The blinky plugin can be used to visualize RX/TX activity on a GPIO LED.

C:
```c
pcan_blinky_instance_t blinky;

void some_init(void) {
  pcan_blinky_led_t led = {
      .port = GPIOB,
      .pin = GPIO_PIN_0,
      .behavior = PCAN_BLINKY_LED_ACTIVE_HIGH,
  };

  pcan_blinky_create(&blinky, led);
  pcan_blinky_register_plugin(&pcan, &blinky);
}
```

C++:
```c++
PCanThreadX<{ .max_subscription_count = 32 }> pcan(&hfdcan1, {CATEGORY, INSTANCE});

void some_init() {
  pcan_blinky_led_t led = {
      .port = GPIOB,
      .pin = GPIO_PIN_0,
      .behavior = PCAN_BLINKY_LED_ACTIVE_HIGH,
  };

  static PCanBlinky blinky(led);
  pcan.register_plugin(blinky);
}
```
