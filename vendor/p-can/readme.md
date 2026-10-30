# Propulse-CAN
Propulse-CAN (or just P-CAN) is a simple CAN driver and protocol for STM32.
It uses a simple publish-subscribe architecture stripped down to the bare minimum
This library can be used on both bare-metal and RTOS systems.
It currently ships with built in support for ThreadX.

If you are just interested in how to get started just skip down to the *Examples* section.

> [!IMPORTANT]
> A clock frequency of 50MHz must be provided to the CAN FD peripheral P-CAN is going to use.
> The bundled speed presets assume a 50MHz FDCAN kernel clock.

> [!IMPORTANT]
> Aways remember to call `.init()` and `.poll()`!

### Speed presets

P-CAN ships with a few convenience presets for the nominal and data phase timing:

- `PCAN_SPEED_CONFIG_1_MBIT`
- `PCAN_SPEED_CONFIG_2_MBIT`
- `PCAN_SPEED_CONFIG_5_MBIT`

The data-speed preset is only used when bit rate switching is enabled.

In C++, `PCanBareMetal` and `PCanThreadX` default to 1 Mbit nominal speed, 5 Mbit data speed,
and bit rate switching disabled.
If you need a different configuration, pass a custom `PCanInstanceConfig`.

## Concepts

P-CAN adds a few concepts to CAN to organize how messages are sent. 

### Message IDs

Each message in CAN has an ID. 
This ID is used both for identifying the messages contents and recipient, 
but also as a priority in case two messages collide.
The lower the ID the higher the message's priority.
To keep things organized the standard 11 bit CAN ID has been divided into the following sections:

| Section  | Length |  Description |
| ---------|--------|-------------|
| Priority | 1 bit  | Allows setting priority independently of the other sections. |
| Type   | 1 bit  | Indicates whether the message is a command (0) or telemetry (1). |
| Category | 3 bits | Part of a physical node's ID. Indicates which category it is. E.g. thermocouple, servos, e-reg, etc... |
| Instance | 3 bits | Specifies which physical instance whithin the category the message is to/from.
| Channel  | 3 bits | Allows for different endpoints within a single physical node. It may for exampl be different thermo couples or servos. |

Together, the category and instance sections make up a _node id_. 
Each physical board connected to the CAN bus should have a unique node id.

### Commands & Telemetry 

Messages in P-CAN are divided into two types: commands and telemetry.

Telemetry messages should contain data from sensors, status of the node, time synchronization or other data which needs to be communicated.
The ID of a telemetry message is the sender and they are broadcast to all interessted nodes.

Command messages contain configuraiton, set points or similar.
The ID of a command message is the target node and they are unicast to specific nodes.

## Recomendations

While P-CAN is neutral in how it is used, there are some best practices to create robust real time commmunication.

### Idempotency

Commands should be created in such a way that sending multiple instances of the same commands gives the same result.
E.g. in stead of the command "next state" the command "go to state x" should be used.

### Command Confirmation

P-CAN provides no message type for acknowledgements.
This is by design as all command results can be inferred from telemetry.
Therefore, a third type would only contribute to complexity of the system.
In stead, the success of commands should be confirmed by verifying with telemetry.

## Implementation

P-CAN supports both C and C++. Its core is written in C, while a lightweight C++ wrapper provides a convenient driver object.

## Examples

Detailed API usage, function-by-function guidance, and examples are documented in [api.md](api.md).
If you want fully working examples head on over to [examples.md](examples.md).

## Additional Features 

In addition to sending and receiving messages, P-CAN has some utilities for time synchronization, bus monitoring, and status LEDs.

### Time Synchronization

A time synchronization module is available. It allows for synchronizing time between all nodes on a bus.

> [!IMPORTANT]
> `on_period_elapsed` must be called from your timer period elapsed callback.
> In C this means calling `pcan_timesync_on_period_elapsed(...)`.
> In C++ this means calling `PCanTimesync::on_period_elapsed(...)`.

See [api.md](api.md) for setup details and examples.

### Bus Monitoring

This is WIP. When finished this will include functonality for setting up heartbeats from each node and keeping track of how many are alive.

### Status LEDs

A blinky module is available for showing RX/TX activity on a status LED.

- In C, use `pcan_blinky_create(...)` and `pcan_blinky_register_plugin(...)`.
- In C++, use `PCanBlinky` and register it with `pcan.register_plugin(...)`.

See [api.md](api.md) for setup details and examples.
