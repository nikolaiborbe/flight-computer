# Servo Driver Reference

This driver provides a transport-agnostic servo API used across our CAN nodes. The driver is built with the following in mind:

- The **ECU** should command servos without caring whether they are local or remote.
- The **actuator node** should safely drive the physical servo (PWM + power enable + optional feedback).
- **CAN-FD** provides the transport between ECU and actuator nodes.

The system defines a single abstract interface (`actuators::servo::Servo`) and a few concrete implementations:

- **`PwmServo`** — runs on actuator nodes and drives a physical servo via PWM + enable pin + optional ADC feedback and current sense.
- **`CanServo`** — runs on the ECU and proxies servo commands/status over CAN.
- **`CanServoAdapter`** — runs on actuator nodes and translates CAN commands to local `PwmServo` calls, while publishing telemetry/config back to the ECU.

This design allows higher-level ECU code to depend on *one* API, independent of transport and physical location of the servo hardware.

---

## Repository layout

- `servo.hpp`  
  Public interface types and the abstract `Servo` class.

- `pwm_servo.hpp/.cpp`  
  Local driver: PWM output, power control, command timing, optional feedback/current sense, calibration.

- `can_servo.hpp/.cpp`  
  ECU-side proxy: sends commands and consumes telemetry.

- `can_servo_adapter.hpp/.cpp`  
  Actuator-side bridge: receives commands, calls `PwmServo`, publishes status/config.

- `servo_model.hpp/.cpp`  
  Servo model database (limits, pulse range, boot time, speed).

- `servo_codec.hpp`  
  CAN frame encoding/decoding (commands + telemetry).

---

## Runtime architecture

### ECU (ThreadX / multithreaded)

The ECU instantiates one `CanServo` per remote servo endpoint. `CanServo`:

- publishes command frames over CAN
- receives telemetry frames
- exposes the `Servo` API to application code

### Actuator node (superloop)

The actuator node owns the physical driver and adapter:

- `PwmServo` drives the hardware and manages power, movement profile, feedback and calibration.
- `CanServoAdapter` translates CAN commands into `PwmServo` calls and periodically publishes telemetry/config.

Conceptually:

ECU application → `CanServo` → CAN-FD → `CanServoAdapter` → `PwmServo` → PWM/Enable/ADC

---

## Servo interface

The interface intentionally stays small:

- Motion: `beginPosition(PositionCommand)`
- Calibration: `beginCalibration(CalibrationConfig)`
- Power lease: `requestWakeup(WakeupRequest)`
- Observability: `getStatus()`, `getConfig()`, `getModel()`

### Motion semantics

A `PositionCommand` specifies:

- `angle_deg`: target angle (absolute or relative)
- `duration_ms`: if non-zero, a linear ramp from start angle to target angle over this duration
- `delay_ms`: optional delay before motion begins (in addition to boot delay)
- `StartPolicy`: `kPreempt` or `kNoInterrupt`

The local implementation clamps angles to the selected model's mechanical limits.

### Power semantics

The servo may require power-up and boot time. `PwmServo` manages this via an enable pin and the model's `boot_time_ms`.

- `requestWakeup()` extends a keep-alive lease (power stays requested long enough to boot and execute motion).
- `ServoConfig::power_mode` controls whether the servo stays on (`kKeepOn`) or powers down when idle (`kAutoOff` + `idle_timeout_ms`).

### Telemetry semantics

`PwmServo` reports:

- `commanded_angle_deg`: current commanded setpoint being output
- `target_angle_deg`: active command target
- `measured_angle_deg`: optional feedback-derived angle (availability depends on configuration/calibration)
- `current_ma`: optional current sense (availability depends on configuration)

`CanServo` returns the **last received telemetry** from the actuator node; it may be stale if telemetry is not arriving.

---

## CAN addressing

Each servo endpoint is identified by:

```cpp
struct CanAddress {
  uint8_t category;
  uint8_t instance;
  uint8_t channel;
};
```

Both sides must use the same address for a given servo:

- ECU: `CanServo(pcan, address)`
- Actuator: `CanServoAdapter(pcan, address, pwm_servo)`

The adapter should only accept commands addressed to its local servo, and should publish telemetry tagged with the same address.

---

## Examples

### 1) Actuator node: local PWM servo + CAN adapter (superloop)

```cpp
#include <cstdint>

#include "pwm_channel.hpp"
#include "digital_out.h"
#include "pwm_servo.hpp"
#include "pcan_core.hpp"
#include "can_servo_adapter.hpp"
#include "board_adc.hpp"   // board-specific ADC wiring

using namespace actuators::servo;

// PWM + enable
PwmChannel pwm_channel{&htim1, TIM_CHANNEL_1};
DigitalOut enable_pin{SERVO_ENABLE_GPIO_Port, SERVO_ENABLE_Pin};

// ADC inputs (optional)
io::adc::Input fb_angle = input(board::adc::InputId::kServoMainFuelAngle);
io::adc::Input i_sense  = input(board::adc::InputId::kCurrentSense);

// Local physical servo
PwmServo main_fuel_servo{
  pwm_channel,
  enable_pin,
  ServoModelId::kPowerHD_T70_12V,
  fb_angle,
  i_sense
};

// CAN + adapter
PCan pcan{1, 2, &hfdcan1};
CanServoAdapter adapter{pcan, CanAddress{1, 2, 0}, main_fuel_servo};

void tick(uint32_t now_ms) {
  // Poll CAN
  pcan.poll();

  // Update local servo driver
  main_fuel_servo.update(now_ms);

  // Publish telemetry and apply incoming commands
  adapter.update(now_ms);
}
```

### 2) ECU: create a `CanServo` proxy and command motion

```cpp
#include "can_servo.hpp"

using namespace actuators::servo;

CanAddress main_fuel_addr{1, 2, 0};
CanServo main_fuel_servo{pcan, main_fuel_addr};

void CommandMainFuelExample() {
  // Move to 45 degrees over 300 ms after a 50 ms delay
  PositionCommand cmd{};
  cmd.angle_deg = 45.0f;
  cmd.duration_ms = 300;
  cmd.delay_ms = 50;
  cmd.position_mode = PositionMode::kAbsolute;
  cmd.start_policy = StartPolicy::kPreempt;

  (void)main_fuel_servo.beginPosition(cmd);
}

void ReadTelemetryExample() {
  const ServoStatus st = main_fuel_servo.getStatus();
  // st contains last-known commanded/target/measured/current and power/mode
}
```

---

## Calibration

If analog feedback hardware is present, the actuator node can run a calibration routine to populate `AngleFeedbackConfig`:

- Drive to PWM min, wait settle, sample ADC N times and average → `v_at_min`
- Drive to PWM max, wait settle, sample ADC N times and average → `v_at_max`
- Store values in config and enable feedback

ECU usage:

```cpp
CalibrationConfig cfg{};
cfg.settle_ms = 2000;
cfg.samples = 8;

main_fuel_servo.beginCalibration(cfg);
```

---

## Threading model

- `PwmServo::update()` is expected to be called from exactly one context.
- `PwmServo` command submission can be called from other contexts and is internally guarded.
- `CanServo` is a proxy; thread-safety depends on how `PCan` is used in your environment.

---

## Adding a new servo model

To add support for a new servo type:

1. Add a new `ServoModelId` in `servo_model.hpp`.
2. Add a corresponding `ServoModel` entry in `servo_model.cpp`.
3. Construct `PwmServo` with that model on the actuator node.

This keeps servo-specific behavior out of transport code and out of application logic.

---

## Roadmap / TODO

- Command ACK/result reporting over CAN
- Telemetry timestamps / sequence counters for freshness detection
- Fault reporting (overcurrent, ADC out-of-range, calibration failure reasons)
- Stricter command validation (finite floats, sane bounds on durations/delays/keep-alive)
