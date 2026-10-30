# EE871 ESP-IDF Diagnostic Basic Bring-Up CLI

This is a diagnostic/basic bring-up example. It owns the E2 GPIO lines for
demonstration and injects callbacks into the framework-neutral EE871 driver.
Production applications should integrate the callbacks into their own GPIO or
bus manager instead of copying this example's bus ownership model directly.

EE871-E2 uses GPIO-style E2 signaling, not ESP-IDF `driver/i2c_master` or
hardware I2C. If multiple tasks can touch the same `EE871` instance or shared
E2 lines, serialize access outside the driver.

The CLI provides the same user-visible command set as the Arduino bring-up CLI:
help, version, scan, probe, recover, driver health, CO2 reads,
feature/capability inspection, configuration/calibration helpers, register/raw
access, diagnostics, bus reset, trace, self-test, stress, and mixed stress
workflows.

`operation` prints the latest persistent-operation evidence without bus I/O.
Maintenance commands reject dirty state, unresolved possible effects and
possible address changes before invoking a write. `resync` checks current
coherence; it does not prove original intent or automatically reopen admission.
The [persistent-maintenance guide](../../../docs/EE871_E2_HIL_RUNNER.md#persistent-maintenance)
describes the evidence and application-owned reconciliation.

`bounds` calculates maximum requested-delay sums from the CLI configuration
without touching the bus. Callback execution and scheduler overhead are outside
the result. It does not perform the live tests run by `timing`; see the
[requested-delay API](../../../README.md#programmatic-requested-delay-bounds).

`co2fastsample` and `co2avgsample` explicitly call the checked MV3/MV4 APIs.
They report the raw word and its validity, checked validity, status/error-code
evidence, and each step's attempted flag and exact transfer status. A value
read failure stops before status; a later failure preserves earlier evidence.
Sensor status bit 3 rejects the checked sample without creating a transport
health failure. Error detail is read only when advertised and unknown codes
remain visible. These commands are manual diagnostics and are not added to
automated HIL/soak plans.

Checked success means successful requested transfers and clear sensor status.
It does not establish warm-up, freshness, concentration accuracy, application
range or product model. Status is read after the value and can trigger the
next measurement and reset its interval counter only when the global interval
is greater than 15 s and the previous value is older than 10 s. This ordering
does not prove a new conversion. The existing `read`, `co2fast` and `co2avg`
commands remain raw. See the
[manual command guide](../../../docs/EE871_E2_HIL_RUNNER.md#manual-checked-co2-diagnostics)
for evidence semantics.

The `sniff` command prints decoded edges synchronously from the transport path.
This perturbs E2 timing; sniffer-enabled traffic is diagnostic only and is not
valid timing or protocol-stability evidence.

- Default SCL: GPIO7
- Default SDA: GPIO6
- Default E2 address: `0`
- Pull-ups: external pull-ups are expected; internal weak pull-ups are disabled
  by default in `ee871_idf::init()`.

Use a bidirectional open-drain level shifter between the 3.3 V ESP32 GPIOs and
the sensor-side E2 bus. The sensor-side pull-ups are 4.7 kOhm to 100 kOhm and bus
high must be 3.6-5.2 V (recommended 4.5-5.0 V). The cable-length guideline is
10 m maximum. Change the GPIO constants in `main/main.cpp` for your board.

Build and monitor from the repository root in an initialized ESP-IDF 6.0.1 or
later environment:

The example discovers the root library through `EXTRA_COMPONENT_DIRS` and its
`REQUIRES "EE871-E2"` entry uses the checkout directory as the ESP-IDF component
name. Keep the repository directory named `EE871-E2`, or update that `REQUIRES`
entry to match the checkout directory name.

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup flash monitor
```

Use `idf.py -C examples/idf/basic_bringup set-target esp32s2 build` for
ESP32-S2 validation. The driver core does not configure GPIO, own the bus, or
log.

The example leaves `readNackRetries` at its default of zero. Its command loop
owns the driver and calls `tick()` between commands; it does not implement a
production sampling or automatic recovery policy. Public bus operations and
persistent-write verification block for their documented bounded durations.

The CLI and `ee871_idf::makeConfig()` install native FreeRTOS `delayMs`/`yield`
callbacks for post-write quiet waits. The default 10 ms slices preserve the
configured total requested wait, including the final partial slice; delay
precedes yield after every slice. The adapter rounds upward with tick-phase
allowance so it does not under-wait a request. Protocol signaling, flash STOP
stretching and the fixed read-retry pause continue using microsecond timing.
Zero waits invoke no long-wait callback. Core defaults remain optional/null,
with a per-millisecond microsecond fallback for existing integrations.

These callbacks run in the owner's task with external serialization retained;
they must be bounded and must not recursively call the driver. Void callbacks
cannot report failures or let the driver prove elapsed time. No hardware
scheduler or watchdog latency improvement is claimed.

See the [ESP-IDF integration guide](../../../docs/IDF_PORT.md) for component
dependencies, callback contracts, timing, and application integration. Build
and hardware qualification results are recorded in the
[validation matrix](../../../docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md).
