# AGENTS.md - EE871 E2 Production Embedded Guidelines

## Role and Target
You are a professional embedded software engineer building a production-grade EE871 CO2 sensor library using the E2 bus.

- Target: ESP32-S2 / ESP32-S3, Arduino framework, PlatformIO.
- Goals: deterministic behavior, long-term stability, clean API contracts, portability, no surprises in the field.
- These rules are binding.

---

## Repository Model (Single Library)

```
include/EE871/          - Public API headers only (Doxygen)
  CommandTable.h        - Control bytes, custom addresses, bit masks
  Status.h
  Config.h
  EE871.h
  Version.h             - Auto-generated (do not edit)
src/                    - Implementation (.cpp)
examples/
  01_*/
  common/               - Example-only helpers (Log.h, BoardConfig.h, E2Transport.h,
                          E2Diagnostics.h, BuildConfig.h)
platformio.ini
library.json
README.md
CHANGELOG.md
AGENTS.md
```

Rules:
- `examples/common/` is NOT part of the library. It simulates project glue and keeps examples self-contained.
- No board-specific pins/bus setup in library code; only in `Config`.
- Public headers only in `include/EE871/`.
- Examples demonstrate usage and may use `examples/common/BoardConfig.h`.
- Keep the layout boring and predictable.

---

## Core Engineering Rules (Mandatory)

- Deterministic: no unbounded loops or waits; all timeouts use explicit deadlines.
- Lifecycle: `Status begin(const Config&)`, `void tick(uint32_t nowMs)`, `void end()`.
- E2 bus operations are synchronous but bounded by documented timeouts; `tick()` is for periodic polling or scheduling.
- No `delay()` in library code. Only `delay_us()` from the E2 HAL for bit timing.
- No heap allocation in steady state (no `String`, `std::vector`, `new` in normal ops).
- No logging in library code; examples may log.
- No macros for constants; use `static constexpr`. Macros only for conditional compile or logging helpers.

---

## E2 Bus Transport + HAL (Required)

- The library MUST NOT own the bus. It never touches `Wire` or I2C drivers directly.
- `Config` MUST accept a transport adapter for the E2 HAL:
  - `set_scl(level)` / `set_sda(level)` where level=1 means release line (open-drain high), level=0 means pull low.
  - `read_scl()` / `read_sda()` for clock stretching and data sampling.
  - `delay_us(t)` for bit timing.
- Transport errors MUST map to `Status` (no leaking platform-specific error codes).
- The library MUST NOT configure bus timeouts, pull-ups, or pins.

---

## E2 Electrical + Timing Requirements (Mandatory)

- Two-wire open-drain bus (CLK, DATA, GND). Idle is high via pull-ups.
- Pull-ups: 4.7k to 100k; bus-high voltage 3.6-5.2 V (recommended 4.5-5.0 V).
- Use a bidirectional open-drain level shifter if the MCU is 3.3 V.
- Cable length guideline: <= 10 m.
- Clock rate 500-5000 Hz; `tCLKH >= 100 us`, `tCLKL >= 100 us`.
- START: DATA high->low while CLK high, wait >= 4 us, then pull CLK low.
- STOP: DATA low->high while CLK high.
- Data stable while CLK high; transitions only while CLK low (except START/STOP).
- Clock stretching: slave may hold CLK low up to 25 ms after any bit; total per byte <= 35 ms.
  - When releasing CLK high, read back `read_scl()` and enforce timeouts.

---

## E2 Framing, Control Byte, PEC (Mandatory)

- Each byte is 8 bits MSB first plus ACK/NACK 9th bit.
- Control byte:
  - b7..b4: MainCommand
  - b3..b1: DeviceAddress (0-7)
  - b0: R/W (1 = read, 0 = write)
- PEC is the low byte of the sum of all transmitted bytes.
  - Read: `PEC = (ControlByte + DataByte) & 0xFF`
  - Write: `PEC = (ControlByte + AddressByte + DataByte) & 0xFF`
- Always verify PEC for every transaction.
- Read sequence: START -> ControlByte(read) -> DataByte -> PEC -> master NACK -> STOP.
- Write sequence: START -> ControlByte(write) -> AddressByte -> DataByte -> PEC -> STOP.

---

## Required Transaction Recipes

- Read normal byte (control byte read):
  - START, ControlByte(read), DataByte, PEC, master NACK, STOP, verify PEC.
- Read custom memory at address A:
  - Write 0x50 with pointer high=0x00, pointer low=A, PEC, STOP.
  - Read with 0x51, verify PEC. For multi-byte, repeat reads; pointer auto-increments.
- Write custom memory byte at address A:
  - Write 0x10 with AddressByte=A, DataByte=value, PEC, STOP.
  - Wait for commit with timeout, then read back to verify.

---

## EE871 Device Requirements

- Default bus address 0; configurable via custom memory `0xC0`.
- Identification:
  - Group = 0x0367 (read low via 0x11, high via 0x41).
  - Subgroup = 0x09 (read via 0x21).
  - Available measurements = 0x08 (read via 0x31, bit3 = CO2).
- Status byte (0x71) bit3 indicates CO2 error; reading status triggers a new measurement.
- Measurement values:
  - MV3 (fast response) low/high via 0xC1 / 0xD1.
  - MV4 (averaged) low/high via 0xE1 / 0xF1.
  - Read low byte first, then high byte to latch paired data.
  - MV1 and MV2 are not defined for EE871; treat as unsupported.
- Custom memory access:
  - Set pointer with 0x50 (address high, address low).
  - Read with 0x51; pointer auto-increments.
  - Write with 0x10 (address, data).
- Error codes are read from custom memory `0xC1` when status bit3 indicates error.
  - CO2 error codes: 1 (supply voltage low), 200 (sensor counts low),
    201 (sensor counts high), 202 (supply voltage breakdown at peak).

### Important custom memory addresses
- 0x00/0x01 firmware version, 0x02 E2 spec version.
- 0x07-0x09 feature flags (operating functions, modes, special features).
- 0xA0-0xAF serial number (read-only), 0xB0-0xBF part name (read/write).
- 0xC0 bus address, 0xC1 error code.
- 0xC6/0xC7 global measurement interval (0.1 s units), 0xCB specific CO2 interval factor.
- 0xD3 filter, 0xD8 operating mode, 0xD9 auto adjustment control/status.
- 0xFE/0xFF pointer visibility.

---

## Measurement Timing (EE871 CO2)

- Warm-up: 5-10 s after power-up before relying on readings.
- Measurement time: ~0.7 s.
- Global interval: 15-3600 s (typical default 15 s).
- Status read can trigger a measurement if last measurement is older than 10 s.
  - New data available ~5-10 s after trigger; trigger resets interval counter.

---

## Write Timing and Flash Behavior (Mandatory)

- 0x10 or 0x50 writes can take up to 150 ms; slave may hold CLK low during this time.
- 0xC6/0xC7 interval write commits after both bytes; delay up to 300 ms.
- After any write, wait with a deadline and read back to verify.

---

## Driver Architecture: Managed Synchronous Driver

The driver follows a managed synchronous model with health tracking:

- All public E2 operations are blocking but bounded by documented timeouts.
- `tick()` may be used for periodic polling, measurement scheduling, or recovery policies.
- Health is tracked via tracked transport wrappers; public API never calls `_updateHealth()` directly.
- Recovery is manual via `recover()`; the application controls retry strategy.

### DriverState (4 states only)

```cpp
enum class DriverState : uint8_t {
  UNINIT,    // begin() not called or end() called
  READY,     // Operational, consecutiveFailures == 0
  DEGRADED,  // 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    // consecutiveFailures >= offlineThreshold
};
```

State transitions:
- `begin()` success -> READY
- Any E2 transfer failure in READY -> DEGRADED
- Success in DEGRADED/OFFLINE -> READY
- Failures reach `offlineThreshold` -> OFFLINE
- `end()` -> UNINIT

### Transport Wrapper Architecture

All E2 bus operations go through layered wrappers:

```
Public API (readStatus, readMeasurement, customRead, customWrite)
    v
Protocol helpers (readControlByte, customRead/write)
    v
TRACKED wrappers (_e2TransferTracked, _e2WriteTracked)
    v  <- _updateHealth() called here ONLY
RAW wrappers (_e2TransferRaw, _e2WriteRaw)
    v
Transport callbacks (Config::set_scl/set_sda/read_scl/read_sda/delay_us)
```

**Rules:**
- Public API methods NEVER call `_updateHealth()` directly.
- Protocol helpers use TRACKED wrappers -> health updated automatically.
- `probe()` uses RAW wrappers -> no health tracking (diagnostic only).
- `recover()` tracks probe failures (driver is initialized, so failures count).

### Health Tracking Rules

- `_updateHealth()` called ONLY inside tracked transport wrappers.
- State transitions guarded by `_initialized` (no DEGRADED/OFFLINE before `begin()` succeeds).
- NOT called for config/param validation errors (INVALID_CONFIG, INVALID_PARAM).
- NOT called for precondition errors (NOT_INITIALIZED).
- `probe()` uses raw E2 transfers and does NOT update health (diagnostic only).

### Health Tracking Fields

- `_lastOkMs` - timestamp of last successful E2 operation
- `_lastErrorMs` - timestamp of last failed E2 operation
- `_lastError` - most recent error Status
- `_consecutiveFailures` - failures since last success (resets on success)
- `_totalFailures` / `_totalSuccess` - lifetime counters (wrap at max)

---

## Versioning and Releases

Single source of truth: `library.json`. `Version.h` is auto-generated and must never be edited.

SemVer:
- MAJOR: breaking API/Config/enum changes.
- MINOR: new backward-compatible features or error codes (append only).
- PATCH: bug fixes, refactors, docs.

Release steps:
1. Update `library.json`.
2. Update `CHANGELOG.md` (Added/Changed/Fixed/Removed).
3. Update `README.md` if API or examples changed.
4. Commit and tag: `Release vX.Y.Z`.

---

## Naming Conventions

- Member variables: `_camelCase`
- Methods/Functions: `camelCase`
- Constants: `CAPS_CASE`
- Enum values: `CAPS_CASE` or `X1`, `X2` for short forms
- Locals/params: `camelCase`
- Config fields: `camelCase`
