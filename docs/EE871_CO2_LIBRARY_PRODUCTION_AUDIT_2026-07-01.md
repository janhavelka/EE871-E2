# EE871 CO2 Library Production Audit - 2026-07-01

> Historical point-in-time audit. Later implementation and COM20 HIL work has
> addressed some findings, but not every production gap. Use
> [README.md](README.md) for document routing and
> [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md)
> for the current evidence ledger; verify each finding against current code.

## Scope

Audited the EE871-E2 library for production CO2 sensor use against the repository guidelines, extracted E2/EE871 protocol notes, public API contracts, tests, examples, and packaging metadata.

Files reviewed included:

- `include/EE871/*.h`
- `src/EE871.cpp`
- `test/test_basic.cpp`
- `test/support/FakeE2Transport.h`
- Arduino and ESP-IDF examples
- `platformio.ini`, `library.json`, `idf_component.yml`, root `CMakeLists.txt`
- README, validation docs, and stored HIL summaries

No hardware HIL was run during this audit.

## Overall Verdict

The library has a solid framework-neutral shape and the basic E2 framing implementation is close to the vendor examples. The main production blockers are not style issues. They are state-machine and field-semantics problems:

- valid long write stretches can time out,
- `begin()` can accept the wrong or non-CO2 device,
- `OFFLINE` is not latched for normal operations,
- raw CO2 reads are the only CO2 read API and examples normalize ignoring status/range/warm-up,
- persistent maintenance commands in examples can write destructive values from invalid input.

I would not call this production-ready for unattended field deployment until the High findings below are fixed and covered by native fake tests.

## Positive Findings

- Core library code stays framework-neutral: no Arduino, ESP-IDF GPIO, `Wire`, logging, heap containers, or owned bus objects in `include/` or `src/`.
- Control-byte bit placement, command nibbles, EE871 group constants, MV3/MV4 low/high mapping, and PEC formulas match the extracted protocol references.
- The read transaction's data-byte ACK before PEC matches the vendor `knl_E2bus_readByteFromSlave()` routine, even though some local recipe text is abbreviated.
- Native tests, Arduino example builds, and repository contract scripts pass in the current checkout.

## Findings

### 1. High - Write transactions cannot tolerate documented 150 ms / 300 ms write clock stretching

**Evidence**

- `waitSclHigh()` enforces `bitTimeoutUs` and `byteTimeoutUs`: `src/EE871.cpp:37`.
- Defaults are 25 ms per bit and 35 ms per byte: `include/EE871/Config.h:67`.
- `_writeCommandRaw()` uses the same short ACK/STOP timing for write PEC and final STOP: `src/EE871.cpp:1197`, `src/EE871.cpp:1218`.
- Write delay is applied only after a successful STOP: `src/EE871.cpp:589`, `src/EE871.cpp:635`.
- Repository rules say `0x10`/`0x50` writes can take up to 150 ms, and interval pair commit up to 300 ms.

**Risk**

A valid EE871 can hold SCL low during a flash/pointer write longer than the generic byte timeout. The driver can report `TIMEOUT` on a valid write, mark health degraded/offline, or leave persistent state uncertain.

**Simple robust solution**

Introduce a write-specific bounded wait path instead of reusing normal read byte timing:

- Keep normal per-bit/per-byte limits for regular read bytes.
- For `0x10` and `0x50`, allow a post-write/final-ACK/final-STOP deadline based on `writeDelayMs`.
- For the second byte of the `0xC6/0xC7` interval pair, allow `intervalWriteDelayMs`.
- Keep the operation synchronous and bounded; do not add background polling.
- Add native tests where SCL is held low for just under and just over the write deadline.

### 2. High - Custom pointer writes do not observe the required `0x50` write timing

**Evidence**

- `setCustomPointer()` sends `0x50` and immediately returns: `src/EE871.cpp:507`.
- `customRead()` immediately starts `0x51` reads after `setCustomPointer()`: `src/EE871.cpp:537`.
- `begin()` also sets pointer `0x07` and immediately reads feature bytes: `src/EE871.cpp:300`.

**Risk**

Custom memory reads can fail or read the wrong location on hardware that needs the documented bounded time after `0x50`. Feature caching during `begin()` is especially exposed.

**Simple robust solution**

Create one pointer-write helper used by `begin()`, `setCustomPointer()`, and `customRead()`:

- send `0x50` through raw or tracked transport as appropriate,
- apply the write-specific bounded deadline from finding 1,
- then start `0x51` reads.

Do not duplicate pointer write logic in `begin()`.

### 3. High - `begin()`, `probe()`, and `recover()` do not validate full EE871 CO2 identity/capability

**Evidence**

- `begin()` validates only group `0x0367`: `src/EE871.cpp:271`.
- `probe()` validates only group: `src/EE871.cpp:388`.
- `recover()` calls only `readGroup()`: `src/EE871.cpp:419`.
- Subgroup and available CO2 bit constants exist: `include/EE871/CommandTable.h:94`.
- `readSubgroup()` and `readAvailableMeasurements()` exist but are not part of startup/probe/recover validation: `src/EE871.cpp:670`, `src/EE871.cpp:681`.

**Risk**

The driver can initialize against a non-EE871, a wrong subgroup, or a device that does not advertise CO2. That violates the fail-closed requirement and makes later CO2 reads misleading.

**Simple robust solution**

Add a single internal identity helper:

- read group low/high,
- read subgroup and require `0x09`,
- read available measurements and require bit `0x08`,
- return precise transport errors when I/O fails,
- return `NOT_SUPPORTED` for wrong identity or missing CO2 capability.

Use raw reads in `begin()` and `probe()`. Use tracked reads in `recover()` so recovery failures update health.

### 4. High - Normal bus APIs can self-recover from `OFFLINE`

**Evidence**

- Tracked wrappers always call `_updateHealth()` with no OFFLINE precondition: `src/EE871.cpp:1132`, `src/EE871.cpp:1225`.
- Any tracked success resets state to `READY`: `src/EE871.cpp:1239`.
- Public normal operations such as `readStatus()` go straight into tracked reads: `src/EE871.cpp:685`.

**Risk**

After the driver reaches `OFFLINE`, a normal read/write can touch the bus and move the driver back to `READY`. The application loses explicit control over recovery cadence.

**Simple robust solution**

Add a small normal-operation guard before tracked transfers:

- if initialized and `DriverState::OFFLINE`, return `BUSY` or a dedicated precise existing status such as `BUSY` with message `"Driver offline; call recover()"`,
- do not touch E2 lines in that path,
- allow only `probe()`, `recover()`, `busReset()`, `checkBusIdle()`, and cache-only diagnostics to bypass it,
- make `recover()` use a scoped private bypass and full identity validation.

### 5. High - CO2 read API has no checked sample path for status errors, range, warm-up, or stale data

**Evidence**

- Public measurement APIs are only raw `readCo2Fast()` and `readCo2Average()`: `include/EE871/EE871.h:527`.
- Implementations only read MV3/MV4 low/high and return OK on transport success: `src/EE871.cpp:699`.
- `Status::Err` has no sensor-domain error such as `CO2_SENSOR_ERROR`: `include/EE871/Status.h:14`.
- README quick start prints `readCo2Average()` directly every second: `README.md:143`.
- CLI `read`, `co2fast`, `co2avg`, and stress commands also treat raw value-read success as CO2 success: `examples/01_basic_bringup_cli/main.cpp:925`, `examples/01_basic_bringup_cli/main.cpp:994`.

**Risk**

The easiest and documented read path can report a ppm value while the sensor status bit says the last CO2 measurement failed, while the value is out of range, during warm-up, or while data is stale after a status-triggered measurement.

**Simple robust solution**

Keep raw APIs source-compatible, but add explicit checked APIs:

- `Co2ReadResult` with ppm, ppm validity, status byte, status validity, optional error code, and component read statuses.
- `readCo2AverageSample(Co2ReadResult&)` and `readCo2FastSample(Co2ReadResult&)`.
- Read MV3/MV4 first, then read status second.
- If status bit 3 is set, read error code when supported and return an append-only `CO2_SENSOR_ERROR`.
- Validate ppm against a documented configured range and return `OUT_OF_RANGE` without counting it as E2 transport health failure.
- Report warm-up/stale/trigger timing as result metadata or documentation, not hidden scheduling policy.
- Update README and examples so `read`/production-oriented paths use checked samples, while `co2avg`/`co2fast` remain clearly labeled raw reads.

### 6. High - Example persistent-write commands can write from invalid input and lack an unlock/confirmation

**Evidence**

Arduino example uses `String::toInt()`, which returns `0` for invalid tokens:

- `addr xyz` writes bus address `0`: `examples/01_basic_bringup_cli/main.cpp:1313`.
- `factor xyz` writes interval factor `0`: `examples/01_basic_bringup_cli/main.cpp:1337`.
- `filter xyz` writes filter `0`: `examples/01_basic_bringup_cli/main.cpp:1352`.
- `mode xyz` writes mode `0`: `examples/01_basic_bringup_cli/main.cpp:1365`.
- `offset xyz` writes offset `0`: `examples/01_basic_bringup_cli/main.cpp:1378`.
- `gain xyz` writes gain `0`: `examples/01_basic_bringup_cli/main.cpp:1390`.

ESP-IDF example has explicit fallback to `0` on parse failure:

- `addr`: `examples/idf/basic_bringup/main/main.cpp:2176`
- `interval`: `examples/idf/basic_bringup/main/main.cpp:2195`
- `filter`: `examples/idf/basic_bringup/main/main.cpp:2229`
- `mode`: `examples/idf/basic_bringup/main/main.cpp:2247`
- `offset`: `examples/idf/basic_bringup/main/main.cpp:2263`

**Risk**

A typo in a bench CLI can change persistent configuration, calibration, bus address, or operating mode. Some values require power-cycle recovery or can invalidate calibration.

**Simple robust solution**

Refactor CLI parsing before any persistent write:

- use strict token parsing with exact argument count,
- reject invalid tokens instead of coercing to zero,
- validate ranges before calling the library,
- require an explicit maintenance unlock/confirmation for persistent writes,
- make read-only commands available without unlock,
- add native parser tests for invalid input.

### 7. High - `begin()` silently ignores feature-cache read failures

**Evidence**

- `begin()` sets pointer to `0x07` and reads `0x07..0x09`: `src/EE871.cpp:300`.
- If any feature read fails, the code continues with all feature flags left at zero: `src/EE871.cpp:314`.

**Risk**

A real `NACK`, `TIMEOUT`, `BUS_STUCK`, or `PEC_MISMATCH` during capability discovery becomes a successful initialization with every optional feature disabled. Later APIs return false `NOT_SUPPORTED`, hiding the original transport failure.

**Simple robust solution**

Do not guess feature state on failed reads:

- after full identity/CO2 validation succeeds, require feature-cache reads to succeed and return the first precise failure, or
- add an explicit `capabilitiesKnown` diagnostic state and make guarded APIs return that precise state.

The simpler production path is to make feature-cache read failure fatal to `begin()`.

### 8. Medium - Accepted single-byte persistent write failures do not set dirty/unverified diagnostics

**Evidence**

- `_customWriteDirect()` can complete the write command, then fail readback verification: `src/EE871.cpp:583`, `src/EE871.cpp:591`.
- Public `customWrite()` returns that error without marking dirty: `src/EE871.cpp:551`.
- Single-byte persistent wrappers use `customWrite()` directly: bus address `src/EE871.cpp:801`, CO2 interval factor `src/EE871.cpp:845`, filter `src/EE871.cpp:864`, operating mode `src/EE871.cpp:879`, auto-adjust `src/EE871.cpp:912`.
- Multi-byte paths such as part name explicitly mark dirty on failure: `src/EE871.cpp:781`.

**Risk**

A persistent write can be accepted by the device but fail during STOP, delay, or verification. The driver reports an error but diagnostics do not say persistent state may need inspection.

**Simple robust solution**

Centralize persistent-write classification:

- define which custom-memory addresses are persistent/configuration/maintenance,
- have `_customWriteDirect()` expose whether the write was accepted,
- mark dirty/unverified for accepted write failures or verify mismatches on persistent addresses,
- keep volatile/control-only writes separate if they are not persistent.

### 9. Medium - `recover()` can mask a stuck bus with a later error

**Evidence**

- `recover()` calls `busReset()` and ignores the result: `src/EE871.cpp:414`.
- It then returns the result of `readGroup()`: `src/EE871.cpp:419`.

**Risk**

If reset proves `BUS_STUCK`, the final status can be replaced by a later `TIMEOUT` or `NACK`. That loses the most useful field diagnostic.

**Simple robust solution**

Return `BUS_STUCK` immediately when `busReset()` cannot free the bus. Only run tracked identity validation after reset succeeds.

### 10. Medium - Responding-but-wrong identity is reported as `DEVICE_NOT_FOUND`

**Evidence**

- `begin()` wrong group: `src/EE871.cpp:289`.
- `probe()` wrong group: `src/EE871.cpp:404`.
- `readGroup()` wrong group: `src/EE871.cpp:665`.
- `readSubgroup()` wrong subgroup: `src/EE871.cpp:676`.

**Risk**

The device did respond, so this is not definite absence. Reporting `DEVICE_NOT_FOUND` hides an unsupported or wrong device and conflicts with the repository status rules.

**Simple robust solution**

Return `NOT_SUPPORTED` for wrong group, wrong subgroup, and missing CO2 bit. Keep `NACK`, `TIMEOUT`, and `BUS_STUCK` for transport/presence failures.

### 11. Medium - Timing configuration does not enforce the required clock-rate envelope or byte deadline honestly

**Evidence**

- Config validation enforces only minimum `clockLowUs`/`clockHighUs`: `src/EE871.cpp:213`.
- E2 requirements specify 500-5000 Hz and `tCLKH/tCLKL >= 100 us`.
- `waitSclHigh()` checks byte timeout only while waiting for SCL high: `src/EE871.cpp:43`.
- Fixed setup/high/low delays are added in `writeBit()`/`readBit()` but no final byte-deadline check rejects impossible configs: `src/EE871.cpp:89`, `src/EE871.cpp:104`.

**Risk**

The driver can be configured below the 500 Hz minimum, or with byte timeout values that are already too small for the configured bit timing. Tests use a fake with very small byte timeouts, so this is not caught.

**Simple robust solution**

Normalize timing validation in one helper:

- require `clockLowUs >= 100` and `clockHighUs >= 100`,
- require effective period not to exceed 2000 us, including the driver's setup delay if it extends the generated period,
- require `byteTimeoutUs` to cover at least a no-stretch byte plus ACK at the configured timing,
- add final elapsed checks after every byte phase.

### 12. Medium - START does not verify SDA is high before issuing START

**Evidence**

- `e2Start()` releases SDA/SCL and waits only for SCL high: `src/EE871.cpp:59`.
- It does not check `readSda()` before pulling SDA low.

**Risk**

An SDA-stuck-low condition can produce an invalid START and later `NACK`/`TIMEOUT` instead of a precise `BUS_STUCK`.

**Simple robust solution**

After releasing both lines and waiting for SCL high, require SDA high. Return `BUS_STUCK` if SDA remains low.

### 13. Medium - CO2 calibration writes are not guarded by custom-adjustment support bits

**Evidence**

- E2 docs define supported-function bytes `0x03..0x06`; custom adjustment is supported only when the relevant bit is set.
- Public APIs expose CO2 offset/gain writes: `include/EE871/EE871.h:483`, `include/EE871/EE871.h:494`.
- Implementations write `0x58..0x5B` directly without checking adjustment support: `src/EE871.cpp:942`, `src/EE871.cpp:976`.

**Risk**

Maintenance calibration writes may be attempted on firmware that does not support them. That can return unclear transport/status behavior or alter undocumented memory.

**Simple robust solution**

Extend feature discovery narrowly:

- read and cache only the relevant `0x03..0x06` supported-function bytes,
- add `hasCo2OffsetGainAdjustment()` or equivalent,
- guard calibration writes with `NOT_SUPPORTED`,
- keep read-only diagnostic reads explicit.

### 14. Low - Some APIs check feature support before invalid caller parameters

**Evidence**

- `writeMeasurementInterval()` checks `hasGlobalInterval()` before range: `src/EE871.cpp:606`.
- `writeBusAddress()` checks `hasAddressConfig()` before address range: `src/EE871.cpp:805`.

**Risk**

Invalid application input can be hidden as `NOT_SUPPORTED` on devices without that feature. That makes caller bugs harder to diagnose.

**Simple robust solution**

After `_initialized`, validate caller parameters first, then check device feature support.

### 15. Medium - Native fake coverage does not match the field-facing API surface

**Evidence**

- `test/test_basic.cpp` has good coverage for core health/dirty paths, but many APIs are only represented by `NOT_INITIALIZED` checks: `test/test_basic.cpp:211`.
- Registered tests end at 31 cases: `test/test_basic.cpp:585`.
- Missing success/unsupported/NACK/timeout/boundary/state tests include bus address, filter, mode, interval factor, auto-adjust, serial/part-name success, checked CO2 status behavior, calibration feature support, and OFFLINE fast-fail behavior.

**Risk**

The existing test suite passes while multiple production-contract failures remain undetected.

**Simple robust solution**

Add a table-driven native fake coverage matrix for every public field-facing API:

- success,
- unsupported feature,
- boundary values,
- NACK or timeout,
- PEC mismatch where relevant,
- OFFLINE behavior,
- dirty/unverified state when persistent writes are accepted then fail.

### 16. Medium - Validation evidence and packaging claims are ahead of recorded proof

**Evidence**

- README says version metadata is `1.0.0` and lists HIL PASS evidence: `README.md:20`.
- Stored safe HIL artifact reports library `0.3.0`: `hil_results/safe_default/ee871_20260601T185912Z/summary.md:33`.
- The deeper hardware matrix notes the caveat: `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md:177`.
- Functional matrix rows are labeled `S2, S3` while the summary says S2 HIL is not recorded: `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md:21`, `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md:276`.
- Package metadata advertises ESP-IDF support: `library.json:38`, `idf_component.yml:5`, while README says pure ESP-IDF build proof remains to be verified: `README.md:39`.

**Risk**

Release consumers may read validation claims as applying to the current `1.0.0` metadata, both S2 and S3 hardware, and ESP-IDF builds. The recorded evidence does not support that broad claim.

**Simple robust solution**

Make the validation ledger mechanically honest:

- mirror the `0.3.0` HIL caveat in README and CHANGELOG,
- split matrix rows by board or mark PASS rows S3-only,
- record a current `1.0.0` HIL run before release claims,
- run and record pure `idf.py` builds or qualify ESP-IDF support as unverified.

### 17. Low - Arduino example does not surface local electrical assumptions

**Evidence**

- Board config gives pins and timing, but not pull-up voltage, level shifter, or cable assumptions: `examples/common/BoardConfig.h:28`.
- E2 transport just configures open-drain GPIO: `examples/common/E2Transport.h:27`.
- Electrical constraints are documented elsewhere: `docs/EE871_E2_Protocol_and_Register_Map.md:57`.

**Risk**

Users may copy the example without the required external pull-ups, 3.3 V level shifting, bus voltage, or cable constraints.

**Simple robust solution**

Add a short example-local README/startup note with:

- 4.7k-100k pull-ups,
- bus high voltage 3.6-5.2 V, recommended 4.5-5.0 V,
- bidirectional open-drain level shifter for 3.3 V MCUs,
- cable length guidance,
- one-owner/serialized bus access reminder.

## Verification Run During Audit

Commands run in this checkout:

```text
pio test -e native
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
pio run -e ex_bringup_s3
pio run -e ex_bringup_s2
idf.py --version
```

Results:

- `pio test -e native`: PASS, 31/31 native tests.
- `check_core_timing_guard.py`: PASS.
- `check_cli_contract.py`: PASS.
- `check_idf_example_contract.py`: PASS.
- `pio run -e ex_bringup_s3`: SUCCESS.
- `pio run -e ex_bringup_s2`: SUCCESS.
- `idf.py --version`: not available in this shell, so pure ESP-IDF build was not run.

The passing tests/builds do not cover the high-risk behavioral gaps above.

## Suggested Fix Order

1. Fix write timing and pointer-write handling.
2. Add full identity/capability validation and correct `NOT_SUPPORTED` status mapping.
3. Latch `OFFLINE` for normal operations and make `recover()` the only online transition path.
4. Add checked CO2 sample APIs and update README/examples to distinguish raw vs checked reads.
5. Harden persistent-write dirty handling and CLI parsing/unlock.
6. Expand fake coverage around the production contracts.
7. Re-run and update validation evidence after fixes.
