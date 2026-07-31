# COM20 Niche Diagnostics

Date: 2026-07-30

This report condenses five ad-hoc, ANSI-normalized Platform `54.03.20` console
captures. The original text files were removed after their distinct results
were transcribed here. This is historical evidence for the ESP32-S3 revision
0.2 / 4 MB flash / 2 MB QSPI PSRAM bench, not evidence for the current
platform.

## Identity And Capability Reads

- Identity: group `0x0367`, subgroup `0x09`, available measurements `0x08`
  with the CO2 bit present.
- Status: `0x00`, without a CO2 error.
- Sensor firmware: `1.4`; E2 specification version: `4`.
- Feature bytes: operating functions `0x93`, operating modes `0x00`, special
  features `0x00`.
- Serial bytes: `31 39 32 30 39 33 35 36 30 32 33 36 38 41 00 00`
  (`1920935602368A..` when non-printable bytes are shown as dots).
- Part name: `EE871`; bus address: `0`; interval: `150 ds`.
- The old CLI also decoded raw `0x55` values from unsupported configuration
  registers. Those observations are retained only as the pre-fix behavior;
  they are not valid supported operating-mode values.

## Read/Write Guards

All 11 selected checks passed:

- `addr 8`, `interval 149`, `interval 36001`, and `mode 4` returned
  `OUT_OF_RANGE`.
- Same-value `addr 0`, `factor 85`, `filter 85`, and `mode 1` writes returned
  `NOT_SUPPORTED` from capability guards and did not reach persistent memory.
- `autoadj start` returned `NOT_SUPPORTED`.
- Final driver state was READY with zero consecutive failures, and persistent
  state was clean with no resync required.

The part-name capture separately performed a same-value `EE871` write and
verified an `EE871` readback. Its broader interactive session contained
unrelated harness-expectation failures, so only this verified block is
retained; the session was never classified as an overall PASS.

## GPIO/E2 Diagnostics

- Idle levels: SCL high and SDA high.
- Pin-toggle test: both GPIOs could be pulled low and released high.
- Clock pulses: `5/5` low and `5/5` high samples passed in the full diagnostic.
- Address scan: address `0` found with valid PEC; addresses `1..7` did not
  respond.
- Library command test: all nine control-byte transactions passed with valid
  PEC.
- Timing discovery received ACK plus valid PEC at all six supported diagnostic
  points (`1000`, `500`, `250`, `200`, `150`, and `100 us` per half-cycle).
  It also responded at `75` and `50 us`; those two points are characterization
  outside the documented supported timing range and are not configuration
  recommendations.

## Trace, Sniffer, And Mixed Stress

- Buffered trace statistics after a real status transaction reported
  `Pending: 0`, `Dropped: 0`, and capacity `512`.
- The protocol sniffer decoded the selected status transaction without
  changing the driver result.
- `stress_mix 500` completed `500/500` with zero errors.
- Health delta for that mixed block was `+937` successes and `+0` failures.
- Final state was READY, online, with zero consecutive failures; persistent
  state remained clean.

## Scope

These results cover identity, capability, validation guards, GPIO/E2 signaling,
PEC behavior, diagnostic timing characterization, trace buffering, sniffer
operation, and mixed safe reads. They do not validate CO2 accuracy,
calibration, ESP32-S2 hardware, pure ESP-IDF hardware, or long-term stability.
