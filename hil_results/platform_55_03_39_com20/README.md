# ESP32-S3 COM20 Evidence — Platform 55.03.39

This directory contains curated prior-platform evidence captured on
2026-07-31 from the ESP32-S3 revision 0.2 bench target with 4 MB flash and
2 MB QSPI PSRAM. The firmware used pioarduino `55.03.39`,
Arduino-ESP32 `3.3.9`, ESP-IDF `5.5.4`, and EE871-E2 `1.0.1`.

Authoritative artifacts:

- `final_v1_0_1_exact_source/`: targeted safe/extended HIL on the final
  firmware source, 144/144 PASS; selftest 26 PASS / 0 FAIL / 1
  unsupported-mode SKIP; stress 500/500; final READY, zero transport failures,
  persistent state clean.
- `serial_discriminator_v1_0_1_10000/`: serial-only `dirty` discriminator,
  10,000/10,000 PASS in 14.078 s with every reply exactly 201 bytes. This
  command performs no E2 operation.
- `final_v1_0_1_scheduled_nack_9min/`: accelerated scheduled-read regression,
  108 cycles / 543.594 s; 564 ordinary PASS, two fully framed MV3
  control-byte NACK attempts recovered by one 1,500 ms harness retry each,
  and zero hard failures, review, SKIP, reconnect, or counter regression.
- `operating_mode_fail_closed_v1_0_1/`: unsupported `mode` returned
  `NOT_SUPPORTED`, decoded no stale value, left tracked transport counters
  unchanged, and preserved READY state.

Generated PASS runs retain their Markdown command ledger (or compact JSON for
the focused discriminators). The scheduled regression JSON retains every row
and the two abnormal NACK excerpts. Redundant ordinary-PASS serial output and
duplicated JSON were removed during the 2026-07-31 evidence condensation.

The two scheduled NACKs were about 105 s apart at nearly identical phase modulo
the configured 15 s interval. This records schedule-correlated
recurrence at the E2 control-byte ACK boundary and distinguishes the events
from host USB reply truncation. It does not identify an electrical or
sensor-internal cause.

This directory contains targeted regression evidence, not a completed
long soak, CO2 accuracy test, calibration test, or pure
ESP-IDF validation.
