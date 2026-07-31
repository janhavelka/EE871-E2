# Arduino-ESP32 3.2.0 HWCDC Serial-Only Characterization

This is a retrospective lab note, not a raw transcript.

On 2026-07-31, before upgrading the COM20 ESP32-S3 firmware from pioarduino
`54.03.20` / Arduino-ESP32 `3.2.0`, the state-only CLI command `dirty` was sent
1,000 times while keeping COM20 open. `dirty` performs no E2 bus operation.

Observed:

- 989 complete replies and 11 timeouts.
- Incomplete receives ended at 9, 73, or 137 bytes: the same offset plus exact
  64-byte USB packet increments.
- Sending the next command without reopening COM20 released the exact queued
  128-byte suffix and prompt.
- Adding duplicate `Serial.flush()` calls produced the same 989/1,000 result;
  that experimental change was reverted.
- No COM-port exception occurred.

The raw serial stream from this focused run was not retained, so these numbers
must be cited as an interactive characterization rather than transcript-backed
HIL. The retained eight-hour soak independently contains the incomplete
replies. The packet-boundary/queued-suffix behavior matches the Arduino-ESP32
HWCDC producer/ISR lost-wakeup race fixed by upstream PR #12606.

After upgrading to pioarduino `55.03.39` / Arduino-ESP32 `3.3.9`, the retained
then-current discriminator completed 10,000/10,000 identical 201-byte
`dirty` replies.
