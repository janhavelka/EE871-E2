# EE871-E2 ESP-IDF Portability Status

Last audited: 2026-03-01

## Current Reality
- Library core is already platform-agnostic (no direct Arduino timing or GPIO calls).
- Transport is fully callback-driven through `EE871::Config`:
  - `setScl`, `setSda`, `readScl`, `readSda`, `delayUs`
- Time base for health counters is supplied by caller via `tick(nowMs)`.

## ESP-IDF Adapter Requirements
Provide E2 bus callbacks backed by ESP-IDF GPIO and delay primitives:
1. Open-drain style line control callbacks for SCL/SDA.
2. Line read callbacks for SCL/SDA.
3. Microsecond delay callback (typically `esp_rom_delay_us`).

## Minimal Adapter Pattern
```cpp
static void setLine(bool high, void* user);
static bool readLine(void* user);
static void delayUs(uint32_t us, void*) { esp_rom_delay_us(us); }

EE871::Config cfg{};
cfg.setScl = setScl;
cfg.setSda = setSda;
cfg.readScl = readScl;
cfg.readSda = readSda;
cfg.delayUs = delayUs;
```

## Porting Notes
- Keep calling `tick(nowMs)` from your scheduler loop.
- Keep E2 timing settings (`clockLowUs`, `clockHighUs`, timeouts) consistent with hardware pull-ups and bus capacitance.
- Status/error behavior depends on robust callback error handling.

## Verification Checklist
- Native tests pass (`pio test -e native`).
- Bring-up CLI builds and responds on target board.
- Bus diagnostics (`diag`) and selftest outputs are valid for connected and disconnected cases.
