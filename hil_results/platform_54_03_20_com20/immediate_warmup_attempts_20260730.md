# COM20 Immediate Warm-Up Capture Attempt Ledger

Date: 2026-07-30

This ledger classifies the retained immediate-warm-up evidence. Only
`immediate_warmup_final_20260730.txt` is the final clean F-08 evidence.

## `immediate_warmup_delayed_start_20260730.txt`

- COM20 was fully absent for 5.750 seconds.
- The capture waited 12.094 seconds for a startup prompt that was not emitted
  to that late-attaching native-USB session, so it missed the immediate
  warm-up window.
- Commands were aligned after that wait.
- One `co2fast` operation returned a bounded `NACK` 20.094 seconds after COM20
  reappeared. The immediately following `co2avg` and `status` succeeded, all
  later commands succeeded, and final health was READY with zero consecutive
  failures and one lifetime failure.
- Result: FAIL. Retained as delayed-start transport evidence, not F-08.

## Discarded Framing-Development Captures

Two development captures were rejected because command-to-response attribution
shifted around the native-USB startup/final prompt. They were used to diagnose
the CLI framing defect, then removed during transcript cleanup because neither
is valid hardware evidence. The resulting newline-terminated prompt and
drain-before-prompt behavior are covered by parser/contract tests and the
post-fix safe HIL run.

## Prompt-Framing Fix Verification

- Before the final run, the operator was instructed to remove power from both
  the ESP32-S3 and EE871 for at least five seconds and confirmed completion.
  The raw log independently records COM20 absent for 8.766 seconds, but the
  sensor power rail was not instrumented. All sample times are therefore
  relative to COM20 reappearance, not a measured sensor power-restoration edge.
- Automated safe HIL after the newline prompt change passed 10/10:
  `prompt_newline_postfix/ee871_20260730T152316Z/`.
- `immediate_warmup_final_20260730.txt` then captured an 8.766-second COM20
  disconnect, startup prompt at 0.250 seconds, and 33 correctly attributed
  warm-up commands.
- Values/status transitioned from `0 ppm`/`0x08` through four seconds to
  `678 ppm`/`0x00` at five seconds.
- Final state was READY with 65 transport successes, zero failures, clean
  persistent state, and interval `150 ds`.
- Result: PASS. This is the retained F-08 evidence.

The final PASS is evidence for bounded immediate warm-up behavior and correct
serial attribution. It does not erase the delayed-start bounded NACK, and it
does not validate CO2 accuracy or calibration accuracy.
