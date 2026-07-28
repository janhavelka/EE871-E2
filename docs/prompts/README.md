# EE871 General-Purpose Hardening and Firmware Integration Prompt Series

Date: 2026-07-28

This directory contains implementation prompts derived from
[`EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md`](../EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md).

The series deliberately separates:

- protocol and device behavior that belongs in the reusable `EE871-E2`
  library;
- product-independent E2 ownership infrastructure that belongs in a consuming
  firmware;
- selected-product schema, cadence, UI, persistence, and hardware policy.

The library must remain usable by TunnelMonitor-node, Co2Control, and unrelated
firmware without knowing about their tasks, queues, pins, schemas, or policies.

For the downstream `TunnelMonitor-node` repository, however, product ownership
is fixed: E2 and EE871 are enabled and used only by the compile-time
`Co2Control` product. The existing `TunnelMonitor` product and every other
production product must remain E2-disabled and contain no EE871 dependency,
pins, runtime, device row, data mapping, health row, or operator surface.

## Prompt Order

Run the prompts in order. Each prompt assumes the earlier prompt has been
implemented, reviewed, and left with a passing baseline.

| Order | Prompt | Repository | Purpose |
| --- | --- | --- | --- |
| 1 | [`01_ee871_protocol_timing_and_fault_precision.md`](01_ee871_protocol_timing_and_fault_precision.md) | `EE871-E2` | Fix write/pointer timing, START/STOP fault precision, configuration bounds, and documented blocking bounds. |
| 2 | [`02_ee871_lifecycle_identity_and_capabilities.md`](02_ee871_lifecycle_identity_and_capabilities.md) | `EE871-E2` | Add optional absent startup, latched OFFLINE behavior, full identity validation, and atomic capability caching. |
| 3 | [`03_ee871_checked_co2_samples.md`](03_ee871_checked_co2_samples.md) | `EE871-E2` | Add general checked MV3/MV4 CO2 sample procedures with correct ordering and sensor-domain errors. |
| 4 | [`04_ee871_persistent_maintenance_and_release.md`](04_ee871_persistent_maintenance_and_release.md) | `EE871-E2` | Complete persistent-write uncertainty, calibration guards, fake coverage, docs, and next-minor release preparation. |
| 5 | [`05_firmware_e2_product_and_electrical_decision.md`](05_firmware_e2_product_and_electrical_decision.md) | `TunnelMonitor-node` | Concretize Co2Control as the sole E2 product and resolve its pins, electrical interface, role, data, timing, dependency, and negative product-isolation authority. |
| 6 | [`06_firmware_generic_e2_owner_contracts.md`](06_firmware_generic_e2_owner_contracts.md) | `TunnelMonitor-node` | Add chip-neutral E2 contracts, owner type, backend boundary, diagnostics, and fake tests without selecting EE871 in a product. |
| 7 | [`07_firmware_ee871_module_and_gpio_backend.md`](07_firmware_ee871_module_and_gpio_backend.md) | `TunnelMonitor-node` | Exact-pin the release and add a thin EE871 module, ESP32 open-drain backend, and reusable worker wrapper without instantiating a product. |
| 8 | [`08_firmware_ee871_profile_data_and_qualification.md`](08_firmware_ee871_profile_data_and_qualification.md) | `TunnelMonitor-node` | Integrate EE871 only into Co2Control and prove every other production product remains E2/EE871-free. |

## Superseded Prompt

[`ee871_tunnelmonitor_fit_prompt_20260627.md`](ee871_tunnelmonitor_fit_prompt_20260627.md)
is retained as historical input, but it must not be executed after this series
was created. It correctly identified several API gaps, but it:

- omitted legal `0x10`/`0x50` long completion/stretch handling;
- omitted custom-pointer completion before `0x51`;
- omitted bounded configuration/WCET requirements;
- explicitly excluded the persistent uncertainty work now required;
- accepted too broad a class of `AllowAbsent` startup failures;
- used mixed-case enum values contrary to the current repository convention;
- combined reusable library work and downstream fit without a complete
  firmware-owner/profile implementation.

## Common Execution Rules

Every AI coder running one of these prompts must:

1. Start in `C:\Users\HonzovoSpectre\Documents\Projects`.
2. Enter only the repository named by that prompt.
3. Read that repository's `AGENTS.md` completely before editing.
4. Read the audit and the preceding prompt handoff/report.
5. Record branch, commit, version/dependency state, and baseline commands before
   implementation.
6. Preserve unrelated dirty work and never use destructive Git cleanup.
7. Prefer deleting duplication or extracting a small private helper over adding
   a forwarding shim, parallel state machine, or second source of truth.
8. Keep public enums append-only and preserve existing public API source
   compatibility unless the prompt explicitly says otherwise.
9. Use fixed-size state, static strings, and bounded loops. Do not add heap
   allocation to steady paths.
10. Run the prompt's validation commands and report failures honestly.
11. Do not claim ESP-IDF, HIL, waveform, long-run, or fault-injection evidence
    that was not actually produced.
12. Do not commit, tag, push, or modify a sibling repository unless the prompt
    explicitly authorizes it.

## Design Boundary

The final architecture is:

```text
product cadence / measurement / CLI / web
  -> fixed firmware command contracts and runtime facade
    -> one E2 runtime wrapper (queue, tracked results, worker)
      -> one E2Task owner + one GPIO backend
        -> selected-composition EE871 module/config/binding
          -> general synchronous EE871-E2 library
            -> injected open-drain GPIO callbacks
```

The EE871 library owns E2 signaling, framing, PEC, EE871 identity/capability
checks, checked sensor procedures, and truthful mutation evidence. The
firmware owns physical pins, level shifting, the owner task, queues, immutable
deadlines, retry cadence, warm-up/freshness, sample schema, storage/Cloud/UI,
authorization, and watchdog policy.

Only Co2Control instantiates the firmware side of this architecture in
production. Shared append-only contracts and explicit native/HIL test builds
may reference E2, but no other production product may compile or expose its
runtime/device surface.

## Release Gate

Do not exact-pin the library into production firmware until Prompts 1-4 are
complete. Prompt 04 intentionally stops at release-ready source: an authorized
release operation must separately commit/tag/push the exact version. Prompt 05
then verifies that immutable tag and commit before downstream exact-pinning.

Do not execute runtime integration from Prompts 6-8 until Prompt 5's
Co2Control electrical, schema, environment, and maintenance preconditions and
the non-Co2Control exclusion matrix are explicitly resolved.
