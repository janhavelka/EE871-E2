# EE871-E2 Hardening Prompt 05 — Hardware Validation Matrix, Final Integration Review, and Release/Merge Report

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty from unrelated user changes, stop and report it.

This prompt finalizes the hardening pass. It should not start large new implementation work unless a critical regression is found.

## Subagents to spawn

1. `final-diff-review-agent`
   - Review the full branch diff.
   - Look for broad refactors, accidental generated files, framework leakage, or docs overstating validation.

2. `hardware-validation-agent`
   - Create a realistic EE871-E2 hardware validation matrix and CLI sequence.
   - Do not claim hardware results unless commands are actually run.

3. `release-readiness-agent`
   - Decide what is merge-ready, release-ready, and still pending.
   - Identify public API changes and compatibility notes.

## Implement only this scope

### 1. Review the full diff

Run:

```bash
git diff --stat main...HEAD
git diff --check
git status --short
```

Inspect the actual diff. Confirm:

- no build artifacts;
- no accidental tarballs;
- no framework headers in core;
- no broad formatting churn;
- all tests are meaningful;
- docs match actual behavior.

### 2. Create hardware validation matrix

Create or update:

```text
docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md
```

Include concrete scenarios:

- power-up `begin()`;
- probe no-health-side-effects;
- read status;
- read CO2 fast/average values;
- PEC success on normal reads;
- persistent interval write/readback/power-cycle persistence;
- CO2 offset/gain write/readback, if safe to run;
- unplug/replug recovery;
- line stuck low/high if hardware jig exists;
- wrong wiring/no sensor;
- warm-up behavior;
- stale sample behavior, if observable;
- ESP32-S2 and ESP32-S3 boards separately.

Mark every item:

- `NOT RUN`;
- `PASS`;
- `FAIL`;
- `BLOCKED`;
- `NOT APPLICABLE`.

Default to `NOT RUN` unless actually run.

### 3. Add safe CLI/hardware test recipe

Add a concise hardware validation recipe to docs or README, for example:

```text
version
drv
probe
status
co2
co2avg
features
selftest
recover
```

Use the actual CLI command names in the repository. Do not invent commands. If commands do not exist, document the closest available commands or recommend adding them in future work.

Include warnings before persistent writes:

- persistent writes may change sensor configuration;
- they may have long delays;
- only run on a bench sensor where configuration changes are acceptable.

### 4. Finalize comprehensive hardening report

Complete:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

It must include:

- Date.
- Branch.
- Starting audit findings.
- Prompt-by-prompt change summary.
- Public API changes.
- Core changes.
- Test infrastructure changes.
- ESP-IDF/CI changes.
- Documentation changes.
- Exact commands run and exact results.
- Commands not run and why.
- Hardware validation status.
- Known remaining gaps.
- Future work needed for full industry-grade claim.
- Merge readiness verdict.
- Release readiness verdict.
- Compatibility/breaking-change notes.

Do not claim industry-grade unless all validation evidence exists.

Suggested verdict wording:

- “Ready to merge after CI passes” if tests/builds/docs are good but hardware validation is pending.
- “Not yet field/industry-grade until hardware fault validation and pure IDF CI pass.”
- “Production-oriented hardening complete; physical validation remains.”

### 5. Optional: package validation

Run package validation if project uses PlatformIO packaging:

```bash
python -m platformio pkg pack
```

Delete generated tarball after validation unless the project intentionally tracks it.

### 6. Final validation commands

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
python -m platformio pkg pack
git diff --check
git status --short
```

If `idf.py` is available:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Record exact results.

Delete generated package artifacts if appropriate, then re-run:

```bash
git status --short
```

### 7. Commit and sync

Commit with a focused message:

```bash
git add docs README.md .github tools test include src examples CMakeLists.txt idf_component.yml
git commit -m "docs: finalize EE871 industry hardening report"
git push
```

Only include files actually changed.

## Final response expected from the coding agent

Return a concise engineering summary:

1. Branch name.
2. Commits made.
3. Tests/checks run and exact results.
4. Hardware validation status.
5. Merge readiness.
6. Release readiness.
7. Remaining work for full industry-grade claim.
