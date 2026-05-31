# EE871-E2 Hardening Prompt 04 — ESP-IDF Build Coverage, CI, and Example Contract Honesty

Continue on the existing branch:

```bash
git checkout hardening/ee871-e2-industry-readiness
git pull --ff-only
git status --short
```

If the worktree is dirty, stop and report it.

This prompt addresses the IDF validation gap. Do not change the core architecture unless a build failure requires a small fix.

## Important context

The audit found IDF artifacts and a native IDF example, but no local or CI proof that pure `idf.py` builds work. PlatformIO Arduino builds are not a substitute for pure ESP-IDF component/example validation.

## Subagents to spawn

1. `idf-build-agent`
   - Inspect root `CMakeLists.txt`, `idf_component.yml`, `examples/idf/basic_bringup`, and any IDF-specific docs.
   - Try to determine what `idf.py` commands should work.

2. `ci-agent`
   - Inspect `.github/workflows/ci.yml`.
   - Add pure ESP-IDF build coverage if feasible.

3. `idf-example-review-agent`
   - Confirm the IDF example remains native IDF.
   - Check GPIO ownership, timing callbacks, nonblocking loop/tick, and diagnostic-vs-production wording.

## Implement only this scope

### 1. Add pure ESP-IDF CI coverage

Update CI to run pure ESP-IDF builds for:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Use an official or common ESP-IDF CI container/action. Keep it simple and maintainable.

Make sure the CI job also runs:

```bash
python tools/check_idf_example_contract.py
```

### 2. Verify IDF component metadata

Check:

- root `CMakeLists.txt`;
- `idf_component.yml`;
- example `CMakeLists.txt`;
- include paths;
- dependency declarations.

Fix only obvious build/config mistakes.

### 3. Preserve framework boundary

Ensure IDF files do not leak into `include/` or `src/`.

Run or update guard scripts if needed, but avoid broad tool rewrites.

### 4. Improve IDF example honesty

Update README/docs to say:

- The IDF example is a diagnostic/basic bring-up example.
- It owns GPIO line setup for demonstration.
- Production users should integrate line callbacks into their application’s GPIO/bus manager.
- External serialization is required if multiple tasks can access the same EE871 instance.
- The device uses E2 GPIO signaling, not ESP-IDF hardware I2C.

### 5. Local IDF build attempt

Attempt:

```bash
idf.py --version
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

If `idf.py` is unavailable, record the exact error. Do not claim local IDF build success.

### 6. Update final report

Append Prompt 04 section to:

```text
docs/EE871_E2_HARDENING_FINAL_REPORT.md
```

Include:

- CI changes.
- Whether local `idf.py` was available.
- Exact local build/CI-relevant commands run.
- Remaining IDF validation gaps.

## Required validation

Run:

```bash
python tools/check_core_timing_guard.py
python tools/check_idf_example_contract.py
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
idf.py --version
git diff --check
```

If `idf.py` exists, run the two `idf.py -C ... build` commands above.

## Commit and sync

Commit with a focused message:

```bash
git add .github CMakeLists.txt idf_component.yml examples docs README.md tools
git commit -m "ci: add EE871 pure ESP-IDF build coverage"
git push
```

Only include files actually changed.
