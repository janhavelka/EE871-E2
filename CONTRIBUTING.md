# Contributing

Thank you for considering contributing to this project!

## Quick Start

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes
4. Run the validation commands below.
5. Commit with a clear message: `git commit -m "feat: add X"`
6. Push and open a Pull Request

## Guidelines

### Code Style
- Follow existing code style (see `.clang-format`)
- Use `constexpr` instead of macros for constants
- Prefer explicit over implicit
- No heap allocations in steady-state library code

### Commits
- Use [Conventional Commits](https://www.conventionalcommits.org/) format:
  - `feat:` new feature
  - `fix:` bug fix
  - `docs:` documentation only
  - `refactor:` code change that neither fixes a bug nor adds a feature
  - `test:` adding or updating tests
  - `chore:` maintenance tasks

### Pull Requests
- Keep PRs focused (one feature/fix per PR)
- Update documentation if needed
- Add a changelog entry under the next unreleased version.
- Ensure CI passes

### Validation

```bash
python scripts/generate_version.py check
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python -m unittest discover -s test -p "test_*.py"
pio test -e native
pio run -e ex_bringup_s3
pio run -e ex_bringup_s2
pio run -e compat_tunnelmonitor_s3
```

On Windows, run PlatformIO through the portable wrapper `scripts\pio.cmd`
(for example `.\scripts\pio.cmd test -e native`) instead of a globally
installed `pio`.

Native ESP-IDF S2/S3 builds run in CI; see the
[ESP-IDF guide](docs/IDF_PORT.md) for local reproduction. Generate the public
documentation with `doxygen Doxyfile` after changing API or guide text.

Keep reusable instructions in the maintained [documentation](docs/README.md).
Consolidate completed hardware results in the validation matrix with exact
firmware identity and limitations. Finished prompts, implementation plans and
audit narratives belong in Git history once their useful content is incorporated.

### What We Accept
- Bug fixes
- Documentation improvements
- Performance improvements (with benchmarks)
- New examples (if they demonstrate a common use case)

### What We Probably Won't Accept
- Breaking API changes without discussion
- Heavy dependencies
- Platform-specific code in the library core
- Features that add heap allocations in steady state

## Questions?

Open a GitHub Discussion or Issue for questions.
