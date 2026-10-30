# EE871-E2 Documentation

Use the maintained guides below for development and integration. Hardware
results live in one validation matrix, with exact firmware versions and limits.

## Maintained guides

| Document | Purpose |
| --- | --- |
| [Library README](../README.md) | Installation, API usage, lifecycle, timing budgets, and retry policy. |
| [Changelog](../CHANGELOG.md) | Release history and migration notes. |
| [Protocol and register map](EE871_E2_Protocol_and_Register_Map.md) | E2 electrical/timing requirements, transactions, and EE871 registers. |
| [ESP-IDF guide](IDF_PORT.md) | Native component integration, GPIO adapter, example setup, and build commands. |
| [HIL runner guide](EE871_E2_HIL_RUNNER.md) | Reproducible serial checks, soak/discriminator tools, and verdict rules. |
| [Validation matrix](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) | Software checks, retained hardware evidence, and untested scenarios. |
| [Contributing](../CONTRIBUTING.md) | Development and validation workflow. |

## Vendor references

The original PDFs are retained as the authority for device behavior:

- [E2 interface specification v4.1](E2_interface_specification_v4_1.pdf)
- [E2 interface application note AN0105](E2_interface_utilising_AN0105.pdf)
- [EE871 CO2 application note AN1611-1](EE871_E2_CO2_interface_AN1611-1.pdf)
- [EE871 E2 interface addendum](EE871_E2_interface_addendum.pdf)
- [EE871 digital interface user guide](EE871_digital_interface_user_guide.pdf)
- [EE871/EE240 wireless user guide](EE871_EE240_wireless_user_guide.pdf)

Searchable versions are kept alongside the originals:

- [Curated notes](extracted-md/README.md) summarize electrical requirements,
  commands, registers, modes, initialization, and variant differences.
- [Raw PDF extracts](pdf-extracted-md/README.md) preserve searchable source
  text. Extraction can disturb tables and notation; verify ambiguous details
  against the original PDF before changing driver behavior.

## Documentation maintenance

Keep current instructions in these guides and release changes in the changelog.
Consolidate hardware observations in the validation matrix rather than adding
another campaign report. Record exact source/build identity, measured results,
failures, and unrun scenarios; software tests are not hardware qualification.

Completed prompts and audit narratives are archived in Git history after their
usage contracts and verification records are incorporated into maintained
guides. The completed October work is available at commit `795f970`:
[implementation prompts and per-finding outcomes](https://github.com/janhavelka/EE871-E2/tree/795f9708754300c6c3f0782998291cfa095e8e40/docs/prompts)
and [retained-branch audit](https://github.com/janhavelka/EE871-E2/blob/795f9708754300c6c3f0782998291cfa095e8e40/docs/RETAINED_BRANCH_AUDIT.md).
These are historical records, not pending implementation instructions. Branch
retirement remains a separate maintenance decision.

Generated Doxygen output and local test captures are not maintained source
documents. Keep lasting usage and compatibility contracts in the README,
public headers and guides above.
