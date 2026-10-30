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

Keep the original wired-E2 sources as the authority for device behavior. The
protocol guide above is the single maintained summary; separate chapter notes
and generated PDF transcriptions are not maintained.

| Source | Purpose |
| --- | --- |
| [E2 interface specification v4.1](E2_interface_specification_v4_1.pdf) | Generic physical layer, framing, commands and custom memory. |
| [Application note AN0105](E2_interface_utilising_AN0105.pdf) | Vendor master examples and device-specific communication exceptions. |
| [CO2 application note AN1611-1](EE871_E2_CO2_interface_AN1611-1.pdf) | CO2 values, triggering, intervals, flash timing and power modes. |
| [EE871 E2 addendum](EE871_E2_interface_addendum.pdf) | EE871 identity, supported fields and electrical/timing requirements. |
| [EE871 digital interface user guide](EE871_digital_interface_user_guide.pdf) | Wired product variants, connector, supply and range. |

Use the product-specific E2 sources when they qualify the generic protocol.
Confirm the ordered probe variant; Modbus and EE240 wireless-system facts do
not define this driver's E2 contract. Search the PDFs directly and check the
original diagrams and tables when a value is ambiguous.

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

The redundant chapter notes, raw PDF extracts and wireless-system guide are
available in the [reference archive at `22d1299`](https://github.com/janhavelka/EE871-E2/tree/22d1299f039e512bd1f8230b5f95f51271124a48/docs).

Generated Doxygen output and local test captures are not maintained source
documents. Keep lasting usage and compatibility contracts in the README,
public headers and guides above.
