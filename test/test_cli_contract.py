from __future__ import annotations

import contextlib
import io
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import check_cli_contract as arduino
import check_idf_example_contract as idf


class CliContractTests(unittest.TestCase):
    def check_mutation(self, old: str, new: str, expected: str) -> None:
        for checker, relative in (
            (arduino, "examples/common/E2Diagnostics.h"),
            (idf, "examples/idf/basic_bringup/main/main.cpp"),
        ):
            with self.subTest(checker=checker.__name__):
                target = ROOT / relative
                original_read = pathlib.Path.read_text
                source = target.read_text(encoding="utf-8")
                self.assertIn(old, source)
                mutated = source.replace(old, new, 1)

                def read(path: pathlib.Path, *args, **kwargs) -> str:
                    if path == target:
                        return mutated
                    return original_read(path, *args, **kwargs)

                output = io.StringIO()
                with mock.patch.object(pathlib.Path, "read_text", read), contextlib.redirect_stdout(output):
                    with self.assertRaises(SystemExit) as raised:
                        checker.main()
                self.assertEqual(1, raised.exception.code)
                self.assertIn(expected, output.getvalue())

    def test_current_contracts_pass(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(0, arduino.main())
            self.assertEqual(0, idf.main())

    def test_scanner_cannot_demote_incompatible_identity(self) -> None:
        self.check_mutation(
            "lastError[addr].code != EE871::Err::NOT_SUPPORTED",
            "true", "never demote NOT_SUPPORTED",
        )

    def test_scanner_error_retention_native(self) -> None:
        # Compile the actual decision block from each example so the runtime
        # regression tests the shipped condition, not a copied implementation.
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "Native scanner regression requires g++")
        functions = []
        for name, relative in (
            ("arduino", "examples/common/E2Diagnostics.h"),
            ("idf", "examples/idf/basic_bringup/main/main.cpp"),
        ):
            source = arduino.strip_cpp_non_code((ROOT / relative).read_text(encoding="utf-8"))
            scanner = arduino.extract_section(source, "void scanAddresses", "struct TimingResult")
            block = re.search(r"if\s*\(\s*lastError\[addr\].*?lastError\[addr\]\s*=\s*st;\s*\}", scanner, re.S)
            self.assertIsNotNone(block)
            functions.append(
                f"EE871::Status {name}(EE871::Status previous, EE871::Status st) {{\n"
                "EE871::Status lastError[1] = {previous}; const int addr = 0;\n"
                + block.group() + "\nreturn lastError[0];\n}"
            )
        source = '#include "EE871/Status.h"\n#include <cassert>\n' + "\n".join(functions)
        source += '''
int main() {
  using EE871::Err;
  using EE871::Status;
  const auto incompatible = Status::Error(Err::NOT_SUPPORTED, "wrong subgroup", 8);
  const auto nack = Status::Error(Err::NACK, "absent");
  const Err laterErrors[] = {Err::BUS_STUCK, Err::TIMEOUT, Err::PEC_MISMATCH, Err::NACK};
  using Retain = Status (*)(Status, Status);
  const Retain implementations[] = {arduino, idf};
  for (Retain retain : implementations) {
    for (Err code : laterErrors) {
      auto result = retain(incompatible, Status::Error(code, "later fault", 99));
      assert(result.code == Err::NOT_SUPPORTED && result.detail == 8);
      assert(result.msg == incompatible.msg);
    }
    assert(retain(Status::Ok(), nack).code == Err::NACK);
    assert(retain(nack, incompatible).code == Err::NOT_SUPPORTED);
    const auto timeout = Status::Error(Err::TIMEOUT, "stretch");
    assert(retain(nack, timeout).code == Err::TIMEOUT);
    assert(retain(timeout, nack).code == Err::TIMEOUT);
    assert(retain(nack, Status::Ok()).ok());
  }
}
'''
        with tempfile.TemporaryDirectory(prefix="ee871-scanner-") as directory:
            cpp = pathlib.Path(directory) / "scanner.cpp"
            executable = pathlib.Path(directory) / "scanner.exe"
            cpp.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++17", "-I", str(ROOT / "include"), str(cpp), "-o", str(executable)],
                capture_output=True, text=True, timeout=60,
            )
            self.assertEqual(0, build.returncode, build.stdout + build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
            self.assertEqual(0, run.returncode, run.stdout + run.stderr)

    def test_scanner_cannot_count_ack_only_responses(self) -> None:
        for increment in ("found ++;", "++ found;"):
            self.check_mutation("candidate.end();", f"candidate.end(); {increment}", "ACK-only")

    def test_libtest_requires_production_read(self) -> None:
        self.check_mutation("driver.readControlByte(", "readByteRaw(", "production control-byte path")

    def test_libtest_rejects_raw_traffic_alongside_production_read(self) -> None:
        self.check_mutation(
            "driver.readControlByte(tests[i].mainCmd, data);",
            "driver.readControlByte(tests[i].mainCmd, data); sendStart(cfg);",
            "raw diagnostic call",
        )

    def test_libtest_explains_health_effects(self) -> None:
        self.check_mutation(
            "Reads are tracked; OFFLINE returns the latched status without bus traffic.",
            "", "explain tracked reads",
        )

    def test_timing_rejects_old_candidates_and_commented_correct_list(self) -> None:
        self.check_mutation(
            "{995, 500, 250, 200, 150, 100}",
            "{1000, 500, 250, 200, 150, 100, 75, 50}; // timings[] = {995, 500, 250, 200, 150, 100}",
            "six in-spec candidates",
        )

    def test_timing_requires_data_setup_in_frequency(self) -> None:
        self.check_mutation(
            "1000000.0f / (10.0f + 2.0f * clockUs)",
            "1000000.0f / (2.0f * clockUs)", "10 us data setup",
        )


if __name__ == "__main__":
    unittest.main()
