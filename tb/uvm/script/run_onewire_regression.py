#!/usr/bin/env python3
"""Run the implemented OneWire controller named-case slice.

The bucket case files remain the scoreboard. This script only records execution
evidence for cases that have live harness support.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
UVM_DIR = SCRIPT_DIR.parent
TB_DIR = UVM_DIR.parent
REPORT_DIR = TB_DIR / "REPORT"

IMPLEMENTED_CASES = [
    "B003",
    "B004",
    "B005",
    "B006",
    "B007",
    "B008",
    "B009",
    "B010",
    "B011",
    "B012",
    "B013",
    "B014",
    "B015",
    "B016",
    "B017",
    "B018",
    "B019",
    "B020",
    "B021",
    "B022",
    "B023",
    "B024",
    "B025",
    "B026",
    "B027",
    "B028",
    "B029",
    "B030",
    "B031",
    "B032",
    "B033",
    "B121",
    "B122",
    "B123",
    "B124",
    "B125",
    "B131",
    "B001",
    "B002",
    "X001",
    "X002",
    "X003",
    "X004",
    "X005",
    "X006",
    "X007",
    "X008",
    "X009",
    "X010",
    "X011",
    "X012",
    "X121",
    "X122",
    "X123",
    "X124",
    "X125",
]

CASE_FEATURES = {
    "B001": [
        "BASIC.zero_temperature_guard",
        "BASIC.sample_valid_status",
        "BASIC.temperature_float32_readback",
    ],
    "B002": [
        "BASIC.six_line_temperature_loop",
        "BASIC.per_line_status_select",
    ],
    "B003": [
        "BASIC.common_uid_meta_header",
        "BASIC.csr_scratchpad",
        "BASIC.capability_readback",
    ],
    "B004": ["BASIC.master_csr_descriptor_range"],
    "B005": ["BASIC.master_csr_descriptor_range"],
    "B006": ["BASIC.master_csr_descriptor_range"],
    "B007": ["BASIC.master_csr_descriptor_range"],
    "B008": ["BASIC.master_csr_descriptor_range"],
    "B009": ["BASIC.master_commit_busy_irq"],
    "B010": ["BASIC.master_commit_busy_irq"],
    "B011": ["BASIC.master_csr_descriptor_range"],
    "B012": ["BASIC.master_csr_descriptor_range"],
    "B013": ["BASIC.master_csr_descriptor_range"],
    "B014": ["BASIC.master_csr_descriptor_range"],
    "B015": ["BASIC.master_csr_descriptor_range"],
    "B016": ["BASIC.master_csr_descriptor_range"],
    "B017": ["BASIC.master_csr_descriptor_range"],
    "B018": ["BASIC.master_csr_descriptor_range"],
    "B019": ["BASIC.master_csr_descriptor_range"],
    "B020": ["BASIC.master_commit_busy_irq"],
    "B021": ["BASIC.master_csr_descriptor_range"],
    "B022": ["BASIC.master_csr_descriptor_range"],
    "B023": ["BASIC.master_commit_busy_irq"],
    "B024": ["BASIC.master_commit_busy_irq"],
    "B025": ["BASIC.master_tx_engine_fsm"],
    "B026": ["BASIC.master_tx_engine_fsm"],
    "B027": ["BASIC.master_tx_engine_fsm"],
    "B028": ["BASIC.master_tx_engine_fsm"],
    "B029": ["BASIC.master_tx_engine_fsm"],
    "B030": ["BASIC.master_tx_engine_fsm"],
    "B031": ["BASIC.master_tx_engine_fsm"],
    "B032": ["BASIC.master_tx_engine_fsm"],
    "B033": ["BASIC.master_tx_engine_fsm"],
    "B121": ["BASIC.capability_readback"],
    "B122": ["BASIC.per_line_status_select"],
    "B123": ["BASIC.processor_go_per_line"],
    "B124": [
        "BASIC.temperature_read_zero_until_valid",
        "BASIC.temperature_float32_readback",
    ],
    "B125": ["BASIC.controller_init_err_readback"],
    "B131": ["BASIC.microsecond_divider_odd_even"],
    "X001": ["ERROR.master_csr_illegal_access"],
    "X002": ["ERROR.master_csr_illegal_access"],
    "X003": ["ERROR.master_csr_illegal_access"],
    "X004": ["ERROR.master_csr_illegal_access"],
    "X005": ["ERROR.master_csr_illegal_access"],
    "X006": ["ERROR.master_csr_illegal_access"],
    "X007": ["ERROR.master_csr_illegal_access"],
    "X008": ["ERROR.master_csr_illegal_access"],
    "X009": ["ERROR.master_csr_illegal_access"],
    "X010": ["ERROR.master_csr_illegal_access"],
    "X011": ["ERROR.master_csr_illegal_access"],
    "X012": ["ERROR.master_csr_illegal_access"],
    "X121": ["ERROR.read_only_csr_noop"],
    "X122": ["ERROR.temperature_write_noop"],
    "X123": ["ERROR.invalid_sel_line_status_zero"],
    "X124": ["ERROR.invalid_sel_line_go_suppressed"],
    "X125": ["ERROR.unmapped_temperature_slot_zero"],
}

def short_case_id(case_id: str) -> str:
    return case_id.strip().upper()


def run_cmd(cmd: list[str], log_path: Path) -> subprocess.CompletedProcess[str]:
    proc = subprocess.run(cmd, cwd=UVM_DIR, text=True, capture_output=True)
    log_path.write_text(proc.stdout + proc.stderr, encoding="utf-8")
    return proc


def parse_features(log_text: str) -> list[str]:
    features: list[str] = []
    for line in log_text.splitlines():
        line = line.lstrip("# ").strip()
        if line.startswith("OW_FEATURE_PASS "):
            features.append(line.split(" ", 1)[1].strip())
    return sorted(set(features))


def write_case_report(case_id: str, proc: subprocess.CompletedProcess[str], log_path: Path) -> dict[str, object]:
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    divider_case = case_id == "B131"
    pass_token = "PSEUDO_CLOCK_DOWN_CONVERTOR_PASS" if divider_case else f"OW_CASE_PASS {case_id}"
    passed = proc.returncode == 0 and pass_token in log_text
    features = parse_features(log_text)
    if passed and not features:
        features = CASE_FEATURES.get(case_id, [])
    report_path = REPORT_DIR / f"{case_id}.md"
    status = "PASS" if passed else "FAIL"
    command = "make run_divider" if divider_case else f"make run CASE={case_id} MODE=isolated"
    report_path.write_text(
        "\n".join(
            [
                f"# {case_id}",
                "",
                f"| field | value |",
                f"|---|---|",
                f"| status | {status} |",
                f"| command | `{command}` |",
                f"| log | `{log_path.relative_to(TB_DIR)}` |",
                f"| features covered | {', '.join(features) if features else 'none'} |",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return {
        "case_id": case_id,
        "status": status,
        "returncode": proc.returncode,
        "features": features,
        "log": str(log_path.relative_to(TB_DIR)),
        "report": str(report_path.relative_to(TB_DIR)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-compile", action="store_true", help="Assume make compile has already completed.")
    parser.add_argument(
        "--cases",
        nargs="*",
        default=IMPLEMENTED_CASES,
        help="Subset of case IDs to run, for example B001 or X121.",
    )
    args = parser.parse_args()
    cases = [short_case_id(case_id) for case_id in args.cases]

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    run_dir = REPORT_DIR / ("run_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    run_dir.mkdir()

    if not args.skip_compile:
        compile_log = run_dir / "compile.log"
        compile_proc = run_cmd(["make", "compile"], compile_log)
        if compile_proc.returncode != 0:
            print(f"compile failed; see {compile_log}", file=sys.stderr)
            return compile_proc.returncode

    results = []
    for case_id in cases:
        if case_id not in CASE_FEATURES:
            print(f"unknown or unimplemented case: {case_id}", file=sys.stderr)
            return 2
        log_path = run_dir / f"{case_id}.log"
        if case_id == "B131":
            proc = run_cmd(["make", "run_divider"], log_path)
        else:
            proc = run_cmd(["make", "run", f"CASE={case_id}", "MODE=isolated"], log_path)
        result = write_case_report(case_id, proc, log_path)
        results.append(result)
        print(f"{case_id}: {result['status']}")
        if result["status"] != "PASS":
            break

    summary = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "implemented_cases": IMPLEMENTED_CASES,
        "results": results,
        "covered_features": sorted({feature for result in results for feature in result["features"]}),
    }
    (REPORT_DIR / "onewire_regression_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    return 0 if all(result["status"] == "PASS" for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
