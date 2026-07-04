#!/usr/bin/env python3
"""
Prepare, run, and summarize a Quartus out-of-context timing experiment.

Typical use:
    python3 eval_ooc/tools/quartus_ooc.py prepare eval_ooc/inst_pipeline/config.json
    python3 eval_ooc/tools/quartus_ooc.py run     eval_ooc/inst_pipeline/config.json
    python3 eval_ooc/tools/quartus_ooc.py sta     eval_ooc/inst_pipeline/config.json
    python3 eval_ooc/tools/quartus_ooc.py summarize eval_ooc/inst_pipeline/config.json
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import gen_quartus_ooc


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def find_repo_root(config_path: Path, config: dict[str, Any]) -> Path:
    if "repo_root" in config:
        root = Path(config["repo_root"])
        if not root.is_absolute():
            root = config_path.parent / root
        return root.resolve()

    for path in [config_path.parent.resolve(), *config_path.parent.resolve().parents]:
        if (path / ".git").exists():
            return path
    return Path.cwd().resolve()


def config_run_dir(config_path: Path, config: dict[str, Any]) -> Path:
    q = config["quartus"]
    top = config["top_module"]
    device = re.sub(r"[^A-Za-z0-9_]", "_", q["device"]).lower()
    clock_mhz = int(float(config.get("clock_mhz", 100.0)))
    run_name = q.get("run_name", f"quartus_{device}_{clock_mhz}")
    raw = config.get("run_dir", f"runs/{run_name}")
    path = Path(raw)
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()


def critical_path_count(config: dict[str, Any], override: int | None) -> int:
    if override is not None:
        return override
    return int(config.get("critical_paths", 20))


def prepare(config_path: Path, critical_paths: int | None = None) -> Path:
    return gen_quartus_ooc.generate(config_path, critical_paths)


def docker_run(config_path: Path, run_dir: Path) -> None:
    config = load_json(config_path)
    repo_root = find_repo_root(config_path, config)
    image = config.get("quartus", {}).get("docker_image", "raetro/quartus:17.0")

    try:
        run_rel = run_dir.resolve().relative_to(repo_root)
    except ValueError as exc:
        raise SystemExit(f"run directory {run_dir} is not under repo root {repo_root}") from exc

    command = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{repo_root}:/work",
        "-w",
        f"/work/{run_rel.as_posix()}",
        image,
        "bash",
        "./run_quartus.sh",
    ]
    subprocess.run(command, check=True)


def docker_sta(config_path: Path, run_dir: Path) -> None:
    config = load_json(config_path)
    repo_root = find_repo_root(config_path, config)
    image = config.get("quartus", {}).get("docker_image", "raetro/quartus:17.0")

    try:
        run_rel = run_dir.resolve().relative_to(repo_root)
    except ValueError as exc:
        raise SystemExit(f"run directory {run_dir} is not under repo root {repo_root}") from exc

    command = [
        "docker",
        "run",
        "--rm",
        "-v",
        f"{repo_root}:/work",
        "-w",
        f"/work/{run_rel.as_posix()}",
        image,
        "bash",
        "./run_sta.sh",
    ]
    subprocess.run(command, check=True)


def number(value: str) -> float:
    return float(value.replace(",", ""))


def int_number(value: str) -> int:
    return int(value.replace(",", ""))


def parse_resource(text: str, labels: list[str]) -> int | None:
    for label in labels:
        pattern = rf"{re.escape(label)}\s*(?:;|:)?\s*([0-9][0-9,]*)"
        match = re.search(pattern, text, flags=re.I)
        if match:
            return int_number(match.group(1))
    return None


def collect_reports(run_dir: Path) -> list[Path]:
    roots = [run_dir / "reports", run_dir / "output_files"]
    reports: list[Path] = []
    for root in roots:
        if root.exists():
            reports.extend(sorted(root.glob("*.rpt")))
    return reports


def collect_logs(run_dir: Path) -> list[Path]:
    root = run_dir / "logs"
    if not root.exists():
        return []
    return sorted(root.glob("*.log"))


def extract_messages(text: str, pattern: str, limit: int = 20) -> list[str]:
    messages: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if re.search(pattern, stripped):
            messages.append(stripped)
            if len(messages) >= limit:
                break
    return messages


def extract_errors(text: str, limit: int = 20) -> list[str]:
    return extract_messages(text, r"\bError\b", limit)


def extract_warnings(text: str, limit: int = 20) -> list[str]:
    return extract_messages(text, r"\b(Critical Warning|Warning)\b", limit)


def extract_slacks(text: str) -> list[float]:
    patterns = [
        r"\bSlack\s*[:=]\s*(-?[0-9][0-9,]*(?:\.[0-9]+)?)\s*ns?",
        r"\bslack\s+is\s+(-?[0-9][0-9,]*(?:\.[0-9]+)?)",
        r"\bWorst-case\s+setup\s+slack\s*(?:is|:)?\s*(-?[0-9][0-9,]*(?:\.[0-9]+)?)",
    ]
    values: list[float] = []
    for pattern in patterns:
        for match in re.finditer(pattern, text, flags=re.I):
            values.append(number(match.group(1)))
    return values


def extract_fmax(text: str) -> list[dict[str, Any]]:
    """Parse the STA "Fmax Summary" panel(s) - the real core Fmax.

    Each corner model emits a panel:
        ;        Slow 1100mV 100C Model Fmax Summary        ;
        ; Fmax       ; Restricted Fmax ; Clock Name ; Note  ;
        ; 108.41 MHz ; 108.41 MHz      ; i_CLK      ;       ;
    Only rows inside such a panel are collected, each tagged with its corner
    model. This avoids matching unrelated "MHz" lines such as the Active
    Serial "100 MHz Internal Oscillator" configuration note.
    """
    values: list[dict[str, Any]] = []
    title_re = re.compile(r";\s*(.*?)\bFmax Summary\b", flags=re.I)
    row_re = re.compile(
        r"^\s*;\s*([0-9][0-9,]*(?:\.[0-9]+)?)\s*MHz\s*;"   # Fmax
        r"\s*([0-9][0-9,]*(?:\.[0-9]+)?)\s*MHz\s*;"        # Restricted Fmax
        r"\s*([^;]*?)\s*;",                                # Clock Name
        flags=re.I,
    )
    in_panel = False
    model = None
    for line in text.splitlines():
        title = title_re.search(line)
        if title:
            model = title.group(1).strip() or None
            in_panel = True
            continue
        if not in_panel:
            continue
        row = row_re.match(line)
        if row:
            values.append({
                "mhz": number(row.group(1)),
                "restricted_mhz": number(row.group(2)),
                "clock": row.group(3).strip(),
                "model": model,
            })
        elif line.lstrip().lower().startswith("this panel reports fmax"):
            in_panel = False
    return values


def extract_worst_path(text: str) -> dict[str, Any] | None:
    lines = text.splitlines()
    best_idx = None
    best_slack = None
    slack_re = re.compile(r"\bSlack\s*[:=]\s*(-?[0-9][0-9,]*(?:\.[0-9]+)?)", re.I)
    for idx, line in enumerate(lines):
        match = slack_re.search(line)
        if not match:
            continue
        value = number(match.group(1))
        if best_slack is None or value < best_slack:
            best_slack = value
            best_idx = idx

    if best_idx is None or best_slack is None:
        return None

    start = max(0, best_idx - 16)
    end = min(len(lines), best_idx + 28)
    window = lines[start:end]
    path = {"slack_ns": best_slack, "excerpt": [line.rstrip() for line in window if line.strip()]}
    for key in ("From Node", "To Node", "Launch Clock", "Latch Clock", "Data Arrival Path"):
        regex = re.compile(rf"{re.escape(key)}\s*[:=]\s*(.*)", re.I)
        for line in window:
            match = regex.search(line)
            if match:
                path[key.lower().replace(" ", "_")] = match.group(1).strip()
                break
    return path


def split_timing_path_blocks(text: str) -> list[list[str]]:
    lines = text.splitlines()
    path_indexes = [idx for idx, line in enumerate(lines) if re.match(r"Path #\d+:", line.strip())]
    if path_indexes:
        blocks: list[list[str]] = []
        for pos, start in enumerate(path_indexes):
            end = path_indexes[pos + 1] if pos + 1 < len(path_indexes) else len(lines)
            blocks.append(lines[start:end])
        return blocks

    slack_indexes = [
        idx
        for idx, line in enumerate(lines)
        if re.search(r"\b(?:Slack\s*[:=]|setup\s+slack\s+is)\s*-?[0-9][0-9,]*(?:\.[0-9]+)?", line, flags=re.I)
    ]
    blocks = []
    for pos, start_idx in enumerate(slack_indexes):
        start = start_idx
        while start > 0:
            prev = lines[start - 1].strip()
            if not prev:
                break
            if re.match(r"^[=\-+; ]+$", prev):
                break
            if re.search(r"\bSlack\s*[:=]", prev, flags=re.I):
                break
            start -= 1

        end = slack_indexes[pos + 1] if pos + 1 < len(slack_indexes) else len(lines)
        blocks.append(lines[start:end])
    return blocks


def extract_table_nodes(block: list[str]) -> list[str]:
    nodes: list[str] = []
    in_data_path = False
    after_data_marker = False
    for line in block:
        stripped = line.strip()
        if re.match(r";\s*Data\s+Arrival\s+Path\s*;", stripped, flags=re.I):
            in_data_path = True
            continue
        if in_data_path and re.search(r"Data\s+Required\s+Path|Slack", stripped, flags=re.I):
            break
        if not in_data_path:
            continue
        if not stripped.startswith(";"):
            continue
        cells = [cell.strip() for cell in stripped.strip(";").split(";")]
        cells = [cell for cell in cells if cell]
        if not cells:
            continue
        if any(re.search(r"Total|Incr|Type|Element|Fanout|Location", cell, flags=re.I) for cell in cells):
            continue
        node = cells[-1]
        if re.fullmatch(r"(launch edge time|clock path|source latency|data path|latch edge time|clock pessimism removed|clock uncertainty)", node, flags=re.I):
            if re.fullmatch(r"data path", node, flags=re.I):
                after_data_marker = True
            continue
        if not after_data_marker:
            continue
        if node and node not in nodes:
            nodes.append(node)

    if nodes:
        return nodes

    for line in block:
        stripped = line.strip()
        if "|" not in stripped:
            continue
        for token in re.findall(r"[A-Za-z_][A-Za-z0-9_$:[\].~|/\\-]*", stripped):
            if "|" in token and token not in nodes:
                nodes.append(token)
    return nodes


def extract_path_property(block: list[str], key: str) -> str | None:
    for line in block:
        stripped = line.strip()
        if not stripped.startswith(";"):
            continue
        cells = [cell.strip() for cell in stripped.strip(";").split(";")]
        cells = [cell for cell in cells if cell]
        if len(cells) >= 2 and cells[0].lower() == key.lower():
            return cells[1]

    joined = "\n".join(block)
    regex = re.compile(rf"{re.escape(key)}\s*[:=]\s*(.*)", re.I)
    match = regex.search(joined)
    if match:
        return match.group(1).strip()
    return None


def extract_critical_paths(text: str, limit: int) -> list[dict[str, Any]]:
    paths: list[dict[str, Any]] = []
    for block in split_timing_path_blocks(text):
        joined = "\n".join(block)
        slack_match = re.search(
            r"\b(?:Slack\s*[:=]|setup\s+slack\s+is)\s*(-?[0-9][0-9,]*(?:\.[0-9]+)?)",
            joined,
            flags=re.I,
        )
        if not slack_match:
            continue
        index_match = re.search(r"Path #(\d+):", joined)
        path: dict[str, Any] = {
            "slack_ns": number(slack_match.group(1)),
            "excerpt": [line.rstrip() for line in block if line.strip()],
        }
        if index_match:
            path["path_index"] = int(index_match.group(1))
        for key in ("From Node", "To Node", "Launch Clock", "Latch Clock"):
            value = extract_path_property(block, key)
            if value:
                path[key.lower().replace(" ", "_")] = value
        for key in ("Data Arrival Time", "Data Required Time"):
            value = extract_path_property(block, key)
            if value and re.match(r"-?[0-9][0-9,]*(?:\.[0-9]+)?$", value):
                path[key.lower().replace(" ", "_") + "_ns"] = number(value)
        nodes = extract_table_nodes(block)
        if nodes:
            path["nodes"] = nodes
        paths.append(path)

    paths.sort(key=lambda item: item["slack_ns"])
    return paths[:limit]


def critical_path_report(run_dir: Path) -> Path | None:
    candidates = [
        run_dir / "reports" / "critical_paths.rpt",
        run_dir / "output_files" / "critical_paths.rpt",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def summarize(config_path: Path, run_dir: Path, critical_paths: int | None = None) -> dict[str, Any]:
    config = load_json(config_path)
    critical_count = critical_path_count(config, critical_paths)
    reports = collect_reports(run_dir)
    logs = collect_logs(run_dir)
    critical_report = critical_path_report(run_dir)
    summary: dict[str, Any] = {
        "top_module": config["top_module"],
        "clock_mhz": float(config.get("clock_mhz", 100.0)),
        "device": config["quartus"]["device"],
        "critical_paths_requested": critical_count,
        "run_dir": str(run_dir),
        "reports": [str(path.relative_to(run_dir)) for path in reports],
        "logs": [str(path.relative_to(run_dir)) for path in logs],
        "timing": {},
        "resources": {},
    }

    all_slacks: list[float] = []
    all_fmax: list[dict[str, Any]] = []
    worst_path: dict[str, Any] | None = None
    resource_text = ""
    errors: list[str] = []
    warnings: list[str] = []

    for report in reports:
        text = report.read_text(encoding="utf-8", errors="ignore")
        if report.suffix == ".rpt":
            all_slacks.extend(extract_slacks(text))
            all_fmax.extend(extract_fmax(text))
            candidate = extract_worst_path(text)
            if candidate and (worst_path is None or candidate["slack_ns"] < worst_path["slack_ns"]):
                worst_path = candidate | {"report": str(report.relative_to(run_dir))}
        if ".fit." in report.name or ".map." in report.name:
            resource_text += "\n" + text

    for log in logs:
        text = log.read_text(encoding="utf-8", errors="ignore")
        for error in extract_errors(text):
            if error not in errors:
                errors.append(error)
        for warning in extract_warnings(text):
            if warning not in warnings:
                warnings.append(warning)
        if len(errors) >= 20:
            errors = errors[:20]
            break

    if errors:
        summary["status"] = "failed"
        summary["errors"] = errors
    elif warnings:
        summary["status"] = "complete_with_warnings"
        summary["warnings"] = warnings[:20]
    else:
        summary["status"] = "complete" if reports else "not_run"
    if all_slacks:
        summary["timing"]["worst_slack_ns"] = min(all_slacks)
    if all_fmax:
        # Dedup: the same Fmax panel can appear in more than one parsed report.
        seen: set[tuple] = set()
        unique_fmax: list[dict[str, Any]] = []
        for f in all_fmax:
            key = (f.get("clock"), f.get("model"), f.get("mhz"), f.get("restricted_mhz"))
            if key not in seen:
                seen.add(key)
                unique_fmax.append(f)
        summary["timing"]["fmax_candidates"] = unique_fmax[:20]
        # Sign-off Fmax = worst (lowest) restricted Fmax across all corners/clocks.
        restricted = [f["restricted_mhz"] for f in unique_fmax if f.get("restricted_mhz") is not None]
        if restricted:
            summary["timing"]["fmax_mhz"] = min(restricted)
    if worst_path:
        summary["timing"]["worst_path"] = worst_path
    if critical_report:
        text = critical_report.read_text(encoding="utf-8", errors="ignore")
        paths = extract_critical_paths(text, critical_count)
        if paths:
            for path in paths:
                path["report"] = str(critical_report.relative_to(run_dir))
            summary["timing"]["critical_paths"] = paths

    resources = {
        "logic_elements": parse_resource(resource_text, ["Total logic elements", "Combinational ALUTs"]),
        "registers": parse_resource(resource_text, ["Total registers", "Dedicated logic registers"]),
        "pins": parse_resource(resource_text, ["Total pins"]),
        "memory_bits": parse_resource(resource_text, ["Total block memory bits", "Total memory bits"]),
        "dsp_blocks": parse_resource(resource_text, ["DSP block 18-bit elements", "Embedded Multiplier 9-bit elements"]),
        "plls": parse_resource(resource_text, ["Total PLLs"]),
    }
    summary["resources"] = {key: value for key, value in resources.items() if value is not None}

    out = run_dir / "summary.json"
    out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def print_summary(summary: dict[str, Any]) -> None:
    print(f"top: {summary['top_module']}")
    print(f"device: {summary['device']}")
    print(f"target clock: {summary['clock_mhz']} MHz")
    print(f"status: {summary.get('status', 'unknown')}")
    if summary.get("errors"):
        print("first errors:")
        for line in summary["errors"][:8]:
            print(f"  {line}")
    elif summary.get("warnings"):
        print("first warnings:")
        for line in summary["warnings"][:8]:
            print(f"  {line}")
    timing = summary.get("timing", {})
    if "worst_slack_ns" in timing:
        print(f"worst slack: {timing['worst_slack_ns']} ns")
    else:
        print("worst slack: unavailable")
    if timing.get("fmax_mhz") is not None:
        print(f"fmax (restricted, worst corner): {timing['fmax_mhz']} MHz")
    if timing.get("fmax_candidates"):
        print("fmax by corner:")
        for item in timing["fmax_candidates"][:5]:
            mhz = item.get("mhz")
            restricted = item.get("restricted_mhz")
            tag = f", restricted {restricted} MHz" if restricted is not None and restricted != mhz else ""
            print(f"  {mhz} MHz{tag} | {item.get('clock', '')} | {item.get('model', '')}")
    if timing.get("worst_path"):
        path = timing["worst_path"]
        print(f"worst path report: {path.get('report', 'unknown')}")
        if "from_node" in path:
            print(f"from: {path['from_node']}")
        if "to_node" in path:
            print(f"to: {path['to_node']}")
    if timing.get("critical_paths"):
        print(f"critical paths: {len(timing['critical_paths'])}")
        for idx, path in enumerate(timing["critical_paths"], start=1):
            print(f"  #{idx}: slack {path['slack_ns']} ns")
            if "from_node" in path:
                print(f"      from: {path['from_node']}")
            if "to_node" in path:
                print(f"      to: {path['to_node']}")
            if path.get("nodes"):
                print("      nodes:")
                for node in path["nodes"][:40]:
                    print(f"        {node}")
                if len(path["nodes"]) > 40:
                    print(f"        ... {len(path['nodes']) - 40} more")
    if summary.get("resources"):
        print("resources:")
        for key, value in summary["resources"].items():
            print(f"  {key}: {value}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("prepare", "run", "sta", "summarize", "all"))
    parser.add_argument("config", type=Path, help="JSON OOC configuration")
    parser.add_argument("--no-prepare", action="store_true", help="do not regenerate before run/all")
    parser.add_argument("--critical-paths", type=int, default=None, help="number of setup critical paths to report")
    args = parser.parse_args()

    config_path = args.config.resolve()
    config = load_json(config_path)
    run_dir = config_run_dir(config_path, config)

    if args.command in ("prepare", "all") or (args.command in ("run", "sta") and not args.no_prepare):
        run_dir = prepare(config_path, args.critical_paths).resolve()
        print(f"prepared: {run_dir}", flush=True)

    run_error: subprocess.CalledProcessError | None = None
    if args.command in ("run", "all"):
        try:
            docker_run(config_path, run_dir)
        except subprocess.CalledProcessError as exc:
            run_error = exc
    if args.command == "sta":
        try:
            docker_sta(config_path, run_dir)
        except subprocess.CalledProcessError as exc:
            run_error = exc

    if args.command in ("sta", "summarize", "all") or run_error is not None:
        summary = summarize(config_path, run_dir, args.critical_paths)
        print_summary(summary)

    if run_error is not None:
        raise SystemExit(run_error.returncode)


if __name__ == "__main__":
    main()
