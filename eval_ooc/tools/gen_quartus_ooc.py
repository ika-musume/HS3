#!/usr/bin/env python3
"""
Generate a registered-boundary Quartus out-of-context timing project.

The tool intentionally uses only the Python standard library so it can be copied
into other RTL projects. It handles ordinary ANSI-style Verilog/SystemVerilog
module port lists, which is enough for the HS3 pipeline and many standalone
datapath/control modules.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import stat
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Port:
    direction: str
    name: str
    packed: str

    @property
    def is_scalar(self) -> bool:
        return self.packed == ""


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def find_matching(text: str, open_idx: int, open_ch: str, close_ch: str) -> int:
    depth = 0
    for idx in range(open_idx, len(text)):
        ch = text[idx]
        if ch == open_ch:
            depth += 1
        elif ch == close_ch:
            depth -= 1
            if depth == 0:
                return idx
    raise ValueError(f"unmatched {open_ch!r}")


def split_top_level(text: str, sep: str = ",") -> list[str]:
    parts: list[str] = []
    start = 0
    paren = bracket = brace = 0
    for idx, ch in enumerate(text):
        if ch == "(":
            paren += 1
        elif ch == ")":
            paren -= 1
        elif ch == "[":
            bracket += 1
        elif ch == "]":
            bracket -= 1
        elif ch == "{":
            brace += 1
        elif ch == "}":
            brace -= 1
        elif ch == sep and paren == 0 and bracket == 0 and brace == 0:
            part = text[start:idx].strip()
            if part:
                parts.append(part)
            start = idx + 1
    tail = text[start:].strip()
    if tail:
        parts.append(tail)
    return parts


def module_header(text: str, module_name: str) -> tuple[str, str]:
    clean = strip_comments(text)
    match = re.search(rf"\bmodule\s+{re.escape(module_name)}\b", clean)
    if not match:
        raise ValueError(f"module {module_name!r} was not found")

    idx = match.end()
    while idx < len(clean) and clean[idx].isspace():
        idx += 1

    params = ""
    if clean.startswith("#", idx):
        idx += 1
        while idx < len(clean) and clean[idx].isspace():
            idx += 1
        if idx >= len(clean) or clean[idx] != "(":
            raise ValueError(f"module {module_name!r} has malformed parameter list")
        end = find_matching(clean, idx, "(", ")")
        params = clean[idx + 1 : end].strip()
        idx = end + 1

    while idx < len(clean) and clean[idx].isspace():
        idx += 1
    if idx >= len(clean) or clean[idx] != "(":
        raise ValueError(f"module {module_name!r} has no ANSI port list")
    end = find_matching(clean, idx, "(", ")")
    ports = clean[idx + 1 : end].strip()
    return params, ports


def parse_ports(text: str, module_name: str) -> list[Port]:
    _params, port_text = module_header(text, module_name)
    ports: list[Port] = []
    current_dir = ""
    current_packed = ""

    for raw in split_top_level(port_text):
        item = " ".join(raw.split())
        match = re.match(r"^(input|output|inout)\b\s*(.*)$", item)
        if match:
            current_dir = match.group(1)
            rest = match.group(2).strip()
        elif current_dir:
            rest = item
        else:
            raise ValueError(f"port entry lacks direction: {raw!r}")

        rest = re.sub(r"\b(wire|logic|reg|tri|signed|unsigned|var)\b", " ", rest)
        rest = " ".join(rest.split())

        dims = re.findall(r"\[[^\]]+\]", rest)
        name_match = re.search(r"([A-Za-z_][A-Za-z0-9_$]*)\s*(?:=.*)?$", rest)
        if not name_match:
            raise ValueError(f"could not parse port entry: {raw!r}")

        name = name_match.group(1)
        if dims:
            current_packed = " ".join(dims)
        elif match:
            current_packed = ""

        ports.append(Port(current_dir, name, current_packed))

    return ports


def sv_type(port: Port) -> str:
    return f"logic {port.packed}".strip()


def port_decl(port: Port) -> str:
    width = f"    {port.packed}" if port.packed else "           "
    return f"    {port.direction:<6} wire {width}  {port.name}"


def logic_decl(name: str, packed: str, attr: str = "") -> str:
    prefix = f"{attr} " if attr else ""
    width = f"    {packed}" if packed else "           "
    return f"{prefix}logic {width}  {name};"


def sanitize_identifier(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", name)


def render_wrapper(config: dict[str, Any], ports: list[Port]) -> str:
    top = config["top_module"]
    wrapper = config.get("wrapper_module", f"{top}_quartus_ooc")
    clock_port = config["clock_port"]
    reset_ports = set(config.get("reset_ports", []))
    ce_cfg = config.get("clock_enable", {})
    ce_port = ce_cfg.get("port")
    ce_mode = ce_cfg.get("mode", "port")
    ce_ports = configured_clock_enable_ports(config)
    constants = dict(config.get("constant_inputs", {}))
    if ce_port and ce_mode == "tie_high":
        constants[ce_port] = "1'b1"
    elif ce_port and ce_mode == "tie_low":
        constants[ce_port] = "1'b0"

    # In "multicycle"/"port" mode the enable stays a real input and is passed
    # straight to the DUT (like clock/reset) so SDC multicycle exceptions can
    # model the slow CEN-gated domain instead of registering the enable here.
    ce_passthrough = bool(ce_port) and ce_mode in ("port", "passthrough", "multicycle")

    # All declared clocks (e.g. i_CLK_p and its inverse i_CLK_n) pass straight to
    # the DUT - they are clock nets, not boundary-registered data inputs.
    passthrough = set(configured_clock_ports(config)) | set(reset_ports)
    if ce_passthrough:
        passthrough.add(ce_port)
    passthrough |= ce_ports

    # Debug/trace ports (dbg* by default) are observation-only: leaving them off the
    # wrapper (no boundary registers, no virtual pins, DUT outputs unconnected) keeps
    # their wide fanout from adding routing noise to the timing run.
    nc_prefixes = tuple(config.get("no_connect_prefixes", ["dbg"]))
    nc_ports = {p.name for p in ports if p.name.startswith(nc_prefixes)}
    top_ports = [p for p in ports if p.name not in constants and p.name not in nc_ports]
    config["_wrapper_ports"] = top_ports
    input_regs = [p for p in top_ports if p.direction in ("input", "inout") and p.name not in passthrough]
    output_regs = [p for p in top_ports if p.direction in ("output", "inout")]

    lines: list[str] = []
    lines.append("`default_nettype wire")
    lines.append("")
    lines.append("/*")
    lines.append("    Auto-generated Quartus out-of-context timing wrapper.")
    lines.append("    DUT inputs and outputs are registered to create realistic timing endpoints.")
    lines.append("*/")
    lines.append("")
    lines.append(f"module {wrapper} (")
    for idx, port in enumerate(top_ports):
        comma = "," if idx + 1 < len(top_ports) else ""
        lines.append(f"{port_decl(port)}{comma}")
    lines.append(");")
    lines.append("")
    lines.append("///////////////////////////////////////////////////////////")
    lines.append("//////  Boundary Registers")
    lines.append("////")
    lines.append("")
    attr = "(* preserve, noprune *)"
    for port in input_regs:
        lines.append(logic_decl(f"{port.name}_drv", port.packed, attr))
    for port in output_regs:
        lines.append(logic_decl(f"{port.name}_dut", port.packed, "(* keep *)"))
        lines.append(logic_decl(f"{port.name}_q", port.packed, attr))
    for name, value in constants.items():
        orig = next((p for p in ports if p.name == name), None)
        packed = orig.packed if orig else ""
        lines.append(logic_decl(f"{name}_const", packed))
        lines.append(f"assign {name}_const = {value};")
    if input_regs or output_regs:
        lines.append("")
        lines.append(f"always_ff @(posedge {clock_port}) begin")
        for port in input_regs:
            lines.append(f"    {port.name}_drv <= {port.name};")
        for port in output_regs:
            lines.append(f"    {port.name}_q <= {port.name}_dut;")
        lines.append("end")
    for port in output_regs:
        lines.append(f"assign {port.name} = {port.name}_q;")

    lines.append("")
    lines.append("///////////////////////////////////////////////////////////")
    lines.append("//////  DUT")
    lines.append("////")
    lines.append("")

    overrides = config.get("parameter_overrides", {})
    if overrides:
        lines.append(f"{top} #(")
        items = list(overrides.items())
        for idx, (name, value) in enumerate(items):
            comma = "," if idx + 1 < len(items) else ""
            lines.append(f"    .{name:<24}({value}){comma}")
        lines.append(") u_dut (")
    else:
        lines.append(f"{top} u_dut (")

    for idx, port in enumerate(ports):
        comma = "," if idx + 1 < len(ports) else ""
        if port.name in nc_ports:
            # explicitly unconnected debug port; inputs tie low so no Z enters the DUT
            conn = "" if port.direction == "output" else "'0"
        elif port.name == clock_port or port.name in reset_ports or port.name in passthrough:
            conn = port.name
        elif port.name in constants:
            conn = f"{port.name}_const"
        elif port.direction in ("input", "inout"):
            conn = f"{port.name}_drv"
        else:
            conn = f"{port.name}_dut"
        lines.append(f"    .{port.name:<28}({conn}){comma}")
    lines.append(");")
    lines.append("")
    lines.append("endmodule")
    lines.append("")
    lines.append("`default_nettype none")
    return "\n".join(lines) + "\n"


def render_qpf(revision: str) -> str:
    return f'QUARTUS_VERSION = "17.0"\n\nPROJECT_REVISION = "{revision}"\n'


def qsf_file_assignment(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix == ".sv":
        key = "SYSTEMVERILOG_FILE"
    elif suffix in (".v", ".vh"):
        key = "VERILOG_FILE"
    elif suffix in (".vhd", ".vhdl"):
        key = "VHDL_FILE"
    else:
        key = "SOURCE_FILE"
    return f"set_global_assignment -name {key} {path.as_posix()}"


def configured_clock_ports(config: dict[str, Any]) -> set[str]:
    clocks = set(config.get("clock_ports", []))
    if "clock_port" in config:
        clocks.add(config["clock_port"])
    return clocks


def configured_clock_enable_ports(config: dict[str, Any]) -> set[str]:
    ce_cfg = config.get("clock_enable", {})
    if ce_cfg.get("mode") == "dual_phase":
        return {ce_cfg.get("p_port", "i_CEN_p"), ce_cfg.get("n_port", "i_CEN_n")}
    return set()


def render_qsf(config: dict[str, Any], source_paths: list[Path], wrapper_path: Path, sdc_path: Path) -> str:
    q = config["quartus"]
    wrapper = config.get("wrapper_module", f"{config['top_module']}_quartus_ooc")
    ports = config["_wrapper_ports"]
    clock_ports = configured_clock_ports(config)
    ce_ports = configured_clock_enable_ports(config)
    lines = [
        f"set_global_assignment -name FAMILY \"{q.get('family', 'Cyclone V')}\"",
        f"set_global_assignment -name DEVICE {q['device']}",
        f"set_global_assignment -name TOP_LEVEL_ENTITY {wrapper}",
        "set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files",
        "set_global_assignment -name OPTIMIZATION_MODE \"AGGRESSIVE PERFORMANCE\"",
        "set_global_assignment -name PHYSICAL_SYNTHESIS_COMBO_LOGIC ON",
        "set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_DUPLICATION ON",
        "set_global_assignment -name PHYSICAL_SYNTHESIS_REGISTER_RETIMING ON",
        qsf_file_assignment(wrapper_path),
    ]
    if q.get("seed") is not None:
        lines.append(f"set_global_assignment -name SEED {q['seed']}")
    for src in source_paths:
        lines.append(qsf_file_assignment(src))
    lines.append(f"set_global_assignment -name SDC_FILE {sdc_path.as_posix()}")
    for port in ports:
        if port.name in clock_ports or port.name in ce_ports:
            continue
        lines.append(f"set_instance_assignment -name VIRTUAL_PIN ON -to {port.name}")
    lines.append("set_instance_assignment -name PARTITION_HIERARCHY root_partition -to | -section_id Top")
    return "\n".join(lines) + "\n"


def render_sdc(config: dict[str, Any]) -> str:
    clock_port = config["clock_port"]
    period_ns = 1000.0 / float(config.get("clock_mhz", 100.0))
    reset_ports = config.get("reset_ports", [])
    ce_cfg = config.get("clock_enable", {})
    # i_CLK_n is supplied as the board-level inverse of i_CLK_p; model it as a
    # generated clock so STA treats the two as phase-related (half-period setup
    # on the GPR launch/capture paths) rather than asynchronous.
    inverted_clocks = config.get("inverted_clocks", {})
    lines = [
        f"create_clock -name {clock_port} -period {period_ns:.3f} [get_ports {{{clock_port}}}]",
    ]
    for inv_port, src_port in inverted_clocks.items():
        lines.append(
            f"create_generated_clock -name {inv_port} -source [get_ports {{{src_port}}}] "
            f"-invert [get_ports {{{inv_port}}}]"
        )
    # Additional asynchronous clock domains (e.g. the RTC 32.768 kHz crystal
    # input): each gets its own create_clock plus a set_clock_groups
    # -asynchronous cut against the master clock, so the 2FF-synchronizer
    # crossings are not timed. Schema: "async_clocks": {port: mhz}. The port
    # must also appear in "clock_ports" so the wrapper passes it through.
    async_clocks = config.get("async_clocks", {})
    for aport, amhz in async_clocks.items():
        aperiod = 1000.0 / float(amhz)
        lines.append(
            f"create_clock -name {aport} -period {aperiod:.3f} [get_ports {{{aport}}}]"
        )
    if async_clocks:
        groups = " ".join(f"-group {{{p}}}" for p in async_clocks)
        lines.append(
            f"set_clock_groups -asynchronous -group {{{clock_port}}} {groups}"
        )
    lines += [
        "",
        "# The DUT is wrapped by input/output registers, so external IO delays are not",
        "# modeled in this OOC run. Async reset release timing is handled at integration.",
    ]
    for reset in reset_ports:
        lines.append(f"set_false_path -from [get_ports {{{reset}}}]")

    # Dual-phase CEN mode models a 200 MHz master clock with architectural
    # registers enabled every other edge. Same-phase CEN paths (CEN_p->CEN_p,
    # CEN_n->CEN_n) get the relaxed two-cycle (100 MHz) setup window; cross-phase
    # paths stay at one master cycle. The key cross-phase path is the EX effective
    # address: ID/EX (CEN_p) -> address calc -> the cache BRAM request/address flop
    # (CEN_n), which must close in one 5 ns half-cycle.
    if ce_cfg.get("mode") == "dual_phase":
        # EA capture flops are the CEN_n request/address latch (cache reqn_* in the
        # real cache, or the OOC wrapper's model of it). They take the EX address
        # within one master cycle, so they must NOT get the same-phase relaxation.
        ea_to = ce_cfg.get("ea_capture_registers", "*|cache*reqn_*")
        # Accept a single glob or a list of globs: every CEN_n capture flop on the
        # half-cycle (the address flop bram_*, the way-select data_out do_*, and the
        # request-valid reqn_*) must be pinned to one master cycle - a single reqn_*
        # pin silently left the AGU->bram_addr address path on the relaxed 10 ns
        # multicycle, so it was never measured at 5 ns.
        ea_to_list = ea_to if isinstance(ea_to, list) else [ea_to]
        arch_period = 2.0 * period_ns
        lines += [
            "",
            "# Dual-phase clock-enable model. The master clock above is the fast (200",
            "# MHz) fabric clock; architectural registers are enabled every other edge",
            "# by CEN_p / CEN_n, so register-to-register paths get two master cycles",
            f"# ({arch_period:.3f} ns). A blanket register multicycle expresses that",
            "# directly. (An earlier per-enable get_fanouts split was unreliable: the",
            "# enable fanout does not resolve to register keepers post-fit, so the",
            "# relaxation silently dropped and every path was held to one cycle.)",
            "set_multicycle_path -setup 2 -from [get_registers *] -to [get_registers *]",
            "set_multicycle_path -hold  1 -from [get_registers *] -to [get_registers *]",
            "",
            "# Exception: the EX effective address must reach every CEN_n capture flop",
            "# (cache BRAM request/address bram_*, way-select do_*, valid reqn_*) in one",
            "# master cycle (CEN_p launch -> CEN_n capture). set_max_delay outranks the",
            "# multicycle, pinning each half-cycle endpoint to the 5 ns budget.",
        ]
        for pat in ea_to_list:
            lines.append(
                f"set_max_delay {period_ns:.3f} -to [get_registers {{{pat}}}]"
            )

        # The reverse half-cycle: a CEN_n source register launches to a CEN_p
        # capture register in one master cycle. The canonical case is the cache
        # load word (do_d_rdata, CEN_n) forwarded through the ID operand-select
        # mux into the ID/EX operand flops (CEN_p). The blanket multicycle above
        # would grade this at two cycles (10 ns) and hide it; a -from/-to
        # set_max_delay pins ONLY that source->dest pair to one cycle, leaving
        # the genuine CEN_p->CEN_p forwards (ex_result -> ID/EX) on the 10 ns
        # window. Each entry is {from:[globs], to:[globs]}.
        for spec in ce_cfg.get("forward_capture_paths", []):
            if spec.get("comment"):
                lines += ["", f"# {spec['comment']}"]
            from_list = spec.get("from", [])
            to_list = spec.get("to", [])
            for f_pat in from_list:
                for t_pat in to_list:
                    lines.append(
                        f"set_max_delay {period_ns:.3f} "
                        f"-from [get_registers {{{f_pat}}}] "
                        f"-to [get_registers {{{t_pat}}}]"
                    )

        # CEN_n LAUNCH registers: everything a half-cycle (CEN_n-enabled) source
        # register reaches is captured on the following CEN_p edge - one master
        # cycle. -from-only precedence sits BELOW -from/-to in TimeQuest, so the
        # same_phase_overrides below re-relax cen_n->cen_n cones (cache resolve)
        # back to the two-cycle window without pair-by-pair enumeration.
        launch_list = ce_cfg.get("launch_registers", [])
        if launch_list:
            lines += [
                "",
                "# CEN_n launch registers: one master cycle into every CEN_p capture.",
            ]
            for pat in launch_list:
                lines.append(
                    f"set_max_delay {period_ns:.3f} -from [get_registers {{{pat}}}]"
                )

        # Same-phase (CEN_n->CEN_n) cones re-relaxed to the architectural window.
        # A -from AND -to exception outranks the -from-only pins above.
        for spec in ce_cfg.get("same_phase_overrides", []):
            if spec.get("comment"):
                lines += ["", f"# {spec['comment']}"]
            f_pats = " ".join(spec.get("from", []))
            t_pats = " ".join(spec.get("to", []))
            lines.append(
                f"set_max_delay {arch_period:.3f} "
                f"-from [get_registers {{{f_pats}}}] "
                f"-to [get_registers {{{t_pats}}}]"
            )

        # Architecturally-false arcs (e.g. M10K mixed-port write->read modeling
        # inside simple-dual-port RAMs whose CEN phases separate the two edges).
        for spec in ce_cfg.get("false_paths", []):
            if spec.get("comment"):
                lines += ["", f"# {spec['comment']}"]
            f_pats = " ".join(spec.get("from", []))
            t_pats = " ".join(spec.get("to", []))
            entry = f"set_false_path -from [get_registers {{{f_pats}}}]"
            if t_pats:
                entry += f" -to [get_registers {{{t_pats}}}]"
            lines.append(entry)

    # Mode-independent architecturally-false arcs (top-level config key). Same schema
    # as the dual-phase clock_enable.false_paths: {comment, from, to} with to optional.
    for spec in config.get("false_paths", []):
        if spec.get("comment"):
            lines += ["", f"# {spec['comment']}"]
        f_pats = " ".join(spec.get("from", []))
        t_pats = " ".join(spec.get("to", []))
        entry = f"set_false_path -from [get_registers {{{f_pats}}}]"
        if t_pats:
            entry += f" -to [get_registers {{{t_pats}}}]"
        lines.append(entry)

    lines.append("")
    return "\n".join(lines)


def render_critical_paths_tcl(revision: str, critical_paths: int,
                              ea_patterns: list[str] | None = None) -> str:
    # The global top-N is sorted by absolute slack, which the 10 ns full-cycle paths
    # dominate; a 5 ns half-cycle path sitting near zero slack never surfaces there.
    # So additionally emit a targeted setup report PER half-cycle endpoint group, to
    # reports/halfcycle_paths.rpt, so the AGU->bram_addr 5 ns slack is always visible.
    targeted = ""
    for i, pat in enumerate(ea_patterns or []):
        # First pattern OVERWRITES the file (no -append) so each run starts a fresh report;
        # the rest append. (-append on every line let stale runs accumulate and mislead.)
        append = "" if i == 0 else "-append "
        targeted += (
            f"report_timing -setup -npaths 5 -detail full_path "
            f"-to [get_registers {{{pat}}}] {append}-file reports/halfcycle_paths.rpt\n"
        )
    return f"""project_open {revision}
create_timing_netlist
read_sdc
update_timing_netlist
report_timing -setup -npaths {critical_paths} -detail full_path -file reports/critical_paths.rpt -panel_name {{Top Setup Critical Paths}}
{targeted}delete_timing_netlist
project_close
"""


def render_sta_script() -> str:
    return """#!/bin/bash
set -euo pipefail

mkdir -p reports logs
/opt/intelFPGA/quartus/bin/quartus_sta -t critical_paths.tcl 2>&1 | tee logs/quartus_critical_paths.log
exit "${PIPESTATUS[0]}"
"""


def render_run_script(revision: str) -> str:
    return f"""#!/bin/bash
set -euo pipefail

REV="{revision}"
mkdir -p reports logs

set +e
quartus_sh --flow compile "$REV" 2>&1 | tee logs/quartus_compile.log
status=${{PIPESTATUS[0]}}
set -e

cp -f output_files/*.map.rpt reports/ 2>/dev/null || true
cp -f output_files/*.fit.rpt reports/ 2>/dev/null || true
cp -f output_files/*.sta.rpt reports/ 2>/dev/null || true
cp -f output_files/*.flow.rpt reports/ 2>/dev/null || true

if [ "$status" -eq 0 ]; then
    ./run_sta.sh
fi

exit "$status"
"""


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        cfg = json.load(handle)
    cfg["_config_path"] = str(path)
    cfg["_config_dir"] = str(path.parent)
    return cfg


def resolve_from_config(config: dict[str, Any], raw: str) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    return (Path(config["_config_dir"]) / path).resolve()


def relpath(path: Path, base: Path) -> Path:
    return Path(os.path.relpath(path.resolve(), base.resolve()))


def generate(config_path: Path, critical_paths: int | None = None) -> Path:
    config = load_config(config_path.resolve())
    if critical_paths is not None:
        config["critical_paths"] = critical_paths
    top = config["top_module"]
    source_paths = [resolve_from_config(config, src) for src in config["sources"]]
    top_source = resolve_from_config(config, config.get("top_source", config["sources"][0]))
    text = top_source.read_text(encoding="utf-8")
    ports = parse_ports(text, top)
    config["_parsed_ports"] = ports

    q = config["quartus"]
    revision = q.get("revision", f"{top}_quartus_ooc")
    run_name = q.get("run_name", f"quartus_{sanitize_identifier(q['device']).lower()}_{int(float(config.get('clock_mhz', 100.0)))}")
    run_dir = resolve_from_config(config, config.get("run_dir", f"runs/{run_name}"))
    run_dir.mkdir(parents=True, exist_ok=True)

    wrapper_path = run_dir / f"{config.get('wrapper_module', f'{top}_quartus_ooc')}.sv"
    sdc_path = run_dir / f"{revision}.sdc"
    qsf_path = run_dir / f"{revision}.qsf"
    qpf_path = run_dir / f"{revision}.qpf"
    run_script = run_dir / "run_quartus.sh"
    sta_script = run_dir / "run_sta.sh"
    critical_tcl_path = run_dir / "critical_paths.tcl"
    manifest_path = run_dir / "ooc_manifest.json"

    wrapper_path.write_text(render_wrapper(config, ports), encoding="utf-8")
    sdc_path.write_text(render_sdc(config), encoding="utf-8")

    cwd = Path.cwd().resolve()
    source_rel = [relpath(src, run_dir) for src in source_paths]
    wrapper_rel = relpath(wrapper_path, run_dir)
    sdc_rel = relpath(sdc_path, run_dir)
    qsf_path.write_text(render_qsf(config, source_rel, wrapper_rel, sdc_rel), encoding="utf-8")
    qpf_path.write_text(render_qpf(revision), encoding="utf-8")
    critical_count = int(config.get("critical_paths", 20))
    ea_cfg = config.get("clock_enable", {}).get("ea_capture_registers", [])
    ea_patterns = ea_cfg if isinstance(ea_cfg, list) else [ea_cfg]
    critical_tcl_path.write_text(
        render_critical_paths_tcl(revision, critical_count, ea_patterns), encoding="utf-8")
    run_script.write_text(render_run_script(revision), encoding="utf-8")
    sta_script.write_text(render_sta_script(), encoding="utf-8")
    run_script.chmod(run_script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    sta_script.chmod(sta_script.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    manifest = {
        "config": str(relpath(config_path.resolve(), cwd)),
        "run_dir": str(relpath(run_dir, cwd)),
        "revision": revision,
        "top_module": top,
        "wrapper_module": config.get("wrapper_module", f"{top}_quartus_ooc"),
        "clock_port": config["clock_port"],
        "clock_mhz": float(config.get("clock_mhz", 100.0)),
        "critical_paths": critical_count,
        "device": q["device"],
        "docker_image": q.get("docker_image", "raetro/quartus:17.0"),
        "generated": {
            "wrapper": wrapper_path.name,
            "sdc": sdc_path.name,
            "qsf": qsf_path.name,
            "qpf": qpf_path.name,
            "critical_paths_tcl": critical_tcl_path.name,
            "run_script": run_script.name,
            "sta_script": sta_script.name,
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return run_dir


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="JSON OOC configuration")
    parser.add_argument("--critical-paths", type=int, default=None, help="number of setup paths to request from TimeQuest")
    args = parser.parse_args()
    run_dir = generate(args.config, args.critical_paths)
    print(run_dir)


if __name__ == "__main__":
    main()
