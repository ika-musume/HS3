#!/usr/bin/env bash
# HS3 simulation runner (Verilator).
#
# usage: sim/run_sim.sh <source_dir> <testbench.sv> [sim_args...]
#
#   <source_dir>    design tree; every *.sv/*.v under it is compiled
#   <testbench.sv>  top testbench; module name must match the file basename
#   [sim_args...]   forwarded to the simulation binary (e.g. +verilator+seed+123)
#
# examples:
#   sim/run_sim.sh src          sim/HS3_tb.sv        # full SoC
#   sim/run_sim.sh src/cpu_core sim/cpu_core_tb.sv   # core only
#
# All tb-support files under sim/ (vendor memory models in sim/models) are
# compiled along with the design; other *_tb.sv are excluded so exactly one
# testbench is elaborated. Build artifacts go to sim/obj_dir_<top>.

set -euo pipefail

if [ $# -lt 2 ]; then
    sed -n '2,16p' "$0"; exit 1                     #print the usage header above
fi

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$1"
TB_FILE="$2"
shift 2

[ -d "$SRC_DIR" ]  || { echo "error: source dir '$SRC_DIR' not found" >&2; exit 1; }
[ -f "$TB_FILE" ]  || { echo "error: testbench '$TB_FILE' not found"  >&2; exit 1; }

TOP="$(basename "$TB_FILE" .sv)"
OBJ_DIR="$SIM_DIR/obj_dir_$TOP${VLT_DEFINES:+_def}"    #defines get their own build dir

#gather sources: design tree + sim/ support files, minus every *_tb.sv
#(the target tb is appended explicitly), minus build outputs
mapfile -t SOURCES < <(
    find "$SRC_DIR" "$SIM_DIR" \( -name 'obj_dir*' -prune \) -o \
        \( -name '*.sv' -o -name '*.v' \) ! -name '*_tb.sv' -print \
    | sort -u
)

#lint waivers are scoped to tb/vendor files (see verilator_waivers.vlt);
#warnings in design sources still fail the build
#VLT_DEFINES: optional extra -D switches (e.g. VLT_DEFINES=-DHS3_RDW_HOSTILE_CACHE
#builds the adversarial M10K DONT_CARE collision model for the RDW audit)
verilator --binary -j 0 -O3 --sv \
    ${VLT_DEFINES:-} \
    "$SIM_DIR/verilator_waivers.vlt" \
    --top-module "$TOP" \
    -Mdir "$OBJ_DIR" \
    "${SOURCES[@]}" "$TB_FILE"

exec "$OBJ_DIR/V$TOP" "$@"
