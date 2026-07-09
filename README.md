# HyperSquid3 - An SH7709-compatible barebone SystemVerilog core

HyperSquid3 (HS3) is a synthesizable SystemVerilog implementation of the Hitachi/Renesas
**SuperH-3** RISC processor, modelled on the **SH7709S**. It is a near cycle-accurate core - 
the pipeline, cache, bus controller, and on-chip peripherals reproduce the
SH7709S hardware manual's timing behaviour, not just its instruction set.

- **Target:** Low-cost/mainstream FPGAs
- **Physical status:** Quartus out-of-context restricted Fmax ≈ **77-79 MHz**
  across seeds today; the remaining gap to 100MHz is a small number of protected
  single-cycle CPU datapath loops, not the cache, bus, or peripherals.

> For the microarchitecture and timing rationale - *what the RTL actually does and
> why it is shaped the way it is* - read **[docs/HS3_Core_Hardware.md](docs/HS3_Core_Hardware.md)**.
> This README is the orientation and usage guide.

---

## What's in the box

The design is a single-clock (`i_CLK` + clock-enable `i_CEN`) SoC subset. No clock
gating anywhere - every register is a synchronous DFF with a clock enable, per the
Cyclone V target.

| Block | Files | Summary |
|---|---|---|
| **Integer pipeline** | [src/cpu_core/int_pipe.sv](src/cpu_core/int_pipe.sv), [int_pipe_pkg.sv](src/cpu_core/int_pipe_pkg.sv), [agu.sv](src/cpu_core/agu.sv) | Classic SH 5-stage in-order pipeline (IF/ID/EX/MA/WB), full forwarding, 2R2W GPR file, MAC, and a single time-shared address adder (no redirect mux). |
| **Unified cache** | [src/cpu_core/cache.sv](src/cpu_core/cache.sv), [cache_mem.sv](src/cpu_core/cache_mem.sv), [cache_pkg.sv](src/cpu_core/cache_pkg.sv) | 16 KB, 4-way, 16-byte line, unified I+D, pseudo-LRU. The lookup is folded into the pipeline stage (no request/response handshake), so cache hits sustain IPC ≈ 1. |
| **Exceptions / control** | [src/cpu_core/exc_handler.sv](src/cpu_core/exc_handler.sv), [ctrl_reg.sv](src/cpu_core/ctrl_reg.sv) | Exception/interrupt entry and the P4 exception MMIO registers; SR/GBR/VBR and friends. |
| **Bus fabric** | [src/peri/ibus_splitter.sv](src/peri/ibus_splitter.sv), [ibus_bridge.sv](src/peri/ibus_bridge.sv) | The on-chip bus tiers (L / I bus 1 / I bus 2 / P bus) matching SH7709S Fig 1.1. |
| **BSC** | [src/peri/bsc.sv](src/peri/bsc.sv) | External bus controller on the **real chip pin set** - SDRAM engine, ordinary/burst-ROM with wait-states, a generic mirror port for the surrounding SoC, and the BSC register file. |
| **Peripherals** | [cpg_wdt.sv](src/peri/cpg_wdt.sv), [intc.sv](src/peri/intc.sv), [tmu.sv](src/peri/tmu.sv), [rtc.sv](src/peri/rtc.sv), [ioport.sv](src/peri/ioport.sv) | Clock-pulse generator + watchdog, interrupt controller, 3× timer unit, RTC (on its own 32.768 kHz domain), and the 12 I/O ports / PFC with real Table-18.1 pin sharing. |
| **Chip top** | [src/HS3.sv](src/HS3.sv) | Ties the core, fabric, BSC, and peripherals into the `HS3` module with the SH7709S external pin set. |

Two elaboration targets are provided:

- **`cpu_core`** - the CPU core alone (pipeline + cache + control/exception logic).
- **`HS3`** - the full SoC top, including the BSC and all on-chip peripherals.

---

## Getting started

### Prerequisites

- **Verilator** ≥ 5.032 (the primary simulator).
- A C++ toolchain (Verilator builds a native binary).
- Bash.

### Running the testbenches

Simulations are driven through one wrapper script, [sim/run_sim.sh](sim/run_sim.sh):

```bash
sim/run_sim.sh <source_dir> <testbench.sv> [sim_args...]
```

It compiles every `*.sv`/`*.v` under `<source_dir>` plus the `sim/` support files
(other `*_tb.sv` are excluded so exactly one testbench elaborates), builds with
`verilator --binary -j 0 -O3 --sv` into `sim/obj_dir_<top>/`, and runs the binary.
Extra arguments are forwarded to the simulation (e.g. `+verilator+seed+123`).

**Core-only** (no external models needed):

```bash
sim/run_sim.sh src/cpu_core sim/cpu_core_tb.sv
# ... expected tail:
# cpu_core_tb: PASS (91 tests)
```

**Full SoC** (see the vendor-model note below):

```bash
sim/run_sim.sh src sim/HS3_tb.sv
# ... expected tail:
# HS3_tb: PASS (57 tests)
```

Both suites also print an IPC micro-benchmark, e.g. from `cpu_core_tb`:

```
[BENCH] straight-line NOP:  203 / 506  -> IPC = 0.401  (non-cacheable bypass path)
[BENCH] cached add loop:   1137 / 1167 -> IPC = 0.974  (cache-hit path)
[BENCH] cached store loop:  415 / 748  -> IPC = 0.554  (unified single-port ceiling)
```

### Vendor memory models (for `HS3_tb` only)

The full-SoC testbench verifies the BSC against a Micron **MT48LC2M32B2** SDRAM
model and a Macronix **MX29LV320E** NOR-flash model. Those vendor Verilog files are
**not** redistributed here - only the patch recipes that make them Verilator-clean
are tracked, in [sim/models/](sim/models/):

- `mt48lc2m32b2.verilator.patch`
- `MX29LV320E.verilator.patch`

Obtain each vendor model, place it in `sim/models/` under the expected filename
(`mt48lc2m32b2.v`, `MX29LV320E.v`), and apply the matching `.patch`. The core-only
`cpu_core_tb` does not use these models and runs on a clean checkout.

---

## Repository layout

```
src/
  HS3.sv              chip top (SH7709S external pin set)
  cpu_core/           pipeline, cache, control, exceptions, AGU
  peri/               BSC, INTC, TMU, RTC, CPG/WDT, I/O ports, bus fabric
sim/
  run_sim.sh          Verilator build+run wrapper
  cpu_core_tb.sv      core-only testbench (91 tests + IPC bench)
  HS3_tb.sv           full-SoC testbench (57 tests, vendor-model bus checks)
  models/             vendor memory-model patch recipes (models themselves untracked)
  verilator_waivers.vlt   lint waivers, scoped to tb/vendor files only
docs/
  HS3_Core_Hardware.md    microarchitecture & timing design doc - start here
  *.pdf                    SH7709S / SH-3 / SH-1·SH-2 / SH7604 reference manuals
```

Lint waivers are scoped to testbench and vendor files; a Verilator warning in any
design source under `src/` still fails the build.

---

## Documentation & references

- **[docs/HS3_Core_Hardware.md](docs/HS3_Core_Hardware.md)** - the design-intent
  companion: clocking philosophy, pipeline, cache lookup model, bus tiers, BSC,
  peripherals, and the timing-closure analysis. Read this before diving into RTL.
- **Vendor manuals** in [docs/](docs/) - the SH7709S hardware manual and the SH-3
  software manual are the primary references; the SH-1/SH-2 manual is the pipeline
  baseline and the SH7604 manual is the cache-operation reference. RTL comments cite
  these by page (`see p.NNN`).

---

## License

BSD 2-Clause. See [LICENSE](LICENSE).
