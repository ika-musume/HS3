# HS3 — Core Hardware Description

A microarchitecture and timing overview of the HS3 SystemVerilog SH-3 (SH7709S)
core. This document describes *what the RTL actually does* and *why it is shaped
the way it is for timing* — it is the design-intent companion to the vendor PDFs
in this folder. Manual page citations (`p.NNN`) point at the SH7709S hardware
manual or the SH-3 software manual unless noted.

**Target:** Intel Cyclone V FPGA, 100 MHz deliverable. **Verification:** Verilator
(`cpu_core_tb` = 64/64, `HS3_tb` = 53/53). **Physical status:** Quartus out-of-context
(OOC) restricted Fmax ≈ **80–83 MHz** today; the gap to 100 MHz is three protected
single-cycle datapath loops (see §0 and §5), not the cache, bus, or peripherals —
every OOC critical path is CPU-internal.

---

## 0. Clocking, reset, and the timing philosophy

### Clock scheme

The shipping core is **single-clock**: one architectural clock `i_CLK` plus one
architectural clock-enable `i_CEN`. There is **no clock gating** anywhere (an ASIC
technique unavailable on Cyclone V) — every register is a synchronous DFF with a
clock-enable (`(* direct_enable *) wire cen = i_CEN;` in `int_pipe.sv`). The
testbenches drive `i_CEN = 1` at a 10 ns period (100 MHz). One `i_CEN` edge = one
architectural cycle.

| Domain | Clock | Rate | Owner |
|---|---|---|---|
| CPU + cache + on-chip fabric | `i_CLK`/`i_CEN` | 100 MHz | whole core |
| PCB-facing SDRAM engine + refresh | `i_CLK` + `i_BCEN` | 50 MHz enable (B-φ = core/2, p.207) | BSC |
| Bus clock output pin | `o_CKIO` | B-φ phase register | CPG |
| RTC oscillator | `i_EXTAL2` | 32.768 kHz, **genuinely asynchronous** | RTC only |

The **RTC is the only true second clock domain.** Everything the RTC exposes to the
CPU crosses back into the `i_CLK` domain through tick-synchronizers and no-reset
toggle flags; the manual's own ~91.6 µs command latency (p.421) absorbs the CDC
skew. `o_CKIO` clocks the board SDRAM directly (the TB clocks its Micron model from
the pin).

### A note on the half-clock (`dclk`) lineage

Some source comments reference `cen_p`/`cen_n` and a "dclk half-clock." That is the
timing-R&D ancestor of the current design: a 2× (200 MHz) master with two
mutually-exclusive enables on alternating edges, so a `cen_n` register was a
*half-cycle* register (zero added latency) that gave the EX address path a clean
5 ns window. It reached its 5 ns goals (AGU, cache address capture) but converged on
the **same** architectural operand cone as the single-clock design (~78 MHz master →
~78 MHz arch). The single-clock **model-b** cache (§2) was then adopted as the
shipping structure. Read `cen_p`/`cen_n` in comments as design history, not two live
clocks — the RTL ports are `i_CLK` + `i_CEN` only.

### The hard rule: no bubbles, IPC first

The overriding design directive: **never regress IPC versus the SH7709S-faithful
baseline.** No inserted pipeline bubbles, no extra handshake/acknowledge beats. On
Cyclone V the goal is to *minimize* setup slack, not necessarily to reach perfect
closure — residual negative slack is accepted when the only alternative would cost a
cycle. Any new register must land on an *already-enabled* edge its consumers already
wait for. This is why the three residual timing walls (§5) are left open rather than
pipelined away.

---

## 1. Pipeline

### Stage structure

Classic SH 5-stage in-order pipeline (SH-1/SH-2 baseline, extended per SH-3):

```
  IF  ──▶  ID  ──▶  EX  ──▶  MA  ──▶  WB
 fetch   decode   execute  memory   write-back
        +regread  +AGU     access
```

| File | Lines | Role |
|---|---|---|
| `int_pipe.sv` | 3073 | the integer pipeline (IF/ID/EX/MA/WB, hazard/forward, GPR, MAC) |
| `int_pipe_pkg.sv` | 916 | inter-stage packet typedefs (`ifid`/`idex`/`exma`/`mawb`), decode |
| `agu.sv` | 68 | the single time-shared address adder |
| `int_pipe_mem.sv`, `ctrl_reg.sv` | | memory-op sequencer, control registers (SR/GBR/VBR/…) |
| `exc_handler.sv` | 325 | exception/interrupt entry + the P4 exception MMIO registers |

The stage registers are the packet structs; a **single global stall** (from the
cache, §2) freezes every stage register at once — there is no per-stage skid buffer,
FIFO, or outstanding-transaction tracking. Back-pressure *is* the freeze.

### Address generation — one adder, no mux

All addresses (sequential PC+2, PC-relative branch targets, data effective
addresses) come from **one** 32-bit AGU adder, time-shared:

```
  o_ADDR = X + (en · Y) + ci        // agu.sv
    X  = i_USE_BASE ? agu_base : fetch_pc
    en = {NULL, FORCE, r_t, ~r_t}[en_mode]   // T folds into the addend LUT
    ci = +2 for the PC+2 case
```

The timing constraint driving this shape: on Cyclone V a ~5 ns half-clock is *one
LUT stage plus carry*. **No discrete mux may sit before or after the adder on that
path** — every source/operand selection is folded *into the adder's own LUTs* (the
`en`-null trick, operands pre-selected in ID). The conditional-branch case is the
elegant payoff: `BT`/`BF` set `use_base = en = r_t`, so the *same* AGU configuration
emits the branch target when taken (`T=1`) or `fetch_pc+2` when not-taken — the
redirect mux is gone. PC-relative targets are precomputed in ID
(`idex.immediate = pc+4+disp`). The AGU OOC'd at +0.079 ns @ 5 ns (203 MHz).

### Forwarding and the register file

- **GPR = 2R2W** built from 4× 32×32 simple-dual-port M10K banks
  (`{wb0,wb1} × {r0,r1}`) plus a 32×1 flip-flop LVT (live-value table); a
  registered LVT select is one 2:1 mux after the RAM read. Same-edge write/read
  hazards are closed by one-cycle WB shadow lanes (`wb0z`/`wb1z`) in the operand
  early legs.
- **Forwarding:** EX-result and MA-result forward into the ID/EX operand latches.
  The forward-source select is pre-decoded and (where it survives timing) registered
  at the IF/ID edge from the "next-ifid" cone, so it is a live recompute equivalent
  without a late combinational path.

### Control-flow timing (this is cycle-law, cite it)

| Case | Penalty | Reference |
|---|---|---|
| Non-delayed `BT`/`BF`, **taken** | **2 fetch bubbles** | SW manual Fig 10.40, p.476 |
| Non-delayed `BT`/`BF`, **not-taken** | 0 | Fig 10.41 |
| Delayed branches (`BRA/BSR/JMP/JSR/RTS/RTE/BRAF/BSRF`) | 0 (delay-slot slack) | §10.2.3, p.432 |

The 2-bubble taken penalty is itself the *proof* that IF is a single 1-cycle stage
fetching back-to-back at one instruction/cycle (the condition resolves in EX; the two
shadow-fetched instructions are discarded). `RTS` reads PR through the existing ID
control-register interlock — no new PR forward path.

### Load-use

A cache **load hit costs zero stall cycles**; only a 1-slot load-use interlock
remains, and even that is avoided for `MOV.L` (longword loads need no aligner, so
they are zero-bubble). Byte/halfword loads carry the aligner/sign-extend in the
`cen_n→cen_p` window; its residual slack is accepted rather than paid for with a
bubble (the no-bubbles rule).

### Measured IPC (verified against the current `main` tree)

| Workload | Retires / cycles | IPC | Meaning |
|---|---|---|---|
| Straight-line NOPs, **non-cacheable** (P2 bypass) | 201 / 706 | **0.285** | front-end fetch ceiling with the halfword-reuse buffer |
| Dependent add loop, **cacheable hit** | 1147 / 1167 | **0.983** | ≈ 1.0; the 1.7 % gap is the 2 taken-branch bubbles per iteration |
| 100 %-store loop, **cacheable hit** | 418 / 827 | **0.505** | the 0.5 unified-single-port ceiling (each store = 1 fetch + 1 data-port cycle) |

The bypass path was lifted 0.199 → 0.285 by a **halfword-reuse buffer** (`ibyp_buf_*`
in `cache.sv`): a non-cacheable longword read serves its sibling halfword from a
registered buffer instead of a second bus transaction (2 cycles, no bus run). The
buffer mirrors memory — loaded only from fault-free reads, blanket-invalidated on any
external write.

---

## 2. Cache

### Geometry (SH7709S p.103–104)

**Unified** instruction+data cache — one tag/data/LRU array, **not** split I$/D$.

| Parameter | Value |
|---|---|
| Size | 16 KB |
| Associativity | 4-way set-associative |
| Line | 16 bytes (4 longwords) |
| Sets | 256 (`16384 / 4 / 16`) |
| Tag | PA[28:10], 19 bits (PA is 29-bit; PA[31:29] are region shadow) |
| Index | PA[11:4], 8 bits |
| Replacement | 6-bit pseudo-LRU, Table 5.2 (one-hot victim decode, one LUT/bit) |

Files: `cache.sv` (1277, the wrapper + FSM), `cache_mem.sv` (157, the M10K banks),
`cache_pkg.sv` (108, geometry + `tag_of`/`cacheable`/`lru_*`/`merge_word` helpers).

### The lookup model — folded into the pipe, no handshake

The single most important structural fact: **the cache lookup is folded into the
pipeline stage; a hit is *not* a request/response transaction.** IF *is* the I-side
access; MA *is* the D-side access. This is what makes IPC=1 reachable (a req/rsp
handshake structurally caps IPC at 0.5). The shipping implementation is **model-b**,
a two-beat overlapped lookup:

- **Beat 0 (request cycle):** the pipe presents the live AGU address. The **accept**
  (`req_ready`) is decided from registered state and the *previous* access's resolve
  (`z_ok`, below). At the edge the RAMs capture the read index and `bram_*` captures
  the request descriptor.
- **Beat 1 (resolve cycle):** tag/data `q` are compared against `bram_*`; a hit is
  answered **combinationally** and consumed by the pipe at the closing edge — the
  same edge the *next* access is captured. Back-to-back hits therefore run at one per
  cycle. A live hit response the pipe can't consume this cycle **retires into a
  registered `rsp_*` flag** (loss-free hold), so no response is ever overwritten.

There is no held-response slot for hits (the resolve *is* the response), which is
exactly why an earlier "held-slot overlap" attempt deadlocked and was abandoned.

### `z_ok` — the single global stall

`z_ok` is the one global-stall term. It blocks a new accept whenever the previous
access's resolve still occupies the cache — a cacheable **miss**, a write-through
store dispatch, or a CCR flush. On a stall, **every** stage register in the core
freezes together. By design `z_ok` carries the tag-compare into `req_ready`; it is
*the* critical path of the design (tag q → compare → `z_ok` → `req_ready` → issue
tail).

### One port ⇒ MA-priority arbitration

IF and MA present addresses to the **same** single read port. Arbitration is
**MA-priority**: when both want the same cycle, the data access wins and the fetch
stalls one cycle (the "MA contends with IF" fetch bubble, p.454–455). This is why
memory-heavy code runs below 1 IPC — loads/stores structurally steal fetch slots.
The store ceiling of 0.5 is this port limit, and it is *correct* RISC behavior
(adding a second read port was explicitly rejected).

### Write policy, fills, MMIO fold

- **Per-region write policy** (p.105, 110–111): P1→`CCR.CB`, P0/U0/P3→`~CCR.WT`
  (`wb_mode`), with a `U` (dirty) bit in the tag. WB write hit = cache+U; WT write
  hit = cache+memory; WB write miss = write-allocate; WT write miss = memory only.
- **Write-back buffer:** one line (p.111–112, Fig 5.5); line fills are 4 sequential
  longword reads, word-0-first.
- **Store fast path:** a write-back store hit commits its strobed bytes at the
  resolve edge and retires in one MA cycle (a store is a *notify*, not an ack — the
  pipe retires it via `ma_complete`); the next access's read at that same edge gets
  the just-written bytes through `cache_mem`'s write-through bypass registers
  (write-before-read order; Cyclone V M10K has no silicon mixed-port new-data).
- **MMIO fold:** CCR/CCR2 and the exception registers (TRA/EXPEVT/INTEVT/INTEVT2/TEA)
  are served as a *local-register access class* through the cache's shared `do_d`
  output flop, matched off the **latched** address — deliberately keeping MMIO decode
  off the AGU 5 ns fan-out. Writes are fire-and-forget; reads return a 1-cycle
  registered response. Memory-mapped cache windows: tag `0xF0xx_xxxx`, data
  `0xF1xx_xxxx` (p.112–114).
- **Non-cacheable bypass:** P2 (`101`) and P4 (`111`) are non-cacheable control
  spaces (`cacheable()` in `cache_pkg`); the reset PC lives in P2 (`0xA000_0000`).

### Miss / external-bus FSM

Only a miss leaves the running state. The miss/refill/write-back/bypass/MMIO states
each last a full cycle (each state's RAM read lands one state later), and drive the
external I-bus (§3). The FSM is unchanged from the earlier design; the model-b work
replaced only the hit/accept/respond path.

---

## 3. Bus structure

### On-chip tiers (SH7709S Fig 1.1, p.6)

The core follows the SH7709S internal bus hierarchy. **Latency law:** the on-chip
fabric is **zero-wait, fire-and-forget**; handshake latency is permitted *only* on
the bridge tiers (I bus 2, P bus) and the external BSC leg — and only if OOC shows a
bus register would cost less Fmax than the pipeline's own worst path (it never has:
zero bus/peripheral cone appears in any OOC top-20).

| Bus | `addr[31:29]` | Timing | What rides it |
|---|---|---|---|
| **L bus** | `111` (P4) | 1-cycle R/W, zero-wait | CPU-direct control regs (CCR, exception MMIO) |
| **I bus 1** | — | zero-wait to the BSC | the cache's master port (`I_BUS`) toward memory |
| **I bus 2** | — | handshake (bridge) | CPG/WDT, INTC register file (behind the BRIDGE) |
| **P bus** | P4 / area-1 | 2-cycle read (bridge) | TMU, RTC, I/O ports (in-BSC bridge) |

Fabric glue: `ibus_splitter.sv` (86, mux-only, zero beats, `owner_q` steers the
response), `ibus_bridge.sv` (159, IDLE→ACCESS→RESP, right-justified writes,
lane-replicated reads), `peri_bus_if.sv` (the slim register-bus interface).

### BSC — the external bus controller (`bsc.sv`, 1383 lines)

The BSC exposes the **real SH7709S chip pin set** (Table 10.1, PCMCIA-less) at the
`HS3` top: `A[25:0]`, a split data bus (`o_D_O`/`o_D_OE`/`i_D_I`, `inout` only at
board level), `BS_n`, `CS0/2–6_n`, `RD_WR`, `RAS3L/U_n`, `CASL/U_n`, `WE_n[3:0]`
(= DQM), `RD_n`, `i_WAIT_n`, `CKE`, and `BREQ_n`/`BACK_n`. Front-end route classes:

- **SDRAM engine** — MRS / single / burst-of-4 / auto-refresh / self-refresh /
  bank-active with per-bank open rows, tWR guard, CL read pipe. Timing is driven
  exactly by the `MCR`/`WCR2` registers (CL/RCD/tRP), on the 50 MHz `i_BCEN` enable.
  Reproduces the natural SDRAM latency of the original board (most emulated code runs
  from SDRAM), §10.3.4 figs 10.14–10.28.
- **Ordinary / burst-ROM** — wait-states from `WCR2` (first access ∈ {0,1,2,3,4,6,8,10}),
  `i_WAIT_n` sampled after the programmed waits; burst-ROM continuation uses the
  pitch table. Reads sample `i_D_I` live at the completing bus edge (raw async
  ROM/SRAM works with no handshake). An `ord_run` grid-align flop keeps every pin
  edge on the 20 ns bus grid (a real bug the 70 ns NOR flash exposed).
- **Generic mirror port** — a zero-beat pass-through toward the surrounding SoC's own
  controllers (fabric SDRAM ctrl / HPS DDR3); this is the IPC-parity path. All data
  rides the physical D pins; the generic port is pure address/control.
- **Register / dummy** — the BSC's own POR-only register file (`0xFFFFFF50–74`).

Verified against a Micron MT48LC2M32B2 SDRAM model and a Macronix MX29LV320E NOR
flash model (patched vendor copies in `sim/models/`), including **boot-from-flash**
(16-bit two-sub-cycle fetches) and autoselect. Hard-won correctness points:
bus-ownership interlocks between the ordinary controller and the SDRAM engine;
self-refresh park must not count as bus-busy (else fetches deadlock); and
**TAS.B atomicity vs BREQ** — the bus must not be released between a locked pair's
read and write (p.320). Cache line fills present as `req_burst` on I bus 1.

---

## 4. Peripherals

All on-chip peripherals are timing-free in OOC (zero cones in any top-20); the CPU
remains the critical path. Full-SoC OOC (all peripherals) lands ≈ 81–82 MHz.

| Module | File | Function |
|---|---|---|
| **CPG / WDT** | `cpg_wdt.sv` (282) | FRQCR/STBCR/STBCR2 clock-pulse generator; Pφ divider N∈{1,2,3,4,6}; watchdog timer with keyed `0x5A`/`0xA5` writes + reset stretcher; owns `o_BCEN` and the `o_CKIO` pin (B-φ = core/2, p.207 — FRQCR has no CKOEN, CKIO always drives in modes 0–2). |
| **INTC** | `intc.sv` (455) | Full §6 interrupt controller: IRQ / IRL / IRLS / PINT / NMI, a 37-entry **2-stage registered priority resolver**, `INTEVT2`, `o_INT_ACK`/`o_NMI_ACK`. Interrupt inputs now tap the I/O pads (below). |
| **TMU** | `tmu.sv` (262) | 3× 32-bit auto-reload down-counters; shared Pφ prescaler taps (P/4, /16, /64, /256); external TCLK clock (per CKEG, 2FF + edge detect); ch2 input capture (TCPR2, ICPF); underflow interrupts `TUNI0-2`/`TICPI2` → INTC (IPRA). |
| **I/O ports / PFC** | `ioport.sv` (232) | All 12 ports (A–L, SCP) as `pcr[]`/`pdr[]` arrays with per-port capability masks (drive/pull-up), PFC mode muxing (`MD1 ? pin : (DRV & DR)`), the PGCR PTG0 quirk (p.577). |
| **RTC** | `rtc.sv` (407) | §13, **two clock domains**: the `i_EXTAL2` 32.768 kHz oscillator (7-bit prescaler → RTCCLK 16.384 kHz + 256 Hz tap) and the bus domain (R64CNT, BCD calendar, alarms, periodic interrupt). CDC by tick-sync + no-reset toggles. Counters/alarms never pin-reset (Table 13.2). Feeds TMU `i_RTCCLK`/`i_RTC_TICK`. |

**Real-chip pin sharing (Table 18.1):** the dedicated `i_IRQ`/`i_IRLS`/`i_PINT`
inputs are **deleted** — interrupt sources ride the port pads
(`i_IRQ = {SCPT7, PTH4-0}`, `i_IRLS = PTF3-0`, `i_PINT = {PTF, PTC}`); only NMI stays
dedicated. `PTF` is shared PINT8-15/IRLS3-0, and `PTH7` mode-00 hands its pad to the
TMU TCLK. This matches the SH7709S philosophy that peripheral function and GPIO
multiplex on the same physical pins.

---

## 5. Timing summary — where the Fmax goes

Single-clock OOC restricted-Fmax history (Quartus 17, Cyclone V, `set_max_delay`
false-paths on the M10K RDW arcs):

| Configuration | Worst slack @ 10 ns | Fmax |
|---|---|---|
| Core only, single-clock port (sclk fit4) | −2.556 ns | 79.64 MHz |
| Core only, after operand/hazard predecode (rounds 5–7) | −2.085 ns (seed 3) | **82.75 MHz** |
| Full SoC + peripherals (RTC session) | −2.329 ns | 81.11 MHz |

The remaining walls are the **architecture's protected single-cycle loops** — they
cannot be pipelined without inserting a bubble, which the IPC-first rule forbids:

1. **ALU→forward loop:** `idex.alu_op` → dynamic shifter → `alu_result` → EX-forward
   → operand-select → operand capture (a full-budget path by design).
2. **Load-use loop:** cache tag-compare → `ld_word` (aligner) → operand forward — the
   1-cycle load-use path.
3. **MA-forward loop:** `exma.*` → `idex.src` — the memory-stage result forward.

Three independent experiments confirmed saturation: max CAD effort produced a
bit-identical result; class-kill rounds only rotate the plateau ±0.4 ns; and a clean
structural kill of the worst class (async-read MLAB GPR) *net-regressed* because it
congested the operand LAB neighborhood. The operand cluster is congestion-bound, not
logic-bound. Plausible remaining levers toward 100 MHz: a LogicLock floorplan pin of
the operand cluster, or a C6 speed grade if the board allows — **not** an operand-
capture pipeline beat (that would break cycle accuracy).

---

## References

- `SH7709S_Hardware_Manual[REJ09B0081-0500O].pdf` — cache (§5, p.103–114), BSC
  (§10), INTC (§6), TMU, RTC (§13), ports (§18), CPG (p.207–212).
- `SH-3_SH-3E_SH3-DSP_Software_Manual.pdf` — pipeline timing (Fig 10.40/10.41
  p.476), branch/PR rules (§10.2.3 p.432).
- `SH-1_SH-2_Programming_Manual.pdf` — 5-stage pipeline baseline.
- `SH7604_Hardware_Manual[ADE-602-085C].pdf` — cache operation reference.
</content>
</invoke>
