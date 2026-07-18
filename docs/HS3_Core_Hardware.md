# HS3 — Core Hardware Description

A microarchitecture and timing overview of the HS3 SystemVerilog SH-3 (SH7709S)
core. This document describes *what the RTL actually does* and *why it is shaped
the way it is for timing* — it is the design-intent companion to the vendor PDFs
in this folder. Manual page citations (`p.NNN`) point at the SH7709S hardware
manual or the SH-3 software manual unless noted.

**Target:** Intel Cyclone V FPGA, 100 MHz deliverable. **Verification:** Verilator
(`cpu_core_tb` = 103/103, `HS3_tb` = 84/84 — see §7). **Physical status:** Quartus
out-of-context (OOC) restricted Fmax **≈ 78–82 MHz across seeds (best fit −2.15 ns
/ 80.3 MHz, best 5-seed mean −2.39 ns)**
on the full SoC (§6); the gap to 100 MHz is a flat plateau of protected single-cycle
datapath loops (see §0 and §6), not the cache, bus, or peripherals — every OOC
critical path is CPU-internal, and none passes through the interrupt/exception
machinery.

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
skew. `o_CKIO` clocks the board SDRAM directly, in the **datasheet phase** (rises
at the command edges); the board delays the device clock for the tOD margin —
the TB gives its Micron model a half-cycle transport delay on the clock net
(see the BSC section).

### A note on the half-clock (`dclk`) lineage

Some source comments reference `cen_p`/`cen_n` and a "dclk half-clock." That is the
timing-R&D ancestor of the current design: a 2× (200 MHz) master with two
mutually-exclusive enables on alternating edges, so a `cen_n` register was a
*half-cycle* register (zero added latency) that gave the EX address path a clean
5 ns window. It reached its 5 ns goals (AGU, cache address capture) but converged on
the **same** architectural operand cone as the single-clock design (~78 MHz master →
~78 MHz arch). The single-clock **model-b** cache (§3) was then adopted as the
shipping structure. Read `cen_p`/`cen_n` in comments as design history, not two live
clocks — the RTL ports are `i_CLK` + `i_CEN` only.

### The hard rule: no bubbles, IPC first

The overriding design directive: **never regress IPC versus the SH7709S-faithful
baseline.** No inserted pipeline bubbles, no extra handshake/acknowledge beats. On
Cyclone V the goal is to *minimize* setup slack, not necessarily to reach perfect
closure — residual negative slack is accepted when the only alternative would cost a
cycle. Any new register must land on an *already-enabled* edge its consumers already
wait for. This is why the residual timing walls (§6) are left open rather than
pipelined away. Correctness terms discovered by the interrupt-collision suites
(§2, §7) are folded into existing cones under the same rule — as registered
qualifiers at final gate levels, never as new beats.

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
| `int_pipe.sv` | 3388 | the integer pipeline (IF/ID/EX/MA/WB, hazard/forward, GPR, MAC, MA sequencer) |
| `int_pipe_pkg.sv` | 848 | inter-stage packet typedefs (`ifid`/`idex`/`exma`/`mawb`), decode |
| `agu.sv` | 68 | the single time-shared address adder |
| `int_pipe_mem.sv`, `ctrl_reg.sv` | 102 / 111 | GPR M10K banks, control registers (SR/GBR/VBR/SSR/SPC) |
| `exc_handler.sv` | 345 | exception/interrupt entry + the P4 exception MMIO registers |

The stage registers are the packet structs; a **single global stall** (from the
cache, §3) freezes every stage register at once — there is no per-stage skid buffer,
FIFO, or outstanding-transaction tracking. Back-pressure *is* the freeze.

### Instruction fetch — 32-bit pair fetch, like the real chip

The IF stage fetches **one longword per bus access and executes both halfwords**
from it, matching the SH7709S's 32-bit instruction fetch (two 16-bit opcodes per
access, p.454). Mechanically:

- A fetch response for an **even** address carries the addressed opcode plus its
  odd **sibling** (`rsp_pair` / `rsp_inst_sib` on the L bus). The sibling parks in
  a one-entry **pair slot** in the pipe and feeds IF/ID on the next issue with *no
  bus request* — steady-state code makes one cache/bus access per two instructions,
  which is what frees data-side port slots (§3) and lifts memory-heavy IPC.
- The pair slot is kill-exact: a taken branch, external redirect, or WB fault kills
  the held sibling by the same terms that kill IF/ID (`pair_serve`/`pair_capture`
  fold every kill gate); a paired response never pairs across a fault.
- On the **non-cacheable** path the same longword economy holds: the bypass keeps a
  halfword-reuse buffer (`ibyp_buf_*` in `cache.sv`) that serves the sibling of a
  non-cacheable longword read without a second external transaction. The buffer
  mirrors memory — loaded only from fault-free reads, blanket-invalidated on any
  external write.
- A wrong-path outstanding fetch is marked by a sticky **drop flag** (`fetch_drop`)
  written at every kill/accept edge and consumed when the stale response arrives
  (consume-and-discard, so the single fetch slot can never wedge). The flag's
  update is an absorbing set/clear chain — once wrong-path, always wrong-path until
  consumed — which is what makes it safe under back-to-back kills (a branch kill
  followed by an interrupt entry while the same fetch is still outstanding).

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
- **Read-ahead address tail:** the RAM read addresses are computed one cycle ahead
  from the "next-ifid" cone (what IF/ID will hold during the read-data cycle). The
  three sources (held pair-slot sibling / live fetch response / current IF/ID) each
  precompute their own bank-qualified addresses, and the late serve/insert selects
  — which carry the whole pipeline-advance loop — cross exactly **one 3:1 mux
  level** at the M10K address pin, driven by merge-blocked `(* keep *)` select
  twins placed with the GPR cluster (the R3 tail late-select; a simulation
  assertion pins the composition to the reference shared-mux form every cycle).
- **Forwarding:** EX-result and MA-result forward into the ID/EX operand latches
  through per-port *registered lanes* patched at the head of EX (EX-head
  forwarding): every forward mux-select is a single FF and every data leg launches
  from a register. The select is pre-decoded at the IF/ID edge from the "next-ifid"
  cone.
- **Bank select:** the active GPR bank (`SR.MD & SR.RB`) is mirrored in a local
  registered copy (`r_bank1`) so the read-address cone launches from a flop, never
  from the cross-module live SR. The mirror snoops every event that can change the
  bank coherently with the read-address capture it must serve: a retiring
  `LDC ...,SR` (one-cycle lookahead off the WB packet), and an **RTE restore** with
  a two-phase arm — the RTE-in-WB cycle (lookahead, because the serialized RTE
  target's BRAM read is addressed one cycle before the restore commits) held
  through the commit-pulse cycle, both taking the bank from SSR. External redirects
  need no arm: they flush IF/ID and the mirror reconverges off live SR inside the
  shadow. A simulation assertion pins the mirror to the live SR under every live
  packet, with the RTE restore's legal two-cycle lead carved out.

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
they are zero-bubble). Byte/halfword loads carry the aligner/sign-extend cone; its
residual slack is accepted rather than paid for with a bubble (the no-bubbles rule).

### The MA sequencer — two-phase memory ops

Multi-access instructions (`MAC.W/L` second read, byte read-modify-write
`AND.B/OR.B/XOR.B/TST.B/TAS.B` write phase) are owned by a small MA-side sequencer
(`ma_seq`, bottom of `int_pipe.sv`). The EX primary access and the sequencer's
second access share one D request descriptor whose data fields select on the
*registered* phase bit (`second_access`) so the deep request-valid cone stays off
the address and store-data paths. The EX-side request valid is gated with the same
phase bit: the two phases can therefore never overlap on the bus — on the phase-two
completion cycle (when the pipeline's advance opens combinationally with the
response) the next instruction's request presents one cycle later, when the phase
bit has cleared. Any acceptance in the overlap cycle would have carried stale
phase-two fields, so the gate costs nothing legitimate. Locked pairs (`TAS.B` and
the GBR byte-RMW forms except `TST.B`) assert `req_lock` on both legs; the BSC
holds bus ownership across the pair (§4).

### Measured IPC (locked laws, verified on every run)

| Workload | Retires / cycles | IPC | Meaning |
|---|---|---|---|
| Straight-line NOPs, **non-cacheable** (P2 bypass) | 203 / 506 | **0.401** | front-end ceiling with pair fetch over the external bus |
| Dependent add loop, **cacheable hit** | 1137 / 1167 | **0.974** | ≈ 1.0; the 2.6 % gap is the 2 taken-branch bubbles per iteration |
| 100 %-store loop, **cacheable hit** | 415 / 748 | **0.554** | the unified-single-port ceiling, softened by pair fetch (a fetched longword covers two stores' issue slots) |

The core bench measures these ratios on every run; `HS3_tb` **asserts** them as
cycle-exact parity laws (the SoC fabric must add zero beats), so any structural
change that moves a cycle count fails the suite.

---

## 2. Interrupts and exceptions — precise acceptance

`exc_handler.sv` owns event selection and the exception register file
(TRA/EXPEVT/INTEVT/TEA as P4 MMIO through the cache's register-access fold, §3);
`ctrl_reg.sv` applies the SR/SSR/SPC updates through one arbiter (reset >
reset-like > entry > RTE restore > pipeline write-back). This is a bare-metal
SH7709S handler: memory faults map to CPU address errors; MMU/TLB vectors are
intentionally absent.

### Event priority and the same-edge yield

One architectural event enters per cycle: **general exception / TRAPA → RTE →
NMI → maskable interrupt**. The interrupt/NMI *acks* yield to a same-edge
synchronous event — the loser stays latched in the INTC and enters after the
handler, so a request is never consumed without its entry (`o_INT_ACK` implies an
entry, checked suite-wide). A general exception raised while `SR.BL=1` is a
**reset-like** event: manual-reset recovery with `EXPEVT=0x020` (§4.6,
p.100–101). NMI honors `BL` unless `ICR1.BLMSK` overrides it.

### The acceptance boundary — one exported invariant

The pipeline owns the whole "is this edge a legal acceptance point" invariant and
exports it as a single bit, `o_INT_BOUNDARY`:

- an instruction retired this edge (interrupts complete the current instruction);
- no **delayed-branch pair** is open — a retired branch whose slot is still owed
  defers acceptance (§4.5.3, p.98–100), so an interrupt can never split the pair;
- no **accepted data access or locked-RMW/MAC sequence** is in flight in MA —
  killing an accepted access would orphan its bus response (wedging the shared
  response channel) and killing between the legs of a locked pair would split an
  indivisible sequence. A *not-yet-granted* request stays killable: L-bus request
  withdrawal is legal, and stores commit at accept (notify semantics) so a killed
  re-executed store is idempotent.

### The restart PC — a commit-time register

The interrupt SPC (the PC the handler returns to) is an **architectural register
maintained at the commit point**, `arch_next_pc`, not a scan of the pipeline. At
each retirement it takes the retiring instruction's successor: the EX redirect
target for a taken non-delayed branch (`nd_taken` packet bit), the redirect target
at the *slot's* commit for a taken delayed pair (`pair_taken_q` + the
single-outstanding `rdir_target_q`, which in-order EX cannot re-arm before its
consumer commits), and `pc+2` otherwise. RTE flows through the same path (its
"target" is SPC). Acceptance is only legal on a retire edge, so the register is
always fresh at any boundary, and because interrupts never split a pair, a
mid-pair value is never consumed. `SPC` therefore never points at a delay slot,
never loses a taken branch, and is independent of the fetch frontier's state —
by construction rather than by per-case mux arms.

Synchronous events keep their own SPC rules: TRAPA saves `pc+2` (it retires),
delay-slot faults save the *branch* (`pc−2`, slot flag set, `EXPEVT=0x1A0` for
slot-illegal), and other faults save the faulting PC.

### Running-state mirrors

EX-stage consumers read SR bits from local running registers (`r_t`, `r_s`,
`r_m`, `r_q`) deposited as each producer leaves EX/MA and resynchronized to the
committed SR on redirects and on pipeline drain; the GPR bank mirror (`r_bank1`,
§1) follows the same doctrine with explicit LDC-commit and two-phase RTE-restore
snoop arms. The rule these mirrors implement: **every SR-derived select launches
from a register that is coherent with the committed SR at the edge its consumer
captures** — including across an RTE that returns to a different register bank or
mask level, which the SR-race and random-interrupt suites sweep exhaustively (§7).

---

## 3. Cache

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

Files: `cache.sv` (1344, the wrapper + FSM), `cache_mem.sv` (157, the M10K banks),
`cache_pkg.sv` (108, geometry + `tag_of`/`cacheable`/`lru_*`/`merge_word` helpers).

### The lookup model — folded into the pipe, no handshake

The single most important structural fact: **the cache lookup is folded into the
pipeline stage; a hit is *not* a request/response transaction.** IF *is* the I-side
access; MA *is* the D-side access. This is what makes IPC=1 reachable (a req/rsp
handshake structurally caps IPC at 0.5). The shipping implementation is **model-b**,
a two-beat overlapped lookup:

- **Beat 0 (request cycle):** the pipe presents the live AGU address. The **accept**
  (`req_ready`) is decided from registered state and the *previous* access's resolve
  (`z_ok`, below). At the edge the RAMs capture the read index — sourced from the
  pipe's private 12-bit index-slice twin `LBus.req_addr_idx`, not the shared adder
  (the R4 lever, §6) — and `bram_*` captures the request descriptor.
- **Beat 1 (resolve cycle):** tag/data `q` are compared against `bram_*`; a hit is
  answered **combinationally** and consumed by the pipe at the closing edge — the
  same edge the *next* access is captured. Back-to-back hits therefore run at one per
  cycle. A live hit response the pipe can't consume this cycle **retires into a
  registered `rsp_*` flag** (loss-free hold), so no response is ever overwritten.
- **Pair delivery:** an even I-side hit (or fill/bypass read) returns the addressed
  halfword *and* its sibling (`rsp_pair`); a hit never faults and a registered
  response excludes a same-side live hit, so the pair qualifier is a pure
  registered-flag product (§1, instruction fetch).

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
Pair fetch halves the fetch-side demand on the port (one access per two
instructions), which is where the 0.554 store-loop IPC comes from; the residual
ceiling is this port limit, and it is *correct* RISC behavior (adding a second
read port was explicitly rejected).

### Write policy, fills, MMIO fold

- **Per-region write policy** (p.105, 110–111): P1→`CCR.CB`, P0/U0/P3→`~CCR.WT`
  (`wb_mode`), with a `U` (dirty) bit in the tag. WB write hit = cache+U; WT write
  hit = cache+memory (a WT hit on a still-dirty line writes through its own word
  and neither drains nor cleans the resident dirty data — `CCR.CF` later discards
  it, a locked law); WB write miss = write-allocate; WT write miss = memory only.
- **Write-back buffer:** one line (p.111–112, Fig 5.5); line fills are 4 sequential
  longword reads, word-0-first. A mid-line fill fault invalidates the victim way
  (earlier beats already overwrote its words) rather than leaving a stale-valid
  hybrid line.
- **Store fast path:** a write-back store hit commits its strobed bytes at the
  resolve edge and retires in one MA cycle (a store is a *notify*, not an ack — the
  pipe retires it via `ma_complete`); the next access's read at that same edge gets
  the just-written bytes through `cache_mem`'s write-through bypass registers
  (write-before-read order; Cyclone V M10K has no silicon mixed-port new-data).
- **MMIO fold:** CCR/CCR2 and the exception registers (TRA/EXPEVT/INTEVT/TEA)
  are served as a *local-register access class* through the cache's shared `do_d`
  output flop, matched off the **latched** address — deliberately keeping MMIO decode
  off the AGU 5 ns fan-out. Writes are fire-and-forget; reads return a 1-cycle
  registered response.
- **Memory-mapped array windows** (p.112–114): tag `0xF0xx_xxxx`, data
  `0xF1xx_xxxx`; way in `A[13:12]`, set in `A[11:4]`, associative bit `A[3]`. Tag
  reads return `{tag, LRU, U, V}`. An **associative write is the purge
  primitive**: a tag match with `U=1` writes the line back, then invalidates
  (miss = silent no-op). A **non-associative tag write to a dirty entry drains it
  first** before installing the new tag/V/U/LRU — direct array writes cannot
  silently destroy dirty data. Data-array pokes are visible to subsequent cached
  hits. All of this is locked by a dedicated law suite (§7).
- **Non-cacheable bypass:** P2 (`101`) and P4 (`111`) are non-cacheable control
  spaces (`cacheable()` in `cache_pkg`); the reset PC lives in P2 (`0xA000_0000`).
  Locked accesses (`TAS.B` and friends) always bypass the array.
- **PREF** allocates through the normal D-fill path but is architecturally
  fire-and-forget: a faulting PREF fill is silently abandoned (no allocation, no
  exception), interrupt or not.

### Miss / external-bus FSM

Only a miss leaves the running state. The miss/refill/write-back/bypass/MMIO states
each last a full cycle (each state's RAM read lands one state later), and drive the
external I-bus (§4). Interrupt acceptance composes freely with every excursion —
including the 256-set `CCR.CF` flush walk and the memory-mapped array states — via
the boundary invariant of §2 (the collision suites sweep an acceptance edge across
each excursion type).

---

## 4. Bus structure

### On-chip tiers (SH7709S Fig 1.1, p.6)

The core follows the SH7709S internal bus hierarchy. **Latency law:** the on-chip
fabric is **zero-wait, fire-and-forget**; handshake latency is permitted *only* on
the bridge tiers (I bus 2, P bus) and the external BSC leg — and only if OOC shows a
bus register would cost less Fmax than the pipeline's own worst path (it never has:
zero bus/peripheral cone appears in any OOC top-20).

| Bus | `addr[31:29]` | Timing | What rides it |
|---|---|---|---|
| **L bus** | `111` (P4) | 1-cycle R/W, zero-wait | CPU-direct control regs (CCR, exception MMIO) |
| **I bus 1** | — | zero-wait to the BSC | two masters — the cache and the DMAC — merged by `ibus_arb` |
| **I bus 2** | — | handshake (bridge) | CPG/WDT, INTC register file (behind the BRIDGE) |
| **P bus** | P4 / area-1 | 2-cycle read (bridge) | TMU, RTC, I/O ports (in-BSC bridge) |

Fabric glue: `ibus_splitter.sv` (86, mux-only, zero beats, `owner_q` steers the
response), `ibus_bridge.sv` (159, IDLE→ACCESS→RESP, right-justified writes,
lane-replicated reads), `peri_bus_if.sv` (the slim register-bus interface), and
`ibus_arb.sv` (147) — the CPU/DMAC arbiter on I bus 1: one 2:1 mux with a
**registered owner select** parked on the CPU (the `reqn_*`/addr 5 ns class stays
one LUT level), flipping only at idle and response-done boundaries, never on an
accept edge; `i_DMA_HOLD` from the DMAC keeps a transfer unit (and a whole burst)
indivisible against the CPU. Zero added beats on the CPU leg — IPC parity is a
locked law.

**Bus contracts (checked suite-wide, §7):** an unaccepted D request re-presents
identical fields every cycle until accepted or withdrawn (withdrawal = a pipeline
kill, legal; mutation, never); an unconsumed D response re-presents identically
(loss-free retirement); an external MEM-bus request is never withdrawn or mutated
once presented; locked accesses alternate strictly read→write (one open pair).

### BSC — the external bus controller (`bsc.sv`, 1952 lines)

The BSC exposes the **real SH7709S chip pin set** (Table 10.1, PCMCIA-less) at the
`HS3` top: `A[25:0]`, a split data bus (`o_D_O`/`o_D_OE`/`i_D_I`, `inout` only at
board level), `BS_n`, `CS0/2–6_n`, `RD_WR`, `RAS3L/U_n`, `CASL/U_n`, `WE_n[3:0]`
(= DQM), `RD_n`, `i_WAIT_n`, `CKE`, `BREQ_n`/`BACK_n`, plus the pad-state
exports `o_A_PU`/`o_D_PU`/`o_RASCAS_OE`, `IRQOUT_n`, and the MCS0–7 selects riding
the port-C pads. Front-end route classes:

- **SDRAM engine** — MRS / single / burst / auto-refresh / self-refresh /
  bank-active with per-bank open rows, tWR guard, CL read pipe. Timing is driven
  exactly by the `MCR`/`WCR2` registers (CL/RCD/tRP), on the 50 MHz `i_BCEN` enable.
  Reproduces the natural SDRAM latency of the original board (most emulated code runs
  from SDRAM), §10.3.4 figs 10.14–10.28. **Full table-10.13 address multiplexing**:
  every AMX family decodes — row = `addr >> {8,9,10}` on `A16–A1`, the
  column phase holds the row values on a per-mode `A16–A13` mask (which keeps the
  bank bits standing on the device BA pins), the precharge flag rides the
  device-A10 pin (`A12`/`A11` by bus width), single-rail modes (`1101`/32-bit,
  `1110`/16-bit) never drive the U rails, and the open-row table compares
  device-meaningful row bits only. **BCR2 selects a 16-bit SDRAM bus width** per
  area: a line moves as 8 half-word beats on `A3:A1` (fig 10.15 text), singles
  split into halves on `D15–D0` under the DQMLU/DQMLL rails. **BS marks the Td
  data cycles** ("asserted in each of cycles Td1–Td4", p.283) via a CL-pipeline
  predictor — commands no longer carry it — and **single reads drive only their
  own DQM byte lanes** (p.276), which the Micron model honors lane-by-lane.
- **Ordinary / burst-ROM** — the full fig-10.6 state model: T1 + *n*·Tw + T2
  (first-access waits ∈ {0,1,2,3,4,6,8,10} from `WCR2`); `i_WAIT_n` decides the
  Tw→T2 transition, sampled mid-state when `WCR1.WAITSEL`=1 (fig 10.11) or at
  the boundary when 0 (a config silicon calls "not guaranteed"); burst-ROM
  continuation beats total the pitch-table states exactly and always sample
  WAIT; a write-back burst ignores the pin (p.274). `WCR1` inter-access idles
  (1–3) gate the **pin launch** on area switches and read→write turnaround —
  the generic-port handshake fast path never pays, which is what keeps the
  IPC-parity law beat-exact.
  **Strobe shapes follow the fig-23.16 AC shapes:** RD/WEn
  assert at **mid-T1** and negate at **mid-T2** (tRSD/tWED at the CKIO falls),
  CSn negates at mid-T2 (tCSD2), read data is sampled **at the mid-T2 fall** —
  tRDH1 = 0 ns lets the device release data the moment RD rises, so any later
  sample is unbuildable on real parts. A split-16 longword now runs as two full
  bus cycles with WE **rising between the halves** (an async device latches at
  the rise — the old held-WE envelope would have lost the first half on real
  silicon). **8-bit ports** complete the width matrix of tables
  10.7–10.12: a datum wider than the port walks its byte addresses low-to-high
  on `D7–D0` with WE0 only, one full bus cycle each, endian-mirrored register
  lanes (both endians decoded; bus arbitration never splits the multi-cycle
  walk, §10.3.8). **Line bursts (cache fill/drain, DMAC 16-byte units) are paced
  by the controller itself**, not by the per-beat bus calls — a one-outstanding
  call/return can never chain beats gap-free, so the head call opens a 4-beat
  envelope: reads prefetch into a line buffer that the calls drain, write calls
  queue ahead of the pins with posted acks (the 4th call completes with the
  envelope so a unit fault stays visible). Beats chain back-to-back with **zero
  idle states** (fig 11.11); on a BCR1 burst-ROM area, read beats 2–4 total the
  pitch-table states, always sample WAIT, and **CSn/RD-WR/DACK stay low across
  the whole run** with only A3–A0 stepping at the beat-launch posedges ("CS0 is
  not negated, only the address is changed", p.304; figs 10.29/10.30, 23.19/23.20)
  while RD re-pulses per beat mid-launch→mid-data-state; on plain areas every
  beat is a basic cycle re-framing CSn (fig 11.11), as are all burst WRITE beats
  (figs 10.29/10.30 notes), which additionally ignore the WAIT pin (p.304:
  16-byte DMA writes, single-address dev→mem, cache write-back). The envelope
  is indivisible against BREQ/refresh (silicon would split plain-area units at
  bus-cycle boundaries, §10.3.8 — HS3 holds the run; bounded and conservative).
  All ord pin edges sit **on the 20 ns bus grid**: launch on the `ord_run` grid
  flop, beat chains at the close boundary, release at the last T2-close
  (`!ord_done`), never at a core handshake edge; only a generic-port hsk *early*
  completion may advance/release off-grid (extension-path behavior, documented
  in the header).
- **Generic mirror port** — a zero-beat pass-through toward the surrounding SoC's own
  controllers (fabric SDRAM ctrl / HPS DDR3); this is the IPC-parity path. All data
  rides the physical D pins; the generic port is pure address/control.
- **Register / dummy** — the BSC's own POR-only register file (`0xFFFFFF50–74`).
- **Early-transaction sideband (`o_MON_*`)** — an advisory, observation-only strobe
  for integrators with their own fast memory path (built for ikacore CV1k): one
  registered pulse per committed external transaction *unit* at its internal accept
  edge (`mon_fire = engine-op start | ordinary accept`), carrying the physical
  address, direction, size, and burst flag one cycle before the pin sequence
  begins. It changes no cycle behavior and no arbitration (measured timing-neutral
  across a 5-seed sweep), and is proven by a whole-run match-queue oracle in the
  TB: every strobe is matched 1:1, in order, field-exact against the independently
  detected pin units (341,575 events; measured outstanding depth = 1). Full
  contract, measured lead tables, and timing diagrams live in the CV1k spec
  (`ikacore_CV1k/docs/sh3_sideband.md` §11).

Verified against a Micron MT48LC2M32B2 SDRAM model and a Macronix MX29LV320E NOR
flash model (patched vendor copies in `sim/models/`), including **boot-from-flash**
(16-bit two-sub-cycle fetches) and autoselect. Hard-won correctness points:
bus-ownership interlocks between the ordinary controller and the SDRAM engine;
self-refresh park must not count as bus-busy (else fetches deadlock); and
**TAS.B atomicity vs BREQ** — the bus must not be released between a locked pair's
read and write (p.320). Cache line fills present as `req_burst` on I bus 1.

The TB runs two **whole-run board-bus monitors** (checked as the final test):
a D-bus contention monitor (at most one driver among DUT / TB memories / flash /
SDRAM at any sample — this is what the WCR1 idles and strobe shapes must
guarantee) and a WE-shape monitor (address/data must hold from WE fall to WE
rise while an ordinary write owns the pins — the held-WE split-16 bug class
can never return). The TB's raw memories are strobe-honest: they drive D only
while `RD_n` is low and latch per `WE_n` lane, like the async parts in figs
10.7–10.9; the handshake-mode controller backs off the D bus while the chip
drives it (posted SDRAM engine writes share the pins).

**CKIO pin phase.** `o_CKIO` rises at the command edges, exactly as figs
10.14/23.16 draw it: every bus pin changes at the CKIO rise, and the
mid-state shapes (RD/WEn edges, WAITSEL=1 sampling) sit at the fall. A
synchronous device clocked straight from the pin would sample at the very
edge the pins change, so the board must grant the device the real chip's
tOD margin — on the FPGA an output-delay constraint / clock-tree skew on
the CKIO net (no PLL block needed). The TB models that board adjustment as
a half-cycle transport delay on the Micron model's clock net, so every
command lands in the same device cycle the real board would see.

**Register file.** POR values (BCR1 `H'0000`+ENDIAN, BCR2 `H'3FF0`, WCR1
`H'3FF3`, WCR2 `H'FFFF`, MCR/PCR/MCSCR/refresh group `0`), reserved-bit
masks (BCR2 `3FF0`, WCR1 `BFF3`, MCR `FFFE`, PCR `CFFF`, MCSCR `007F`),
`BCR1.ENDIAN` read-only reflecting the MD5 strap (**0 = big-endian**),
fig-10.5 write keys on the refresh group (word-only, `A5` / `101001`), RFCR
clearing when it exceeds the LMTS limit, and CMF's clear bound to the *next
performed CBR refresh* after the keyed write-0 (p.253). Locked by a
dedicated law suite (§7).

**MCS mask-ROM selects and pad behavior.** MCS0–7 mask-ROM selects (per
MCSCR0–7 / table 10.15: CS0-or-CS2 select, CAP-sized `A25:22` block
compare, CS-shaped assertion) ride the **port-C pads** through the PFC
"other function" mode — and, per p.323, MCS0 claims the CS0 pad itself when
MCSCR0 decodes area 0. **Bus-release pad behavior:** PULA pulls `A25–A0` up
for exactly 4 CKIO after `BACK` asserts (fig 10.41), PULD marks the D pins
whenever the data bus is idle (figs 10.42/10.43), and HIZCNT keeps the
RAS/CAS pads driven through a release (`o_RASCAS_OE`). The **IRQOUT pin**
(pp.320–321) asserts on a pending-not-yet-run refresh (BSC `o_REF_PEND`) or
an unmasked interrupt (`int_level > SR.I3–I0`, BL-independent, NMI always),
so a foreign master returns the bus. Known deviations from the full chip:
PCMCIA, and standby-mode pad states (the SoC has no standby mode; HIZMEM is
stored but unreachable).

### Cache↔BSC interaction latency

Core-cycle timing for a cache miss and the two non-cacheable (P2) accesses,
traced with a warmed loop running from cached SDRAM (auto-precharge mode,
CL2/RCD2, 50 MHz bus, bus otherwise idle). All numbers are core cycles at
100 MHz; `t` = the edge the D-lookup resolves (the tag read is `t−1` — "it first
takes a cycle to find the cache", SH7604 §7.11.2, and HS3 matches).

| event | cached load miss | bypass load (P2) | bypass store (P2) |
|---|---|---|---|
| miss/classify resolved; `CORE_I_BUS` req; splitter; BSC accept | `t` (one edge, zero beats) | `t` | `t` |
| first SDRAM command (ACTV) on pins | `t+1` | `t+1` | posted |
| data beats respond (CL2) | **missed word first** at `t+10`, wrap +2/beat | `t+11` | ack `t+1` |
| the load **retires** (fill-forward) | **`t+13`, any word offset** | `t+14` | `t+2` |
| cache returns to `S_IDLE` (fill tail in background) | `t+17` | `t+12` | (WRIT lands `~t+7`) |

This holds against SH7709S §5.3.2 (p.110), SH7604 §7.11.2/§8.4.3, and the
`attic/SH-master` SH7604 implementation:

1. **Fills wrap and forward from the missed word.** The BSC engine issues
   READ columns in wrap order (READA on the 4th command, fig 10.16), and the
   cache forwards the first beat to the pipe "in parallel with being loaded
   to the cache" (p.110) — miss retirement is offset-independent, and the
   fill tail drains in the background behind the resumed pipe. A fault on a
   later beat does not fault the forwarded access: it silently kills the
   line's validation, and the exception binds to whichever access later
   requests the faulting word itself (the victim-invalidate law is
   unaffected). The background drain takes a launch slot on hit-resolve
   edges, since an early-restarted hit stream otherwise has no request-free
   edge to use.
2. **Dispatch is one edge.** The first SDRAM command issues at the `E_IDLE`
   dispatch edge directly from the live op fields — accept→ACTV is one cycle
   for every SDRAM op.
3. **The front half is already optimal.** Miss determination, cache request,
   splitter, and BSC accept all land on **one edge** (the SH-master reference
   registers its request one cycle later). The latency law of §4 applies: no
   new beats here.
4. **Cache-off is not one cycle faster than a hit dispatch, and that is
   correct.** SH7604 §7.11.2: cache-through reads still pay "an extra cycle …
   to determine the cycle" before the internal-bus read starts. HS3's
   dispatch cost is identical for miss and bypass (both fire at `t`); the
   bypass is faster end-to-end only because it moves one beat instead of
   four.
5. **The posted bypass store matches the SH7604 "one-level write buffer"
   model** (ack at `t+1`, retire `t+2`, WRIT on pins ~`t+9`,
   fire-and-forget). The BSC is one-outstanding, so a *following* external
   access stalls until the posted write completes — same as the manual's
   "during reads, the CPU always has to wait."

Self-modifying-code note: the fill-forward window is tight enough that
`SELFMOD_K` only needs to cover IF/ID and the pair slot holding stale opcodes
(`SELFMOD_K = 2`).

---

## 5. Peripherals

All on-chip peripherals are timing-free in OOC (zero cones in any top-20); the CPU
remains the critical path.

| Module | File | Function |
|---|---|---|
| **CPG / WDT** | `cpg_wdt.sv` (286) | FRQCR/STBCR/STBCR2 clock-pulse generator; Pφ divider N∈{1,2,3,4,6}; watchdog timer with keyed `0x5A`/`0xA5` writes + reset stretcher; owns `o_BCEN` and the `o_CKIO` pin (B-φ = core/2, p.207 — FRQCR has no CKOEN, CKIO always drives in modes 0–2; datasheet phase: rises at the command edges). |
| **INTC** | `intc.sv` (455) | Full §6 interrupt controller: IRQ / IRL / IRLS / PINT / NMI, a 37-entry **2-stage registered priority resolver**, `INTEVT2`, `o_INT_ACK`/`o_NMI_ACK` (the acks latch/clear pending state — the core's ack-implies-entry law makes the handshake lossless). Interrupt inputs tap the I/O pads (below). |
| **TMU** | `tmu.sv` (262) | 3× 32-bit auto-reload down-counters; shared Pφ prescaler taps (P/4, /16, /64, /256); external TCLK clock (per CKEG, 2FF + edge detect); ch2 input capture (TCPR2, ICPF); underflow interrupts `TUNI0-2`/`TICPI2` → INTC (IPRA). |
| **I/O ports / PFC** | `ioport.sv` (236) | All 12 ports (A–L, SCP) as `pcr[]`/`pdr[]` arrays with per-port capability masks (drive/pull-up), PFC mode muxing (`MD1 ? pin : (DRV & DR)`), the PGCR PTG0 quirk (p.577), the `o_PC_FN` grant vector handing port-C pads to the BSC's MCS outputs. |
| **RTC** | `rtc.sv` (407) | §13, **two clock domains**: the `i_EXTAL2` 32.768 kHz oscillator (7-bit prescaler → RTCCLK 16.384 kHz + 256 Hz tap) and the bus domain (R64CNT, BCD calendar, alarms, periodic interrupt). CDC by tick-sync + no-reset toggles. Counters/alarms never pin-reset (Table 13.2). Feeds TMU `i_RTCCLK`/`i_RTC_TICK`. |
| **DMAC + CMT** | `dmac.sv` (764) + `dmac_channel.sv` (195) | Full §11 4-channel DMA controller, a second I-bus-1 master that "calls the BSC like a function" (bus cycles shaped exactly as CPU accesses, p.363). Request sources: auto, the on-chip CMT (Pφ/4-64 compare-match timer, §11.4), external DREQ0/1 (CKIO-falling-edge sampling, DS level/edge, DRAK grant pulses). Units: dual-direct R→W, ch3 dual-indirect (LONG pointer fetch prologue), single-address (DACK-framed one-cycle transfers, `o_D_OE` held off for device-drive writes), byte/word/long/16-byte (4-longword gather/play with a 4×32 buffer). Fixed + round-robin priority (a single 2-bit rotation head — the p.350 rule provably keeps the order a pure rotation), re-resolved every unit boundary. ch2 source reload every 4 transfers. Aborts: NMIF from the INTC's qualified NMI edge, AE from grant-time alignment checks + in-flight bus faults; both halt all channels with TE unset. `DEI0-3` → INTC (0x800–0x860). DACK windows are CSn-framed **inside the BSC** (active-high sideband strobes; AL/RL pad polarity applied in the DMAC). 16-byte unit beats carry `req_burst`, so the BSC chains them as one gap-free run (fig 11.11; one CSn/DACK envelope on burst-ROM areas, fig 23.19; SDRAM engine line ops) and honors the p.304/§11.6-note-12 WAIT-ignore on the write runs. Deviations noted in the header: DREQ sampled every CKIO fall (not the 2-cycle one-step-ahead cadence), SDRAM-area DACK/single-address not wired. |

**Real-chip pin sharing (Table 18.1):** the dedicated `i_IRQ`/`i_IRLS`/`i_PINT`
inputs are **deleted** — interrupt sources ride the port pads
(`i_IRQ = {SCPT7, PTH4-0}`, `i_IRLS = PTF3-0`, `i_PINT = {PTF, PTC}`); only NMI stays
dedicated. `PTF` is shared PINT8-15/IRLS3-0, and `PTH7` mode-00 hands its pad to the
TMU TCLK. The DMAC pins ride Port D the same way: DREQ0/1 tap `PTD4`/`PTD6` as
inputs, DACK0/1 drive `PTD5`/`PTD7` and DRAK0/1 drive `PTD1`/`PTD0` (note the DRAK
swap, Table 18.1) when PDCR grants mode 00. This matches the SH7709S philosophy
that peripheral function and GPIO multiplex on the same physical pins.

---

## 6. Timing summary — where the Fmax goes

Full-SoC OOC (Quartus 17, Cyclone V `5CSEBA6U23I7`, AGGRESSIVE PERFORMANCE,
`set_max_delay` false-paths on the M10K RDW arcs), measured across five seeds on the
final tree:

| Seed | Worst multicorner slack @ 10 ns | Restricted Fmax | Top-20 headline class |
|---|---|---|---|
| dse | −2.68 ns | 78.9 MHz | mixed plateau (advance front, exc-MMIO compare legs) |
| 1 | −2.22 ns | 81.8 MHz | Wall A data leg residue |
| 4 | **−2.15 ns** | **80.3 MHz** | AGU carry → exc-MMIO compares → `o_TEA\|ena` — best HS3 fit recorded |
| 5 | −2.42 ns | 80.2 MHz | exc-MMIO classification → cache dispatch (`state.S_IDLE`) |
| 7 | −2.51 ns | 79.6 MHz | advance front → pair-slot capture |

(Measured 2026-07-18 on branch `sideband` — the complete SoC including the full
DMAC, the **R3 read-ahead tail late-select** (§1), the CV1k early-transaction
sideband (`o_MON_*`, §5 BSC — measured timing-neutral, zero `mon_` cells in any
top-20), and the **R4 cache-index slice twin**: a private 12-bit copy of the whole
AGU cone on `(* preserve *)` select duplicates whose sole consumer is the cache
RAM read index (`LBus.req_addr_idx`), letting the fitter place the slice at the
RAM block. R4 is the best 5-seed result recorded: mean worst slack **−2.39 ns**
vs −2.69 post-R3, every seed improved vs its same-seed anchor, and the old
AGU→RAM-index class (`byp_q`/tag/data read address) fell to 0/20 paths on 4 of 5
seeds. The promoted headline is the **exc-MMIO live classification** family:
adder carry → the shared TRA/EXPEVT/INTEVT/TEA/CCR compares → exception-register
write enables and cache dispatch.)

> **Read this as a plateau, not a ranking.** The design sits on a *flat cluster* of
> single-cycle protected loops; each seed's placement picks a different one as the
> headline, and per-seed coarse Fmax moves by more than real structural changes do
> (identical-RTL fits of the Wall B/FSM cone have swung −2.2..−3.9). Judge any
> change by worst-slack trend *and cone composition across seeds*, never by one
> fit. The 100 MHz deliverable corresponds to worst slack ≥ 0; the measured gap is
> ~2.2–3.2 ns of mostly interconnect (55–65 % of every failing path is routing).
> Post-R4 the plateau is also measured **perturbation-chaotic**: two later small
> levers (R5, R6′) each shifted the 5-seed *mean* by −0.4..−0.7 ns with their own
> logic in zero worst paths, while an identical-RTL re-anchor moved <0.1 — any
> netlist change re-rolls global placement and taxes the saturated issue-enable
> fabric more than a small cone removal buys. That is why the catalog closes here.

The recurring cone classes, all protected by the no-bubbles rule (each is a
single-cycle loop that cannot take a register without costing an architectural
cycle):

1. **Advance loop / request front** — `{mawb.gpr0_data, fwd_*_agu,
   second_access_agu}` → AGU adder → request valid/accept → `idex_allow`
   (fanout ~350) → capture. The oldest and deepest family. R3 cut its deepest
   tail (the GPR read-ahead address, see §1); the residue captures at the
   pair-slot clock enables (a 1-bit CE cone — no late-select applies), the cache
   FSM next-state, and the write-through bypass registers (`byp_q`).
2. **Wall A — I-response → predecode** — cache I-side response formation
   (`bram_addr` → RAM `q` → way/word select → `rsp_inst`) → predecode → the GPR
   read-ahead capture. R3 removed the shared mux tail it used to cross; the
   residue is the live-response data leg itself, which no restructure can
   register without a fetch bubble.
3. **Operand → EX flags** — forward-lane selects → operand mux → the EX adder
   carry chain → T/compare select tree → `exma.t_data` / `r_t`. ~9 levels, ~60 %
   interconnect; a placement-spread cone rather than a logic-depth one.
4. **Exc-MMIO live classification** (promoted by R4) — AGU carry → the shared
   `req_addr == TRA/EXPEVT/INTEVT/TEA/CCR` compares → `o_TEA`/`o_EXPEVT` write
   enables and the cache dispatch arm. Both request-side transforms are measured
   dead ends: a case sub-decode is a provable no-op (the compares are shared with
   the WE gate), and the full sum-addressed carry-free rewrite (R5) fitted
   *worse* — six full-width operand words fan into the compare cluster where one
   sum routed before, and six soft-LUT levels lose to the hard carry chain
   (reverted; anatomy in the campaign log). The unspent idea is consumer-side:
   flattening the enable's priority tail in `exc_handler`.

The interrupt/exception machinery (§2) contributes **no logic to any failing
path**: the acceptance boundary, restart-PC register, bank/flag mirrors, and MA
phase gate are all registered-launch, registered-capture structures placed off the
walls — confirmed by name-search over every top-20 path across seeds.

**The RTL lever catalog is exhausted** (R4 was the last kept entry and R5 the
last measured attempt; the full round history lives in
`eval_ooc/cache_wall_campaign.md`). What remains toward 100 MHz is physical, not
structural: a LogicLock floorplan pinning the AGU/request cluster next to the
cache banks (unlicensed in the current docker Quartus flow), a C6 speed grade, or
wide seed/DSE harvesting (seed 4 shows what a lucky placement yields). Explicitly
**not** on the table: an operand- or address-capture pipeline beat — that would
break cycle accuracy (the no-bubbles rule, §0).

*Flow note:* the OOC flow (`eval_ooc/tools/quartus_ooc.py all <config>`) reuses the
run directory named in the config; the STA/summary regenerate on every run but the
`probe_*.rpt` cone probes do **not** — check file mtimes before reading probes
against a fresh fit.

---

## 7. Verification

Two self-checking Verilator benches gate every change; both must pass bit-exact,
and the three IPC laws (§1) are asserted values, not observations.

| Bench | Scope | Tests |
|---|---|---|
| `cpu_core_tb` | core only (`src/cpu_core`), bus modeled in the tb | **103** |
| `HS3_tb` | full SoC on the real pin set, vendor SDRAM/flash models | **84** |

(Test 84 is the sideband match-queue oracle — a whole-run passive checker that
holds every `o_MON_*` strobe against the independently detected external unit
starts: 1:1, in order, fields exact, reset-flushed; see §5 BSC.)

The suite is built in four layers:

1. **Directed goldens** — every ISA class, addressing mode, exception cause,
   hazard/interlock, cache law (fills, write policies, per-beat fill faults,
   victim drains, flush/CE semantics, LRU thrash, self-modification, the
   memory-mapped tag/data window matrix, WT-flip on dirty lines), fetch-pair laws,
   and the BSC's device-level behaviors (boot-from-flash, refresh, self-refresh,
   `BREQ` arbitration, `i_WAIT_n`). The DMAC's laws lock its cycle behavior the
   same way: DREQ→grant latency, DME→TE durations per bus mode, DACK⊆CS window
   containment, round-robin grant order (a tb write-order log turns each DMA
   write beat's destination page into a hex-literal grant sequence), NMI/AE
   abort-and-resume protocols, and BREQ splitting a dual-address pair at the
   bus-cycle tier with intact results.
2. **Collision sweeps** — an interrupt (and separately an NMI) is swept cycle-by-
   cycle across every machinery excursion: plain execution, miss/fill/drain walks,
   locked `TAS.B` pairs, the `CCR.CF` flush walk, memory-mapped array accesses,
   faulting and clean PREF fills, and two-event collisions (each synchronous
   exception flavor — illegal, slot-illegal, address error, TRAPA, D-fill bus
   fault, I-fill fetch fault — colliding with a pending interrupt at every
   offset). Laws at every offset: exactly one entry per event, `EXPEVT`/`INTEVT`
   never mix, the interrupted computation is transparent, `SPC` is never a delay
   slot, and no request is lost (an ack without an entry fails). SR-write races
   (`LDC ...,SR` flipping IMASK/BL/RB at the acceptance edge) and nested-enable
   re-entry (BL cleared inside a handler with the level still held) are swept the
   same way. `HS3_tb` carries SoC twins of the collision sweeps over the real
   INTC/IRL pin protocol.
3. **Suite-wide passive checkers** — always-on monitors that fail the run from any
   test: an independent true-LRU mirror cross-checked at every LRU write and
   victim choice, the L-bus/MEM-bus stability contracts (§4), locked-pair
   alternation, ack-implies-entry with `INTEVT` settlement, and acceptance
   **coverage histograms** (entries per cache FSM state, restart-PC source arm)
   asserted non-empty so the sweeps provably reach the deep states.
4. **Random oracles** — constrained-random programs (ALU, R14-window loads/stores,
   byte stores, `TAS.B`, `PREF`, plain and delayed conditional branches over live
   T) run once as their own reference and re-run under (a) random I/D wait states
   and (b) random INT/NMI waves at random offsets in three SR flavors (privileged
   RB=1, privileged RB=0, user mode — the latter two flip the register bank on
   every entry/RTE). The architectural end state must be identical and the handler
   count must equal the wave count. This layer subsumes the hand-built offset
   enumeration and is the strongest regression net in the suite. The DMAC adds a
   randomized legal-config differential: 12 drawn configurations (channel, size
   incl. 16-byte, inc/dec/fixed walks, bus mode, count) each run against a
   tb-side golden model under a randomly drawn bus latency — the golden model
   never sees the latency, so passing rounds are the latency-invariance oracle.

Testbench conventions worth knowing (they encode real pitfalls): the `AFFE` guard
word is a delayed branch, so the word after it must stay a NOP; GPRs persist across
the tb's reset, so programs zero their own *active-bank* registers and any detector
registers; interrupt-visible state is initialized *before* the BL-clearing `LDC`
(a wave accepted at the init's own boundary would re-execute the init over the
handler's counts); and `gpr()` sampling at a retire marker races the pipeline's
~7-word lookahead, so multi-step checks use write-once result registers read after
the sentinel.

---

## References

- `SH7709S_Hardware_Manual[REJ09B0081-0500O].pdf` — cache (§5, p.103–114), BSC
  (§10), INTC (§6), exceptions (§4, p.85–101), TMU, RTC (§13), ports (§18), CPG
  (p.207–212).
- `SH-3_SH-3E_SH3-DSP_Software_Manual.pdf` — pipeline timing (Fig 10.40/10.41
  p.476), branch/PR rules (§10.2.3 p.432), interrupt/pair semantics (§4.5.3).
- `SH-1_SH-2_Programming_Manual.pdf` — 5-stage pipeline baseline.
- `SH7604_Hardware_Manual[ADE-602-085C].pdf` — cache operation reference.
