# HS3 — Core Hardware Description

This document gives the microarchitecture and the timing of the HS3 core. HS3 is
a SystemVerilog SH-3 (SH7709S) core. The document tells *what the RTL does* and
*why the RTL has this shape for timing*. It is the design-intent companion to the
vendor PDFs in this folder. Manual page references (`p.NNN`) refer to the SH7709S
hardware manual or to the SH-3 software manual, unless a different manual is given.

**Target:** Intel Cyclone V FPGA, 100 MHz deliverable. **Verification:** Verilator
(`cpu_core_tb` = 103/103, `HS3_tb` = 84/84, refer to §7). **Physical status:** The
Quartus out-of-context (OOC) restricted Fmax is **approximately 78–82 MHz across
seeds (best fit −2.15 ns / 80.3 MHz, best 5-seed mean −2.39 ns)** on the full SoC
(§6).

The gap to 100 MHz is a flat plateau of protected single-cycle datapath loops (refer
to §0 and §6). The cache, the buses, and the peripherals do not cause the gap. All
OOC critical paths are internal to the CPU. No critical path goes through the
interrupt and exception logic.

---

## 0. Clocking, reset, and the timing philosophy

### Clock scheme

The core uses **one clock**: one architectural clock `i_CLK` and one architectural
clock enable `i_CEN`. The core has **no clock gating**. Clock gating is an ASIC
technique, and Cyclone V does not support it. Each register is a synchronous DFF
with a clock enable (`(* direct_enable *) wire cen = i_CEN;` in `int_pipe.sv`).

The testbenches set `i_CEN = 1` and use a 10 ns period (100 MHz). One `i_CEN` edge
is one architectural cycle.

| Domain | Clock | Rate | Owner |
|---|---|---|---|
| CPU, cache, and on-chip fabric | `i_CLK`/`i_CEN` | 100 MHz | full core |
| SDRAM engine and refresh (PCB side) | `i_CLK` + `i_BUS_PCEN` | 50 MHz enable (B-φ = core/2, p.207) | BSC |
| Bus clock output pin | `o_CKIO` | B-φ phase register, clocked by the two bus enables | CPG |
| Bus grid (exported) | `o_BUS_PCEN` / `o_BUS_NCEN` | CKIO rise / CKIO fall enables. This is the ONLY phase source | CPG |
| RTC oscillator | `i_EXTAL2` | 32.768 kHz, **fully asynchronous** | RTC only |

The **RTC is the only real second clock domain.** All RTC signals that go to the CPU
cross into the `i_CLK` domain through tick synchronizers and toggle flags without
reset. The manual gives a command latency of approximately 91.6 µs (p.421). This
latency absorbs the CDC skew.

`o_CKIO` is the clock of the board SDRAM. It has the **datasheet phase**: it rises at
the command edges. The board delays the device clock to get the tOD margin. The TB
models this delay as a half-cycle transport delay on the clock net of the Micron
model (refer to the BSC section).

### Note on the half-clock (`dclk`) design

Some source comments refer to `cen_p`/`cen_n` and to a "dclk half-clock." This was
an earlier timing research design. It used a 2× (200 MHz) master clock with two
enables on alternate edges. Thus a `cen_n` register was a *half-cycle* register with
no added latency, and it gave the EX address path a 5 ns window.

That design got its 5 ns goals (AGU and cache address capture). But it had the
**same** architectural operand cone as the one-clock design (approximately 78 MHz
master → 78 MHz architectural). Thus the project uses the one-clock **model-b**
cache (§3).

Read `cen_p`/`cen_n` in comments as design history. They are not two live clocks.
The RTL ports are only `i_CLK` and `i_CEN`.

### The primary rule: no bubbles, IPC first

The primary design rule is: **do not make the IPC lower than the SH7709S-faithful
baseline.** Do not add pipeline bubbles. Do not add handshake or acknowledge beats.

On Cyclone V, the goal is to make the setup slack as small as possible. Full closure
is not always necessary. The design accepts some negative slack when the only other
solution adds a cycle. A new register must be on an edge that is *already enabled*
and that its consumers already wait for. For this reason, the design does not add
pipeline stages to the remaining timing walls (§6).

The interrupt-collision suites (§2, §7) found some correctness terms. The design
adds these terms to the existing cones with the same rule. Each term is a registered
qualifier at a final gate level. No term adds a beat.

---

## 1. Pipeline

### Stage structure

The core uses the usual SH 5-stage in-order pipeline. The baseline is SH-1/SH-2, with
the SH-3 extensions:

```
  IF  ──▶  ID  ──▶  EX  ──▶  MA  ──▶  WB
 fetch   decode   execute  memory   write-back
        +regread  +AGU     access
```

| File | Lines | Function |
|---|---|---|
| `int_pipe.sv` | 3388 | the integer pipeline (IF/ID/EX/MA/WB, hazard/forward, GPR, MAC, MA sequencer) |
| `int_pipe_pkg.sv` | 848 | packet typedefs between stages (`ifid`/`idex`/`exma`/`mawb`), decode |
| `agu.sv` | 68 | the one time-shared address adder |
| `int_pipe_mem.sv`, `ctrl_reg.sv` | 102 / 111 | GPR M10K banks, control registers (SR/GBR/VBR/SSR/SPC) |
| `exc_handler.sv` | 345 | exception and interrupt entry, and the P4 exception MMIO registers |

The stage registers are the packet structs. The cache (§3) supplies a **single global
stall**. This stall stops all stage registers at the same time. The pipeline has no
skid buffer for each stage, no FIFO, and no tracking of outstanding transactions.
The stall *is* the back-pressure.

### Instruction fetch — 32-bit pair fetch, as on the real chip

The IF stage fetches **one longword for each bus access and executes the two
halfwords** in it. The SH7709S also has a 32-bit instruction fetch (two 16-bit
opcodes for each access, p.454). The mechanism is as follows:

- A fetch response for an **even** address contains the addressed opcode and its odd
  **sibling** (`rsp_pair` / `rsp_inst_sib` on the L bus). The sibling goes into a
  one-entry **pair slot** in the pipe. At the next issue, the pair slot supplies the
  sibling to IF/ID *without a bus request*. In steady state, the code makes one
  cache or bus access for each two instructions. This gives more data-side port
  slots (§3) and increases the IPC of memory-heavy code.
- The pair slot is kill-exact. A taken branch, an external redirect, or a WB fault
  kills the sibling in the slot. The kill terms are the same terms that kill IF/ID
  (`pair_serve`/`pair_capture` include each kill gate). A response pair is never
  made across a fault.
- The **non-cacheable** path has the same longword economy. The bypass has a
  halfword reuse buffer (`ibyp_buf_*` in `cache.sv`). This buffer supplies the
  sibling of a non-cacheable longword read without a second external transaction.
  The buffer is a copy of memory. Only fault-free reads load it, and each external
  write invalidates all of it.
- The external request always has the **longword-aligned** address (A1:A0 = 00).
  This is also true for a fetch at an odd halfword PC, as on the real chip. The pipe
  selects the odd opcode from the response.
- A sticky **drop flag** (`fetch_drop`) marks an outstanding fetch on the wrong path.
  Each kill or accept edge writes the flag. The stale response clears it when it
  arrives (the response is consumed and discarded). Thus the single fetch slot can
  never lock. The flag uses an absorbing set/clear chain: after it is set, it stays
  set until the stale response clears it. This makes it safe for back-to-back
  kills, for example a branch kill and then an interrupt entry while the same fetch
  is outstanding.

### Address generation — one adder, no mux

**One** 32-bit AGU adder makes all addresses. These are the sequential PC+2, the
PC-relative branch targets, and the data effective addresses. The adder is time
shared:

```
  o_ADDR = X + (en · Y) + ci        // agu.sv
    X  = i_USE_BASE ? agu_base : fetch_pc
    en = {NULL, FORCE, r_t, ~r_t}[en_mode]   // T is part of the addend LUT
    ci = +2 for the PC+2 case
```

The timing limit sets this shape. On Cyclone V, a half-clock of approximately 5 ns
is *one LUT stage and the carry chain*. Thus **no discrete mux can be before or
after the adder on this path**. Each source or operand selection is *in the LUTs of
the adder* (the `en`-null method, with operands that ID selects before).

The conditional branch uses this shape. `BT`/`BF` set `use_base = en = r_t`. Thus
the *same* AGU configuration gives the branch target when the branch is taken
(`T=1`), and `fetch_pc+2` when it is not taken. No redirect mux is necessary. ID
calculates the PC-relative targets before EX (`idex.immediate = pc+4+disp`). The
OOC result of the AGU is +0.079 ns at 5 ns (203 MHz).

### Forwarding and the register file

- **GPR = 2R2W.** It has 4× 32×32 simple dual-port M10K banks (`{wb0,wb1} ×
  {r0,r1}`) and a 32×1 flip-flop LVT (live-value table). A registered LVT select
  is one 2:1 mux after the RAM read. One-cycle WB shadow lanes (`wb0z`/`wb1z`) in
  the early operand legs close the write/read hazards on the same edge.
- **Read-ahead address tail:** The pipe calculates the RAM read addresses one cycle
  before the read. It uses the "next-ifid" cone, which is the value that IF/ID holds
  in the read-data cycle. There are three sources: the sibling in the pair slot, the
  live fetch response, and the current IF/ID. Each source calculates its own
  bank-qualified addresses before the select.
  The late serve and insert selects carry the full pipeline-advance loop. They go
  through only **one 3:1 mux level** at the M10K address pin. Merge-blocked
  `(* keep *)` select twins drive this mux, and the fitter puts them near the GPR
  cluster (the R3 tail late-select). A simulation assertion compares this structure
  with the reference shared-mux form in each cycle.
- **Forwarding:** The EX result and the MA result go into the ID/EX operand latches
  through *registered lanes* for each port. The pipe patches these lanes at the
  head of EX (EX-head forwarding). Each forward mux select is one FF, and each data
  leg starts from a register. The pipe decodes the select at the IF/ID edge from the
  "next-ifid" cone.
- **Bank select:** A local registered copy (`r_bank1`) holds the active GPR bank
  (`SR.MD & SR.RB`). Thus the read-address cone starts from a flop, not from the
  live SR in a different module. The copy monitors each event that can change the
  bank. It stays coherent with the read-address capture that it serves.
  - A retiring `LDC ...,SR` updates it, with a one-cycle lookahead from the WB
    packet.
  - An **RTE restore** updates it with a two-phase arm. The first phase is the cycle
    when RTE is in WB. This is a lookahead, because the BRAM read for the RTE target
    occurs one cycle before the restore commits. The arm stays on through the
    commit-pulse cycle. The two phases take the bank from SSR.
  - External redirects do not need an arm. They flush IF/ID, and the copy gets the
    live SR value again in the shadow.

  A simulation assertion compares the copy with the live SR for each live packet.
  The assertion accepts the legal two-cycle lead of the RTE restore.

### Control-flow timing (cycle law, give the reference)

| Case | Penalty | Reference |
|---|---|---|
| Non-delayed `BT`/`BF`, **taken** | **2 fetch bubbles** (3 cycles) | SW manual Fig 10.40, p.476; table 2/1, 3/1 |
| Non-delayed `BT`/`BF`, **not taken** | 0 | Fig 10.41 |
| Delayed `BT/S`/`BF/S`, **taken** | **1 bubble** (2 cycles; the slot fills the other cycle) | Fig 10.42, p.478 |
| `BRA/BSR/JMP/JSR/RTS/RTE/BRAF/BSRF` | **1 bubble** (2 cycles) | Fig 10.44/10.45 |
| Load in the delay slot of a taken branch | +1 (its MA uses the L bus that the target fetch needs) | §10.2.1 IF/MA contention |

These penalties agree with a real SH7709S to the clock. The test is the CV1000 CPU
CACHE HIT benchmark (`sim/cpu_core_tb.sv +cv1k`). The 4-instruction dependent chase
uses 6.0 clocks for each step, on the PCB and in HS3.

**Early target fetch** (int_pipe, "EARLY TARGET FETCH"): The IF of HS3 is pipelined
over 2 cycles (request, then response into IF/ID). If the target request starts
after the redirect, there are 4 slots from the redirect to EX. The chip has 3 slots.
Thus a taken branch in EX puts its target on the L bus in its own EX cycle.

The target uses the time-shared AGU, as a PC-relative load address does. ID loads
the base and the addend:

- The base is 0 for PC-relative forms. It is Rn with the EX-head forward lanes for
  `JMP/JSR/BRAF/BSRF`. It is PR or SPC for `RTS/RTE`.
- The addend is the full target or pc+4.

Thus no new mux level is on the cache address cone. Only the `i_USE_BASE` select of
the AGU gets a shallow term (the registered branch class and the running `r_t`).

The pipe tags the request (`tgt_pend`). The redirect keeps the request and does not
drop it. The cache holds the response until the redirect edge. `fetch_pc` is one
halfword behind the fetched target (`fpc_lag`, in bit 1 of the addend leg of the
AGU). Thus no `target + 2` adder is on the branch-target path.

A delayed branch starts the early fetch only when its slot is in IF/ID. The pipe
uses the late path in two cases: a speculation across a byte-RMW evaluate slot
(stale `r_t`), and a speculation over a changed AGU base. `RTS` reads PR through the
ID control-register interlock. No new PR forward path is necessary.

### Load-use

A cache **load hit adds no stall cycles**. Only a 1-slot load-use interlock remains.
`MOV.L` does not have this interlock, because longword loads do not need the aligner.
Thus `MOV.L` has no bubble. Byte and halfword loads go through the aligner and
sign-extend cone. The design accepts the remaining slack of this cone, because the
only other solution is a bubble (the no-bubbles rule).

### The MA sequencer — two-phase memory operations

Some instructions make more than one access: the second read of `MAC.W/L`, and the
write phase of the byte read-modify-write instructions
`AND.B/OR.B/XOR.B/TST.B/TAS.B`. A small sequencer in MA controls these accesses
(`ma_seq`, at the bottom of `int_pipe.sv`).

The EX primary access and the second access of the sequencer use one D request
descriptor. The data fields of this descriptor select on the *registered* phase bit
(`second_access`). Thus the deep request-valid cone is not on the address and
store-data paths.

The same phase bit gates the EX-side request valid. Thus the two phases never
overlap on the bus. In the completion cycle of phase two, the pipeline advance opens
combinationally with the response. The request of the next instruction starts one
cycle later, when the phase bit is clear. An accept in the overlap cycle would have
stale phase-two fields. Thus the gate does not stop a legal accept.

Locked pairs (`TAS.B` and the GBR byte-RMW forms, but not `TST.B`) set `req_lock`
on the two legs. The BSC keeps bus ownership across the pair (§4).

### Measured IPC (locked laws, verified in each run)

| Workload | Retires / cycles | IPC | Meaning |
|---|---|---|---|
| Straight-line NOPs, **non-cacheable** (P2 bypass) | 203 / 506 | **0.401** | front-end limit with pair fetch over the external bus |
| Dependent add loop, **cacheable hit** | 1137 / 1157 | **0.982** | approximately 1.0; the gap is the taken `BF` (2 bubbles) in each iteration |
| 100 % store loop, **cacheable hit** | 415 / 736 | **0.564** | the limit of the unified single port; pair fetch makes it better (one fetched longword covers the issue slots of two stores) |

The core bench measures these ratios in each run. `HS3_tb` **asserts** them as
cycle-exact parity laws. (The SoC fabric must not add beats.) Thus each structural
change that moves a cycle count makes the suite fail.

---

## 2. Interrupts and exceptions — precise acceptance

`exc_handler.sv` selects the event and holds the exception register file
(TRA/EXPEVT/INTEVT/TEA as P4 MMIO, through the register-access path of the cache,
§3). `ctrl_reg.sv` applies the SR/SSR/SPC updates through one arbiter. The priority
is: reset > reset-like > entry > RTE restore > pipeline write-back.

This is a bare-metal SH7709S handler. Memory faults go to CPU address errors. The
design does not have MMU/TLB vectors.

### Event priority and the same-edge yield

One architectural event enters in each cycle. The priority is: **general exception /
TRAPA → RTE → NMI → maskable interrupt**.

The interrupt and NMI *acks* yield to a synchronous event on the same edge. The
event that loses stays latched in the INTC, and it enters after the handler. Thus
the core never consumes a request without its entry. (`o_INT_ACK` always has an
entry. The full suite checks this.)

A general exception when `SR.BL=1` is a **reset-like** event. The core does a
manual-reset recovery with `EXPEVT=0x020` (§4.6, p.100–101). NMI obeys `BL`, unless
`ICR1.BLMSK` overrides it.

### The acceptance boundary — one exported invariant

The pipeline owns the full "this edge is a legal acceptance point" invariant. It
exports the invariant as one bit, `o_INT_BOUNDARY`. The bit is set when all these
conditions are true:

- An instruction retired on this edge. (An interrupt lets the current instruction
  complete.)
- No **delayed-branch pair** is open. A retired branch with an outstanding slot
  delays acceptance (§4.5.3, p.98–100). Thus an interrupt can never split the pair.
- No **accepted data access** and no **locked-RMW/MAC sequence** is in flight in MA.
  A kill of an accepted access would orphan its bus response, and this would lock
  the shared response channel. A kill between the legs of a locked pair would split
  a sequence that must stay complete.

A request that is *not yet granted* can be killed. A withdrawal of an L-bus request
is legal. Stores commit at accept (notify semantics). Thus, if the core kills a store
and executes it again, the result is the same.

### The restart PC — a commit-time register

The interrupt SPC is the PC to which the handler returns. It is an **architectural
register that the commit point updates** (`arch_next_pc`). It is not a scan of the
pipeline. At each retirement, the register gets the successor of the retiring
instruction:

- For a taken non-delayed branch, it gets the EX redirect target (`nd_taken` packet
  bit).
- For a taken delayed pair, it gets the redirect target at the commit of the *slot*
  (`pair_taken_q` and the single outstanding `rdir_target_q`). In-order EX cannot arm
  this register again before its consumer commits.
- For all other instructions, it gets `pc+2`.

RTE uses the same path (its "target" is SPC). Acceptance is legal only on a retire
edge, thus the register is always current at a boundary. Interrupts never split a
pair, thus the core never consumes a value from the middle of a pair. Because of
this structure, `SPC` never points to a delay slot and never loses a taken branch.
It does not depend on the state of the fetch frontier. No case-specific mux arms
are necessary.

Synchronous events have their own SPC rules:

- TRAPA saves `pc+2` (it retires).
- A fault in a delay slot saves the *branch* (`pc−2`) and sets the slot flag. A
  slot-illegal instruction gives `EXPEVT=0x1A0`.
- Other faults save the PC of the faulting instruction.

### Running-state copies

EX-stage consumers read SR bits from local running registers (`r_t`, `r_s`, `r_m`,
`r_q`). Each producer writes these registers when it leaves EX/MA. On redirects and
on pipeline drain, the registers get the committed SR again. The GPR bank copy
(`r_bank1`, §1) uses the same rule, with an LDC-commit arm and a two-phase
RTE-restore arm.

The rule for these copies is: **each select that comes from SR starts from a
register that is coherent with the committed SR at the edge where its consumer
captures.** This is also true across an RTE that returns to a different register
bank or mask level. The SR-race suite and the random-interrupt suite test all of
these cases (§7).

---

## 3. Cache

### Geometry (SH7709S p.103–104)

The cache is a **unified** instruction and data cache. It has one tag/data/LRU array.
It is **not** split into I$ and D$.

| Parameter | Value |
|---|---|
| Size | 16 KB |
| Associativity | 4-way set-associative |
| Line | 16 bytes (4 longwords) |
| Sets | 256 (`16384 / 4 / 16`) |
| Tag | PA[28:10], 19 bits (PA has 29 bits; PA[31:29] are the region shadow) |
| Index | PA[11:4], 8 bits |
| Replacement | 6-bit pseudo-LRU, Table 5.2 (one-hot victim decode, one LUT for each bit) |

Files: `cache.sv` (1344, the wrapper and the FSM), `cache_mem.sv` (157, the M10K
banks), `cache_pkg.sv` (108, geometry and the
`tag_of`/`cacheable`/`lru_*`/`merge_word` helpers).

### The lookup model — in the pipe, no handshake

This is the most important structural fact: **the cache lookup is part of the
pipeline stage. A hit is *not* a request/response transaction.** IF *is* the I-side
access, and MA *is* the D-side access. This makes IPC=1 possible. (A req/rsp
handshake limits the IPC to 0.5.)

The design uses **model-b**. This is a two-beat overlapped lookup:

- **Beat 0 (request cycle):** The pipe supplies the live AGU address. The cache
  decides the **accept** (`req_ready`) from registered state and from the resolve
  of the *previous* access (`z_ok`, refer to the subsequent section). At the edge,
  the RAMs capture the read index, and `bram_*` captures the request descriptor.
  The read index comes from a private 12-bit index-slice twin in the pipe
  (`LBus.req_addr_idx`), not from the shared adder (the R4 lever, §6).
- **Beat 1 (resolve cycle):** The cache compares the tag/data `q` with `bram_*`. The
  cache gives a hit response **combinationally**. The pipe consumes it at the closing
  edge. On the same edge, the cache captures the *next* access. Thus back-to-back
  hits run at one for each cycle. If the pipe cannot consume a live hit response in
  this cycle, the response **goes into a registered `rsp_*` flag** (hold without
  loss). Thus no response is ever overwritten.
- **Pair delivery:** An even I-side hit, fill read, or bypass read returns the
  addressed halfword *and* its sibling (`rsp_pair`). A hit never faults. A registered
  response cannot occur with a live hit on the same side. Thus the pair qualifier is
  a product of registered flags only (§1, instruction fetch).

Hits do not have a held-response slot, because the resolve *is* the response. A
"held-slot overlap" design cannot work for this reason: it locks.

### `z_ok` — the single global stall

`z_ok` is the one global-stall term. It stops a new accept when the resolve of the
previous access still uses the cache. This occurs for a cacheable **miss**, a
write-through store dispatch, or a CCR flush. During a stall, **all** stage registers
in the core stop together.

`z_ok` carries the tag compare into `req_ready`. This is intentional. It is *the*
critical path of the design (tag q → compare → `z_ok` → `req_ready` → issue tail).

### One port ⇒ MA-priority arbitration

IF and MA supply addresses to the **same** single read port. The arbitration gives
**priority to MA**. When IF and MA need the same cycle, the data access gets the port
and the fetch stops for one cycle. This is the "MA contends with IF" fetch bubble
(p.454–455).

For this reason, memory-heavy code runs at less than 1 IPC: loads and stores use
fetch slots. Pair fetch makes the fetch-side demand on the port half as large (one
access for each two instructions). This gives the 0.564 IPC of the store loop. The
remaining limit is the port limit. This is *correct* RISC behavior. (The project
does not use a second read port.)

### Write policy, fills, and the MMIO path

- **Write policy for each region** (p.105, 110–111): P1 uses `CCR.CB`. P0/U0/P3 use
  `~CCR.WT` (`wb_mode`). The tag has a `U` (dirty) bit.
  - A WB write hit writes the cache and sets U.
  - A WT write hit writes the cache and the memory. A WT hit on a dirty line writes
    only its own word to memory. It does not drain or clean the dirty data in the
    line. A subsequent `CCR.CF` discards this data (a locked law).
  - A WB write miss uses write-allocate.
  - A WT write miss writes only the memory.
- **Write-back buffer:** one line (p.111–112, Fig 5.5). A line fill is 4 sequential
  longword reads, word 0 first. A fill fault in the middle of a line invalidates the
  victim way, because the earlier beats already wrote its words. Thus no line can be
  a stale-valid mix.
- **Store fast path:** A write-back store hit writes its strobed bytes at the
  resolve edge. It retires in one MA cycle. (A store is a *notify*, not an ack. The
  pipe retires it through `ma_complete`.) The read of the next access at the same
  edge gets the new bytes through the write-through bypass registers of
  `cache_mem` (write before read). Cyclone V M10K has no mixed-port new-data mode in
  silicon.
- **MMIO path:** CCR/CCR2 and the exception registers (TRA/EXPEVT/INTEVT/TEA) are a
  *local-register access class*. The cache serves them through its shared `do_d`
  output flop. The match uses the **latched** address. Thus the MMIO decode is not on
  the 5 ns fan-out of the AGU. Writes do not wait for a response. Reads return a
  registered response in 1 cycle.
- **Memory-mapped array windows** (p.112–114): The tag window is `0xF0xx_xxxx` and
  the data window is `0xF1xx_xxxx`. The way is in `A[13:12]`, the set is in
  `A[11:4]`, and the associative bit is `A[3]`. A tag read returns
  `{tag, LRU, U, V}`.
  - An **associative write is the purge operation**. If the tag agrees and `U=1`, the
    cache writes the line back and then invalidates it. A miss does nothing.
  - A **non-associative tag write to a dirty entry drains the entry first**. Then
    the cache writes the new tag/V/U/LRU. Thus a direct array write cannot destroy
    dirty data.
  - Subsequent cached hits see the data-array writes.

  A dedicated law suite locks all of this behavior (§7).
- **Non-cacheable bypass:** P2 (`101`) and P4 (`111`) are non-cacheable control
  spaces (`cacheable()` in `cache_pkg`). The reset PC is in P2 (`0xA000_0000`).
  Locked accesses (`TAS.B` and related instructions) always bypass the array.
- **PREF** allocates through the usual D-fill path. But architecturally, it does
  not wait for a response. If a PREF fill faults, the cache stops it without a
  message (no allocation, no exception). This is also true with an interrupt.

### Miss and external-bus FSM

Only a miss makes the cache leave the running state. Each state for miss, refill,
write-back, bypass, and MMIO lasts one full cycle. (The RAM read of each state
arrives one state later.) These states drive the external I bus (§4).

Interrupt acceptance can occur during each of these excursions. This includes the
256-set `CCR.CF` flush walk and the memory-mapped array states. The boundary
invariant of §2 makes this safe. The collision suites move an acceptance edge
across each excursion type.

---

## 4. Bus structure

### On-chip tiers (SH7709S Fig 1.1, p.6)

The core uses the SH7709S internal bus hierarchy.

**Latency law:** The on-chip fabric has **zero wait states and does not wait for
responses**. Handshake latency is permitted *only* on the bridge tiers (I bus 2,
P bus) and on the external BSC leg. It is permitted only if OOC shows that a bus
register costs less Fmax than the worst path of the pipeline. This has never
occurred: no bus or peripheral cone is in an OOC top 20.

| Bus | `addr[31:29]` | Timing | Function |
|---|---|---|---|
| **L bus** | `111` (P4) | 1-cycle R/W, zero wait | control registers that the CPU accesses directly (CCR, exception MMIO) |
| **I bus 1** | — | zero wait to the BSC | two masters (the cache and the DMAC), merged by `ibus_arb` |
| **I bus 2** | — | handshake (bridge) | CPG/WDT, INTC register file (behind the BRIDGE) |
| **P bus** | P4 / area 1 | 2-cycle read (bridge) | TMU, RTC, I/O ports (bridge in the BSC) |

Fabric modules:

- `ibus_splitter.sv` (86): mux only, zero beats. `owner_q` steers the response.
- `ibus_bridge.sv` (159): IDLE→ACCESS→RESP, right-justified writes,
  lane-replicated reads.
- `peri_bus_if.sv`: the small register-bus interface.
- `ibus_arb.sv` (147): the CPU/DMAC arbiter on I bus 1. It is one 2:1 mux with a
  **registered owner select**. The select stays on the CPU when it is idle. Thus the
  `reqn_*`/address 5 ns class stays at one LUT level. The select changes only at
  idle and at response-done boundaries, never on an accept edge. `i_DMA_HOLD` from
  the DMAC keeps a transfer unit (and a full burst) indivisible against the CPU.

The CPU leg has no added beats. IPC parity is a locked law.

**Bus contracts (the full suite checks them, §7):**

- An unaccepted D request supplies the same fields in each cycle until it is
  accepted or withdrawn. A withdrawal is a pipeline kill and is legal. A change of
  the fields is never legal.
- An unconsumed D response supplies the same fields again (retirement without
  loss).
- An external MEM-bus request is never withdrawn or changed after it starts.
- Locked accesses always alternate read→write (one open pair).

### BSC — the external bus controller (`bsc.sv`, 1952 lines)

The BSC has the **pin set of the real SH7709S chip** (Table 10.1, without PCMCIA) at
the `HS3` top:

- `A[25:0]`
- A split data bus (`o_D_O`/`o_D_OE`/`i_D_I`). It is `inout` only at board level.
- `BS_n`, `CS0/2–6_n`, `RD_WR`, `RAS3L/U_n`, `CASL/U_n`, `WE_n[3:0]` (= DQM),
  `RD_n`, `i_WAIT_n`, `CKE`, `BREQ_n`/`BACK_n`
- The pad-state outputs `o_A_PU`/`o_D_PU`/`o_RASCAS_OE`, and `IRQOUT_n`
- The MCS0–7 selects on the port-C pads

The front end has these route classes:

- **SDRAM engine**
  - It supports MRS, single, burst, auto-refresh, self-refresh, and bank-active with
    open rows for each bank. It has a tWR guard and a CL read pipe.
  - The `MCR`/`WCR2` registers (CL/RCD/tRP) set the timing exactly. The engine runs
    on the 50 MHz `i_BUS_PCEN` enable.
  - It gives the same SDRAM latency as the original board (§10.3.4, figs
    10.14–10.28). Most emulated code runs from SDRAM.
  - **Full table-10.13 address multiplexing:** It decodes all AMX families. The row
    is `addr >> {8,9,10}` on `A16–A1`. In the column phase, a mask for each mode on
    `A16–A13` holds the row values. This mask keeps the bank bits on the device BA
    pins. The precharge flag is on the device-A10 pin (`A12`/`A11`, as the bus width
    sets). Single-rail modes (`1101`/32-bit, `1110`/16-bit) never drive the U rails.
    The open-row table compares only the row bits that the device uses.
  - **BCR2 selects a 16-bit SDRAM bus width** for each area. A line moves as 8
    halfword beats on `A3:A1` (fig 10.15 text). Single accesses split into halves on
    `D15–D0` under the DQMLU/DQMLL rails.
  - **BS marks the Td data cycles** ("asserted in each of cycles Td1–Td4", p.283).
    A CL-pipeline predictor makes BS. The commands do not carry BS.
  - **Single reads drive only their own DQM byte lanes** (p.276). The Micron model
    obeys this for each lane.
- **Ordinary / burst ROM**
  - It has the full state model of fig 10.6: T1 + *n*·Tw + T2. The number of
    first-access waits is one of {0,1,2,3,4,6,8,10}, from `WCR2`.
  - `i_WAIT_n` decides the Tw→T2 transition. When `WCR1.WAITSEL`=1, the BSC samples
    it in the middle of the state (fig 10.11). When `WCR1.WAITSEL`=0, it samples at
    the boundary. (The silicon documentation calls this configuration "not
    guaranteed.")
  - The continuation beats of a burst ROM have the exact state total of the pitch
    table, and they always sample WAIT. A write-back burst ignores the pin (p.274).
  - The `WCR1` idle states between accesses (1–3) delay the **pin launch** on area
    changes and on read→write turnaround. The fast path of the generic-port
    handshake never has this cost. Thus the IPC-parity law stays beat-exact.
  - **The strobe shapes agree with the AC shapes of fig 23.16.** RD/WEn assert at
    **mid-T1** and negate at **mid-T2** (tRSD/tWED at the CKIO falls). CSn negates at
    mid-T2 (tCSD2). The BSC samples read data **at the mid-T2 fall**. tRDH1 = 0 ns
    lets the device release the data when RD rises. Thus a later sample is not
    possible on real parts.
  - A split-16 longword runs as two full bus cycles. **WE rises between the
    halves.** An asynchronous device latches at the rise. Thus a WE that stays low
    across the two halves would lose the first half on real silicon.
  - **8-bit ports** complete the width matrix of tables 10.7–10.12. If a datum is
    wider than the port, the BSC accesses its byte addresses from low to high on
    `D7–D0` with WE0 only. Each byte is one full bus cycle. The register lanes are
    endian-mirrored, and the BSC decodes the two endians. Bus arbitration never
    splits the multi-cycle sequence (§10.3.8).
  - **The controller itself paces line bursts** (cache fill/drain, DMAC 16-byte
    units). The bus calls for each beat do not pace them. A call/return with one
    outstanding call cannot chain beats without gaps. Thus the first call opens a
    4-beat envelope. Reads prefetch into a line buffer, and the calls read from this
    buffer. Write calls go into a queue before the pins and get posted acks. The 4th
    call completes with the envelope, thus a unit fault stays visible.
  - The beats chain back-to-back with **no idle states** (fig 11.11).
    - On a BCR1 burst-ROM area, read beats 2–4 have the exact state total of the
      pitch table and always sample WAIT. **CSn/RD-WR/DACK stay low across the
      full run.** Only A3–A0 change, at the beat-launch posedges ("CS0 is not
      negated, only the address is changed", p.304; figs 10.29/10.30, 23.19/23.20).
      RD pulses again for each beat, from mid-launch to mid-data-state.
    - On plain areas, each beat is a basic cycle that frames CSn again (fig 11.11).
      All burst WRITE beats also do this (notes of figs 10.29/10.30). Burst WRITE
      beats also ignore the WAIT pin (p.304: 16-byte DMA writes, single-address
      dev→mem, cache write-back).
  - BREQ and refresh cannot split the envelope. (Silicon splits plain-area units
    at bus-cycle boundaries, §10.3.8. HS3 holds the full run. This is bounded and
    conservative.)
  - All ord pin edges are **on the 20 ns bus grid**. The launch is on the `ord_run`
    grid flop. Beats chain at the close boundary. The release is at the last
    T2 close (`!ord_done`). No edge is at a core handshake edge. Only an *early*
    completion of a generic-port hsk can advance or release off the grid. This is
    extension-path behavior, and the header of the file gives it.
- **Generic mirror port** — a pass-through with zero beats to the controllers of the
  surrounding SoC (fabric SDRAM controller / HPS DDR3). This is the IPC-parity path.
  All data goes on the physical D pins. The generic port has only address and
  control.
- **Register / dummy** — the register file of the BSC, with POR-only reset
  (`0xFFFFFF50–74`).
- **Early-transaction sideband (`o_MON_*`)** — an advisory strobe for observation
  only. It is for integrators that have their own fast memory path (made for
  ikacore CV1k).
  - It gives one registered pulse for each committed external transaction *unit*,
    at its internal accept edge (`mon_fire = engine-op start | ordinary accept`).
  - The pulse carries the physical address, the direction, the size, and the burst
    flag, one cycle before the pin sequence starts.
  - It does not change cycle behavior or arbitration. A 5-seed sweep shows no
    timing effect.
  - A whole-run match-queue oracle in the TB proves it. Each strobe agrees 1:1, in
    order, and field-exact with the pin units that the TB detects independently
    (341,575 events; measured outstanding depth = 1).
  - The CV1k specification (`ikacore_CV1k/docs/sh3_sideband.md` §11) gives the full
    contract, the measured lead tables, and the timing diagrams.

The verification uses a Micron MT48LC2M32B2 SDRAM model and a Macronix MX29LV320E NOR
flash model (patched vendor copies in `sim/models/`). It includes **boot from flash**
(16-bit fetches with two sub-cycles) and autoselect. Important correctness points:

- The ordinary controller and the SDRAM engine have bus-ownership interlocks.
- Self-refresh park must not count as bus-busy. If it counts, fetches lock.
- **TAS.B atomicity against BREQ:** The BSC must not release the bus between the
  read and the write of a locked pair (p.320).

Cache line fills use `req_burst` on I bus 1.

The TB runs two **board-bus monitors for the full run**. The last test checks them.

- A D-bus contention monitor. At each sample, a maximum of one driver is on the bus
  (DUT, TB memories, flash, or SDRAM). The WCR1 idle states and the strobe shapes
  must make this true.
- A WE-shape monitor. When an ordinary write owns the pins, the address and the data
  must stay stable from the WE fall to the WE rise. Thus the split-16 bug class with
  a held WE cannot occur again.

The raw memories of the TB obey the strobes. They drive D only while `RD_n` is low,
and they latch for each `WE_n` lane, as the asynchronous parts in figs 10.7–10.9 do.
The handshake-mode controller releases the D bus while the chip drives it. (Posted
writes of the SDRAM engine share the pins.)

**CKIO pin phase.** `o_CKIO` rises at the command edges, as figs 10.14/23.16 show.
Each bus pin changes at the CKIO rise. The mid-state shapes (RD/WEn edges, WAITSEL=1
sampling) are at the fall. A synchronous device with its clock directly from the pin
would sample at the same edge where the pins change. Thus the board must give the
device the tOD margin of the real chip. On the FPGA, use an output-delay constraint
or clock-tree skew on the CKIO net (no PLL block is necessary). The TB models this
board adjustment as a half-cycle transport delay on the clock net of the Micron
model. Thus each command arrives in the same device cycle as on the real board.

**Register file.**

- POR values: BCR1 `H'0000`+ENDIAN, BCR2 `H'3FF0`, WCR1 `H'3FF3`, WCR2 `H'FFFF`,
  MCR/PCR/MCSCR/refresh group `0`.
- Reserved-bit masks: BCR2 `3FF0`, WCR1 `BFF3`, MCR `FFFE`, PCR `CFFF`,
  MCSCR `007F`.
- `BCR1.ENDIAN` is read-only. It shows the MD5 strap (**0 = big-endian**).
- The refresh group has the write keys of fig 10.5 (word only, `A5` / `101001`).
- RFCR clears when it is more than the LMTS limit.
- After the keyed write-0, CMF clears at the *next CBR refresh that occurs*
  (p.253).

A dedicated law suite locks this behavior (§7).

**MCS mask-ROM selects and pad behavior.**

- The MCS0–7 mask-ROM selects use MCSCR0–7 / table 10.15: CS0-or-CS2 select, a
  CAP-sized `A25:22` block compare, and a CS-shaped assertion. They go on the
  **port-C pads** through the "other function" mode of the PFC. When MCSCR0 decodes
  area 0, MCS0 uses the CS0 pad itself (p.323).
- **Pad behavior at bus release:** PULA pulls `A25–A0` up for exactly 4 CKIO after
  `BACK` asserts (fig 10.41). PULD marks the D pins when the data bus is idle (figs
  10.42/10.43). HIZCNT keeps the RAS/CAS pads driven through a release
  (`o_RASCAS_OE`).
- The **IRQOUT pin** (pp.320–321) asserts for a pending refresh that did not run
  yet (BSC `o_REF_PEND`). It also asserts for an unmasked interrupt
  (`int_level > SR.I3–I0`, independent of BL, NMI always). Thus a different master
  returns the bus.

Known differences from the full chip: PCMCIA, and the pad states in standby mode.
(The SoC has no standby mode. HIZMEM is stored but has no effect.)

### Cache↔BSC interaction latency

The table gives the core-cycle timing for a cache miss and for the two non-cacheable
(P2) accesses. The trace uses a warm loop that runs from cached SDRAM (auto-precharge
mode, CL2/RCD2, 50 MHz bus, no other bus traffic). All numbers are core cycles at
100 MHz. `t` is the edge where the D lookup resolves. The tag read is at `t−1` ("it
first takes a cycle to find the cache", SH7604 §7.11.2). HS3 agrees with this.

| Event | Cached load miss | Bypass load (P2) | Bypass store (P2) |
|---|---|---|---|
| miss/classify resolved; `CORE_I_BUS` req; splitter; BSC accept | `t` (one edge, zero beats) | `t` | `t` |
| first SDRAM command (ACTV) on pins | `t+1` | `t+1` | posted |
| data beats respond (CL2) | **missed word first** at `t+10`, wrap +2 for each beat | `t+11` | ack `t+1` |
| the load **retires** (fill-forward) | **`t+13`, all word offsets** | `t+14` | `t+2` |
| cache goes back to `S_IDLE` (fill tail in background) | `t+17` | `t+12` | (WRIT arrives at approximately `t+7`) |

This agrees with SH7709S §5.3.2 (p.110), SH7604 §7.11.2/§8.4.3, and the
`attic/SH-master` SH7604 implementation:

1. **Fills wrap and forward from the missed word.** The BSC engine sends the READ
   columns in wrap order (READA on the 4th command, fig 10.16). The cache forwards
   the first beat to the pipe "in parallel with being loaded to the cache" (p.110).
   Thus the miss retirement does not depend on the word offset. The fill tail drains
   in the background while the pipe runs.
   - A fault on a later beat does not make the forwarded access fault. It only stops
     the validation of the line. The exception goes to the access that later
     requests the faulting word itself. (The victim-invalidate law does not change.)
   - The background drain uses a launch slot on hit-resolve edges. Without this, a
     hit stream after an early restart has no edge without a request.
2. **Dispatch is one edge.** The first SDRAM command starts at the `E_IDLE` dispatch
   edge, directly from the live op fields. For each SDRAM op, accept→ACTV is one
   cycle.
3. **The front half is already optimal.** The miss decision, the cache request, the
   splitter, and the BSC accept all occur on **one edge**. (The SH-master reference
   registers its request one cycle later.) The latency law of §4 applies: no new
   beats here.
4. **Cache-off is not one cycle faster than a hit dispatch. This is correct.**
   SH7604 §7.11.2: cache-through reads also have "an extra cycle … to determine the
   cycle" before the internal-bus read starts. In HS3, the dispatch cost of a miss
   and of a bypass is the same (both start at `t`). The bypass is faster from start
   to end only because it moves one beat, not four.
5. **The posted bypass store agrees with the SH7604 "one-level write buffer"
   model** (ack at `t+1`, retire at `t+2`, WRIT on pins at approximately `t+9`, no
   wait for a response). The BSC has one outstanding transaction. Thus a
   *subsequent* external access stops until the posted write completes. The manual
   gives the same behavior: "during reads, the CPU always has to wait."

Self-modifying code: The fill-forward window is short. Thus `SELFMOD_K` must cover
only IF/ID and the pair slot that holds stale opcodes (`SELFMOD_K = 2`).

---

## 5. Peripherals

No on-chip peripheral has a timing problem in OOC (no cones in a top 20). The CPU
stays the critical path.

| Module | File | Function |
|---|---|---|
| **CPG / WDT** | `cpg_wdt.sv` (311) | FRQCR/STBCR/STBCR2 clock pulse generator. Pφ divider N∈{1,2,3,4,6}. Watchdog timer with keyed `0x5A`/`0xA5` writes and a reset stretcher. It owns the bus grid `o_BUS_PCEN`/`o_BUS_NCEN`. (No subsequent module makes a phase again.) It also owns the `o_CKIO` pin (B-φ = core/2, p.207). FRQCR has no CKOEN, thus CKIO always drives in modes 0–2. Datasheet phase: CKIO rises at the command edges. |
| **INTC** | `intc.sv` (455) | Full §6 interrupt controller: IRQ / IRL / IRLS / PINT / NMI. A 37-entry **2-stage registered priority resolver**, `INTEVT2`, `o_INT_ACK`/`o_NMI_ACK`. The acks latch and clear the pending state. The ack-implies-entry law of the core makes the handshake lossless. The interrupt inputs come from the I/O pads (refer to the subsequent text). |
| **TMU** | `tmu.sv` (262) | 3× 32-bit auto-reload down-counters. Shared Pφ prescaler taps (P/4, /16, /64, /256). External TCLK clock (per CKEG, 2FF and edge detect). Ch2 input capture (TCPR2, ICPF). Underflow interrupts `TUNI0-2`/`TICPI2` → INTC (IPRA). |
| **I/O ports / PFC** | `ioport.sv` (236) | All 12 ports (A–L, SCP) as `pcr[]`/`pdr[]` arrays, with a capability mask for each port (drive/pull-up). PFC mode mux (`MD1 ? pin : (DRV & DR)`). The PGCR PTG0 special case (p.577). The `o_PC_FN` grant vector gives the port-C pads to the MCS outputs of the BSC. |
| **RTC** | `rtc.sv` (407) | §13, **two clock domains**. The first is the `i_EXTAL2` 32.768 kHz oscillator (7-bit prescaler → RTCCLK 16.384 kHz and a 256 Hz tap). The second is the bus domain (R64CNT, BCD calendar, alarms, periodic interrupt). The CDC uses tick sync and toggles without reset. A pin reset never resets the counters and the alarms (Table 13.2). It supplies TMU `i_RTCCLK`/`i_RTC_TICK`. |
| **DMAC + CMT** | `dmac.sv` (764) + `dmac_channel.sv` (195) | Full §11 4-channel DMA controller. It is a second I-bus-1 master that "calls the BSC like a function" (its bus cycles have the same shape as CPU accesses, p.363). **Request sources:** auto; the on-chip CMT (Pφ/4-64 compare-match timer, §11.4); external DREQ0/1 (sampled at the CKIO falling edge, DS level/edge, DRAK grant pulses). **Units:** dual-direct R→W; ch3 dual-indirect (LONG pointer fetch prologue); single-address (one-cycle transfers framed by DACK, `o_D_OE` held off for writes that the device drives); byte/word/long/16-byte (4-longword gather/play with a 4×32 buffer). **Priority:** fixed and round-robin. It uses one 2-bit rotation head. The p.350 rule makes sure that the order is always a pure rotation. The DMAC resolves the priority again at each unit boundary. Ch2 reloads the source after each 4 transfers. **Aborts:** NMIF from the qualified NMI edge of the INTC. AE from alignment checks at grant time and from bus faults in flight. The two aborts stop all channels without setting TE. `DEI0-3` → INTC (0x800–0x860). **DACK:** The BSC frames the DACK windows on CSn **inside the BSC** (active-high sideband strobes). The DMAC applies the AL/RL pad polarity. **Bursts:** 16-byte unit beats carry `req_burst`. Thus the BSC chains them as one run without gaps (fig 11.11; one CSn/DACK envelope on burst-ROM areas, fig 23.19; SDRAM engine line ops). The BSC obeys the WAIT-ignore rule of p.304/§11.6 note 12 on the write runs. **Differences** (given in the header): DREQ is sampled at each CKIO fall (not the 2-cycle one-step-ahead cadence). SDRAM-area DACK/single-address is not connected. |

**Pin sharing of the real chip (Table 18.1):** The design does **not** have dedicated
`i_IRQ`/`i_IRLS`/`i_PINT` inputs. The interrupt sources come from the port pads
(`i_IRQ = {SCPT7, PTH4-0}`, `i_IRLS = PTF3-0`, `i_PINT = {PTF, PTC}`). Only NMI stays
dedicated. `PTF` is shared by PINT8-15 and IRLS3-0. In mode 00, `PTH7` gives its pad
to the TMU TCLK.

The DMAC pins use Port D in the same way. When PDCR grants mode 00:

- DREQ0/1 use `PTD4`/`PTD6` as inputs.
- DACK0/1 drive `PTD5`/`PTD7`.
- DRAK0/1 drive `PTD1`/`PTD0`. (Note that DRAK is in the opposite order, Table 18.1.)

This agrees with the SH7709S design: peripheral functions and GPIO share the same
physical pins.

---

## 6. Timing summary — where the Fmax goes

The data is from a full-SoC OOC run (Quartus 17, Cyclone V `5CSEBA6U23I7`,
AGGRESSIVE PERFORMANCE, `set_max_delay` false paths on the M10K RDW arcs). The run
used five seeds on the final tree:

| Seed | Worst multicorner slack at 10 ns | Restricted Fmax | Top-20 headline class |
|---|---|---|---|
| dse | −2.68 ns | 78.9 MHz | mixed plateau (advance front, exc-MMIO compare legs) |
| 1 | −2.22 ns | 81.8 MHz | remaining part of the Wall A data leg |
| 4 | **−2.15 ns** | **80.3 MHz** | AGU carry → exc-MMIO compares → `o_TEA\|ena` (best HS3 fit) |
| 5 | −2.42 ns | 80.2 MHz | exc-MMIO classification → cache dispatch (`state.S_IDLE`) |
| 7 | −2.51 ns | 79.6 MHz | advance front → pair-slot capture |

The measured tree is the complete SoC. It includes:

- the full DMAC,
- the **R3 read-ahead tail late-select** (§1),
- the CV1k early-transaction sideband (`o_MON_*`, §4 BSC). It has no timing effect:
  no `mon_` cells are in a top 20.
- the **R4 cache-index slice twin**. This is a private 12-bit copy of the full AGU
  cone on `(* preserve *)` select duplicates. Its only consumer is the read index of
  the cache RAM (`LBus.req_addr_idx`). Thus the fitter can put the slice near the RAM
  block.

The mean worst slack of the five seeds is **−2.39 ns**. The AGU→RAM-index class
(`byp_q`/tag/data read address) is in 0/20 paths on 4 of 5 seeds. The headline class
is the **exc-MMIO live classification** family: adder carry → the shared
TRA/EXPEVT/INTEVT/TEA/CCR compares → exception-register write enables and cache
dispatch.

> **Read this as a plateau, not as a ranking.** The design is on a *flat cluster* of
> protected single-cycle loops. The placement of each seed selects a different loop
> as the headline. The coarse Fmax of each seed changes more than real structural
> changes do. (Fits of the Wall B/FSM cone with the same RTL went from −2.2 to −3.9.)
> Use the worst-slack trend *and the cone composition across seeds* to judge a
> change. Never use one fit. The 100 MHz deliverable is worst slack ≥ 0. The
> measured gap is approximately 2.2–3.2 ns, and most of it is interconnect (55–65 %
> of each failing path is routing).
>
> The plateau is also **chaotic under perturbation**. Two small levers (R5, R6′)
> each moved the 5-seed *mean* by −0.4 to −0.7 ns, but their own logic was in no
> worst path. A re-anchor with the same RTL moved less than 0.1. Each netlist change
> starts a new global placement. This costs more in the saturated issue-enable
> fabric than a small cone removal gives. For this reason, the catalog stops here.

These are the recurring cone classes. The no-bubbles rule protects all of them. Each
is a single-cycle loop that cannot get a register without an added architectural
cycle.

1. **Advance loop / request front** — `{mawb.gpr0_data, fwd_*_agu,
   second_access_agu}` → AGU adder → request valid/accept → `idex_allow` (fanout
   approximately 350) → capture. This is the oldest and deepest family. R3 removes
   its deepest tail (the GPR read-ahead address, refer to §1). The remaining paths
   capture at the clock enables of the pair slot (a 1-bit CE cone; no late-select
   applies), at the next state of the cache FSM, and at the write-through bypass
   registers (`byp_q`).
2. **Wall A — I-response → predecode** — the I-side response of the cache
   (`bram_addr` → RAM `q` → way/word select → `rsp_inst`) → predecode → the GPR
   read-ahead capture. R3 removes the shared mux tail on this path. The remaining
   path is the live-response data leg. No restructure can put a register on it
   without a fetch bubble.
3. **Operand → EX flags** — forward-lane selects → operand mux → carry chain of the
   EX adder → T/compare select tree → `exma.t_data` / `r_t`. Approximately 9 levels,
   approximately 60 % interconnect. The cause of this cone is the placement spread,
   not the logic depth.
4. **Exc-MMIO live classification** (headline after R4) — AGU carry → the shared
   `req_addr == TRA/EXPEVT/INTEVT/TEA/CCR` compares → the `o_TEA`/`o_EXPEVT` write
   enables and the cache dispatch arm. The two request-side transforms do not help:
   - A case sub-decode does nothing, because the WE gate shares the compares.
   - The full sum-addressed carry-free rewrite (R5) gives a *worse* fit. Six
     full-width operand words go into the compare cluster, where one sum was before.
     Six soft-LUT levels are slower than the hard carry chain. (The campaign log
     gives the details.)

   The one idea that is not tried is on the consumer side: make the priority tail
   of the enable in `exc_handler` flatter.

The interrupt and exception logic (§2) adds **no logic to a failing path**. The
acceptance boundary, the restart-PC register, the bank/flag copies, and the MA phase
gate all start from registers and end at registers. The fitter puts them away from
the walls. A name search over each top-20 path across seeds shows this.

**No more RTL levers are in the catalog.** R4 is the last lever that the design keeps,
and R5 is the last lever that was measured. `eval_ooc/timing_campaign_log.md` gives
the full history of the rounds. The remaining steps to 100 MHz are physical, not
structural:

- a LogicLock floorplan that puts the AGU/request cluster near the cache banks
  (there is no license for it in the current docker Quartus flow),
- a C6 speed grade,
- a wide seed/DSE search (seed 4 shows the result of a good placement).

Do **not** add an operand-capture or address-capture pipeline beat. This would
break cycle accuracy (the no-bubbles rule, §0).

*Flow note:* The OOC flow (`eval_ooc/tools/quartus_ooc.py all <config>`) uses the
run directory that the config gives. The STA and the summary are made again in each
run, but the `probe_*.rpt` cone probes are **not**. Before you read probes for a
new fit, examine the file modification times.

---

## 7. Verification

Two self-checking Verilator benches gate each change. The two benches must pass
bit-exact. The three IPC laws (§1) are asserted values, not observations.

| Bench | Scope | Tests |
|---|---|---|
| `cpu_core_tb` | core only (`src/cpu_core`); the tb models the bus | **103** |
| `HS3_tb` | full SoC on the real pin set, with vendor SDRAM/flash models | **84** |

Test 84 is the sideband match-queue oracle. It is a passive checker for the full run.
It compares each `o_MON_*` strobe with the starts of the external units that the TB
detects independently. The two must agree 1:1, in order, with exact fields, and they
are flushed at reset. Refer to §4 BSC.

The suite has four layers:

1. **Directed goldens**
   - All ISA classes, addressing modes, exception causes, and hazards/interlocks.
   - Cache laws: fills, write policies, fill faults on each beat, victim drains,
     flush/CE semantics, LRU thrash, self-modification, the memory-mapped tag/data
     window matrix, and WT-flip on dirty lines.
   - Fetch-pair laws.
   - Device-level behavior of the BSC: boot from flash, refresh, self-refresh,
     `BREQ` arbitration, and `i_WAIT_n`.
   - DMAC laws, which lock its cycle behavior in the same way: DREQ→grant latency,
     DME→TE durations for each bus mode, DACK⊆CS window containment, round-robin
     grant order, NMI/AE abort-and-resume protocols, and BREQ that splits a
     dual-address pair at the bus-cycle tier with correct results. (For the grant
     order, a tb write-order log changes the destination page of each DMA write beat
     into a hex-literal grant sequence.)
2. **Collision sweeps** — The tests move an interrupt (and, in a different test, an
   NMI) cycle by cycle across each excursion of the logic:
   - plain execution, miss/fill/drain walks, locked `TAS.B` pairs, the `CCR.CF`
     flush walk, and memory-mapped array accesses,
   - faulting and clean PREF fills,
   - two-event collisions. Each synchronous exception type (illegal, slot-illegal,
     address error, TRAPA, D-fill bus fault, I-fill fetch fault) collides with a
     pending interrupt at each offset.

   The laws at each offset are:
   - exactly one entry for each event,
   - `EXPEVT` and `INTEVT` never mix,
   - the interrupted computation is transparent,
   - `SPC` is never a delay slot,
   - no request is lost (an ack without an entry fails).

   The tests also sweep SR-write races (`LDC ...,SR` that changes IMASK/BL/RB at the
   acceptance edge) and nested-enable re-entry (BL cleared in a handler while the
   level stays on). `HS3_tb` has SoC twins of the collision sweeps over the real
   INTC/IRL pin protocol.
3. **Passive checkers for the full suite** — These monitors are always on. They make
   the run fail from any test:
   - an independent true-LRU copy, which the tb compares at each LRU write and each
     victim choice,
   - the L-bus/MEM-bus stability contracts (§4),
   - locked-pair alternation,
   - ack-implies-entry, with `INTEVT` settlement,
   - acceptance **coverage histograms** (entries for each cache FSM state and for
     each restart-PC source arm). The tb asserts that they are not empty. This
     proves that the sweeps get to the deep states.
4. **Random oracles** — Constrained-random programs run once as their own reference.
   (The programs use ALU operations, loads/stores in the R14 window, byte stores,
   `TAS.B`, `PREF`, and plain and delayed conditional branches over live T.) Then
   they run again under:
   - (a) random I/D wait states,
   - (b) random INT/NMI waves at random offsets, in three SR types: privileged RB=1,
     privileged RB=0, and user mode. (The last two change the register bank on each
     entry and RTE.)

   The architectural end state must be the same. The handler count must be equal to
   the wave count. This layer includes all the offsets of the hand-built tests. It
   is the strongest regression check in the suite.

   The DMAC adds a randomized differential test of legal configurations. The test
   uses 12 random configurations (channel, size including 16-byte, inc/dec/fixed
   walks, bus mode, count). Each configuration runs against a golden model in the tb,
   under a random bus latency. The golden model does not see the latency. Thus each
   round that passes is a latency-invariance oracle.

Testbench conventions (each one prevents a real problem):

- The `AFFE` guard word is a delayed branch. Thus the word after it must stay a NOP.
- GPRs keep their values across the reset of the tb. Thus the programs set their
  own *active-bank* registers and all detector registers to zero.
- Initialize the interrupt-visible state *before* the `LDC` that clears BL. If a
  wave is accepted at the boundary of the initialization, the core executes the
  initialization again, and this overwrites the counts of the handler.
- `gpr()` sampling at a retire marker races the lookahead of the pipeline
  (approximately 7 words). Thus multi-step checks use write-once result registers
  that the tb reads after the sentinel.

---

## References

- `SH7709S_Hardware_Manual[REJ09B0081-0500O].pdf` — cache (§5, p.103–114), BSC
  (§10), INTC (§6), exceptions (§4, p.85–101), TMU, RTC (§13), ports (§18), CPG
  (p.207–212).
- `SH-3_SH-3E_SH3-DSP_Software_Manual.pdf` — pipeline timing (Fig 10.40/10.41
  p.476), branch/PR rules (§10.2.3 p.432), interrupt/pair semantics (§4.5.3).
- `SH-1_SH-2_Programming_Manual.pdf` — 5-stage pipeline baseline.
- `SH7604_Hardware_Manual[ADE-602-085C].pdf` — cache operation reference.
