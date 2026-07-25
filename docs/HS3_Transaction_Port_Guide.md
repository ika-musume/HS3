# HS3 Transaction Port (`o_MEM_*`) Guide

One `o_MEM_*` family carries the complete transaction view of the external
bus, in parallel with the unchanged SH7709S pin protocol — every waveform
below draws both, and the pin rows ARE the external bus contract. The port
lets an external memory controller (an SoC integration fronting other RAM —
e.g. 16-bit DDR3 — or the CV1k "fast main" pump) service every external
transaction **without decoding the SH7709S pin protocol**.

**First principle — the port never alters the machine it observes.** Under
no circumstances do the SH3's existing bus cycles (every T-state, wait, and
pin edge) or the pipeline's IPC change: every rail is a dedicated FF
mirroring an already-existing internal enable; no handshake, launch, or
capture path is touched; no new backpressure enters the core. Full-suite
cycle laws bit-exact and identical IPC are the acceptance gate for every
implementation change.

The intended consumer structure is a pair of FIFOs in the controller:

- a **read FIFO (FWFT)** whose head drives `i_D_I`; the BSC samples it at its
  own architecturally fixed instants and confirms each consumed beat with a
  1-`i_CLK` `o_MEM_DE` pop pulse;
- a **write FIFO** that the BSC fills EARLY — one `o_MEM_DE` push pulse +
  `o_MEM_WDATA` beat per internal accept, several CKIO cycles before the pins —
  and the controller drains at its own pace.

The consumer runs in the **same `i_CLK` domain** as HS3 (core clock, 100 MHz
target / 102.4 MHz in the CV1k deployment; CKIO = `i_CLK`/2). Every output is
registered and glitch-free; the consumer samples on every clock edge.

---

## 1. Clock and phase foundation

The CPG is the single source of the bus grid — `cpg_wdt.sv` makes both
enables from `ckio_ph` and clocks the `o_CKIO` pin off them, so nothing
downstream re-derives a phase:

| Enable | Expression | Capture edge lands on |
|---|---|---|
| `i_CEN` | architectural enable | every core edge |
| `i_BUS_PCEN` | `i_CEN & ckio_ph` | **CKIO rising edge** (command edge) |
| `i_BUS_NCEN` | `i_CEN & ~ckio_ph` | **CKIO falling edge** (mid-state edge) |

Both leave the chip (`o_BUS_PCEN`/`o_BUS_NCEN`) next to `o_CKIO`, so a
consumer in this clock domain uses the same grid the pins are built on.
Every bus pin changes at CKIO rises (the BUS_PCEN grid); mid-state events
(RD/WEn shapes, WAIT sampling, ordinary read sampling) sit at the falls. In
the waveforms below, one CKIO cell is 8 characters: the rise at the cell
start, the fall mid-cell; 1 `i_CLK` = 4 characters.

```
CKIO        __/‾‾‾‾‾‾‾\_______/
              ^rise   ^fall = BUS_NCEN edge
              BUS_PCEN edge (commands, pin updates)
```

Phase note: read pop pulses ride the CONSUME edges (ordinary = falls,
SDRAM = rises — both grid phases occur); REQ and write push pulses ride the
`i_CLK` accept grid and are CKIO-phase-free. DE is therefore not a
BUS_PCEN-grid signal.

---

## 2. Signal reference

Transaction rails (all registered, dedicated FFs):

| Signal | Width | Phase | Meaning |
|---|---|---|---|
| `o_MEM_REQ` | 1 | 1 `i_CLK` pulse at the accept edge | one pulse per committed external transaction UNIT |
| `o_MEM_WR` | 1 | field, valid under REQ | 1 = write — the **early direction bit** |
| `o_MEM_ADDR` | 29 | field, valid under REQ | physical address, `[28:26]` = CS area |
| `o_MEM_SIZE` | 2 | field, valid under REQ | I_BUS size encoding |
| `o_MEM_BURST` | 1 | field, valid under REQ | 16-byte unit / burst-ROM envelope |
| `o_MEM_LEN` | 5 | field, valid under REQ | **physical beat count** (1..16) = read pop count; the consumer needs no SIZE×port-width tables |
| `o_MEM_WSTRB` | 4 | field, valid under REQ | 32-bit-lane byte enables; partial strobes occur only on single-beat units (line beats are always `4'hF`) |
| `o_MEM_SADDR` | 1 | field, valid under REQ | single-address DMAC unit: WDATA is NOT sourced by the BSC (§5) |
| `o_MEM_CS_n` | 7 | field (held decode) | area strobe of the held unit, bit n = area n (bit 1 never asserts); key on REQ, not on CS_n edges |
| `o_MEM_DE` | 1 | 1 `i_CLK` pulse per beat | read unit: **pop-confirm** at the consume edge; write unit: **push-valid** at the accept edge |
| `o_MEM_WDATA` | 32 | per push, valid under DE (WR=1) | write beat payload (label `WDATA` in the waveforms) |

Fields are loaded at the accept edge and physically held until the next unit;
they are only *guaranteed* under the REQ pulse. A resumed mid-burst write
drain re-strobes REQ with the first remaining beat's address and the
**remaining** LEN.

Companion rails (they complete the port):

| Signal | Dir | Meaning |
|---|---|---|
| `i_D_I` | in | read data — the read FIFO's FWFT head drives it; the BSC samples at its fixed instants |
| `i_WAIT_n` | in | ordinary/burst-ROM wait pin — the ONLY read backpressure (§5) |
| `i_MEM_RSP_VALID` / `i_MEM_READY` | in | optional early completion of an ordinary access (handshake fast path, IPC-parity); tie low to use the timed path only |
| `o_MEM_RSP_READY` | out | accept pacing for the handshake completions |
| `i_MEM_FAULT` | in | fault injection on a handshake completion |

**Unit mapping** (one REQ each):
- SDRAM head/single op (excluding SDMR/MRS). A line fill strobes once; a
  write drain resumed after a mid-burst yield strobes again with the first
  remaining beat's address and remaining LEN.
- Ordinary / burst-ROM envelope head. Continuation calls ride the envelope
  silently (their write beats appear as DE pushes).

**Exempt (silent on the transaction rails, consumer pin-decodes if needed —
spec R2):** CBR/self-refresh, BRQ_PALL, MRS via the SDMR window, P-bus /
on-chip register / dummy cycles. In an SoC deployment this silence is a
feature: the DDR3 controller runs its own init/refresh and never sees the
legacy SDRAM housekeeping.

**Reset (spec R10):** a manual reset drops an accepted-undispatched op; the
consumer flushes both FIFOs and its match queue on any reset assertion.

---

## 3. `o_MEM_REQ` — 1 `i_CLK` pulse at the accept edge

The accept is an **edge** event: it becomes
fact only at the clock edge where `req_valid && req_ready` are sampled high.
The REQ flop loads at that same edge, so the pulse rises AT the accept edge —
the earliest committed, glitch-free instant that exists. Anything earlier
would export the (wide, late-settling) accept cone combinationally: a glitchy
pin carrying a prediction that a reset can retract, and external routing load
on the accept cone that the dedicated-FF design deliberately avoids.

```
i_CLK        _/‾\_/‾\_/‾\_/‾\_
valid&ready  _/‾‾‾‾‾‾‾\________    accept cone — stays internal
                      |accept edge (commitment instant)
o_MEM_REQ    _________/‾‾‾\____    1 i_CLK pulse, rises AT the accept edge
WR + fields  =========X== held =   loaded at the same edge
```

The pulse is `i_CLK`-aligned but CKIO-phase-free (accepts land on either
phase). REQ leads — or, for an ordinary cycle granted on the accept edge
itself, coincides with — the unit's first pin activity; SDRAM dispatch adds
1.5–2 CKIO of lead (grid pickup + PCEN-registered command, phase-dependent;
2 CKIO when the accept is rise-aligned, as drawn). **For a write unit the REQ edge is also the first
WDATA push** (§4): the head beat's data is committed at the same instant.

**REQ lead — one rule, two launch machines.** REQ always fires at the
accept edge; what differs between waveforms is how far the pin machine is
from launching at that instant:

| Unit class | accept → first pin activity | accept → first read deadline |
|---|---|---|
| SDRAM | 1.5–2 CKIO (engine dispatch, accept-phase-dependent) before ACTV; precharge tails add, bank-active row hits skip ACTV | dispatch + tRCD + CL before the first capture rise — the §5.4 pre-fill budget (A1/A2) |
| ordinary / burst-ROM | 0..1 `i_CLK` (launch grid-align) + WCR1 AnIW idles before T1 | ≥1.5 CKIO + WCR2 first-access waits before the T2 fall, extended without bound by `i_WAIT_n` (A5) |

The SDRAM engine has a dispatch pipeline behind the accept, so its REQ lead
is structural and guaranteed — and it is the shape that needs it, since
SDRAM areas have no WAIT. The ordinary launcher is grid-aligned immediately
behind the accept: T1 opens at the accept edge itself (the coincident
shape, A6) or at the next BUS_PCEN edge. Primitives that draw the accept a
full cell before T1 do so for canvas room; the structural gap is 0..1
`i_CLK` plus programmed AnIW idles. A read gains nothing from beating T1
anyway — T1 carries no read data; the deadline is the SAMPLE FALL, and the
guaranteed budget knobs are WCR2 first-access waits (fixed) and `i_WAIT_n`
(dynamic, §5.3). Nothing earlier than the accept can be exported: that
would be uncommitted pend state — retractable by reset, and its cone is the
accept cone (§10).

**Spacing guarantee — pulses never merge.** The fabric is one-outstanding
(`ibus_arb.sv`: `busy` from accept to `rsp_done`; owner moves only at idle
boundaries, never on an accept edge) and single ordinary writes are NOT
posted (`rsp_valid = gen_ext_done | ord_done` for non-envelope GEN,
`bsc.sv` response mux). A new head is additionally gated by
`eng_busy || eng_go` (ordinary) / engine-idle terms (SDRAM). Consequently two
REQ-firing accepts are always separated by at least a full response-bounded
transaction — no skid buffer exists, and a testbench assertion enforces the
invariant (§10).

---

## 4. `o_MEM_DE` — 1 `i_CLK` pulse per data beat

DE asserts a **single-`i_CLK` pulse per beat**, so it can drive a
synchronous FIFO enable directly (a multi-cycle window would double-pop).
One wire, one rule — *DE rises at the edge where the beat changes hands* —
with the direction fixed by the held `o_MEM_WR`:

**Read units (WR=0): pop-confirm.** The pulse is a registered copy of the
BSC's own consume enable — it rises AT the edge where `i_D_I` is sampled:

| Cycle | Consume edge = DE rise |
|---|---|
| Ordinary / burst-ROM read | the data state's CKIO **fall** (T2/Tb2 mid-state sample, tRDH1 = 0) — one pulse per physical sub/beat |
| SDRAM read | each capture **rise** (the CKIO rise closing the data cell) — burst: one pulse per CKIO |
| Handshake early completion | the core edge where `i_MEM_RSP_VALID`/`i_MEM_READY` is taken (off-grid) |

Wait states never generate a pulse: an inserted Tw simply delays the fall
that carries it, so **the pulse slides with WAIT automatically** — it mirrors
the capture enable, it never predicts it. This is why the fall (not the
T-state's opening rise) is the spec point: the fall is the first instant that
is unconditionally true under every WAIT mode, and it makes DE a
*pop-after-consume* strobe.

```
CKIO         ___/‾‾‾\___/‾‾‾\___/‾‾‾\___
i_D_I        ==<  FIFO head N   >X< N+1 =   FWFT head, stable until popped
BSC capture           ^ consume edge (fall shown: ordinary read)
o_MEM_DE     _________/‾‾‾\_____________    1 i_CLK, rises AT the consume edge
FIFO rd_en                 ^ sampled here → head advances AFTER the capture
```

Pop budget: the FIFO samples DE one `i_CLK` after the consume edge and
advances; the next possible consume edge is ≥2 `i_CLK` away (1-CKIO beat
pitch), so the new head always has ≥1 `i_CLK` to settle on `i_D_I`.

**Write units (WR=1): push-valid.** The pulse rises AT each accepted write
beat's internal accept edge — the instant the data exists inside the BSC —
NOT at the pin cycle. `o_MEM_WDATA` loads at the same edge and is stable
under (and after) the pulse. The head beat's push coincides with REQ;
continuation beats (line drains) push at their own accepts, back-to-back at
the internal accept cadence.

```
i_CLK        _/‾\_/‾\_/‾\_/‾\_/‾\_
beat accept  _/‾‾‾‾‾‾‾\____________   internal (head accept = the REQ edge)
o_MEM_DE     _________/‾‾‾\________   push rises AT the accept edge, like REQ
WDATA        =========X  beat  ====   loaded at the same edge
```

- **Count is contractual, cadence is not**: a unit's push count equals its
  accepted-beat count (LEN on 32-bit areas; LEN×width/32 on narrow areas —
  pushes are LOGICAL 32-bit beats, pin expansion is pin-side only, A8).
  Push spacing (1–2 `i_CLK`) is an implementation detail.
- Lead over the pins: an SDRAM single write's push precedes its WRIT data
  cycle by ~3 CKIO (~6–7 `i_CLK`); drain beats 1–3 gain more (B4/B6). An
  ordinary write's push leads its T1 by the accept→launch gap (worst case 0:
  the coincident-T1 shape, A6).
- `o_MEM_WSTRB` stays a unit-held field — partial strobes only ever occur on
  single-beat units, so no per-beat strobe rail is needed.

**Framing invariant.** All DE pulses in `[REQ(N), REQ(N+1))` belong to unit N
— reads pop strictly FIFO in pin order, writes push strictly in beat order
(drain slots ascend), and a unit's pops/pushes never interleave with another
unit's (one-outstanding fabric + response-bounded accepts, §3). REQ order =
pin order = service order; response order does NOT (posted writes run ahead,
B2/B6) — the consumer queues on REQ, never on responses.

---

## 5. FIFO integration model (the SoC deployment)

The BSC does not know the FIFOs exist — every rule below is consumer-side.

**Read FIFO — FWFT, head on `i_D_I`.**
1. On REQ with WR=0: start fetching `LEN` beats from the backing memory
   (DDR3 etc.), pushing into the read FIFO as they arrive.
2. Keep the FIFO head driven on `i_D_I` from fill until the pop; pop on each
   DE pulse. Beat k must be on `i_D_I` by its consume edge.
3. **Ordinary / burst-ROM areas — the WAIT rule:** while a read unit is
   outstanding and the head is not yet valid, hold `i_WAIT_n` low (plain
   level, no pin-protocol decode needed — WAIT is level-sampled at the
   protocol's fixed instants, and burst-ROM samples it per beat, p.304).
   The BSC inserts Tw states; the consume edge — and the pop pulse — slide
   until the data is there. This is the ONLY read backpressure — the WAIT
   machinery is load-bearing for the port.
4. **SDRAM areas — no WAIT exists.** The controller returns data on the
   BSC's programmed pin schedule: REQ leads the first
   capture by dispatch (≥2 CKIO) + ACTV + tRCD + CL as programmed in MCR,
   and the SDRAM command rails remain fully driven in parallel, so the
   controller may pin-decode them or replay the programmed schedule from
   REQ. It must simply never be late — sizing the MCR timing fields is the
   SoC integrator's budget knob.

**Write FIFO — early fill, leisurely drain.**
1. On each DE pulse with WR=1: capture `{o_MEM_WDATA, held WSTRB}` into the
   write FIFO; the unit is framed by its REQ (address/LEN/fields).
2. Drain to the backing memory at any pace. Depth is bounded small: the
   one-outstanding fabric + engine gating keep at most ~2 units' beats in
   flight — 8–16 entries is generous. Units are throttled by the pin
   protocol, never by the FIFO.
3. **Ordering rule:** never serve a read past an older undrained write
   (RAW). A single REQ-ordered service queue satisfies this for free.

**Single-address DMAC units (SADDR=1, A10).** The data never passes through
the BSC (the device drives D directly): the DE push is COUNT-only and WDATA
is not sourced. The consumer latches the source data from the D pads during
the DACK frame — the one permanent pin-latch exception.

**Resets:** flush both FIFOs and the match queue on any reset assertion
(spec R10). **Refresh/MRS:** silent on the transaction rails (spec R2) —
pin activity only.

---

## 6. Waveform catalog — primitives

Conventions: one CKIO cell = 8 chars, 1 `i_CLK` = 4 chars; `v` = pin-side
sample point, `^` = DE pulse rise. `WDATA` = `o_MEM_WDATA`. The **pin rows
are the external bus protocol** — the transaction rails ride alongside.
Write primitives include a lead-in cell so the accept-time push is on canvas
(the accept is `i_CLK`-grid — drawn on a CKIO rise for readability only).
Parameters shown (TRCD=1, CL=2, 0 or 1 waits) are examples; the placement
rules, not the offsets, are the spec. SDRAM primitives include the
dispatch lead-in cells so the accept is on canvas; ordinary primitives open
at/near T1 — the accept precedes T1 by at most 1 `i_CLK` + AnIW idles (§3).

### A1 — SDRAM single read (TRCD=1, CL=2)

```
             acc     disp    R0      R1      R2      R3
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CMD                          ACTV    READ-AP nop     nop
D (pump)   --------------------------------- <  D0  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________________   accept — the pre-fill window opens here
o_MEM_WR   _____________________________________________________   0 = read (held)
o_MEM_DE   __________________________________________/‾‾‾\_____   1 i_CLK pop at the capture rise
                                                      ^ R3: beat consumed — the FIFO head may advance
```

REQ→pop = dispatch (1.5–2 CKIO, accept-phase-dependent; 2 as drawn) + tRCD
+ CL: the SDRAM pre-fill budget (§5.4) — REQ leads every pin edge of its
own unit.

### A2 — SDRAM burst read (4 beats)

```
             acc     disp    R0      R1      R2      R3      R4      R5      R6      R7
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
CMD                          ACTV    RD-b0   RD-b1   RD-b2   RD-b3A  nop     nop     nop
D (pump)   ----------------------------------<  b0  ><  b1  ><  b2  ><  b3  >----------------
o_MEM_REQ  __/‾‾‾\___________________________________________________________________________   head, LEN = 4
o_MEM_WR   __________________________________________________________________________________   0 = read (held)
o_MEM_DE   __________________________________________/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___________
                                                      ^R3     ^R4     ^R5     ^R6   4 pops = 4 beats
```

The controller pre-fills the read FIFO from REQ (§5.4); each pop
confirms one consumed beat, 1 CKIO apart at full burst rate.

### A3 — SDRAM single write

```
             acc     disp    R0      R1      R2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CMD                          ACTV    WRITA   nop
D (BSC)    --------------------------<  D0  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________   1 i_CLK at the accept edge
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\_____________________________________   single push — coincides with REQ
WDATA      ==X              D0 (held)                ==   in the FIFO ~3 CKIO before the pin beat
```

Dispatch lead drawn rise-aligned (2 CKIO, §3). The pins drive D0 in the
WRITA cycle per the pin protocol — the push is an early copy of the data,
never a re-timing of the pins.

### A4 — SDRAM burst write (4 beats)

```
             acc     a1      a2      a3
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CMD                          ACTV    WR-b0   WR-b1   WR-b2   WR-b3A  nop
D (BSC)    --------------------------<  b0  ><  b1  ><  b2  ><  b3  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________________________________   head only
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\_____________________________________   4 pushes
WDATA      ==X  b0   X  b1   X  b2   X  b3 (held)                             ==
```

Pushes at the internal accept cadence (a1–a3 = continuation accepts; 2 i_CLK
drawn, 1–2 i_CLK real — **count is contractual, spacing is not**). The whole
line sits in the write FIFO before the first WR command reaches the pins.

### A5 — Ordinary read (1 wait; wait count never moves the pop off T2)

```
              T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn        ‾‾\___________________/‾‾‾‾‾
RD_n       ‾‾‾‾‾‾\_______________/‾‾‾‾‾    assert T1 mid, negate T2 mid
D (pump)   ------------------<  D  >---
o_MEM_REQ  __/‾‾‾\_____________________    at the accept (grid-aligned here)
o_MEM_WR   ____________________________    0 = read (held)
o_MEM_DE   ______________________/‾‾‾\_
                                 ^ pop at the T2 fall = the BSC's own sample edge
```

The FWFT head must be on `i_D_I` by that fall. Not ready → the adapter holds
`i_WAIT_n` low, Tw states are inserted, and the fall — with its pop — slides
(§5.3). REQ→pop is ~1.5 CKIO + waits: the ordinary-read budget — WCR2
first-access waits raise the guaranteed floor, `i_WAIT_n` covers the rest.

### A6 — Ordinary write (1 wait)

```
              acc     T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn        ‾‾‾‾‾‾‾‾‾‾\_____________________/‾‾‾
WEn        ‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________________/‾‾‾
D (BSC)    ----------<           D           >-    driven from T1 (tWDH1 window)
o_MEM_REQ  __/‾‾‾\_____________________________    accept edge
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾    1 = write (held)
o_MEM_DE   __/‾‾‾\_____________________________    push with REQ — wait states never touch it
WDATA      ==X            D (held)            ==
```

Push lead over T1 = the accept→launch gap (grid align + engine pend; worst
case 0 in the coincident-T1 shape). Wait count is irrelevant to the push.

### A7 — Burst ROM read (4 data, first wait 1, burst wait 1)

```
              T1      Tw      Tb2     Twb     Tb2     Twb     Tb2     Twb     T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/
CSn        ‾‾\_______________________________________________________________/‾‾‾‾‾‾‾‾   held for the run
D (pump)   ------------------<  d0   >-------<  d1   >-------<  d2   >-------<  d3  >-
o_MEM_REQ  __/‾‾‾\____________________________________________________________________   head only
o_MEM_WR   ___________________________________________________________________________   0 = read (held)
o_MEM_DE   ______________________/‾‾‾\___________/‾‾‾\___________/‾‾‾\___________/‾‾‾\
                                 ^d0             ^d1             ^d2             ^d3
```

One pop per datum at its sample fall; Twb gaps (and the per-beat WAIT
sample, p.304) slide later pops. With zero burst waits the pops space at
1 CKIO — the A2 rhythm on the ordinary leg.

### A8 — Narrow port: 32-bit write to an 8-bit area (0 waits)

```
              acc     T1      T2      T1      T2      T1      T2      T1      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
A (byte)   ----------<     +0        X     +1        X     +2        X     +3    >---
D7:0 (BSC) ----------<     b0        X     b1        X     b2        X     b3    >---
o_MEM_REQ  __/‾‾‾\___________________________________________________________________   one REQ, LEN = 4
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\___________________________________________________________________   ONE push (logical beat)
WDATA      ==X     {b3,b2,b1,b0} - one 32-bit entry, WSTRB = 1111                   ==
```

LEN counts PIN beats (a read here would pop 4×); the write side pushes ONE
logical 32-bit beat — lane expansion (endian-mirrored, tables 10.7-10.11) is
pin-side only. Deployment rule: FIFO-backed areas should be 32-bit; narrow
areas suit real pad devices.

### A9 — DMAC dual-address transfer (two ordinary cycles, 0 waits)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
bus state    (idle) | T1r     T2r   | (DMAC) | T1w     T2w   |
D          ----------------- <  Dr  >--------< Dw  (BSC)    >--
o_MEM_REQ  ______/‾‾‾\___________________/‾‾‾\_________________
                 R unit                   W unit
o_MEM_WR   ‾‾‾‾‾‾\_______________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   0 = read (R) -> 1 = write (W)
o_MEM_DE   ______________________/‾‾‾\___/‾‾‾\_________________
                                 ^pop @T2r fall  ^push @W accept
WDATA      ==============================X Dw (held)         ==
```

Indistinguishable from CPU traffic; earliest spacing shown. The W unit's
data arrives via the DMAC's bus call, so the push carries it normally.

### A10 — DMAC single-address dev→mem (write-shaped, device-sourced)

```
              acc     T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn/DACK   ‾‾‾‾‾‾‾‾‾‾\___________________/‾‾‾‾‾   DACK framed on CSn (p.363)
WEn        ‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________/‾‾‾‾‾
D (device) --------------<      data      >----   DMA device drives, BSC tri-stated
o_MEM_REQ  __/‾‾‾\_____________________________   SADDR = 1 in the REQ fields
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\_____________________________   push = COUNT only
WDATA      ==X     (not sourced - SADDR unit)==
```

The consumer latches the SOURCE data from the D pads during the DACK frame
(the device holds it through the frame) — the one pin-latch exception (§5).

Notes:
- BSC-internal read latch points: SDRAM at the capture rise
  (`if(i_BUS_PCEN && rd_lat)`, `bsc.sv`), ordinary/burst-ROM at the T2/Tb2
  fall (tRDH1 = 0). The pop pulse is a registered mirror of exactly these.
- Narrow ports: read pops per PHYSICAL sub-beat, write pushes per LOGICAL
  accepted beat (A8); LEN always counts physical beats.
- A handshake early completion (`i_MEM_RSP_VALID`/`i_MEM_READY`) replaces
  the timed sample: its pop pulse fires at the completion core edge instead.

---

## 7. Waveform catalog — sequences

Full timelines with REQ, responses, and both units' DE pulses. `acc` rows
show the `valid && ready` cycle whose closing edge is the accept. Key fabric
facts used: single ordinary writes are not posted; the arbiter is
one-outstanding and re-grants only at idle; new heads are engine-gated.
WDATA rows appear on write units.

### B1 — Ordinary single write → SDRAM read

```
core edge    e0  e1  e2  e3  e4  e5  e6  e7  e8  e9  e10     e12     e14     e16     e18
i_CLK      __/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________/‾‾‾\___________________________________________
             A @e1                           B — ≥2 cycles after rsp A, never e2
o_MEM_REQ  ______/‾‾‾\___________________________/‾‾‾\_______________________________________
             A: 1 i_CLK at accept edge           B: same
o_MEM_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________________________________
                  1 = write (A)                       0 = read (B)
bus state            | T1    | T2    | idle  |eng_go | disp  | ACTV  | READ  | (CL)  |
rsp        __________________________/‾‾‾\___________________________________________________
                             rsp A = ord_done (single ord writes NOT posted)
D          ----------< A write data >----------------------------------------<  d0  >--------
o_MEM_DE   ______/‾‾‾\_______________________________________________________________/‾‾‾\___
                 ^A: push at the accept (= REQ edge)                             ^B: pop at the capture rise
WDATA      ======X A data (held until the next write unit)                                ==
```

A's push leads its T1 pin cell by 1 CKIO here; B's pop confirms the beat
the BSC captured at the closing rise of its data cell. Note B's accept edge
lands on the CKIO **fall** phase: engine take at the next rise, so its
dispatch lead is the §3 floor — 1.5 CKIO to ACTV — whereas A1 draws the
rise-aligned 2-CKIO case. Both shapes are legal; the accept is phase-free.

### B2 — Ordinary single read → SDRAM write (posted)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________/‾‾‾\___________________________________
             A @e1                           B
o_MEM_REQ  ______/‾‾‾\___________________________/‾‾‾\_______________________________
o_MEM_WR   ‾‾‾‾‾‾\_______________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  0 = read (A)                        1 = write (B)
bus state            | T1    | T2    | idle  |eng_go | disp  | ACTV  | WRIT  |
rsp        __________________________/‾‾‾\_______/‾‾‾\_______________________________
                             rsp A (ord_done)    rsp B: POSTED at the accept — before B's pins!
D          ------------------<  Da  >--------------------------------<  Db  >--------
o_MEM_DE   ______________________/‾‾‾\___________/‾‾‾\_______________________________
                                 ^A: T2-fall pop ^B: accept push (+ posted rsp)
WDATA      ======================================X Db (held)                       ==
```

The posted ack means response order runs ahead of pin order; **REQ order
always equals pin order**, so the consumer queues on REQ, never on
responses. B's data + ack + REQ all land at the accept — ~3 CKIO ahead of
its WRIT pin cycle.

### B3 — Ordinary burst-ROM fill (4 beats) → SDRAM head (tightest ord→SDRAM gap)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________/‾‾‾\___________________________
             A @e1                                           B: after last rsp + turnaround
o_MEM_REQ  ______/‾‾‾\___________________________________________/‾‾‾\_______________________
o_MEM_WR   ‾‾‾‾‾‾\____________________________________________________________________________
                  0 = read (A); B reloads the same value at its accept — no visible edge
bus state            | T1    | d0    | d1    | d2    | d3/T2 | idle  | disp  | ACTV  |
D          ------------------<  d0   X  d1   X  d2   X  d3   >--------------------------------
o_MEM_DE   ______________________/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___________________________________________
                                 ^d0     ^d1     ^d2     ^d3  last pop; REQ B ≥1 CKIO later
```

READ/data of unit B continue beyond the right edge (A2 shape). Even in the
tightest case REQ B trails A's last pop — no overlap.

### B4 — Ordinary burst write (cache drain) → SDRAM

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
o_MEM_REQ  ______/‾‾‾\_______________________________________________________________________
             head only — continuation beats appear as DE pushes
o_MEM_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  1 = write (the whole drain unit)
bus state            | T1a   | T2a   | T1b   | T2b   | T1c   | T2c   | T1d   | T2d   |
o_MEM_DE   ______/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___________________________________________________________
                 4 pushes at the accept cadence — the line is banked before the pins finish beat 0
WDATA      ======X  b0   X  b1   X  b2   X  b3 (held)                                                   ==
rsp        ______________________________________________________________________/‾‾‾\_______
             beats 0-2 ack immediately (posted, not drawn); beat 3 answers at envelope end
```

The pins still run T1a..T2d as drawn; they — never the FIFO — throttle the
unit rate. The SDRAM head follows as in B1: accepted ≥2 cycles after the
final response.

### B5 — SDRAM single read → ordinary write (engine-tail gap)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________________/‾‾‾\___________________
             A @e1                        B PENDS at the front end -->| accepted at eng idle
o_MEM_REQ  ______/‾‾‾\___________________________________________________/‾‾‾\_______________
o_MEM_WR   ‾‾‾‾‾‾\_______________________________________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  0 = read (A)                                             1 = write (B)
bus state            | disp  | ACTV  | READ  | (data)| drain | Tpc   | idle  | T1    | T2
rsp        __________________________________________/‾‾‾\___________________________________
                                     rsp A at the capture; B still blocked by eng_busy
D          ----------------------------------<  d0  >------------------------< B write data >
o_MEM_DE   __________________________________________/‾‾‾\_______________/‾‾‾\_______________
                                                     ^A: pop             ^B: push at the accept (= REQ B)
WDATA      ==============================================================X B data (held)
```

The GEN head-ready term requires `!eng_busy && !eng_go`: B's request waits
out the read drain + Tpc tail — the pend delays the ACCEPT itself, so the
push fires only then, still at/before B's own T1 launch.

### B6 — SDRAM posted write → ordinary read (longest pend)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________/‾‾‾\___________________________
             A @e1                                           B @ engine idle
o_MEM_REQ  ______/‾‾‾\___________________________________________/‾‾‾\_______________________
o_MEM_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________________
                  1 = write (A)                                      0 = read (B)
rsp        __________/‾‾‾\___________________________________________________________________
             rsp A POSTED at once — B's request pends from here to the engine-idle edge
bus state            | disp  | ACTV  | WRIT  | Trwl  | Tpc   | idle  | T1    | T2    |
D          --------------------------<  Da  >--------------------------------<  Db  >--------
o_MEM_DE   ______/‾‾‾\___________________________________________________________/‾‾‾\_______
                 ^A: push + posted ack @ accept                                 ^B: pop @ its T2 fall
WDATA      ======X Da (held)                                                              ==
```

A's data is banked ~2.5 CKIO before its WRIT pin cycle. REQ fires at the
**accept**, not at the request — B's long pend produces no early pulse,
preserving commitment semantics.

### B7 — Minimum REQ→REQ spacing summary

| Sequence | Binding constraint | Practical minimum |
|---|---|---|
| ordinary → ordinary | rsp at `ord_done` + master turnaround | full cycle + ~2 `i_CLK` |
| ordinary → SDRAM | last rsp at final data state + turnaround | REQ B ≥1 CKIO after A's last pop (B3) |
| SDRAM → ordinary | engine tail (drain/Tpc or Trwl/Tpc) | REQ B ≥2 CKIO after A's last pop (B5/B6) |
| SDRAM → SDRAM | engine completion + posted-ack consumption | op + tails |

REQ pulses never merge, and every DE pulse in `[REQ(N), REQ(N+1))` belongs
to unit N (§4 framing invariant).

### B8 — Refresh interleave (silent pins)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___ ... __/‾‾‾\___/‾‾‾\___
bus state    | T1    | T2    | idle  | PALL  | (wait)  ...  | REF   | lockout
o_MEM_REQ  __/‾‾‾\___________________________________ ... __________________
             ordinary read unit — the refresh burst is port-silent (spec R2)
o_MEM_WR   ‾‾\_______________________________________ ... __________________   0 = read (held)
o_MEM_DE   ______________/‾‾‾\_______________________ ... __________________
                         ^ pop at the T2 fall
```

Refresh (and MRS/BRQ_PALL) occupy the pins with no REQ and no DE; the next
unit's REQ fires only after the refresh lockout releases the engine.

---

## 8. Waveform catalog — double pump

The double-pump build is the §5 FIFO consumer fronting a **16-bit SDRAM
clocked at 2× CKIO** (102.4 MHz against the 51.2 MHz bus clock; CL=2,
BL=2): each 32-bit beat crosses the narrow side as a LO16/HI16 pair inside
one CKIO, so the 16-bit device matches the 32-bit bus bandwidth exactly.
Every canvas in this section uses the same BSC programming — **MCR.RCD=1
(ACTV → nop → READ, tRCD = 2 CKIO) and CL=2** — one CKIO later than the §6
canvases everywhere downstream of ACTV. The o_MEM_* rails follow the §6
placement rules unchanged: REQ pulses at the accept (acc/disp lead-in as in
§6), read pops sit at the BSC capture rises, write pushes ride the internal
accept cadence. `CLK (DP)`/`CMD (DP)`/`DQ16 (DP)` are the 16-bit device's
own pins; `D (pump)` is the 32-bit bus the BSC samples.

### A1 — SDRAM single read (MCR.RCD=1, CL=2)

```
             acc     disp    R0      R1      R2      R3      R4
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CLK (DP)   __/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾
CMD                          ACTV    nop     READ-AP nop     nop
CMD (DP)                     ACT nop RD  ACT*
DQ16 (DP)  ----------------------------------<LO><HI>-----------------   the two 16-bit halves of D0
D (pump)   ------------------------------------------<  D0  >---------   assembled {HI,LO} after the shift stages
o_MEM_REQ  __/‾‾‾\____________________________________________________   accept — the pre-fill window opens here
o_MEM_WR   ___________________________________________________________   0 = read (held)
o_MEM_DE   __________________________________________________/‾‾‾\____   1 i_CLK pop at the capture rise
                                                             ^ R4: beat consumed
```

The 16-bit command pipeline (102.4 MHz, one entry per DP slot): the
controller turns REQ into its own `ACT`; one nop covers the device tRCD
(2 DP cycles ≈ 19.5 ns); a single `RD` with BL=2 then returns both halves
CL=2 later — `LO16`/`HI16` in the two halves of R2. The CL gap slot
(`ACT*`) is free — the trace shows a refresh/other-bank ACT parked there.
The halves then cross the controller's register/shift stages (~1 CKIO) and
the assembled 32-bit word meets the BSC capture rise at R4. On the BSC
side, RCD=1 pushes the capture one CKIO past §6-A1 (REQ→pop = dispatch +
2 + 2), widening the pre-fill budget by the same CKIO.

All released Cave games use this setup. Verified by a human using a MAME
trace on 2026-07-23. The tRCD for 6N/7N components is 15–20 ns, but the
programmers apparently felt RCD=0 was too short to satisfy the minimum
time. This allows a little more time for commands to be latched from the
SDRAM controller and pass through the shift stages.

### A2 — SDRAM burst read (4 beats)

```
             acc     disp    R0      R1      R2      R3      R4      R5      R6      R7
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CLK (DP)   __/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾
CMD                          ACTV    nop     RD-b0   RD-b1   RD-b2   RD-b3A  nop     nop
CMD (DP)                     ACT nop RD0     RD1     RD2     RD3A
DQ16 (DP)  ----------------------------------<l0><h0><l1><h1><l2><h2><l3><h3>------------------
D (pump)   ------------------------------------------<  b0  ><  b1  ><  b2  ><  b3  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________________________________________________   head, LEN = 4
o_MEM_WR   ____________________________________________________________________________________   0 = read (held)
o_MEM_DE   __________________________________________________/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\_____
                                                             ^R4     ^R5     ^R6     ^R7   4 pops = 4 beats
```

Seamless BL=2 pipelining sustains one 32-bit beat per CKIO on the 16-bit
part: one `RD` per beat, issued every other DP slot (`RD0`…`RD3A`, the
last with autoprecharge), so `DQ16` streams l0…h3 gap-free — RDn at slot
k returns its halves at k+2/k+3 (CL=2), landing exactly under the next
command. The free odd slots between RDs can carry refresh/other-bank
commands, as in A1. The same ~1 CKIO of shift stages separates each
`DQ16` pair from its assembled beat on `D (pump)`; pops then run 1 CKIO
apart at the capture rises, one CKIO later than §6-A2 throughout (RCD=1).

### A3 — SDRAM single write

Pushes are DP-independent — they fire at the accept, before any pump
stage:

```
             acc     disp    R0      R1      R2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CMD                          ACTV    nop     WRITA   nop
D (BSC)    ----------------------------------<  D0  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________________   1 i_CLK at the accept edge
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\_____________________________________________   single push — coincides with REQ
WDATA      ==X                  D0 (held)                    ==   in the FIFO ~4 CKIO before the pin beat
```

The pins drive D0 in the WRITA cycle per the pin protocol (one CKIO later
than §6-A3, RCD=1). The 16-bit drain has no fixed alignment to the BSC
cycle, so no DP rows are drawn: the controller empties the write FIFO at
its own time as one `WR` (BL=2) plus a LO16/HI16 pair per word — write
data accompanies the command on the DP side, no CL applies.

### A4 — SDRAM burst write (4 beats)

```
             acc     a1      a2      a3
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CMD                          ACTV    nop     WR-b0   WR-b1   WR-b2   WR-b3A  nop
D (BSC)    ----------------------------------<  b0  ><  b1  ><  b2  ><  b3  >----------
o_MEM_REQ  __/‾‾‾\_____________________________________________________________________   head only
o_MEM_WR   __/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MEM_DE   __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\_____________________________________________   4 pushes
WDATA      ==X  b0   X  b1   X  b2   X  b3 (held)                                    ==
```

Pushes at the internal accept cadence (a1–a3 = continuation accepts; 2
i_CLK drawn, 1–2 i_CLK real — **count is contractual, spacing is not**),
exactly as §6-A4. With RCD=1 the whole line sits in the write FIFO a full
4 CKIO before the first WR command reaches the pins; the drain then runs
four back-to-back BL=2 pairs (or one BL=8) on the 16-bit side, again at
the controller's leisure.

---

## 9. Consumer contract

1. Sample all `o_MEM_*` on every `i_CLK` edge. All outputs are registered —
   no phase alignment needed, no pulse can be missed.
2. On REQ=1: enqueue `{WR, ADDR, SIZE, BURST, LEN, WSTRB, SADDR}`. One pulse
   = one unit; pulses never merge. **Service units strictly in REQ order** —
   this is also the RAW/WAW guard: never serve a read past an older
   undrained write.
3. Pair DE pulses to units by the framing invariant: every pulse in
   `[REQ(N), REQ(N+1))` belongs to unit N. Pin order = REQ order (response
   order is NOT: posted writes run ahead, B2/B6).
4. Read units (WR=0): fetch LEN beats and fill the FWFT read FIFO; keep the
   head on `i_D_I` from fill until the pop; pop on each DE pulse (pop count
   = LEN). Ordinary/burst-ROM areas: hold `i_WAIT_n` low while the head is
   not valid. SDRAM areas: meet the programmed pin schedule — the command
   rails stay driven in parallel for decode/replay; never be late.
5. Write units (WR=1): on each DE pulse capture `{WDATA, held WSTRB}` into
   the write FIFO (push count = accepted beats; = LEN on 32-bit areas);
   drain at leisure. SADDR units: pushes are count-only — latch the source
   data from the D pads during the DACK frame.
6. On any reset assertion: flush both FIFOs and the match queue (spec R10).
7. Refresh/MRS pin activity is port-silent — pin-decode it if needed
   (spec R2).

---

## 10. Implementation notes

- All transaction rails are dedicated FFs in the BSC — the port never loads
  the accept cone or engine cones with external routing (the CV1k campaign
  measured accept-cone-shape changes as Fmax-negative).
- REQ + fields: implemented as dedicated registers in the BSC's
  transaction-port section — LEN from the SIZE × area-port-width decode the
  BSC already owns (`ord_w8_c`/`ord_w16_c`, `sd16_a2`/`sd16_a3`), SADDR,
  and every field REQ-loaded and held (never live-muxed to the pins).
- DE pulse = a registered mirror of the ACTUAL consume/accept enables:
  `i_BUS_PCEN && rd_lat` (SDRAM read capture), the `ord_t2` fall-sample term
  including the `gen_ext_done` handshake injection (ordinary/burst-ROM), and
  the write-accept enables (`fe_acc_eng && req_write` + the generic-leg
  twin) for pushes. No prediction, no cone loading.
- WDATA (+ held WSTRB): one 32-bit register loaded under the same accept
  enables; ~45 new FF bits total including LEN/SADDR.
- The port deliberately provides NO level-held request view (a consumer
  needing one regenerates it as REQ-sets / LEN'th-DE-clears) and NO
  pre-accept pend visibility (REQ = commitment).
- The HS3_tb harness keeps its raw-memory models and hsk controller on a
  WHITEBOX copy of the internal live request view (hierarchical taps,
  expression-identical to the deleted live mux) so every model trigger and
  cycle law keeps its exact timing — the first principle holds by
  construction, and the port itself is exercised only by the oracles.
- Testbench contract (implemented in HS3_tb, whole-run passive): the match
  queue checks every REQ against its dispatched unit — fields + LEN +
  SADDR exact, FIFO order, reset flush; the DE oracle checks the pulse
  rail EVENT-FOR-EVENT against a registered whitebox twin of the
  consume/accept truth, frames every pulse into its [REQ(N), REQ(N+1))
  unit, and compares read units' delivered pops against the port's own
  LEN (SDRAM units always; GEN units in raw mode — the hsk leg consumes
  logical beats and its faults abort tails). Pin-sample pops are barred
  from silent engine bursts, judged at cause time (the registered pulse
  trails its consume edge by one cycle); hsk completions and write pushes
  are off-pin events that may land under a concurrent refresh. The port
  must be dead across reset. A full FIFO reference-adapter model is a
  possible future addition.
- First principle (hard gate, not an expectation): existing bus cycles and
  pipeline IPC are unchanged under all circumstances — no internal
  handshake, launch, or capture-path edits; new registers only on
  already-existing enable edges. Full suites + cycle laws bit-exact and
  identical IPC before any port change merges.
- OOC re-check after integration (plateau-tax history: every new change
  shape costs −0.3..−0.7 ns until proven otherwise); route the new HS3
  ports in the hand OOC top.
