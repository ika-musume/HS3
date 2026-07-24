# HS3 Early Transaction Monitor (`o_MON_*`) Guide

Specification of the BSC's early-transaction sideband: one advisory-with-guarantees
port that lets an external memory controller (the CV1k "fast main" pump,
`ikacore_CV1k sh3_sideband.md`) snoop every committed external transaction before
and while it appears on the pins. Nothing is received back; the external pin
protocol is the unchanged SH7709S contract.

The consumer runs in the **same `i_CLK` domain** as HS3 (core clock, 100 MHz
target / 102.4 MHz in the CV1k deployment; CKIO = `i_CLK`/2). Every MON output
is registered and glitch-free: REQ is a 1-`i_CLK` pulse, DE a CKIO-cycle-granular
window. The consumer samples on every clock edge.

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
start, the fall mid-cell.

```
CKIO        __/‾‾‾‾‾‾‾\_______/
              ^rise   ^fall = BUS_NCEN edge
              BUS_PCEN edge (commands, pin updates)
```

---

## 2. Signal reference

| Signal | Width | Phase | Meaning |
|---|---|---|---|
| `o_MON_REQ` | 1 | 1 `i_CLK` pulse at the accept edge | one pulse per committed external transaction UNIT |
| `o_MON_WR` | 1 | field, valid under REQ | 1 = write — the **early direction bit** |
| `o_MON_ADDR` | 29 | field, valid under REQ | physical address, `[28:26]` = CS area |
| `o_MON_SIZE` | 2 | field, valid under REQ | I_BUS size encoding |
| `o_MON_BURST` | 1 | field, valid under REQ | 16-byte unit / burst-ROM envelope |
| `o_MON_DE` | 1 | window, 1 CKIO cycle per data beat | valid data crosses D each asserted CKIO cycle |

Fields are loaded at the accept edge and physically held until the next unit;
they are only *guaranteed* under the REQ pulse.

**Unit mapping** (`mon_fire`, `bsc.sv` MON section):
- SDRAM head/single op (`fe_eng_start`, excluding SDMR/MRS). A line fill
  strobes once; a write drain resumed after a mid-burst yield strobes again
  with the first remaining beat's address.
- Ordinary / burst-ROM envelope head (`fe_acc && fe_gen && !ord_bcont`).
  Continuation calls ride the envelope silently.

**Exempt (silent on MON, consumer pin-decodes — spec R2):** CBR/self-refresh,
BRQ_PALL, MRS via the SDMR window, P-bus / on-chip register / dummy cycles.

**Reset (spec R10):** a manual reset drops an accepted-undispatched op; the
consumer flushes its match queue on any reset assertion.

---

## 3. `o_MON_REQ` — 1 `i_CLK` pulse at the accept edge

The accept is an **edge** event: it becomes fact only at the clock edge where
`req_valid && req_ready` are sampled high. The REQ flop loads at that same
edge, so the pulse rises AT the accept edge — the earliest committed,
glitch-free instant that exists. Anything earlier would export the (wide,
late-settling) accept cone combinationally: a glitchy pin carrying a
prediction that a reset can retract, and external routing load on the accept
cone that the dedicated-FF design deliberately avoids.

```
i_CLK        _/‾\_/‾\_/‾\_/‾\_
valid&ready  _/‾‾‾‾‾‾‾\________    accept cone — stays internal
                      |accept edge (commitment instant)
o_MON_REQ    _________/‾‾‾\____    1 i_CLK pulse, rises AT the accept edge
WR + fields  =========X== held =   loaded at the same edge
```

The pulse is `i_CLK`-aligned but CKIO-phase-free (accepts land on either
phase). REQ leads — or, for an ordinary cycle granted on the accept edge
itself, coincides with — the unit's first pin activity; SDRAM dispatch adds
≥2 CKIO cycles of lead.

**Spacing guarantee — pulses never merge.** The fabric is one-outstanding
(`ibus_arb.sv`: `busy` from accept to `rsp_done`; owner moves only at idle
boundaries, never on an accept edge) and single ordinary writes are NOT
posted (`rsp_valid = gen_ext_done | ord_done` for non-envelope GEN,
`bsc.sv` response mux). A new head is additionally gated by
`eng_busy || eng_go` (ordinary) / engine-idle terms (SDRAM). Consequently two
REQ-firing accepts are always separated by at least a full response-bounded
transaction — no skid buffer exists, and a testbench assertion enforces the
invariant (§8).

---

## 4. `o_MON_DE` — data-enable window

DE asserts one full CKIO cycle per data beat: it rises at the beat's opening
CKIO rise, holds through the cycle, and falls at the closing rise. All
transitions sit on the BUS_PCEN grid (CKIO rises); the minimum assert is
1 CKIO cycle = 2 `i_CLK`. Contiguous beats (SDRAM bursts) merge into one
longer window; the asserted CKIO-cycle count always equals the beat count.

```
CKIO         __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
o_MON_WR     __________________________________   held level: 0 = read / 1 = write
o_MON_DE     __________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______   2-beat window (rise -> rise)
                           v       v
                     per-beat sample falls (mid-window)
```

Generation is a dedicated BUS_PCEN-registered output FF fed by the same
next-state that drives the internal window trackers — transitions only at
CKIO rises, no combinational gating on the pin, no cone loading:

```systemverilog
always_ff @(posedge i_CLK) if(i_BUS_PCEN) de_q <= window_next;  //rise-to-rise window
```

**Window edge semantics.** The **opening rise** is the launch/command edge
(SDRAM WRIT beat, ordinary T-state start). The **mid fall** is the per-beat
sample instant: the ordinary-leg read sample, and the point where write data
is guaranteed stable — the recommended consumer sample point. The **closing
rise** is the SDRAM read capture edge / next beat boundary. DE is stable for
2 `i_CLK` per beat: the consumer counts beats by sampling once per CKIO cycle
(e.g. at falls), or by counting `i_CLK` samples and halving.

**Window placement rules** (per beat):

| Cycle | Window (per beat) | Beats |
|---|---|---|
| SDRAM read | the CKIO cycle ending at each capture rise (brackets the fall) | burst: contiguous |
| SDRAM write | the CKIO cycle opening at each WRIT command rise | burst: contiguous |
| Ordinary read | T2 only — wait states never stretch or move it | 1 per sub-cycle |
| Ordinary write | T1 only — data+address live from cycle start | 1 per sub-cycle |
| Burst ROM read | each data state (Tb2, Tb2, … T2); Twb gaps stay low | 1 per datum |
| DMAC cycles | identical to the above — **no special case** | — |

Notes:
- The SH3 BSC internally latches SDRAM read data at the window's **closing
  rise** (`if(i_BUS_PCEN && rd_lat)`, `bsc.sv`); the fall is the nominal window
  center. Ordinary/burst-ROM reads latch at the **fall** (`ord_t2` data-mid,
  tRDH1 = 0).
- Narrow ports: an 8/16-bit area splits a datum into full sub-cycles
  (`ord_subs`/`ord_ba`, endian-mirrored lanes) — one window cycle per
  **physical beat**, so a 32-bit write to an 8-bit area = 1 REQ + 4 asserted
  cycles. The consumer derives beat count from SIZE + the area's static port
  width.
- Single-address (DACK) dev→mem transfers are write-shaped cycles where the
  DMA device drives D (`o_D_OE` off): the window is T1 like any write, and
  the consumer latches the **source data** from D itself — the data never
  passes through the BSC. The device holds D through the DACK/CSn frame, so
  the consumer may defer its latch to any later fall in the cycle.

**Overlap rule.** A REQ pulse never overlaps another unit's DE window — new
accepts are response-bounded behind the previous unit's data phase (§3
spacing). A unit's own REQ can coincide with the first cycle of its own DE
in exactly one shape: an ordinary write accepted on a BUS_PCEN edge with the bus
free (T1 starts at the accept edge). Everywhere else REQ and DE abut at
worst. This is a property of the current fabric, protected by assertion —
the consumer may rely on it but the rails remain formally independent.

---

## 5. Waveform catalog — primitives

Conventions: one CKIO cell = 8 chars, `R#` = rises, `v` = data sample point,
`^` = internal BSC capture edge. Parameters shown (TRCD=1, CL=2, 0 or 1
waits) are examples; the placement rules, not the offsets, are the spec.
REQ fires at the unit's accept, at or before the first window shown.

### A1 — SDRAM single read (TRCD=1, CL=2)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
             R0      R1      R2      R3
CMD          ACTV    READ-AP nop     nop
D (pump)   ----------------- <  D0  >----------
o_MON_WR   ____________________________________   0 = read (held)
o_MON_DE   __________________/‾‾‾‾‾‾‾\_________
                                 v   ^ BSC capture (window close)
```

### A2 — SDRAM burst read (4 beats)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
             R0      R1      R2      R3      R4      R5      R6      R7
CMD          ACTV    RD-b0   RD-b1   RD-b2   RD-b3A  nop     nop     nop
D (pump)   ------------------<  b0  ><  b1  ><  b2  ><  b3  >----------------
o_MON_WR   __________________________________________________________________   0 = read (held)
o_MON_DE   __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________
                                 v   ^   v   ^   v   ^   v   ^   4 CKIO cycles = 4 beats
```

### A3 — SDRAM single write

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
             R0      R1      R2
CMD          ACTV    WRITA   nop
D (BSC)    ----------<  D0  >----------
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __________/‾‾‾‾‾‾‾\_________
                         v latch here (or at the closing rise)
```

### A4 — SDRAM burst write (4 beats)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
             R0      R1      R2      R3      R4      R5
CMD          ACTV    WR-b0   WR-b1   WR-b2   WR-b3A  nop
D (BSC)    ----------<  b0  ><  b1  ><  b2  ><  b3  >----------
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________
                         v       v       v       v
```

### A5 — Ordinary read (1 wait; wait count never moves the window off T2)

```
              T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn        ‾‾\___________________/‾‾‾‾‾
RD_n       ‾‾‾‾‾‾\_______________/‾‾‾‾‾    assert T1 mid, negate T2 mid
D (pump)   ------------------<  D  >---
o_MON_WR   ____________________________   0 = read (held)
o_MON_DE   __________________/‾‾‾‾‾‾‾\_
                                 v BSC sample = T2 fall (mid-window)
```

### A6 — Ordinary write (1 wait)

```
              T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn        ‾‾\_____________________/‾‾‾
WEn        ‾‾‾‾‾‾\_________________/‾‾‾
D (BSC)    --<           D           >-   driven from T1 (tWDH1 window)
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __/‾‾‾‾‾‾‾\_________________   T1 window — early, wait-independent
                 v addr+data stable here
```

### A7 — Burst ROM read (4 data, first wait 1, burst wait 1)

```
              T1      Tw      Tb2     Twb     Tb2     Twb     Tb2     Twb     T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/
CSn        ‾‾\_______________________________________________________________/‾‾‾‾‾‾‾‾   held for the run
D (pump)   ------------------<  d0   >-------<  d1   >-------<  d2   >-------<  d3  >-
o_MON_WR   ___________________________________________________________________________   0 = read (held)
o_MON_DE   __________________/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\
                                 v               v               v               v
```

With zero burst waits the per-datum windows abut into one contiguous assert
(the A2 pattern) — counting stays one CKIO cycle per datum either way.

### A8 — Narrow port: 32-bit write to an 8-bit area (0 waits)

```
              T1      T2      T1      T2      T1      T2      T1      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
A (byte)   --<     +0        X     +1        X     +2        X     +3    >---
D7:0 (BSC) --<     b0        X     b1        X     b2        X     b3    >---
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______
                 v               v               v               v
```

One REQ, four window cycles (one per sub-T1), lane order endian-mirrored
(tables 10.7-10.11).

### A9 — DMAC dual-address transfer (two ordinary cycles, 0 waits)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
bus state    (idle) | T1r     T2r   | (DMAC) | T1w     T2w   |
D          ----------------- <  Dr  >--------< Dw  (BSC)    >--
o_MON_REQ  ______/‾‾‾\___________________/‾‾‾\_________________
                 R unit                   W unit
o_MON_WR   ‾‾‾‾‾‾\_______________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   0 = read (R) -> 1 = write (W)
o_MON_DE   __________________/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_________
                             T2 (read)       T1 (write)
```

Indistinguishable from CPU traffic; earliest spacing shown.

### A10 — DMAC single-address dev→mem (write-shaped, device-sourced)

```
              T1      Tw      T2
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
CSn/DACK   ‾‾\___________________/‾‾‾‾‾   DACK framed on CSn (p.363)
WEn        ‾‾‾‾‾‾\_______________/‾‾‾‾‾
D (device) ------<      data      >----   DMA device drives, BSC tri-stated
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __/‾‾‾‾‾‾‾\_________________   T1 window, identical to A6
                 v latch the SOURCE data from D (held through the DACK frame)
```

---

## 6. Waveform catalog — sequences

Full timelines with REQ, responses, and both units' DE. `acc` rows show the
`valid && ready` cycle whose closing edge is the accept. Key fabric facts
used: single ordinary writes are not posted; the arbiter is one-outstanding
and re-grants only at idle; new heads are engine-gated.

### B1 — Ordinary single write → SDRAM read

```
core edge    e0  e1  e2  e3  e4  e5  e6  e7  e8  e9  e10     e12     e14     e16     e18
i_CLK      __/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________/‾‾‾\___________________________________________
             A @e1                           B — ≥2 cycles after rsp A, never e2
o_MON_REQ  ______/‾‾‾\___________________________/‾‾‾\_______________________________________
             A: 1 i_CLK at accept edge           B: same
o_MON_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________________________________
                  1 = write (A)                       0 = read (B)
bus state            | T1    | T2    | idle  |eng_go | disp  | ACTV  | READ  | (CL)  |
rsp        __________________________/‾‾‾\___________________________________________________
                             rsp A = ord_done (single ord writes NOT posted)
D          ----------< A write data >----------------------------------------<  d0  >--------
o_MON_DE   __________/‾‾‾‾‾‾‾\_______________________________________________/‾‾‾‾‾‾‾\_______
               A: T1 window                           B: data window (rise-to-rise) v   ^
```

### B2 — Ordinary single read → SDRAM write (posted)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________/‾‾‾\___________________________________
             A @e1                           B
o_MON_REQ  ______/‾‾‾\___________________________/‾‾‾\_______________________________
o_MON_WR   ‾‾‾‾‾‾\_______________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  0 = read (A)                        1 = write (B)
bus state            | T1    | T2    | idle  |eng_go | disp  | ACTV  | WRIT  |
rsp        __________________________/‾‾‾\_______/‾‾‾\_______________________________
                             rsp A (ord_done)    rsp B: POSTED at the accept — before B's pins!
D          ------------------<  Da  >--------------------------------<  Db  >--------
o_MON_DE   __________________/‾‾‾‾‾‾‾\_______________________________/‾‾‾‾‾‾‾\_______
                       A: T2 window v                        B: WRIT-cycle window
```

The posted ack means response order runs ahead of pin order; **REQ order
always equals pin order**, so the consumer queues on REQ, never on responses.

### B3 — Ordinary burst-ROM fill (4 beats) → SDRAM head (tightest ord→SDRAM gap)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________/‾‾‾\___________________________
             A @e1                                           B: after last rsp + turnaround
o_MON_REQ  ______/‾‾‾\___________________________________________/‾‾‾\_______________________
o_MON_WR   ‾‾‾‾‾‾\____________________________________________________________________________
                  0 = read (A); B reloads the same value at its accept — no visible edge
bus state            | T1    | d0    | d1    | d2    | d3/T2 | idle  | disp  | ACTV  |
D          ------------------<  d0   X  d1   X  d2   X  d3   >--------------------------------
o_MON_DE   __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________________________
                                              window closes here ^ REQ B ≥1 CKIO later
```

READ/data of unit B continue beyond the right edge (A2 shape). Even in the
tightest case REQ B trails A's closing edge — no overlap.

### B4 — Ordinary burst write (cache drain) → SDRAM

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
o_MON_REQ  ______/‾‾‾\_______________________________________________________________________
             head only — beats are silent continuations
o_MON_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  1 = write (the whole drain unit)
bus state            | T1a   | T2a   | T1b   | T2b   | T1c   | T2c   | T1d   | T2d   |
o_MON_DE   __________/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______/‾‾‾‾‾‾‾\_______________
                 one window per beat's T1 (4 asserted CKIO cycles = 4 beats)
rsp        ______________________________________________________________________/‾‾‾\_______
             beats 0-2 ack immediately (posted, not drawn); beat 3 answers at envelope end
```

The SDRAM head follows as in B1: accepted ≥2 cycles after the final response.

### B5 — SDRAM single read → ordinary write (engine-tail gap)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________________/‾‾‾\___________________
             A @e1                        B PENDS at the front end -->| accepted at eng idle
o_MON_REQ  ______/‾‾‾\___________________________________________________/‾‾‾\_______________
o_MON_WR   ‾‾‾‾‾‾\_______________________________________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                  0 = read (A)                                             1 = write (B)
bus state            | disp  | ACTV  | READ  | (data)| drain | Tpc   | idle  | T1    | T2
rsp        __________________________________________/‾‾‾\___________________________________
                                     rsp A at the capture; B still blocked by eng_busy
D          ----------------------------------<  d0  >------------------------< B write data >
o_MON_DE   __________________________________/‾‾‾‾‾‾‾\_______________________/‾‾‾‾‾‾‾\_______
                               A: data window v      ^            B: T1 window
```

The GEN head-ready term requires `!eng_busy && !eng_go`: B's request waits
out the read drain + Tpc tail. REQ B trails A's window close by ~2.5 CKIO
cycles and abuts B's own T1 window.

### B6 — SDRAM posted write → ordinary read (longest pend)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
acc        __/‾‾‾\___________________________________________/‾‾‾\___________________________
             A @e1                                           B @ engine idle
o_MON_REQ  ______/‾‾‾\___________________________________________/‾‾‾\_______________________
o_MON_WR   ______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\___________________________
                  1 = write (A)                                      0 = read (B)
rsp        __________/‾‾‾\___________________________________________________________________
             rsp A POSTED at once — B's request pends from here to the engine-idle edge
bus state            | disp  | ACTV  | WRIT  | Trwl  | Tpc   | idle  | T1    | T2    |
D          --------------------------<  Da  >--------------------------------<  Db  >--------
o_MON_DE   __________________________/‾‾‾‾‾‾‾\_______________________________/‾‾‾‾‾‾‾\_______
                                         v A: WRIT window (after rsp A!)         v B: T2 window
```

REQ fires at the **accept**, not at the request — B's long pend produces no
early pulse, preserving commitment semantics.

### B7 — Minimum REQ→REQ spacing summary

| Sequence | Binding constraint | Practical minimum |
|---|---|---|
| ordinary → ordinary | rsp at `ord_done` + master turnaround | full cycle + ~2 `i_CLK` |
| ordinary → SDRAM | last rsp at final data state + turnaround | REQ B ≥1 CKIO after A's window close (B3) |
| SDRAM → ordinary | engine tail (drain/Tpc or Trwl/Tpc) | REQ B ≥2 CKIO after A's window close (B5/B6) |
| SDRAM → SDRAM | engine completion + posted-ack consumption | op + tails |

REQ pulses never merge and never overlap another unit's DE window (own-unit
coincidence: §4 overlap rule).

### B8 — Refresh interleave (silent pins)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___ ... __/‾‾‾\___/‾‾‾\___
bus state    | T1    | T2    | idle  | PALL  | (wait)  ...  | REF   | lockout
o_MON_REQ  __/‾‾‾\___________________________________ ... __________________
             ordinary read unit — the refresh burst is MON-silent (spec R2)
o_MON_WR   ‾‾\_______________________________________ ... __________________   0 = read (held)
o_MON_DE   __________/‾‾‾‾‾‾‾\_______________________ ... __________________
```

Refresh (and MRS/BRQ_PALL) occupy the pins with no REQ and no DE; the next
unit's REQ fires only after the refresh lockout releases the engine.

---

## 7. Waveform catalog — double pump

### A1 — SDRAM single read (reg_RCD=0, reg_CL=2)

```
CKIO       \___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
               | R0    | R1    | R2    | R3    |
CMD              ACTV    RD-AP   nop     nop
D (pump)   ------------------- <  D0  >----------
o_MON_REQ  /‾‾‾\_________________________________
o_MON_WR   \_____________________________________   0 = read (held)
o_MON_DE   ____________________/‾‾‾‾‾‾‾\_________
                                       ^ BSC capture (window close)
CMD(DP)    /‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾
               | ACT
                   | nop
                       | RD
                           | ACT (refresh)
                               | LO16
                                   | HI16
```
**No CV1000 game uses this setup.**

### A2 — SDRAM single read (reg_RCD=1, reg_CL=2)

```
CKIO       \___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
               | R0    | R1    | R2    | R3    | R4    |
CMD              ACTV    nop     RD-AP   nop     nop
D (pump)   ----------------------------<  D0   >---------
o_MON_REQ  /‾‾‾\_________________________________________
o_MON_WR   \_____________________________________________   0 = read (held)
o_MON_DE   ____________________________/‾‾‾‾‾‾‾\_________
                                               ^ BSC capture
CMD(DP)    /‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾\_/‾
               | ACT
                   | nop
                       | RD
                           | ACT (refresh)
                               | LO16
                                   | HI16
```
All released Cave games use this setup. Verified by a human using a MAME trace
on 2026-07-23. The tRCD for 6N/7N components is 15–20 ns, but the programmers
apparently felt RCD=0 was too short to satisfy the minimum time. This allows
a little more time for commands to be latched from the SDRAM controller and
pass through the shift stages.



### A2 — SDRAM burst read (4 beats)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___
             R0      R1      R2      R3      R4      R5      R6      R7
CMD          ACTV    RD-b0   RD-b1   RD-b2   RD-b3A  nop     nop     nop
D (pump)   ------------------<  b0  ><  b1  ><  b2  ><  b3  >----------------
o_MON_WR   __________________________________________________________________   0 = read (held)
o_MON_DE   __________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_______________
                                 v   ^   v   ^   v   ^   v   ^   4 CKIO cycles = 4 beats
```

### A3 — SDRAM single write

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
             R0      R1      R2
CMD          ACTV    WRITA   nop
D (BSC)    ----------<  D0  >----------
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __________/‾‾‾‾‾‾‾\_________
                         v latch here (or at the closing rise)
```

### A4 — SDRAM burst write (4 beats)

```
CKIO       __/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾‾‾\___/‾
             R0      R1      R2      R3      R4      R5
CMD          ACTV    WR-b0   WR-b1   WR-b2   WR-b3A  nop
D (BSC)    ----------<  b0  ><  b1  ><  b2  ><  b3  >----------
o_MON_WR   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾   1 = write (held)
o_MON_DE   __________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_________
                         v       v       v       v
```

---

## 8. Consumer contract

1. Sample all `o_MON_*` on every `i_CLK` edge. All outputs are registered —
   no phase alignment needed, no pulse can be missed.
2. On REQ=1: enqueue `{ADDR, SIZE, BURST, WR}`. One pulse = one unit; pulses
   never merge.
3. Pair DE windows to units strictly FIFO — pin order equals REQ order
   (response order does not: posted writes run ahead, B2/B6).
4. Asserted DE CKIO-cycle count per unit = physical beat count = SIZE / port
   width (BURST: line size / port width; burst-ROM per datum).
5. Writes: latch D at each window's mid fall (or closing rise — data is
   driven through the whole window; single-address cycles carry
   device-sourced data). Reads: drive D through each asserted window per the
   pin protocol; DE paces and counts the beats.
6. On any reset assertion: flush the match queue (spec R10).
7. Refresh/MRS pin activity is MON-silent — pin-decode it (spec R2).

---

## 9. Implementation notes

- All MON outputs are dedicated FFs in the BSC MON section — the port never
  loads the accept cone or engine cones with external routing (the CV1k
  campaign measured accept-cone-shape changes as Fmax-negative).
- REQ + fields: the existing `mon_req_q`/`mon_*_q` registers already
  implement this spec exactly (1-cycle pulse at the accept edge). No change.
- DE window terms mirror existing BUS_PCEN-domain state: `rd_lat` (SDRAM read),
  `sd_dq_oe` (SDRAM write; never fires for MRS), `ord_t2` (ordinary/burst-ROM
  data states), the T1/BS-state marker (ordinary writes, incl. single-address).
  Output FF: `de_q <= window_next` under `i_BUS_PCEN` — same next-state as the
  internal trackers, so no added latency and no cone loading.
- Testbench assertions: (1) no unit accepted while another REQ pulse is high;
  (2) DE transitions only at CKIO rising edges; (3) per-unit asserted
  CKIO-cycle count equals the beat count; (4) REQ precedes or coincides with
  the unit's first DE cycle, never follows it; (5) zero REQ/DE activity
  across refresh/MRS bursts; (6) MON silent across reset + consumer flush
  point.
- OOC re-check after integration (plateau-tax history: every new change shape
  costs −0.3..−0.7 ns until proven otherwise).

## 10. Decision log

| # | Decision | Rationale |
|---|---|---|
| 1 | SDRAM read internal capture stays at the closing rise | fall-capture halves device flight time; window shape identical |
| 2 | Single `o_MON_DE`; direction stays the REQ-phase `o_MON_WR` | early direction preserved; one data rail |
| 3 | REQ = 1 `i_CLK` pulse at the accept edge | earliest committed glitch-free instant; combinational export rejected |
| 4 | DE = full 1-CKIO-cycle windows on the BUS_PCEN grid (rise→rise) | beat = CKIO cycle; mid-window fall = sample point; toothed low-half variant evaluated and reverted |
| 5 | No REQ skid/pipeline | fabric one-outstanding proof; tb assertion instead |
| 6 | DMAC cycles fully uniform (single-address = T1 window) | simplicity; consumer latches source data from D |
| 7 | Narrow ports: one window cycle per physical beat | beat-granular pump; count from SIZE + port width |
| 8 | Consumer domain = core `i_CLK` (102.4 MHz in CV1k) | supersedes the 2-cycle / CKIO-grid REQ variants |
