# Timing campaign — cache-side walls @ 100 MHz on Cyclone V (5CSEBA6U23I7)

Successor to the forwarding campaign (see memory `exhead-forward-dse` / `v3-exhead-handoff`).
v3 EX-head forwarding is committed (`f9aae05`) and retired the forwarding classes from
every top-20; the plateau is now set by two cache-side walls. This log tracks their
attack. **A round that isn't logged didn't happen.**

## Ground truth
- Branch / baseline commit: `dse` @ `f9aae05` (v3 EX-head forwarding).
- Verification gate: `cpu_core_tb` PASS (77) + `HS3_tb` PASS (53), bit-exact, and IPC
  laws exact: bypass 201/706=0.284, cache-hit 1137/1167=0.974, store 414/848=0.488.
- Flow: `python3 eval_ooc/tools/quartus_ooc.py all eval_ooc/HS3/config.json` (SoC, ~7 min,
  Wall B lives here) / `... eval_ooc/cpu_core/config.json` (core, ~4 min, Wall A). Docker
  `raetro/quartus:17.0`. `sta` subcommand re-runs TimeQuest on the existing db (no refit).
- Constraint audit: `HS3_quartus_ooc.sdc` — clocks i_CLK(10 ns)/i_EXTAL2(async), M10K
  RDW write-port datain/we false-paths (verified: the byp_q **read**-side path and the
  byp_q FF itself are NOT false-pathed → Wall B is a real, constrained path, not fiction).
- Noise floor: core seeds 1/3/5/7 = −3.1 / **−2.472** / −2.8 / −2.8 (handoff §4) ⇒ spread
  ~0.6 ns, so ±~0.3 ns around seed 3. SoC noise floor not yet independently swept (only
  seed 3 = −2.252); judge SoC rounds primarily by CONE ELIMINATION, not coarse Fmax.
- 85 MHz gate ⟺ worst slack ≥ **−1.765 ns** @ 10 ns.

## Class table (update every fit)
| Class | Launch → capture | Best ever | Prev | Current | Status / next lever |
|---|---|---|---|---|---|
| Advance loop (SoC, NEW) | `{exma.gpr1_data,fwd_*_agu,second_access_agu}` → AGU adder → `early_d_req_valid` → `idex_allow`(fo 346) → `nx_read0` → GPR `portb_address_reg` | −2.920 | (below top-20 @ f9aae05) | **−2.920** (re-baseline, seed3) | only unspent lever: tail late-select (see re-baseline note) |
| Wall A (SoC) | `cache\|bram_addr` → `hit_w/hit_rsp_i` → `rsp_inst` → predecode → `nx_read0` → GPR `portb_address_reg` | −2.728 | −2.461 (core) | −2.813 (SoC seed3) | same capture as advance loop; shares any nx_read0 tail fix |
| o_TEA (SoC, NEW) | same AGU front → `idex_allow` → `i_req_fire` → `exc_handler\|o_TEA` | −2.737 | (below top-20) | −2.737 | enable-side of the same loop; not headline |
| Wall B / FSM (SoC) | `int_pipe\|{second_access_agu,fwd_*_agu}` → `cache\|state.*` decode | −1.905 | −1.905 | −2.595 (re-triage, worst now elsewhere) | placement-coupled to byp_q; needs floorplan / front cut |
| Wall B / byp_q (SoC) | `ma_seq\|second_access_agu` → `cache_data_bank_wt\|byp_q[3]` | −1.920 (re-baseline) | −2.220 | −1.920 | SHELVED: trades against FSM, can't move plateau (R1+R2) |

**Wall B has two capture flavors off one shared AGU-request front.** The expensive
common cone is `{second_access_agu, mawb.gpr0_data, fwd_*_agu} → always20 (2nd-pending
gate) → Mux362 (AGU base mux) → AGU adder → tag_raddr` (~data_raddr). It then fans to:
(a) `byp_q` (data-bank WT bypass) — dominates the **SoC** top-20; (b) `state.S_FLUSH` /
`S_STORE_WR` FSM next-state — dominates the **core** top-20. R1 attacks only flavor (a)'s
tail. A later round attacking the SHARED front (tag_raddr IC / AGU handoff) would move
both flavors at once but touches AGU/forwarding placement — higher risk, do it after the
cheap tail cuts. Note: the two fits triage the same physical cone differently
(byp_q worst in SoC, FSM worst in core); expect R1 to help the SoC number and leave the
core FSM flavor for its own round.

## Measured Wall B cone (fit#0, SoC seed 3, `HS3/runs/quartus_cyclonev_100_dse`)
Every one of the top-20 SoC setup paths is this class. Worst = −2.220 ns, **9 logic
levels, 66 % data interconnect**. Node-by-node (arrival ns → incr):
```
4.781  launch  second_access_agu~DUPLICATE (ma_seq)          X29_Y31
6.144  +1.36   always20~1  (ma_second_pending gate)          X25_Y31   IC 0.877
8.125  +1.98   Mux362~0    (AGU base mux select)             X33_Y27   IC 1.472  <== big
8.412  +0.29   agu_x[3]                                      X33_Y27
10.18  +1.77   o_ADDR[3]   (AGU adder carry-chain start)     X42_Y27   IC 1.022
10.57  +0.39   o_ADDR[4..6] carry                            X42_Y27
12.23  +1.66   tag_raddr~12 (= data_raddr index, from AGU)   X42_Y21   IC 1.179  <== big
13.45  +1.21   g_way[3]|Equal0~2  (10-bit i_WADDR==i_RADDR)   X45_Y21   IC 0.859
14.20  +0.76   g_way[3]|Equal0~4                              X45_Y21
14.54  +0.33   g_way[0]|byp_q~4  (AND i_WE)                   X45_Y21
16.08  +1.55   g_way[2]|byp_q~2  (AND i_BWE[3])              X39_Y17   IC 1.470  <== big
16.34  +0.26   g_way[2]|byp_q[3]  capture                     X39_Y17
```
Structural reading: the launch is the v3 AGU control FF (`second_access_agu`, forces the
`agu_base_q` leg on a 2nd access); it selects the AGU base mux, drives the adder, and the
AGU-computed read index (`data_raddr = req_addr[11:2]`) crosses into the cache and feeds
the write-through-bypass collision compare `i_WADDR == i_RADDR` in `cache_data_bank_wt`.
The compare is logically IDENTICAL across all four ways (`data_raddr`/`data_waddr` are the
same nets for every `g_way[*]`; only `i_WE`/`data_we[gw]` differs). The fitter CSE-merges
it into ONE compare (nodes named in `g_way[3]`/`g_way[0]`) and routes the result to
`g_way[2]`'s `byp_q` FF across a **1.470 ns inter-bank hop** (Y21→Y17). Three ~1.5 ns
interconnect hops dominate: Mux362 (AGU-side), tag_raddr (AGU→cache handoff), byp_q~2
(inter-bank scatter). 66 % of the data delay is routing, not cells.

## Re-baseline — 2026-07-05, current tree (86da99e + exc_handler manual-reset sync fix)
Fit of `HS3/runs/quartus_cyclonev_100_dse` (seed 3, AGGRESSIVE PERFORMANCE) on the tree
with fetch-pair + interrupt-precision + int×cache-collision fixes landed since `f9aae05`.
**Worst −2.920 multicorner (77.4 MHz)** vs −2.252 at the f9aae05 baseline. The top-20 is
now two classes, BOTH capturing at the GPR read-ahead address regs (`nx_read0` →
`portb_address_reg`): the NEW advance loop (14/20) and Wall A (6/20). byp_q/FSM left the
panel (probes: byp_q −1.920, FSM −2.595 — re-triage around the new headline class).
- Worst-path anatomy (probe + summary.json): 8 levels, 60 % IC, 12.18 ns data delay.
  Head = the SAME measured-full AGU front (GPR data → `Mux374` base mux 1.54 IC → adder).
  New middle/tail = `o_ADDR[1:0]` → `early_d_req_valid` (now also gated on
  `!i_REDIRECT_VALID` and same-edge `d_rsp_fault` — interrupt-correctness terms, spec
  per [[no-bubbles-ipc-first]]) → `idex_allow` (fanout 346) → `ifid_ld_dat` (fo 72,
  already a placement duplicate) → `nx_pd.rib` → `nx_read0` → 1.04 IC → M10K addr reg.
- The −0.67 delta vs f9aae05 bought IPC 0.284→0.401 bypass / 0.488→0.554 store plus 8
  interrupt bugs — a correct trade under the IPC-first law. SoC noise ±~0.3-0.4 means the
  exact size is fuzzy; the class swap is real.
- **LogicLock is NOT licensed** in `raetro/quartus:17.0` (warning 292013) — plateau
  lever 1 (floorplan) is unavailable in this flow.
- Only unspent RTL lever: a TAIL round on `nx_read0` — precompute the 3 arm candidates
  (pair_serve / ifid_ld / hold) from registered state and late-select at the M10K address
  pin with per-cluster `idex_allow`/serve duplicates near the GPR M10Ks (last-level-mux
  pattern from the halfcycle campaign). Ceiling if fully successful: back to the measured
  −2.2..−2.4 plateau (~80 MHz) — it does NOT beat the plateau; the AGU front (measured
  full), FSM (−2.6) and o_TEA (−2.74) queue directly underneath. 85 MHz gate (−1.765)
  not reachable on current evidence.
- **Verdict: no definite headroom. RTL FROZEN at the current tree** unless the tail
  round is explicitly commissioned to recover the ~80 MHz number.

## Standing walls (measured-full — do NOT re-attack without new information)
- (none yet)

## Measured negatives (reverted experiments — do NOT re-try)
- **byp_q per-way `(* keep *)` compare** (R1, fit `HS3/runs/quartus_cyclonev_100_dse_r1`,
  seed 3): the lever WORKED for its class — byp_q −2.220 → **−2.000** (+0.220). But it
  net-REGRESSED the design: the 3 extra un-shared 10-bit comparators congested the cache
  region and blew up the co-located FSM next-state cone −1.905 → **−2.776** (−0.871),
  making the SoC worst −2.776. Reverted (`cache_mem.sv` restored to `f9aae05`). Lesson:
  the cache region is CONGESTION-bound, not logic-bound — do NOT add area (duplication)
  there; the next lever must REDUCE cache-region cost. Also: byp_q had only ~0.3 ns of
  headroom over the FSM cone, so it was never the real ceiling — do not re-attack byp_q
  until the FSM flavor is retired and byp_q is measured binding again.

## Measured FSM cone (baseline `HS3/runs/quartus_cyclonev_100_dse`, probe_fsm.rpt, seed 3)
FSM `state.*` worst = **−1.905** (2nd class at baseline; becomes worst once byp_q is
touched). Path: `second_access_agu` → `Mux355` (AGU mux) → `u_agu_d` adder
`o_ADDR[10..14]|cout/sumout` (carry all the way to region/MMIO high bits) → cache
`Equal15~2/~3/~4` (a 3-level address-region decode) → `state~34/~35` → state FF. 9 levels.
The decode `Equal15` consumes the FULL AGU-computed high address bits, so the whole
carry chain (o_ADDR up to bit ≥14) is IN the FSM next-state cone. The request's
region/cacheability class is, however, largely knowable from registered request fields —
it need not be re-derived from the fresh AGU sum each cycle.

## Designed next steps (written before they're built)
- **Round 2 (recommended): shallow-grant the FSM next-state.** Cut the AGU high-bit carry
  chain out of the `Equal15` region decode: pre-classify the request region/cacheability
  from registered fields (or a narrow early address slice) so the FSM next-state
  late-selects on a NARROW decode instead of the full AGU sum. Target: retire the
  `o_ADDR[10..14] → Equal15 → state` dependency; must NOT add cache-region area (see R1
  negative) and must NOT touch v3 forwarding logic (Mux355 select is v3 territory —
  physical/placement only there). Gate bit-exact; watch the 3 dclk deadlock lessons.
- byp_q is a LOW-VALUE target (0.3 ns headroom + congests FSM) — shelved, not a wall.
- Wall B, after R1: attack the AGU→cache handoff `tag_raddr~12` IC (1.179) and/or the
  `Mux362` AGU base-mux IC (1.472). Mux362 is v3 forwarding territory — do NOT reopen the
  forwarding *logic*; only physical duplication/placement of the grant→index net is fair.
- Wall B alt: check whether the byp_q collision needs the full 10-bit {index,word} compare
  or only the low index/word bits the store-write-back actually collides on — cutting the
  AGU carry-chain high bits out of the compare cone would shorten the late input.
- Wall A (core): shorten the cache I-response way/word select feeding predecode, or feed
  the GPR read-ahead from a narrower early slice of `rsp_inst`.

## Rounds
### Round 2 — byp_q compare HOIST (one shared copy, no added area)  (fit `..._dse_r2`, REVERTED)
- Hypothesis: R1 proved byp_q is scatter-bound (it improved +0.220) but its per-way
  DUPLICATION added 3 comparators and congested the FSM cone (−0.871). The opposite
  structure — HOIST the `i_WADDR==i_RADDR` compare to ONE copy in `cache.sv`
  (`data_collide`), 1-bit `i_COLLIDE` to all four banks — is area-NEUTRAL; predict byp_q
  improves WITHOUT the FSM collateral, so SoC worst drops below −2.220.
- Change: `cache_mem.sv` `i_COLLIDE` port on `cache_data_bank_wt`; `cache.sv`
  `wire data_collide = (data_raddr == data_waddr)` once, wired to all 4 banks. Bit-exact.
- Regression: cpu_core_tb 77/77 + HS3_tb 53/53, IPC laws exact ✓.
- Result (probe both dbs, default corner): **byp_q −2.220 → −1.956 (+0.264, best byp_q
  yet — even better than R1's −2.000)**, but FSM `state.*` −1.905 → **−2.245 (−0.340)**.
  Multicorner worst (summary.json) −2.252 → −2.396. byp_q left the top-20; the FSM flavor
  (`state.S_DBYP_REQ`) is the worst again.
- Verdict: **REVERTED.** The hoist beat duplication for byp_q AND cut the FSM collateral
  in half (−0.340 vs R1's −0.871) — but the FSM cone STILL regressed and the overall worst
  stayed flat/within-noise. Two rounds now agree: byp_q and the FSM cone are
  PLACEMENT-COUPLED in the same congested cache region; any change that shifts byp_q's
  placement ripples into the FSM cone, which gives back the gain. See standing wall below.

## Standing wall — cache-region placement-balanced plateau (measured R1+R2)
The SoC/core worst sits on a ~−2.2..−2.4 plateau shared by TWO placement-coupled cache
cones off one AGU-request front: `byp_q` (WT bypass) and `state.*` (FSM next-state via a
region/MMIO decode). R1 (duplicate) and R2 (hoist) BOTH improved byp_q (to −2.00 / −1.96)
but BOTH regressed the FSM cone (−2.776 / −2.245), netting flat-to-worse. The front is
66 % interconnect spanning int_pipe AGU (Y27–31) ↔ cache (Y26): the delay is DISTANCE +
the live-address classification the no-handshake folded lookup requires, NOT reducible
logic depth. This matches doc §5's congestion verdict and R1's area lesson. **byp_q is
SHELVED — it trades against FSM and cannot move the plateau alone.**

Levers that COULD move the plateau (all bigger / need a decision):
1. **Floorplan (LogicLock):** pin the AGU-output/request cluster and the cache FSM/banks
   into adjacent regions to kill the int_pipe↔cache interconnect and decouple the two
   cones. Doc §5's recommended lever. Tooling/iteration cost; not an RTL edit.
2. **Shorten the SHARED front:** the AGU→`tag_raddr`/region-decode handoff feeds BOTH
   cones; cutting it helps both at once. But it is v3/forwarding-adjacent (the launch is
   `second_access_agu`/`fwd_*_agu` → `Mux355/362` AGU base mux) — physical/placement only
   per the v3 fence; logic changes there reopen forwarding (see [[v3-exhead-handoff]]).
3. **C6 speed grade** if the board allows (doc §5).

### Round 1 — byp_q per-way local compare  (fit `..._dse_r1`, REVERTED)
- Hypothesis: Wall B's tail is a physical inter-bank scatter, not depth. The
  `i_WADDR == i_RADDR` collision compare is shared across all four `cache_data_bank_wt`
  instances (identical operands), so the fitter computes it once and routes the 1-bit
  result to the far bank's `byp_q` across a 1.470 ns Y21→Y17 hop. Forcing a **per-way
  `(* keep *)` compare** gives each `byp_q` its own compare co-located with its bank RAM,
  which should retire the `byp_q~2` inter-bank IC.
- Change: `cache_mem.sv` `cache_data_bank_wt` — per-instance
  `(* keep *) logic collide = (i_WADDR == i_RADDR)`, used in the `byp_q` capture.
  Bit-exact by construction (same boolean, blocks only cross-instance CSE).
- Regression: cpu_core_tb 77/77 + HS3_tb 53/53, IPC laws exact ✓ (bit-exact confirmed).
- Result (probe on both dbs, seed 3): **byp_q −2.220 → −2.000 (+0.220, hypothesis
  CONFIRMED for its class)**, but FSM `state.*` −1.905 → **−2.776 (−0.871 collateral)**;
  SoC worst regressed −2.220 → −2.776. byp_q left the top-20; FSM flavor now dominates it.
- Verdict: **REVERTED.** The lever proved byp_q is scatter-bound (it improved), but the
  duplication congested the cache region and the FSM cone (untouched logic) lost far more
  than byp_q gained. byp_q's 0.3 ns headroom over FSM made it a low-value target anyway.
  Net learning: attack the FSM next-state (Round 2), and never ADD area in the cache region.

### Re-baseline after BSC Group C (2026-07-07, seeds 3/4/5) — NEUTRAL
- RTL delta since the last re-measure: the whole BSC compliance campaign tail
  (Group B pin shapes, CKIO datasheet phase, §10.2 register audit, Group C:
  full AMX decode, 16-bit SDRAM bus, 8-bit ports, MCS-on-PTC pads, release
  pads + IRQOUT) — ~1,700 inserted lines, all BCEN-domain engine/pad logic.
  `HS3_ooc_top` gained the four new pad-state pins (o_RASCAS_OE/o_A_PU/
  o_D_PU/o_IRQOUT_n) so their cones register at the boundary.
- Worst slack (i_CLK): seed3 −2.92→−2.92 (±0.00), seed4 −2.86→−3.22 (−0.36),
  seed5 −2.61→−2.78 (−0.17). Mean −2.80→−2.97; every delta inside the ±0.4 ns
  fit-noise band. Restricted Fmax 78.32 / 75.62 / 78.22 MHz (best 78.3 vs
  baseline best 79.3 — coarse-Fmax cluster noise, judge by slack/cones).
- Cones: **zero BSC/peripheral logic in any seed's top-20** — every path is
  the known int_pipe forwarding/pair plateau (fwd_lane/fwd_dep/fwd_wbsel,
  mawb.gpr→pair_inst, ma_seq second_access/req_sent) + cache bram_addr.
  The Group C logic is timing-invisible; the frozen cache-wall verdict and
  the one unspent lever (nx_read0 tail late-select) stand unchanged.
- Resources: 8,363/8,340/8,346 ALMs (20%), registers 6,516/6,520/6,509
  (baseline 6,531/6,608/6,567 — fitter duplication noise swallows the ~60
  new flops), block memory bits identical (158,208).
