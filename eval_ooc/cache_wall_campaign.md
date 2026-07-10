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
| Advance loop (SoC) | `{exma.gpr1_data,fwd_*_agu,second_access_agu}` → AGU adder → `early_d_req_valid` → `idex_allow`(fo 346) → `nx_read0` → GPR `portb_address_reg` | −2.500 (R3) | −2.920 | **−2.589** (R3, 3/20 seed3, was 14/20) | tail late-select LANDED (R3); residue = AGU front, measured full |
| Pair-slot enable (SoC, NEW headline) | `mawb.gpr0_data` → advance front → `pair_capture/ifid_ld` CE → `pair_inst/pair_pc` | −2.489 (R3) | (below top-20) | **−2.625** (R3 seed3 headline) | 1-bit CE cone — no late-select possible; needs front cut / floorplan |
| Wall A (SoC) | `cache\|bram_addr` → `hit_w/hit_rsp_i` → `rsp_inst` → predecode → `nx_read0` → GPR `portb_address_reg` | −2.574 (R3) | −2.813 | **−2.574** (R3 seed1 headline) | R3 removed the shared mux tail; residue = live-response data leg |
| o_TEA (SoC) | same AGU front → `idex_allow` → `i_req_fire` → `exc_handler\|o_TEA` | −2.737 | −2.737 | (below top-20 @ R3) | enable-side of the same loop; not headline |
| Wall B / FSM (SoC) | `int_pipe\|{second_access_agu,fwd_*_agu}` → `cache\|state.*` decode | −1.905 | −2.595 | −3.243/−2.816 (R3 s4/s5 headliners, placement swing) | placement-coupled to byp_q; needs floorplan / front cut |
| Wall B / byp_q (SoC) | `ma_seq\|second_access_agu` → `cache_data_bank_wt\|byp_q[3]` | −1.920 (re-baseline) | −2.220 | −1.913 (R3 s7) | SHELVED: trades against FSM, can't move plateau (R1+R2) |

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
  [2026-07-10: commissioned and LANDED as R3 — see the R3 entry at the tail of this log.
  The freeze now applies to the post-R3 tree; the lever list is empty.]

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

### Re-baseline after DMAC phases 1-2 (2026-07-08, seeds 3/4/5) — NEUTRAL
- RTL delta since the BSC Group C re-baseline: DMAC register block (`dmac.sv` +
  `dmac_channel.sv` x4: SAR/DAR/DMATCR/CHCR quads, DMAOR, full CMT counter) on a
  new BSC P-bus window 0x04000020-77, DEI->INTC wiring, the IBus_1 DMAC sideband
  (req_dack/_ch/_al, req_saddr; cache ties off), and `ibus_arb` inserted between
  the cache master and the splitter (owner flop parked on CPU, single 2:1 mux
  with registered select, DMAC leg tied off). Suites 67/67 + 103/103, laws
  bit-exact (the arb's zero-cost idle path held).
- Worst slack (i_CLK): seed3 −2.92→−3.03 (−0.11), seed4 −3.22→−3.20 (+0.02),
  seed5 −2.78→−2.70 (+0.08). Mean −2.97→−2.98; every delta deep inside the
  ±0.4 ns fit-noise band. Restricted Fmax 76.78 / 75.76 / 78.76 MHz.
- Cones: **zero arb/dmac logic in any seed's top-20** — the plateau is the same
  int_pipe forwarding/pair + AGU set (u_agu_d, fwd_lane_b_agu, pair_pc/
  pair_capture, gpr_2r2w, nx_read0). The 2:1 arb mux on the reqn/addr class is
  timing-invisible at this fit; the frozen cache-wall verdict stands.
- Resources: registers 7,018/6,985/7,002 (baseline 6,516/6,520/6,509 — the
  ~+490 is the DMAC regfile + CMT + arb owner/track flops), memory bits
  identical (158,208).
- Note: these runs flagged Quartus 10240 latch-inference on the parameter-absent
  CHCR bits (reset-only assignment when the feature param is false) —
  functionally benign (bits sweep to GND) and fixed in-tree right after by
  gating the assigned VALUE instead of the assignment; next OOC should log clean.
- Config note: the three new sources were added to eval_ooc/HS3/config*.json.

### Re-baseline after DMAC phases 3-4 (2026-07-09, seeds 3/4/5) — NEUTRAL
- RTL delta since the phases-1-2 re-baseline: the full DMAC transfer engine
  (auto/CMT/external-DREQ requests, dual-direct + single-address units, fixed
  priority, cycle-steal/burst, channel iteration datapaths), the arb rsp-done
  owner-flip fix, BSC DACK windows + single-address D_OE gate, Port D pad
  merges (o_PD_FN), DREQ samplers on the CKIO-fall cen. Suites 75/75 +
  103/103, laws bit-exact throughout.
- Worst slack (i_CLK): seed3 −3.03→−2.92 (dead on the frozen baseline),
  seed4 −3.20→−2.23 (best HS3 fit recorded), seed5 −2.70→−3.79. Mean
  −2.98 vs the −2.97 baseline: NEUTRAL. Per-seed spread widened to ±0.8 —
  placement noise on the known plateau (seed5's worst path is the catalogued
  int_pipe fwd_lane/address_error/early_d_req_valid cone; identical-RTL probe
  runs have swung −2.61..−3.20 before). Restricted Fmax 79.87/81.74/72.54.
- Cones: **zero dmac/arb logic in any seed's top-20** across all three seeds.
  The engine's request/address muxes (registered grant selects, flat req
  cones through the arb 2:1) are timing-invisible as designed.
- Resources: registers 7,202/7,228/7,257 (phases-1-2: 7,018/6,985/7,002 —
  the ~+230 is the sequencer, DREQ/DRAK samplers, and single-address paths),
  memory bits identical (158,208).

### Final re-baseline after DMAC phases 5-6 (2026-07-09, seeds 3/4/5) — NEUTRAL
- RTL delta since phases 3-4: 16-byte 4-beat units (4x32 gather buffer,
  registered +4 address stepping), ch3 indirect pointer-fetch states, ch2
  source reload (SAR shadow + 4-counter in the channel), round-robin
  priority (2-bit rr_head rotation), NMIF from the INTC's qualified NMI
  edge (new intc o_NMI_EDGE port), AE address errors (grant-time alignment
  masks + in-flight rsp_fault abandonment). Suites 83/83 + 103/103, all
  laws bit-exact throughout.
- Worst slack (i_CLK): seed3 −2.92 (dead on the frozen baseline again),
  seed4 −3.98, seed5 −2.30. Mean −3.07 vs the −2.97/−2.98 historical
  means: NEUTRAL within the documented plateau spread (identical-RTL
  probes have swung −2.2..−3.9). Restricted Fmax 79.58/71.55/81.27 —
  seed5's 81.27 is the best HS3 fit recorded.
- Headlines are all the catalogued CPU advance-loop family: seed3
  bram_addr→M10K address capture, seed4 fwd_lane_b_agu→AGU→M10K address
  capture, seed5 fwd_dep_a_agu→AGU→exc-MMIO o_TEA decode.
- Cones: **zero dmac/arb logic in any seed's top-20**. The specials
  (16-byte beats, pointer states, rotation mux) and abort checks
  (alignment masks off registered CHCR/SAR/DAR, flag set-priority) all
  landed off the walls; the alignment cone feeds only the grant enable,
  which was already a multi-level IDLE-only qualifier.
- Resources: registers 7,430/7,343/7,402 (phases 3-4: 7,202/7,228/7,257 —
  the ~+180 is the 4x32 buffer, rr_head/beat/sz16 state, sar_init shadow
  + ro_cnt on ch2, and the AE/NMIF plumbing), memory bits identical
  (158,208). Latch-inference log clean (the phases-1-2 10240 note stays
  fixed).
- DMAC campaign CLOSED: all six plan phases landed with per-phase OOC
  gates; the DMAC never entered a top-20 path at any phase.

### Post-merge re-baseline — full DMAC incl. phase-7 burst envelope (2026-07-09, seeds 1/3/4/5/7) — NEUTRAL
- Context: DMAC merged to `main` (`e1c1658 add dmac (#5)`). This is the FIRST OOC of
  the phase-7 ordinary-path burst envelope (bsc.sv `obuf` prefetch + `req_burst` from
  the 16-byte DMAC units + the ibus_arb fault-vs-`burst_open` clear) — the pending
  item 7.6 (`[[dmac-progress]]`) is now closed. FIVE seeds this pass (added 1 and 7 to
  the standard 3/4/5, matching the core noise-floor set) via
  `quartus_ooc.py all eval_ooc/HS3/config{,_s1,_s4,_s5,_s7}.json`, run in parallel.
- Worst multicorner slack @ 10 ns / restricted Fmax:
  seed1 −2.898 / 77.53, seed3 −2.920 / 77.83, seed4 **−2.673 / 78.91 (BEST SLACK)**,
  seed5 −2.716 / 78.64, seed7 −2.960 / 77.16. Mean −2.833 vs the −2.97/−2.98/−3.07
  historical means: NEUTRAL. Cluster spread 0.287 ns — the whole 5-seed set fits
  inside the documented ±0.4 fit-noise band. seed4 swung −3.98→−2.673 and seed5
  −2.30→−2.716 vs the phase-6 run on ~identical RTL: pure placement noise, exactly the
  plateau the doc warns about (judge by cones, not coarse Fmax).
- Headlines are FIVE DIFFERENT catalogued CPU cones — one per seed, the plateau
  signature: seed1 cache-tag `di_q` → GPR read-ahead M10K address (Wall A); seed3
  `fwd_dep_a_agu` → `fetch_pending_pc` (advance loop / AGU front); seed4 `fwd_lane_b`
  → EX adder → `exma.gpr0_data` (operand→EX result, ~10 LUT levels); seed5
  `fwd_lane_b_agu` → `exc_handler|o_TEA` (advance front → exception-MMIO decode); seed7
  `second_access_agu` → cache `byp_q` (Wall B WT-bypass).
- Cones: **ZERO dmac/arb/bsc/peripheral logic in ANY of the 5 seeds' top-20.** The
  phase-7 burst envelope (obuf 4:1 prefetch into the OWN_GEN rsp leaf, ordp_env/cnt
  trackers, `req_burst` on the 16-byte units) lands entirely off the walls, like every
  prior BSC change — item 7.6's "expect neutral" prediction confirmed.
- Resources (best seed 4): 9,828 / 41,910 ALMs (23 %), 7,617 registers (phase-6:
  7,430/7,343/7,402 — the ~+200 is the burst-envelope trackers + obuf twins), block
  memory bits 158,336 (phase-6: 158,208 — the +128 is the `obuf[0:3]` 4×32-bit
  prefetch buffer, inferred as a small RAM, Quartus 276020). Latch-inference log CLEAN
  (the phases-1-2 10240 fix holds); zero errors; warnings all benign (unused-signal
  lint, LogicLock-unlicensed, OOC pin/SDC-filter notes).
- Verdict: NEUTRAL — the frozen cache-wall plateau is unchanged and the DMAC (all 7
  phases) remains timing-invisible. Best-slack fit **seed 4, −2.673 ns / 78.91 MHz** is
  the reference used to refresh docs/HS3_Core_Hardware.md §6.

### R3 — nx_read0 tail late-select (2026-07-10, seeds 1/3/4/5/7) — **KEPT**
- The commissioned "one unspent lever": the GPR read-address tail in `int_pipe.sv`.
  Before, the late selects `pair_serve`/`ifid_ld` (both fold `id_issue`→`idex_allow`
  fo 346, plus the cache hit resolve via `if_accept`) crossed the `nx_inst`/`nx_pd`
  shared muxes, the `active_gpr_id` map LUTs, and the `rib`/`need_a` muxes before the
  M10K `portb_address_reg` pin. Now each arm's addresses are precomputed from its OWN
  inst/pd (`nx_pair_*`/`nx_rsp_*`/`nx_hold_*`; pair/hold arms fully registered, rsp arm
  = the live response, Wall A's data leg) and the selects cross exactly ONE 3:1 mux
  level at the pin (2 sel + 3 data = 5 inputs/bit). Selects are merge-blocked
  `(* keep *)` twins for the GPR cluster (`pair_serve_gpr`/`ifid_ld_gpr` re-rooted on a
  private `idex_allow_gpr`/`id_issue_gpr`; the ifid_ld twin folds its kill terms into
  `i_rsp_ready`, whose drop/redirect/branch arms drop out dead). The shared `nx_inst`/
  `nx_pd` muxes remain for the hz_* flop captures; a translate_off assertion pins the
  late-select composition to the shared-mux original every cycle.
- Verification: cpu_core_tb 103/103 + HS3_tb 83/83, cycle laws exact (in-suite),
  equivalence assertion silent. No IPC change (pure combinational restructure).
- Worst multicorner slack @ 10 ns / restricted Fmax:
  seed1 −2.574 / 79.53, seed3 −2.625 / 79.21, seed4 −3.243 / 75.51,
  seed5 −2.816 / 78.03, seed7 **−2.192 / 82.02 (best HS3 fit ever recorded)**.
  Mean −2.690 vs the −2.833 post-merge baseline (+0.143 as a mean — inside noise;
  the verdict is the cone evidence below, per the doc's own rule).
- Cone verdict — the target class is structurally cut:
  - seed3: `portb_address_reg` captures 20/20 → **3/20** (−2.589/−2.534/−2.504); the
    survivors route through `ifid_ld_gpr` = the ONE-level select (8 hits in
    critical_paths.rpt), so what remains is the measured-full advance FRONT, not the
    tail. New seed3 headline = the same front re-capturing at the pair-slot CEs
    (`pair_inst`/`pair_pc`, −2.625) — a previously-below-headline plateau member,
    promoted exactly as the re-baseline note predicted ("does NOT beat the plateau").
  - seed1: Wall A headline (`bram_addr`→`portb_address_reg`) −2.898 → −2.574: the
    shared-mux tail it used to cross is gone; residue is the live-response data leg.
  - seeds 4/5: Wall B/FSM headliners (−3.243/−2.816) — the documented
    placement-coupled swing (identical-RTL probes −2.2..−3.9); zero R3 logic in cone.
  - seed7: −2.192/82.02 with a diverse residue panel (FSM, operand forward, byp_q) —
    the predicted "−2.2..−2.4 plateau (~80 MHz)" ceiling realized.
- Resources (seed3): 9,927 ALMs (24%) vs 9,828 baseline (+~100 = 3-arm id-map
  duplication + twins), registers 7,780 (fitter-duplication noise band), block memory
  bits identical 158,336. Latch-inference log clean.
- Verdict: **KEPT.** The advance-loop→nx_read0 tail and Wall A's shared tail are
  eliminated as classes; the plateau successor is the pair-slot ENABLE cone (1-bit CE,
  no late-select applies) + Wall B/FSM — both need a floorplan (LogicLock unlicensed
  here) or an AGU-front cut. No RTL levers remain on the list; RTL RE-FROZEN post-R3.
