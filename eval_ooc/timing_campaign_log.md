# HS3 timing campaign log — OOC fits @ 100 MHz on Cyclone V (5CSEBA6U23I7)

Durable, append-only record of every OOC fit: the cache-wall attack rounds, the
per-feature re-measures (BSC, DMAC, transaction ports), and the RTL packages cleared
to land against them. Started as the cache-wall campaign — successor to the forwarding
campaign (see memory `exhead-forward-dse` / `v3-exhead-handoff`). v3 EX-head forwarding
is committed (`f9aae05`) and retired the forwarding classes from every top-20; the
plateau is set by the two cache-side walls below. **A round that isn't logged didn't
happen.**

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

### CV1k sideband + re-anchor (2026-07-18, seeds 1/3/4/5/7, branch `sideband`) — NEUTRAL as designed
- Context: ikacore_CV1k requested an early-transaction sideband (`o_MON_*`, spec at
  `~/Desktop/ikacore_CV1k/docs/sh3_sideband.md`) — one registered pulse per committed
  external transaction UNIT at its internal accept edge (bsc.sv `mon_fire =
  (fe_eng_start && !fe_sdmr) | (fe_acc && fe_gen && !ord_bcont)`), fields latched at
  fire. Observation-only; laws bit-exact (HS3_tb 84/84 incl. a NEW whole-run
  match-queue oracle: 341,575 strobes matched 1:1 with pin units, measured
  outstanding depth = 1, zero mismatches; cpu_core_tb 103/103).
- Sweep A (wrapper did NOT yet connect o_MON — sideband swept; = an identical-RTL
  re-probe of the post-R3 baseline on fresh CAD state):
  s1 −2.574, s3 −2.609, s4 −2.912, s5 −2.766, s7 −2.032; mean −2.579. The +0.111
  mean drift vs R3's logged −2.690 on identical logic re-measures the plateau noise.
- Sweep A2 (o_MON connected through `HS3_ooc_top`, `mon_*` regs confirmed in fit):
  s1 −2.912, s3 −2.758, s4 −2.253, s5 −2.502, s7 −2.883; mean −2.662. Δmean vs A
  −0.083 (inside ±0.4); per-seed swings ±0.66/−0.85 in BOTH directions = plateau
  placement noise. **Zero `mon_` cells in any seed top-20.** Headliners: bram_addr →
  (BSC fe_* live decode) → req_ready → cache `state.S_IDLE` (the handshake-loop /
  Wall B-FSM family) + S_FLUSH/S_DRAIN self-loops — all catalogued classes.
- S1 audit (CV1k §10.2 suggestion 1): inferred altsyncram already carries
  `MIXED_PORT_FEED_THROUGH_MODE = "dont_care"` for tag (`pgn1`), data (`lpo1`), LRU
  (`vdn1`) — `no_rw_check` works in this flow. The CV1k STA's
  `PORT_B_WRITE_ENABLE_REG` launch class does not exist in a faithful build; their
  fits predate R3 (their §10.1 quotes the pre-R3 catalog). Action for CV1k: re-drop
  current RTL, verify the map-report RAM parameter, keep the ramstyle string intact.

### R4 — cache-index slice twin (2026-07-18, seeds 1/3/4/5/7, branch `sideband`) — **KEPT; best mean + best single fit ever**
- The campaign's named "AGU-front cut", executed as a per-consumer duplicate (CV1k §10.2
  suggestion 2's request-side cousin): a private 12-bit copy of the WHOLE u_agu_d cone
  (base/addend forward muxes + adder, `l_addr_idx`), re-rooted on a third (* preserve *)
  select/state set (`*_idx`: fwd lane/wbsel/dep a+b, idex_is_data, ma_seq second/req_sent).
  Sole consumer: `LBus.req_addr_idx[11:2]` -> cache `grant_idx`/`grant_word` (tag/LRU/data
  RAM read index + the WT-bypass compares). Sum bits [11:0] close under [11:0] operands, so
  the twin == `ea_addr_sum[11:0]` every cycle (translate_off assertion). en mirror =
  `agu_en_mode[0]` (exact: u_agu_d runs i_R_T=0). All other req_addr consumers unchanged.
- Verification: cpu_core_tb 103/103 + HS3_tb 84/84 (incl. sideband oracle bit-identical:
  341,575 strobes, depth 1), laws exact, equivalence assertion silent. Zero IPC change
  (pure duplication).
- Worst multicorner slack @ 10 ns / restricted Fmax (vs the A2 sideband anchor):
  s1 −2.220/81.83 (+0.692), s3 −2.677/78.88 (+0.081), s4 **−2.149/80.32 (+0.104, BEST
  SINGLE FIT EVER, prev −2.192)**, s5 −2.417/80.19 (+0.085), s7 −2.508/79.57 (+0.375).
  Mean **−2.394** vs A2 −2.662 (+0.268) and vs the R3 logged mean −2.690 (+0.296) —
  EVERY seed improved vs its A2 twin; three seeds fit ≥80 MHz restricted.
- Cone verdict — the AGU->RAM-index class is structurally cut:
  - `byp_q`/`Equal0`(cache)/`tag_raddr`/`data_raddr` appear in **0/20 paths on 4 of 5
    seeds**; the `_idx` twin logic itself appears in **zero** top-20 (placed with the
    RAMs, as designed). s5's surviving "Equal0" hits are `u_exc_handler|Equal0` — a
    DIFFERENT class (below).
  - New plateau headline family = **exc-MMIO live classification**: AGU adder carry →
    the shared 4-address compare networks (`o_LMMIO_HIT_LIVE` == the case arms) → `o_TEA[*]|ena`
    (s4 headline −2.149) and → cache dispatch/`state.S_IDLE` (s5). Plus the operand→EX
    (s3/s7 `fwd_*`→`exma.gpr0_data`) and FSM/handshake-loop members rotating per seed.
- CV1k §10.2 suggestion-3 verdict (measured): the cheap 2-bit case sub-decode in
  exc_handler is a PROVABLE NO-OP — the four compare networks are shared with
  HIT_LIVE (the o_TEA CE's WE gate), so the binding adder→compare→ena arc survives it.
  The real lever for this family is SUM-ADDRESSED (carry-free) classification computed
  in int_pipe off the operand muxes and shipped like req_addr_idx — designed, NOT
  built (next session). Suggestion 4 (way-select duplicate): measured absent from all
  panels — skipped.
- Resources (s4): 10,093 ALMs (24 %; +~170 = 12-bit twin + sideband regs), 7,861
  registers, block memory bits identical 158,336. Latch-inference log CLEAN.
- Verdict: **KEPT.** RTL freeze lifted for this branch; post-R4 state = best 5-seed
  mean and best single fit recorded for this design.

### R5 — sum-addressed exc-MMIO classification (2026-07-18, seeds dse/1/4/5/7, branch `sideband`) — **MEASURED NEGATIVE, REVERTED**
- The R4-designed lever, built as specced: carry-free `base+addend==K` decode
  (per-bit "required carry-in x^y^K == neighbor's provided carry-out, K[i-1] ? x&y :
  x|y", AND-reduced; NO carry chain) for all SIX exact-address MMIO compares
  (TRA/EXPEVT/INTEVT/TEA FFD0/D4/D8/FC + CCR FFEC + CCR2 A400_00B0), computed in
  int_pipe on a FOURTH private-select copy of the u_agu_d cone (`_cls`, the R4 `_idx`
  recipe at FULL width), shipped as `LBus.req_mmio_cls[5:0]` with a translate_off
  equality assertion. All six consumer sites rewired, including the THIRD compare
  site found in the audit: exc_handler's write-commit `unique case(L_BUS.req_addr)`
  (the literal o_TEA/o_EXPEVT `|ena` endpoint), plus HIT_LIVE, lmmio_sel_q, cache
  live_is_ccr/ccr2 + bram_is_ccr/ccr2 captures.
- Verification: cpu_core_tb 103/103 + HS3_tb 84/84 first try, equality assertion
  silent both runs, sideband oracle bit-identical (341,575 strobes, depth 1). The
  change was functionally perfect - the verdict below is purely physical.
- Worst multicorner slack @ 10 ns / restricted Fmax (vs R4 same-seed):
  dse −2.488/80.08 (+0.189), s1 −2.767/78.33 (−0.547), s4 −3.113/76.26 (−0.964),
  s5 −2.742/78.48 (−0.325), s7 −2.970/77.10 (−0.462). Mean **−2.816 vs R4 −2.394
  (−0.422, outside the ±0.4 floor, 4/5 seeds worse)**. Registers 7,966 (+105).
- Cone anatomy (crit reports archived `scratchpad/C_reports/`): the adder→compare
  arc IS gone as designed - but the twin **became the new, slower headline** on 3/5
  seeds: `fwd_lane_*_cls → agu_wb fold → operand mux → match → WideAnd tree →
  o_LMMIO_EXC_WE → priority chain → o_EXPEVT/o_TEA |ena` (dse −2.487 11/20 cls
  paths, s5 −2.742 6/20, s7 −2.970 15/20). s4's −3.113 is the catalogued
  advance-front/pair-slot placement swing (0/20 cls).
- WHY it lost (the pattern-#16 boundary, measured): (1) sum-addressed classify needs
  FULL-width x and y, so ~six 32-bit operand words (exma.gpr0/1, mawb words, shadow,
  agu_base_q, src_b, fetch_pc) fan into the classification cluster where before only
  the ONE 32-bit sum routed there - on an interconnect-dominated fit that trade
  inverts pattern #15's premise; (2) ~6 soft-LUT levels (fold+mux+match+AND32) LOSE
  to Cyclone V's hard carry chain (~35 ps/bit ≈ 1.1 ns for 32 bits with zero
  inter-bit routing) in raw delay - the carry chain was never the expensive part of
  this family, the compare fan-in placement was. Contrast R4, which won because its
  slice kept 12-bit operands (narrow inputs) and a single consumer cluster.
- Residual truth for this family: the deep tail is the CONSUMER side - the
  `o_LMMIO_EXC_WE → general_reset_like/exc/nmi/int priority chain → |ena` levels in
  exc_handler. A consumer-side flatten ("no higher-priority event" pre-rail + 2-input
  ena AND) is the only unspent idea; the request-side classify is now a recorded dead
  end in both forms (S3 sub-decode = provable no-op; R5 sum-addressed = measured
  negative).
- Revert: hand-reversed edit-by-edit (branch state uncommitted-style staging);
  `git status` clean vs the pre-R5 `staging` commit 985f8ee = byte-exact restoration,
  re-verified 103/103 + 84/84. **Post-R5 state == post-R4 state (mean −2.394, best
  s4 −2.149/80.32 MHz).**

### R6' — 2-bit address_error slice twin (2026-07-18, seeds dse/1/4/5/7, branch `sideband`) — **MEASURED NEGATIVE, REVERTED; campaign closed**
- The lever surfaced by the R6 scoping audit (which killed R6-as-specced: the dominant
  S_IDLE arcs are pipe-internal — operand front → o_ADDR[0] → address_error →
  early_d_req_valid → idex_allow → S_IDLE — and never enter the BSC; the
  bram_addr→fe_* round-trip leg sat at panel position 20/20). Built as the R4 recipe
  at [1:0]: a fourth `_err` preserve set + private 2-bit operand-mux/adder slice
  (`l_addr_err`, carry-free at [1:0]) + mirrored MAC term; consumer split -
  `early_ex_fault`'s address term reads the twin (sole load = the request-valid
  cone), `ex_result` fault recording keeps the original; equality sim-asserted.
  int_pipe-only, ~11 registers + a few LUTs.
- Verification: 103/103 + 84/84 first try, assertion silent, oracle bit-identical.
- Worst multicorner slack @ 10 ns (vs R4 same-seed): dse −2.759 (−0.082),
  s1 −3.135 (−0.915), s4 −3.407 (−1.258), s5 −3.012 (−0.595), s7 −3.229 (−0.721).
  Mean **−3.108 vs R4 −2.394 (−0.714)** — worse than R5. `_err` present in the
  netlist (10 named regs, map.rpt) but in **0/20 top paths on every seed**; the
  panels are the catalogued families re-rolled worse (o_TEA/o_TRA|ena headliners
  −2.76..−3.23, s4 S_IDLE ×20 at −3.407). Reports in `scratchpad/D_reports/`.
- THE LOAD-BEARING FINDING (two rounds of evidence): R5 and R6' both regressed the
  mean −0.4..−0.7 with the new logic in ZERO top-20 paths, while identical-RTL
  re-anchors (A vs A2) moved only −0.08. The plateau is **perturbation-chaotic**:
  any netlist change - even 11 registers - re-rolls global placement across the
  flat cluster, and each additional preserve-mirror set adds D/CE load on the
  saturated enable fabric (idex_allow fo ~350, exma_allow, id_lane_*). The twin
  pattern's marginal cost now exceeds any single remaining cone's removal value.
  R4 was the last change big enough to pay for its own perturbation.
- Verdict: **REVERTED** (edit-by-edit, `git status src/` clean vs staging 985f8ee,
  suites re-passed). **CAMPAIGN CLOSED at R4** (mean −2.394, best s4 −2.149/80.32).
  Remaining ideas (consumer-side ena-tail flatten, R6 area class) are recorded but
  NOT recommended: expected effect is below the measured perturbation cost. Next
  step toward 100 MHz is physical only: LogicLock floorplan (unlicensed here),
  C6 speed grade, or wide DSE/seed harvesting on the frozen netlist.

### R7 — fetch_pending_pc CE preload (2026-07-18, seeds dse/1/4/5/7, branch `sideband`) — **MEASURED NEGATIVE, REVERTED**
- External proposal experiment 1: widen the 32-FF `fetch_pending_pc` CE from
  `i_req_fire` to `(!fetch_pending || if_accept)` (a proven factor of
  `early_i_req_raw_valid`, so every `ifid_ld_dat`-consumed value is identical;
  empty slot tracks junk). Removes `req_ready` (handshake loop), `l_is_data`
  (the `fwd_dep_a_agu` arc), `fault_hold`, `wb_fault_kill`, `rsp_pair_ok` from
  that CE cone. Shadow-register equivalence assertion at the consuming edge.
- Verification: cpu_core_tb 103/103 + HS3_tb 84/84 first try, assertion silent,
  sideband oracle bit-identical (341,575 strobes, depth 1). Zero IPC change.
- Worst multicorner slack @ 10 ns (vs R4 same-seed): dse −2.820 (−0.143),
  s1 −2.729 (−0.509), s4 −2.684 (−0.535), s5 −2.777 (−0.360), s7 −2.676 (−0.168).
  Mean **−2.737 vs R4 −2.394 (−0.343, 5/5 seeds worse)**. Registers 7,854–7,925.
- Cones: `fetch_pending_pc` 0/20 on all seeds — but it was ALREADY absent from
  the R4 panels (its headline was one seed of the 2026-07-09 post-merge baseline),
  so no binding cone was cut. Headliners = catalogued families re-rolled:
  exc-MMIO `|ena` (dse/s1/s5), **pair-slot payload CE (s4: `fwd_lane_b_agu` →
  `pair_pc`/`pair_inst`, −2.684 — LIVE, the R3-successor class)**, ma_seq
  `req_sent` (s7). Same shape as R5/R6': change lands off the walls, placement
  re-roll eats the mean. A CE *narrowing* (removal) is not exempt from the
  perturbation tax when its target class is not binding.
- Verdict: **REVERTED** (`git checkout`, byte-exact vs 8744ebe). Lesson: only
  attack classes present in the CURRENT panels; the s4 pair-slot headline is the
  one live candidate → R8.

### R8 — pair-payload CE preload (2026-07-18, seeds dse/1/4/5/7, branch `sideband`) — **MEASURED NEGATIVE, REVERTED; closure re-confirmed**
- External proposal experiment 2, the one candidate whose class was LIVE in the
  R7 panels (s4 `fwd_lane_b_agu` → `pair_pc`/`pair_inst` −2.684; also the R3
  successor). The 66-FF payload (`pair_pc`/`pair_inst`/`pair_pd`) moves to a bare
  `!pair_ready` CE — `pair_capture` provably implies an empty slot (its
  `if_accept` comes through the `!pair_ready` arm of `i_rsp_ready`) and every
  consumer is `pair_ready`-gated. Shadow-payload assertion on all valid cycles.
- Verification: cpu_core_tb 103/103 + HS3_tb 84/84 first try, assertion silent,
  sideband oracle bit-identical. Zero IPC change.
- Worst multicorner slack @ 10 ns (vs R4 same-seed): dse −2.810 (−0.133),
  s1 −2.477 (−0.257), s4 −2.791 (−0.642), s5 −2.959 (−0.542), s7 −2.863 (−0.355).
  Mean **−2.780 vs R4 −2.394 (−0.386, 5/5 seeds worse)**.
- Cones: the target class IS cut — pair payload **0/20 on every seed** (was the
  live s4 headline). But the family just rotated: **s5's new headline is
  `fwd_lane_b_agu` → `fetch_pending_pc` (R7's target class, back on the original
  RTL)**, plus S_IDLE handshake (dse/s4/s7) and operand→EX (s1). The advance-front
  composition has many 32-bit landing zones (fetch_pending_pc, pair payload,
  fetch_pc via fpc_ce, exma.gpr0_data, S_IDLE); cutting one member promotes
  another and the perturbation tax (−0.3..−0.7 mean) lands regardless.
- Verdict: **REVERTED** (byte-exact vs 8744ebe). 2026-07-18 evidence stack:
  identical-RTL re-anchor −0.08; R5 −0.42; R6' −0.71; R7 −0.34; R8 −0.39. FOUR
  independent change-shapes (twin add ×2, CE preload ×2 — including one that cut
  a live headline cone) all pay the same tax. **CAMPAIGN REMAINS CLOSED at R4**
  (mean −2.394, best s4 −2.149/80.32). Proposal experiments 3-6 (need_a arm,
  exc-write round trip, priority flatten, MAC mux fold) NOT run: all target
  classes now measured as rotation members, expected value below the tax. Path
  to 100 MHz stays physical: floorplan / speed grade / seed harvesting.

## 2026-07-22 — o_MON_DE integration re-check (Early_Monitor_Guide.md section 8)

- Change: the MON data-enable window flop in bsc.sv (`rd_td_nx` + E_WR
  next-state + ord FSM mirror into one BCEN-registered sink FF) + the
  o_MON_DE port through HS3/HS3_ooc_top (wrapper regenerated - the hand
  top had to route the new port or the cone is swept).
- Verification: HS3_tb 85/85 (new whole-run DE oracle: pin-truth equality
  every CKIO cycle, 12486 DE cycles, 3423 units beat-count-checked). Zero
  law movement, zero IPC change (pure sink FF, no feedback into any cone).
- Worst multicorner slack @ 10 ns: dse **-2.440 / 80.39 MHz** (single
  seed). Within the historical band (3-seed re-measure -2.61..-3.20, R4
  best -2.149); `mon_de` appears in **0/20** critical paths - headline is
  the familiar fwd_lane_b_agu -> AGU -> exc_handler TEA cone.
- Verdict: **NEUTRAL, KEPT** - same shape as the proven-neutral o_SB
  strobe (sink flop off already-registered state).

## 2026-07-25 — o_MEM_* merged transaction port re-measure (3 seeds) — NEUTRAL

- Change: the o_MON_*/o_MEM_* merge in bsc.sv (docs/Early_Monitor_Guide.md):
  REQ pulse + held fields gain LEN[4:0]/SADDR/registered WSTRB/CS_n, the DE
  window mirror is DELETED (replaced by the registered pulse OR of the BSC's
  own consume/accept enables), WDATA rail added, the combinational live
  mirror + cs_area generate retired. ~45 new FF bits, all off existing
  enables, no feedback into any cone. HS3/HS3_ooc_top port lists updated
  (o_MON_* gone, LEN/SADDR/DE/WDATA at the registered boundary).
- Verification: HS3_tb 85/85 + cpu_core_tb 103/103, zero law movement
  (first principle held; tb models moved to whitebox live-view taps).
- Worst multicorner slack @ 10 ns (vs R4 same-seed): dse −2.72 (−0.04),
  s4 −2.641 (−0.49), s5 −2.982 (−0.57). Mean −2.781 vs R4 −2.414 (−0.37) —
  inside the measured plateau-tax band (−0.3..−0.7, paid by every change
  shape incl. identical-RTL re-anchors) and the historical identical-RTL
  swing (−2.61..−3.20). Restricted Fmax 78.62 / 79.11 / 77.03 MHz.
- Cones: **mon_* in 0/20 paths on all three seeds.** Headliners are the
  catalogued plateau families re-rolled: dse = advance front →
  fetch_pending_pc/pair_pc/o_TEA + cache FSM→S_IDLE; s4 = fwd_wbsel/fwd_dep
  → exma.gpr0_data + S_DRAIN_REQ→S_IDLE; s5 = ma_seq second_access/req_sent
  + fwd_wbsel_a_agu → cache S_IDLE. The u_bsc cells inside dse/s4 S_IDLE
  paths (fe_sdmr~0/comb~12/fe_gen) are the PRE-EXISTING accept handshake
  (fe_gen → req_ready → cache next-state, the R8-era S_IDLE family), not
  new port logic.
- Resources: registers 7,901 / 7,898 / 7,932, memory bits 158,336 (=).
- Verdict: **NEUTRAL, KEPT** — same shape as the o_SB strobe and o_MON_DE
  precedents (dedicated FFs off already-registered state). Campaign remains
  closed at R4; this is a bookkeeping re-measure, not a round.

## 2026-07-28 — interrupt-boundary WB guard (`mawb.mem_done`) re-measure (3 seeds) — NEUTRAL/POSITIVE

- Change: `o_INT_BOUNDARY` gained a fourth term, `!wb_mem_done`. `ma_inflight` is
  EX/MA-scoped, so a memory op that has LEFT MA but not yet committed sat in MA/WB
  with its external access already accepted (a store is a notify, issued at the
  EX/MA edge; a load's response is already consumed). An acceptance edge there
  killed the WB packet and SPC pointed at it, so the access was RE-ISSUED after the
  handler — invisible in architectural state, a duplicated bus transaction on any
  device register with side effects. One new `mawb_t` bit (`mem_done`) set in
  `ma_result`, one AND at the boundary. Functional detail in the tb note below.
- Verification: cpu_core_tb 104/104 (new golden [93] counts BUS writes, not GPR side
  effects: 60-offset sweep, WT-cacheable so the pipeline actually runs back-to-back)
  + HS3_tb 85/85, zero law movement. IPC 0.401 / 0.974 / 0.554 — unchanged.
  Suite-wide window census: 79 acceptance edges over 2405 open cycles BEFORE,
  **0 / 0 AFTER**. NOTE: the bug does NOT reproduce on the bypass path — low IPC
  always bubbles between a commit and the store's WB cycle, so a bypass-only probe
  passes vacuously (0 window cycles). This is why 103 prior tests never saw it.
- Baseline discipline: HEAD re-fit in the SAME session reproduced the 2026-07-25
  numbers EXACTLY (dse -2.720, s4 -2.641, s5 -2.982), so the flow is deterministic
  per seed and these deltas are attributable, not seed noise.
- Worst multicorner slack @ 10 ns (same-session baseline -> change):
  dse -2.720 -> **-2.351 (+0.369)**, s4 -2.641 -> -2.729 (-0.088),
  s5 -2.982 -> **-2.796 (+0.186)**. Mean -2.781 -> **-2.625 (+0.156)**.
  Restricted Fmax 78.62/79.11/77.03 -> **80.97**/78.56/78.15 MHz.
- Cones: `mem_done` / `int_boundary` / `retire_int_defer` in **0/20 on all three
  seeds**. Headliners both sides are the catalogued plateau families re-rolled:
  advance front (`fwd_lane_*`/`fwd_wbsel_*`/`mawb.gpr0_data`/`second_access`/
  `req_sent`) into {S_IDLE, fetch_pending_pc, pair_pc, exma.gpr0_data, o_EXPEVT,
  bram_addr}. Direct TimeQuest probe on the dse netlist (no refit): exactly ONE
  `mawb.mem_done` FF, worst path FROM it **-0.189**, worst path TO it **+2.413** —
  ~2.2 ns clear of the headline.
- Resources: registers 7,963 / 7,879 / 7,888 (was 7,901 / 7,898 / 7,932),
  memory bits 158,336 (=).
- Verdict: **NEUTRAL, arguably positive** — and notably the FIRST change shape in
  this log that did not pay the -0.3..-0.7 plateau tax; dse's -2.351 is the best
  dse fit recorded. Read conservatively: 2-of-3 seeds better at +0.156 mean is
  inside the seed spread, so the claim is "no measurable cost", not "faster".
  Campaign remains closed at R4. **UNCOMMITTED at the user's instruction.**

## 2026-07-29 — per-packet `pair_taken` (ibara/vec_0 wrong-resume fix) — RTL LANDED, OOC re-measure PENDING

- Change: the interrupt restart-PC machinery's shared `pair_taken_q` register is
  DELETED; taken-ness now rides the packet as a `pair_taken` bit in
  idex_t/exma_t/mawb_t, set on the slot at issue (the same edge its branch
  resolves in EX — `ex_complete` guarantees same-edge) and read at that packet's
  own commit in the `arch_next_pc` mux. Root cause (probe ring, vendor stack
  t=253,306,315..355 ns): the shared flag, armed by a young taken `bra` in EX,
  was consumed one edge later by an OLDER not-taken `bt/s` pair's slot commit —
  the loop-tail idiom `bt/s`(not-taken)+slot then `bra`+slot at IPC 1. An
  interrupt at the `bra`-slot boundary then resumed at slot.pc+2; one boundary
  earlier it jumped to the young target early (skipping the pair). Every
  ibara/vec_0 symptom (the "wrong-word/wrong-line load", poisoned ADDR, lost
  DBG_MBX store, phantom fill, EXPEVT 0x100 detonation) was fall-through from
  that one wrong SPC — no data path was ever wrong. See
  ikacore_CV1k/docs/hs3_vec0_repro.md (RESOLVED section).
- Verification: cpu_core_tb **105/105** (suite renumbered: the mem_done golden
  [93] is now [94]). New golden [89] `test_int_pairtaken_hazard` — the
  bt/s-not-taken + bra loop tail under an interrupt offset sweep — fails BOTH
  polarities on the HEAD core (fall-through executed / body+slot counts lost)
  and is clean on the fix; [94] re-confirmed red on HEAD (17 bus writes for 16
  stores). HS3_tb **85/85**, zero law movement, IPC 0.401/0.974/0.554 unchanged.
  CV1k vec_0 12M-insn re-runs CLEAN on BOTH stacks (`v0_fix_run.log`,
  `ms_vec0_fix_run.log`): 12M cap reached at t=608 ms (2.4x past the old
  fatal), no pump %Fatal, probe canary 0 hits, zero `8c000100` entries and zero
  `0c0029a4` fall-throughs in the traces. The EXACT pre-fix alignment recurs at
  retire ~2.577M (TMU0 accepted at the bra+slot boundary) and RTE now resumes
  at the branch target 0c00291c.
- OOC: **NOT yet measured.** Expected shape: three pipeline-register bits on
  already-enabled edges (idex/exma/mawb loads), no new commit-mux depth (the
  select swaps a dedicated register read for a mawb field read), one register
  DELETED. Same dedicated-FF-off-registered-state family as the o_SB / o_MON_DE
  / mem_done precedents (all 0/20-cone neutral). Run the 3-seed ritual with a
  same-session HEAD re-anchor before quoting numbers; log the result here.
  **MEASURED 2026-07-30** (incidental: the RDW session's v1 s7 fit = this RTL
  plus a no-op attribute drop): worst **−2.394** — inside the flown family and
  the best recent s7. The full 3-seed exists only for the union with the RDW
  OLD_DATA change (next entry) — also neutral. Considered CLOSED.
- Verdict: **CORRECTNESS FIX, KEPT** (SH7709S interrupt semantics are law; the
  IPC-first rule is not in tension — no bubbles, no handshake beats, no new
  stall terms). Campaign remains closed at R4.
  **UNCOMMITTED at the user's instruction, alongside mem_done.**

## 2026-07-30 — cache data-bank RDW = DONT_CARE (ibara bug #3) — OLD_DATA altsyncram LANDED, 3-seed NEUTRAL

- Change: `cache_data_bank_wt` consumed the RAM q on a same-edge write/read
  collision — the UN-strobed lanes of a sub-word store — while
  `ramstyle "M10K, no_rw_check"` fitted the altsyncram with mixed-port RDW =
  **DONT_CARE** (map/fit parameter tables). Verilator models old-data, so the
  divergence is invisible to every simulator by construction: this is ibara
  bug #3 (CV1k `docs/bug3_audit_report.md` — silicon-only random corrupted
  reads, identical across two placements AND a −15% underclock, survives
  pair_taken + mem_done). The file's own claim "the colliding RAM lanes are
  never consumed" was false for sub-word stores. Fix v1 (drop the attribute)
  is a DEAD END: Quartus classes the template dual-clock (warning 276027) and
  still fits DONT_CARE — inference cannot express mixed-port OLD_DATA (its
  only matching tool, pass-through, is new-data). Fix v2 LANDED: the banks
  instantiate `altsyncram` directly (`DUAL_PORT`, byte_size 8, M10K,
  `read_during_write_mode_mixed_ports("OLD_DATA")`) under an
  `ifdef VERILATOR` split; the behavioral twin is lane-equivalent incl. the
  clocken-stall corner (masked lanes never change in the array).
- Verification: adversarial in-tree models (`HS3_RDW_HOSTILE_CACHE` /
  `HS3_RDW_HOSTILE_GPR` ifdefs invert exactly the uncovered collision lanes;
  `VLT_DEFINES` hook in run_sim.sh, `RDWHOSTILE=1` arm in CV1k build_sim.sh).
  Hostile-cache turns cpu_core_tb RED on the selfmod sweep k=3 (a MOV.W
  poke's commit edge colliding with the fetch of the same cell — the measured
  I-side consumer; GOLDEN D is the D-side spec of the same reliance).
  Hostile-GPR **105/105 GREEN** — the gpr_2r2w "shadow forward upstream"
  no_rw_check claim is empirically total, attribute KEPT there. Tag/LRU are
  proof-safe (full-entry bypass ⇒ colliding q dead), attribute KEPT. All 14
  fitted RAMs audited (BSC obuf = Quartus auto pass-through). Normal suites
  on the v2 split: cpu_core_tb **105/105**, HS3_tb **85/85**, bit-identical.
  CV1k FASTBOOT ibara control-vs-hostile traces byte-IDENTICAL at 2M AND 20M
  — but the collision counters read **6,079 exposed-collision read cycles in
  20M insns** (per way 1255/1087/1315/2422): the game hits the write/read
  same-cell alignment ~every 3.3k instructions, and consumption (same line +
  same way + lanes used) is the rare tail — the field-rate shape exactly.
- OOC (3 seeds, same-session v1 s7 re-anchor −2.394): s1 **−2.158 / 82.25
  MHz** (ties the all-time best −2.149), s7 **−2.846**, s5 **−3.312 / 75.12
  MHz** vs baseline family −2.61..−3.20 → **NEUTRAL** (plateau placement
  noise). OLD_DATA parameter verified in the map tables at every seed; banks
  in M10K (26 blocks total), fit legal, zero errors; no RAM path in any
  top-20. The OLD_DATA mode itself prices at zero here.
- Verdict: **CORRECTNESS FIX, KEPT.** Campaign remains closed at R4. Sim can
  never re-confirm this class — remaining gates are CV1k-side: srcs/rtl
  sync, refit, **ibara board soak**. Lesson for every future RAM: the fit
  report's RDW parameter is the only truth; every no_rw_check needs a
  written non-consumption proof (the hostile ifdefs are the audit harness).
  **UNCOMMITTED at the user's instruction, alongside pair_taken/mem_done.**

## 2026-08-01 — transaction-port pre-accept tiers (EREQ + PEND, CV1k tRCD menu) — LANDED, 3-seed NEUTRAL

- Change: `bsc.sv` transaction-port section grows the two pre-accept rails
  from the CV1k tRCD menu (`ikacore_CV1k/docs/hs3_early_addr_notice_spec.md`,
  reply section = the HS3 verdict): `o_MEM_EREQ/EADDR/EWR` = the REQ
  register's own D-net exported live ((* keep *) duplicate of `mon_fire` +
  `fa[28:0]`/`req_write` taps — exactly-1-edge-early pairing BY CONSTRUCTION),
  and `o_MEM_PEND/PADDR/PWR` = registered pend level, class-masked to
  REQ-pairing units (`fe_gen && !ord_bcont` | eng head/resume arm), head
  re-captured every enabled edge (arb owner flip = documented replacement).
  31 new FFs, zero handshake/launch/capture touches. Ports through HS3.sv +
  the hand OOC top; guide doc now specs the three-tier request view.
- Verification: HS3_tb **86/86** (new test 86 = early/pend whole-run oracle:
  EREQ<=>REQ both directions + field equality over 341,575 pairs, REQ never
  overlaps PEND, 2,337 pend-covered REQs PADDR/PWR-exact, own_dma flip
  exemption taken 0 times this run). Match-queue count 341,575 = the
  historical number exactly — laws bit-exact, first principle holds.
- OOC (3 seeds vs the 2026-07-30 family −2.158/−2.846/−3.312): s1 **−2.480 /
  80.13**, s7 **−2.415 / 80.55**, s5 **−2.277 / 81.45** — mean −2.391, the
  best 3-seed family since R4 (−2.394), every seed inside the plateau band;
  `mon_ereq/mon_pend/mon_paddr` in **0/20 paths on all seeds**; sta.rpt
  mtimes verified fresh (the 17.0 stale-report trap checked). Registers
  7996/7989/7924 — in-family. **NEUTRAL, KEPT.** Campaign remains closed at
  R4; the pre-accept tiers price at zero.

## 2026-08-01 — EREQ/EADDR flat-launch restructure (CV1k timing addendum) — LANDED at round 4, round 5 REVERTED

- Ask (spec doc TIMING ADDENDUM): their first Step-A c102 fit put ~7.7 ns /
  7+ serial LUT levels of the exported `mon_ereq_c` cone inside HS3 (cache
  state -> arb addr muxes -> `fe_port~2/~5` -> `fe_acc` -> `fe_eng_start`);
  same ports, same contract, launch restructured toward <=3-4 levels.
- Change (5 measured rounds, best state kept = round 4):
  1. `bsc.sv`: EREQ re-derived as a FLAT kept cone — class kernels collapse
     to `fa[31:26]` x DRAMTP (area 1 is wholly port/dmac/dummy, so the
     20-bit `fe_port`/`fe_dmac` compares provably never reach a unit-class
     accept), parallel one-LUT ready rails (`mon_gen_ok/mon_sdh_ok/
     mon_sdc_ok` + flat `mon_ebusy/mon_blk/mon_wrp_est` re-computes),
     single-LUT final. Equivalence proven every enabled edge by tb oracle E1
     (EREQ vs the registered REQ, both directions, 341,575 pairs).
  2. `ibus_arb.sv`: `o_MON_ADDR/o_MON_WR/o_MON_VLD` = kept copies of the
     request mux; EADDR/EWR now launch here (splitter passes addr/write
     untouched -> value-identical to the BSC's `fa`).
  3. raw pre-splitter valid feeds the cone (`hit_brg`'s 28-bit compare is
     redundant: its windows are P4/area-1, never unit-class).
  4. `cache.sv` `o_MON_IVLD` = kept flat dup of `I_BUS.req_valid` (states
     inlined), piped cache -> cpu_core -> HS3 -> arb monitor mux.
- Round 5 (kept LUT-half split + flat wr/burst exports end-to-end) MEASURED
  NEGATIVE and reverted: s5 put strobe cells in 8/20 top paths (`mon_addr_c
  [27]` fanout 36 — Quartus 17 merges functionally-identical nodes INTO the
  kept dup, making the "dedicated" copy the shared net), s7 fell to −3.436.
  Lesson: (* keep *) pins the net, not the cone; past ~1 dup per field the
  merger inverts the decoupling.
- Verification (final state): HS3_tb **86/86** + cpu_core_tb **105/105**;
  341,575 EREQ pairs exact, laws bit-exact (match-queue = historical count).
- OOC final (= round 4, confirmed by deterministic re-fit): s1 **−2.367 /
  80.86**, s7 **−2.521 / 79.87**, s5 **−2.756 / 78.39** — mean −2.548, best
  family mean of the campaign; strobes **0/20 all seeds**; sta mtimes fresh.
  Export-cone probes (`ereq_depth.tcl` in each run dir, wrapper `_q` regs):
  EREQ slack **+1.5/+2.3/+1.5 @ 10 ns**, 7-8 fitted levels (HS3-internal
  launch 7.2/6.3 ns incl. OOC placement IC); EADDR **4 levels, +2.8 slack**
  (the "near-registered image" delivered). The serial fe_port/fe_acc/
  fe_eng_start chain is GONE from the export.
- Residual: EREQ's 7-8 fitted levels = cache one-hot valid/write OR-trees +
  17.0 duplicate-merging re-welding kept dups (round-5 proof). The literal
  <=3-4-level target needs a REGISTERED launch = Tier 2 next-state mirrors
  (mon_de idiom scaled across cache/arb/bsc) — separate decision, offered.

## 2026-08-01 — Tier-2 registered EREQ launch (arm-riding packages) — LANDED at round 8 by USER DECISION (accepts s1 −0.63)

- Design: each master exports a preserved request PACKAGE {vld, cls_gen,
  cls_sdr, wr, bst}. The cache's rides the FSM itself — written alongside
  every one of the 14 REQ-state entries, the `vic_ld` override, and the 6
  accept exits (no condition re-derived; drift structurally impossible,
  policed by test 86 every run). Class kernels are computed into the flop
  Ds from REGISTERED addresses (`wb_pa`/`fill_base`/`bram_addr`/`cur_addr`;
  only the two live-bypass sites decode the live AGU address). DMAC leg =
  one LUT off its registered sequencer. Arb muxes packages by `own_dma`;
  BSC final = package && one-LUT ready rails. DRAMTP decodes exported
  quasi-static (`o_MON_A2SDR/A3SDR`). `(* preserve *)` on all 5 package
  FFs — REQUIRED: they are sequentially equivalent to state decodes, and
  without it Quartus deletes them and re-derives 8-level combinational
  chains (measured, round 7).
- Four measured rounds (3 seeds each, sims 86/86 + 105/105 green at every
  point; laws bit-exact, 341,575 EREQ pairs exact throughout):
  - R7 no-preserve: FFs deleted by synthesis; EREQ 8 lvl, mean −2.70. Dominated.
  - **R8 full preserve (LANDED): EREQ 4/6/4 lvl, launch slack +1.6/+1.6/+3.2
    @10 ns; s1 −2.994 (cls-capture family in top-20, −0.63 vs baseline),
    s7 −2.450 / s5 −2.670 both CLEAN and better than baseline.**
  - R9 late-select split of the live cls decode: WORSE everywhere
    (−3.24/−2.70/−2.76, all seeds contaminated). Reverted.
  - R10 partial preserve (cls released): −2.83/−2.85/−3.14, still
    contaminated 2/3. Dominated.
- Root cause of the cost: the package flops are TORN endpoints — D-cones
  anchor to the AGU carry tail (live-bypass cls) and the tag-resolve arm
  enables, Q-routes must reach the arb/BSC corner. Three shapings all
  priced −0.3..−0.9 somewhere; this is a floorplan property, not a coding
  artifact.
- Decision: the user chose the round-8 state over the neutral Tier-1
  combinational form (EREQ 7-8 lvl @ +1.5..+2.3) — the CV1k tRCD fix
  gates on launch depth, and s7/s5 improved. The no-harm law is thus
  WAIVED for s1's −0.63 on this feature by explicit user decision
  (AskUserQuestion, 2026-08-01). Tier-1 numbers remain reproducible by
  dropping the five (* preserve *) attributes + reverting the package (see
  this entry + the Tier-1 entry above).

## 2026-08-01 — Tier-2 drain-class bug (CV1k oracle rejection) — FIXED same day

- CV1k's every-edge contract oracle killed their boot at 577 us:
  `REQ without EREQ announce, addr=0c4d5200 wr=1 len=4 burst=1` — the
  FIRST write-back drain. Reproduced deterministically in their sim
  (working tree consumed live via symlink); a time-windowed probe showed
  the package head up with `vld=1 wr=1 bst=1` but **cls=00**.
- Root cause: `mon_cls_drain = mon_cls(wb_pa[27:22], ...)` — but `wb_pa`
  is declared **`logic [31:4]`** (offset range; its own comment says
  "wb_pa[31:29] is 000"), so `[27:22]` reads PA[27:22], not PA[31:26].
  For 0x0C4D5200 those bits decode area 1 -> class 00 -> EREQ never
  arms for the drain. Fix: `wb_pa[31:26]` (one line).
- Why HS3_tb's 341,575-pair oracle missed it: ADDRESS-MAP VACUITY. The
  tb's drain PAs put the wrong bits on area 0 -> class GEN, and the gen
  arm's rails happen to track SDRAM drain-head accepts on those runs -
  E1 held by coincidence of the map, not by correctness. Lesson recorded:
  an oracle that only checks port-vs-port pairing can be satisfied by two
  errors that cancel on one address map.
- Gap closed: HS3_tb E3 GOLDEN-CLASS oracle — every cycle the cache
  package is up (CPU-owned), its registered class/direction must equal a
  fresh decode of the LIVE bus head address x DRAMTP (whitebox
  `mon_pk_cls` vs golden(`u_bsc.fa`)). 377,657 checks/run; it fails on
  the first mis-classed head on ANY map. Under the old bug it trips at
  the first drain.
- Verification: HS3_tb 86/86 (laws bit-exact, 341,575 pairs), cpu_core_tb
  105/105, CV1k `MISTER=1 FASTBOOT=1` datum run clean to $finish at
  125 ms sim (216x past the failure point). OOC 3-seed re-measure: same
  cone shape (6 bits of the same register, different indices) — logged
  below when complete.
- Post-fix OOC 3-seed (the TRUE Tier-2 numbers; the round-8 figures above
  came from the buggy netlist): s1 −2.849 / s7 −2.796 / s5 −2.924 (mean
  −2.856; per-seed cost vs Tier-1 baseline −0.48/−0.28/−0.17, mean −0.31 —
  same accepted trade, placement re-roll spread the `mon_pk_cls` torn-
  endpoint family across all three panels instead of s1 alone). EREQ
  6/6/6 fitted levels @ +1.89/+0.86/+2.60 launch slack; EADDR 5/3/4 @
  +1.8..+2.5. sta mtimes fresh. FINAL LANDED STATE.

## 2026-08-01 — ibara stale-T fix (RTE running-flag resync) + interrupt torture suite

Answer to the CV1k evidence package (`ikacore_CV1k/docs/hs3_interrupt_tbit_bug.md`);
HS3-side report kept verbatim at `docs/hs3_interrupt_tbit_fix.md`.

- Root cause — one mechanism, and it is **NOT** the SSR capture. The
  report's summary guessed "SSR captured holds the pre-writer T"; its own
  silicon evidence disagrees (§5 row 2925: handler-restored SSR =
  `fffffe00`, bit 0 = 0 = architecturally correct), and the new tb
  capture-law oracle (SSR.T latched at every entry vs an architectural
  T-map of the swept loop) PASSES on the pre-fix RTL at every boundary.
  The real defect: EX never reads SR.T from ctrl_reg — it reads the
  running mirrors `r_t/r_s/r_m/r_q` (`int_pipe.sv`, "Running SR.T latch"),
  which resync only on (a) an exception-entry redirect or (b) a fully
  drained pipe. **RTE's SR restore happens inside ctrl_reg and resynced
  neither.** At cache-hit streaming:
  ```
  cycle W    RTE in WB (slot in MA)          rte_pending holds issue
  edge  W    o_RTE_VALID pulse armed
  cycle W+1  SR <= SSR lands in ctrl_reg     mawb = slot (no drain window)
  edge  W+1  target issues into ID/EX
  cycle W+2  target's FIRST instruction in EX reads r_t  <- HANDLER's T
  edge  W+2  drained-pipe resync finally lands           <- one cycle late
  ```
  So whenever the interrupt's SPC lands between a T-writer and its
  consuming branch (the `tst`/`bf` boundary — the NAND-pump geometry), the
  first post-RTE instruction IS a T-consumer and executes with whatever T
  the OS handler left behind. Wrong branch direction = exactly the §4 sim
  trace. Same hole covered S/M/Q (MAC.W saturation mode, DIV1 chains).
  `LDC Rm,SR` is NOT affected — serialization + drain resync close the
  window one cycle earlier (proven by dedicated goldens). The §5 MACL
  anomaly needs no second bug: with a stale-T branch the "wrong path"
  executes **architecturally**, so its `mul` commits MACL legitimately;
  a wrong-path-leak sweep confirms the kill gates are sound.
- RTL: one arm in the int_pipe WB region — `if(o_RTE_VALID) r_* <=
  i_SSR{0,1,9,8}` (exclusive with EX deposits: rte_pending holds issue
  that cycle, so EX is guaranteed a bubble). No new stalls, no handshake
  beats; IPC 0.401/0.974/0.554 bit-identical.
- New torture goldens (cpu_core_tb group 12c, `+torture` subset), with the
  pre-fix verdict of each:

  | Golden | Pre-fix | Post-fix |
  |---|---|---|
  | RTE flag mirror x4 (SETT/CLRT poison, MOVT/branch-first) — deterministic, no interrupts | FAIL (all 4: consumer reads poisoned mirror) | PASS |
  | LDC-SR flag mirror x2 | PASS (serialization covers it) | PASS |
  | T-clobbering-handler sweep (SETT), ibara tst/bf-pair loop | FAIL at both spc=0x5A boundary offsets (early exit, iter=0) | PASS |
  | T-clobbering-handler sweep (CLRT) + SSR.T-vs-SPC capture law | capture law PASS pre-fix (SSR was never wrong) | PASS |
  | DIV1 chain vs DIV0S/SETT handler | FAIL at 11 offsets (M/Q/T corruption) | PASS |
  | MAC.W chain vs SETS handler | PASS | PASS |
  | Wrong-path MUL/CLRMAC/GPR leak sweep | PASS (kill gates sound) | PASS |
  | Held-level RTE re-entry storm (12+ back-to-back entries) | PASS | PASS |

  19 pre-fix failures total.
- Verification: cpu_core_tb 117/117, HS3_tb 86/86, laws bit-exact.
- CV1k-side expectations handed back with the fix (§8 of the report):
  (1) datum ladder stays byte-identical — the fix is invisible without a
  flag-clobbering handler colliding with a writer/consumer boundary;
  (2) the 12.97 s event: mod loop must now complete (col=0, len=0x840),
  no `TCR <= 00000000`; (3) board soak: TCR trap silent, sprite-ID /
  asterisk / FLASH READ ERROR symptoms expected to clear together.
- OOC: re-measured with the session-2 package — see the 2026-08-02 verdict
  below (NEUTRAL, cleared to land). Expectation at the time was neutral:
  the change adds a 5th 1-bit select arm to four flag registers, and `r_t`
  cones have been absent from every top-20 panel since the re-baseline.

## 2026-08-01 (session 2) — RTE delay-slot manual conformance + suite hardening

Same-day hardening pass on top of the stale-T fix (§5 addendum of
`docs/hs3_interrupt_tbit_fix.md`): the mirror resync moved from the retire
pulse to the RTE's EX exit, so the delay slot itself already reads the
restored T/S/M/Q.

- Manual law implemented (sw manual 8.2.53 p.241: "the slot uses the SR
  restored", + section-8 Delay_Slot illegal-slot list). Five RTL deltas:
  (1) mirrors deposit i_SSR flags at the RTE's EX EXIT (replaces the
  session-1 WB-pulse arm; the slot now reads restored T/S/M/Q);
  (2) ctrl_reg RTE-restore arm MERGES same-edge retiring-slot commits
  (ctrl regs + T/S/MQ bits land on top of the restore instead of being
  dropped); (3) sr_id_view: STC-SR decoded as the RTE slot reads
  SSR&MASK; (4) ex_md_view: slot privilege check against restored MD;
  (5) bank1_nx rte_pre_wb arm: the slot's GPR read/write bank = restored
  RB (shallow decode-field select, id_issue kept OFF the address cone);
  plus NEW illegal-slot detection at the issue-time slot marking (any
  PC-changer in a delay slot -> EXPEVT 0x1A0, was previously absent).
- Suite hardening: passive flag-mirror coherence oracle (286,506
  consume-point checks/run, property-checker group), hostile handlers in
  the random INT/NMI capstone (flag-clobber + handler branches, MAC
  save/clobber/restore, SSR/SPC rewrite context-switch shape; 12 trials,
  MACH/MACL + S/M/Q added to the end-state compare), anti-vacuity
  coverage asserts on every torture sweep, and GOLDEN X8 rte-slot laws
  (6 phases). Old test_rte updated to the manual law (slot writes the
  RESTORED bank).
- tb lesson: MAC.W @R14+,@R14+ in the random mix drifts the window base
  off longword alignment -> address error -> vector slide fetches the
  program's own cached data at VBR+0x100 -> reset-like loop. MAC-state
  coverage moved to MUL.L/DMULS.L; MAC.W x S stays in the X5 sweep.
- Verification: cpu_core_tb 118/118, HS3_tb 86/86, IPC 0.401/0.974/0.554
  bit-identical.
- OOC: re-measured 2026-08-02 (entry below) — NEUTRAL, cleared to land.
  Watch items going in were: bank1_nx gained a 3-term registered-field OR
  on its select (feeds the GPR read-address cone - the sensitive family),
  sr_id_view/ex_md_view are shallow control legs, illegal-slot is one
  product in the ID illegal cone.

## 2026-08-02 — OOC verdict for the RTE/interrupt conformance package + gap tests

- 3-seed re-measure (s1/s5/s7, sta.rpt mtimes fresh): s1 −3.282/75.29,
  s5 −2.366/80.87, s7 −2.425/80.48. Mean −2.69 vs the −2.920 frozen
  baseline = NEUTRAL (slightly positive, within the ±0.4 noise band;
  s1 is a plateau-tax placement draw of the known headline class).
- Cone scan: every top-20 head is a PRE-EXISTING wall class — s1 all
  fwd_shadow_a→address_error→ex_complete→fetch/pair (the round-5 weld),
  s5 r_bank1-OUTPUT→operand-select (the historic fit4 o_SR[29] class,
  same −2.36 magnitude) + bram_addr walls, s7 AGU→cache-state. ZERO
  paths through rte_pre_wb / sr_id_view / ex_md_view / illegal-slot /
  ctrl_reg merge / mirror deposits; no endpoint lands ON r_bank1 (the
  widened bank1_nx select is invisible). PACKAGE CLEARED TO LAND.
- Coverage-gap audit follow-ups built while fitting (X9/X10/X11, all
  green = laws locked, no new bugs): RTE-target fetch fault (restored-SR
  →SSR re-capture round trip, SPC/TEA on target), killed in-flight
  LDC→GBR/SSR ctrl-write sweep (the ctrl-file twin of the GPR-leak law),
  nested hostile handlers (BL-cleared outer + clobbering inner, 2-level
  SSR/SPC save/restore chain via R0/R15 + entry-count discrimination).
- Final: cpu_core_tb 121/121, HS3_tb 86/86, IPC 0.401/0.974/0.554.
- Remaining audit items DEFERRED with rationale: SoC twin of the
  hostile-handler suite (INTC-protocol product; core level proven, SoC
  twins historically zero-bug), SLEEP×interrupt (SLEEP does not halt -
  o_SLEEP_VALID unconnected, standby feature deferred with STBCR),
  RTE-slot-fault behavior (manual p.241 disclaims: programmer must
  avoid; current recovery is exercised but the exact SPC choice is a
  documented-choice candidate), exc-MMIO write vs same-edge event
  (documented priority, low value).

## 2026-08-01 — X12 depth-3 mixed-kind nest + interrupt-logic optimization audit

- GOLDEN X12 `test_int_nested_trapa` (torture group, green FIRST run,
  20-offset sweep): interrupt → nested interrupt → TRAPA raised INSIDE
  the inner handler. SH-3 has one physical SSR/SPC pair, so the 3-level
  ladder is software save/restore (R14/R15 outer, R4/R5 inner); every
  level clobbers the running T before its RTE. Locks: depth-3 capture
  (STC SSR/SPC = inner live SR / TRAPA+2), FIRST-post-RTE MOVT reads the
  restored T through the depth-3 poison, final SSR/SPC == outer-saved
  images, main tst/bf loop transparent, entry/ack/EXPEVT/TRA bookkeeping
  exact. tb trap avoided: load_tbit_torture's &remaining=0x100 aliases
  the VBR+0x100 vector through the UNIFIED cache — X12 moves it to 0x180.
- Final: cpu_core_tb 122/122 (torture 18/18), HS3_tb 86/86 (RTL
  untouched), IPC 0.401/0.974/0.554 bit-identical.
- RTL-SKILLS audit of the interrupt conformance logic (static cost-model
  pass + the 3-seed OOC evidence): mirror deposits / ex_md_view /
  illegal-slot override ≈ 1 ALM each on registered inputs; ctrl_reg
  restore-merge = duplication-over-priority (correct trade); sr_id_view
  adds ≤1 level on the rare STC immediate leg (foldable into the
  control_read_value selector if it ever surfaces). ONE real lever
  identified, NOT applied: predecode event_rte into pd_route_t to
  replace the live 16-bit ==16'h002B equality in rte_pre_wb's ifid arm
  (the front of the bank1_nx select → GPR M10K address path, Wall A
  class). Withheld per the skill's own protocol: zero top-20 membership
  across 3 seeds, campaign closed at the plateau (R7/R8 lesson: -0.3..
  -0.7 tax on speculative shape changes). Apply only if bank1_nx /
  bram-addr panels ever show rte_pre_wb membership.

## 2026-08-02 — OOC verdict for i_MEM_HOLD (consumer accept-hold, CV1k request)

- Change shape: one registered top-level input ANDed into the two head-ready
  legs + the SDRAM continuation resume qualifier (bsc.sv), mirrored into the
  flat EREQ arms mon_gen_ok/mon_sdh_ok/mon_sdc_ok (6→7, 6→7, 4→5 inputs).
  New HS3/HS3_ooc_top port routed through the generated wrapper (preserve
  _drv reg = real launch arc into the ready cones).
- 3-seed re-measure (s1/s5/s7, sta.rpt mtimes fresh 2026-08-02 21:06):
  s1 −2.975 / 77.07, s5 −2.973 / 77.26, s7 −3.414 / 75.55. Mean −3.12 vs
  the −2.920 frozen baseline (−0.20) and inside the documented
  identical-RTL family band (−2.2..−3.9; the two prior baseline trios
  themselves differ by up to ±0.9 per seed). s7's −3.414 is the catalogued
  placement-coupled swing of the mon_pk headline class.
- Cone scan: i_MEM_HOLD in ZERO of 60 top-20 paths; no u_bsc cone in any
  top-20; every worst endpoint is the PRE-EXISTING mon_pk_cls/mon_pk_vld
  EREQ-package wall (cache FSM-arm select class, unchanged by this edit).
  NEUTRAL — CLEARED TO LAND.
- Sim gates: HS3_tb 89/89 (new group 20: hold-through-reset defer, random
  1-20-cycle burst torture, TAS-pair stretch, DMAC-burst walk; anti-vacuity
  defer counters 333/64 cycles); tests 1-82 output diff-identical to the
  86/86 baseline at hold=0 (first-principle proof); EREQ<->REQ oracle green
  over 341,962 strobes including injected traffic.

## 2026-08-03 — OOC verdict for the CCR.CF shadow-swap flush (V/U split + scrub)

- Change shape: V/U moved from tag-word bits [20:19] into per-way 512x2 VU RAMs
  (bank-MSB shadow swap), LRU 512x6, tag RAM narrowed to 256x19, S_FLUSH state
  deleted, background scrubber muxed into the VU/LRU write ports (shallow
  scrub_busy state decode as the select), resq_q gains a !flush_swap term,
  store/MRU quals gain !flush_req. Hit-resolve depth unchanged by construction
  (V sourced from a parallel RAM q, same AND level).
- 3-seed re-measure (cpu_core rig, sta.rpt mtimes fresh 2026-08-03 18:22-18:28):
  s1 -2.783 / 78.23, s5 -2.347 / 80.99, s7 -3.045 / 76.66. Mean -2.73 vs the
  -2.920 frozen baseline (+0.19, inside the documented identical-RTL family
  band -2.2..-3.9).
- Cone scan: u_vu / scrub / vu_bank in ZERO of 60 top-20 paths; every worst
  endpoint is the pre-existing int_pipe forwarding-lane class (fwd_lane_a,
  fwd_lane_b_agu, fwd_wbsel_a). NEUTRAL — CLEARED TO LAND.
- Sim gates: cpu_core_tb 122/122, HS3_tb 89/89; IPC 0.401/0.974/0.554 with
  identical retire/cycle counts; three HS3_tb whole-run laws relocked -248/-256
  (the flush walk itself). Flush cost measured: ~258 -> 2 cycles.
