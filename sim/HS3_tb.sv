`default_nettype wire

/*
    HS3 SoC-level testbench.

    Scaffolding follows cpu_core_tb: a Harvard imem/dmem memory model slaves
    the EXT_BUS leg (I/D told apart by a test-only probe of the cache FSM,
    exact because these tests run bypass-only), the scoreboard samples the
    core's dbg_o_* pulses hierarchically, and small hand-assembled SH-3
    programs drive everything end to end.

    Coverage: IPC parity with the cpu_core_tb benches (the splitter must add
    ZERO beats), bridge word/byte/long register access with lane alignment,
    CPG/WDT registers with the 0x5A/0xA5 keyed writes, WDT interval interrupt
    end-to-end (INTEVT/INTEVT2 = 0x560), WDT watchdog resets (EXPEVT 0x000 /
    0x020, WTCSR retention), IRQ level+edge (IRR0 clear protocol), IRL/IRLS,
    NMI (BL/BLMSK), PINT, priority/tiebreak, and the external-leg regression.
    The BSC group runs against real vendor models on the shared board bus:
    Micron MT48LC2M32B2 SDRAM (area 3) and Macronix MX29LV320E NOR flash
    (area 0 boot, 16-bit straps) - both patched copies, see the model headers.

    Programs run from P2 (bypass); the interrupt handler lives at VBR+0x600 =
    0x600 (P0, bypass while CCR.CE=0) = imem index 0x300. Mailboxes are dmem
    longwords at data addresses 0x40/0x44/0x48/0x4C (dmem[0x10..0x13]).

    Session 4 adds the CKIO pin (the SDRAM device clock now comes from the
    chip, not a hierarchy peek) and the RTC on its own EXTAL2 clock domain -
    the tb crystal is a scaled 32.768 kHz surrogate (80 ns period), see the
    generator comment for the exact-law reasoning.

    Group 13 twins the cpu_core_tb interrupt-collision sweeps (tests 87-89)
    onto the real INTC/BSC: IRL requests on the pins vs SDRAM fill/drain/
    refresh machinery, TAS.B locked pairs, and synchronous-exception
    collisions - plus suite-wide passive checkers for lock pairing on
    IBUS1_CORE and the ack/INTEVT/INTEVT2 handshake law.
*/

module HS3_tb;

timeunit 1ns;
timeprecision 1ps;

///////////////////////////////////////////////////////////
//////  DUT Connections
////

logic           clk;
logic           por_n;
logic           rst_n;

//interrupt pins ride the port pads since session 3 (table 18.1): IRQ4-0 =
//PTH4-0, IRQ5 = SCPT7, PINT7-0 = PTC, PINT15-8 / IRLS3-0 = PTF (shared pad!)
logic           nmi_pin;
logic   [5:0]   irq_pin;            //-> PTH4-0 + SCPT7 pads
logic   [7:0]   pint_pin;           //-> PTC pads (PINT7-0)
logic   [7:0]   ptf_pin;            //-> PTF pads: PINT15-8 high-active / IRLS3-0 low-active
logic           tclk_pin;           //-> PTH7 pad (TMU TCLK)
logic   [7:0]   pta_pin;            //-> PTA pads (port tests)
logic   [7:0]   ptg_pin;            //-> PTG pads (PGCR quirk test)
wire    [7:0]   pta_o, pta_oe, pta_pu;      //port A pad ring view
wire    [7:0]   ptd_o, ptd_oe;              //port D (DACK/DRAK pads + bit checks)
logic   [7:0]   ptd_pin;                    //port D pad inputs (PTD4/6 = DREQ0/1, active-low)
wire    [7:0]   pth_o, pth_oe;              //PTH pad ring view (TCLK merge)

IBus_1          MEM_BUS();       //generic-port view; tb memory model slaves it

//generic memory port (flat at the chip boundary since the BSC landed)
logic           wait_n;          //WAIT pin (auto-stretcher below drives it)
wire            mem_req, mem_write, mem_burst, mem_rsp_ready;
wire    [1:0]   mem_size;
wire    [28:0]  mem_addr_p;
wire    [6:0]   mem_cs_n;
wire    [3:0]   mem_wstrb;

//MON transaction monitor / early-transaction sideband (sh3_sideband.md) - oracle checks it whole-run
wire            mon_req, mon_wr, mon_burst;
wire    [1:0]   mon_size;
wire    [28:0]  mon_addr;

//the chip's physical external bus (table 10.1) + the one true bidirectional
//net (controls stay unidirectional; the inout lives only at board level)
wire    [25:0]  a_pin;
wire    [31:0]  d_o;
wire            d_oe;
wire            bs_n, cs0_n, cs2_n, cs3_n, cs4_n, cs5_n, cs6_n;
wire            rd_wr, rasl_n, rasu_n, casl_n, casu_n, rd_n, cke, back_n, bus_oe;
wire            rascas_oe, a_pu, d_pu, irqout_n;    //release pads + IRQOUT (Group C)
wire    [7:0]   ptc_o, ptc_oe;                      //PTC pad ring view (MCS merge)
logic           breq_n = 1'b1;
logic           md4_pin = 1'b1;  //area-0 width straps (knobs; 11=32-bit boot,
logic           md3_pin = 1'b1;  //10=16-bit for the NOR flash tests)
wire    [3:0]   we_n;
wire    [31:0]  d_bus;
assign  d_bus = d_oe ? d_o : 32'hzzzz_zzzz;

//clock pins: CKIO output (drives the SDRAM model below) and the RTC crystal
wire            ckio;
wire            ckio_pcen, ckio_ncen;       //CKIO edge enables (DREQ/WAIT phase checks)
logic           extal2 = 1'b0;

HS3 #(
    .RESET_PC                  (32'hA000_0000),
    .BIG_ENDIAN                (1'b1)
) u_dut (
    .i_POR_n                   (por_n),
    .i_RST_n                   (rst_n),
    .i_CLK                     (clk),
    .i_CEN                     (1'b1),
    .o_CKIO                    (ckio),
    .o_CKIO_PCEN               (ckio_pcen),
    .o_CKIO_NCEN               (ckio_ncen),
    .i_EXTAL2                  (extal2),

    .o_MEM_REQ                 (mem_req),
    .o_MEM_WR               (mem_write),
    .o_MEM_BURST               (mem_burst),
    .o_MEM_SIZE                (mem_size),
    .o_MEM_ADDR                (mem_addr_p),
    .o_MEM_CS_n                (mem_cs_n),
    .o_MEM_WSTRB               (mem_wstrb),
    .i_MEM_READY               (1'b0),
    .i_MEM_RSP_VALID           (raw_mode ? 1'b0 : MEM_BUS.rsp_valid),
    .i_MEM_FAULT               (MEM_BUS.rsp_fault),
    .o_MEM_RSP_READY           (mem_rsp_ready),

    .o_MON_REQ                  (mon_req),
    .o_MON_WR                   (mon_wr),
    .o_MON_ADDR                 (mon_addr),
    .o_MON_SIZE                 (mon_size),
    .o_MON_BURST                (mon_burst),

    .o_A                       (a_pin),
    .o_D_O                     (d_o),
    .o_D_OE                    (d_oe),
    .i_D_I                     (d_bus),
    .o_BS_n                    (bs_n),
    .o_CS0_n                   (cs0_n),
    .o_CS2_n                   (cs2_n),
    .o_CS3_n                   (cs3_n),
    .o_CS4_n                   (cs4_n),
    .o_CS5_n                   (cs5_n),
    .o_CS6_n                   (cs6_n),
    .o_RD_WR                   (rd_wr),
    .o_RAS3L_n                 (rasl_n),
    .o_RAS3U_n                 (rasu_n),
    .o_CASL_n                  (casl_n),
    .o_CASU_n                  (casu_n),
    .o_WE_n                    (we_n),
    .o_RD_n                    (rd_n),
    .i_WAIT_n                  (wait_n),
    .i_MD4                     (md4_pin),
    .i_MD3                     (md3_pin),
    .o_CKE                     (cke),
    .i_BREQ_n                  (breq_n),
    .o_BACK_n                  (back_n),
    .o_BUS_OE                  (bus_oe),
    .o_RASCAS_OE               (rascas_oe),
    .o_A_PU                    (a_pu),
    .o_D_PU                    (d_pu),
    .o_IRQOUT_n                (irqout_n),

    .i_NMI                     (nmi_pin),

    .i_PTA_I                   (pta_pin),
    .o_PTA_O                   (pta_o),
    .o_PTA_OE                  (pta_oe),
    .o_PTA_PU                  (pta_pu),
    .i_PTB_I                   (8'h00),
    .o_PTB_O                   (),
    .o_PTB_OE                  (),
    .o_PTB_PU                  (),
    .i_PTC_I                   (pint_pin),
    .o_PTC_O                   (ptc_o),
    .o_PTC_OE                  (ptc_oe),
    .o_PTC_PU                  (),
    .i_PTD_I                   (ptd_pin),
    .o_PTD_O                   (ptd_o),
    .o_PTD_OE                  (ptd_oe),
    .o_PTD_PU                  (),
    .i_PTE_I                   (8'h00),
    .o_PTE_O                   (),
    .o_PTE_OE                  (),
    .o_PTE_PU                  (),
    .i_PTF_I                   (ptf_pin),
    .o_PTF_PU                  (),
    .i_PTG_I                   (ptg_pin),
    .o_PTG_PU                  (),
    .i_PTH_I                   ({tclk_pin, 2'b00, irq_pin[4:0]}),
    .o_PTH_O                   (pth_o),
    .o_PTH_OE                  (pth_oe),
    .o_PTH_PU                  (),
    .i_PTJ_I                   (8'h00),
    .o_PTJ_O                   (),
    .o_PTJ_OE                  (),
    .o_PTJ_PU                  (),
    .i_PTK_I                   (8'h00),
    .o_PTK_O                   (),
    .o_PTK_OE                  (),
    .o_PTK_PU                  (),
    .i_PTL_I                   (8'h00),
    .i_SCPT_I                  ({irq_pin[5], 7'd0}),
    .o_SCPT_O                  (),
    .o_SCPT_OE                 (),
    .o_SCPT_PU                 ()
);

//bridge the flat generic port onto the model's bundle (mechanical rename)
assign MEM_BUS.req_valid = mem_req;
assign MEM_BUS.req_write = mem_write;
assign MEM_BUS.req_burst = mem_burst;
assign MEM_BUS.req_size  = mem_size;
assign MEM_BUS.req_addr  = {3'b000, mem_addr_p};
assign MEM_BUS.req_wdata = d_o;             //ALL data rides the physical D bus now
assign MEM_BUS.req_wstrb = mem_wstrb;
assign MEM_BUS.req_lock  = 1'b0;
assign MEM_BUS.rsp_ready = mem_rsp_ready;

//mirror-strobe counter: BSC-owned accesses (SDRAM areas) must appear on the
//generic port as accept strobes with their CS asserted (tests snapshot+delta)
integer         mirror_cnt = 0;
always @(posedge clk) begin
    if(mem_req && !mem_owned && !mem_cs_n[3]) mirror_cnt = mirror_cnt + 1;
end

//sticky CKE-low monitor for the self-refresh test (single-writer; the test
//pulses the clear knob - a two-writer variable confuses Verilator)
logic           cke_mon_clr  = 1'b0;
logic           cke_low_seen = 1'b0;
always @(posedge clk) begin
    if(cke_mon_clr)   cke_low_seen <= 1'b0;
    else if(!cke)     cke_low_seen <= 1'b1;
end

//sticky TAS-vs-BREQ invariant: the bus must never be granted while a locked
//RMW pair is open on either BSC path (p.320); single-writer + clear knob
logic           tas_mon_clr = 1'b0;
logic           tas_viol    = 1'b0;
always @(posedge clk) begin
    if(tas_mon_clr) tas_viol <= 1'b0;
    else if((u_dut.u_bsc.e_lock_hold || u_dut.u_bsc.fe_lock_hold) && !back_n)
        tas_viol <= 1'b1;
end

//Micron MT48LC2M32B2 on the SDRAM pins (area 3 in the tests -> CS3). The
//chip's CKIO pin is DATASHEET phase (rises at the command edges), so the
//board must delay/phase-shift the device clock to grant the tOD margin -
//the user's board does it with an FPGA output-delay/PLL adjustment; here
//it is a half-cycle transport delay on the clock net: the device samples
//10 ns after the pins change, 10 ns setup + 10 ns hold, and every command
//lands in the same device cycle as before the phase fix.
logic           sd_clk = 1'b0;
always @(ckio) sd_clk <= #10 ckio;      //transport (NBA) - never swallows pulses

//population knob (Group C): the AMX / 16-bit-bus shape probes reprogram the
//address mux away from this device's 0111 wiring - deselect it so the model
//neither drives D nor complains about the (to IT) ill-formed sequences
logic           sdram_en = 1'b1;
wire            sdram_cs_n = cs3_n | ~sdram_en;

mt48lc2m32b2 u_sdram (
    .Dq                        (d_bus),
    .Dq_in                     (d_o),          //write-data view (patched port, see model header)
    .Addr                      (a_pin[12:2]),  //device A10:A0 = chip A12:A2 (fig 10.12)
    .Ba                        (a_pin[14:13]),
    .Clk                       (sd_clk),
    .Cke                       (cke),
    .Cs_n                      (sdram_cs_n),
    .Ras_n                     (rasl_n),
    .Cas_n                     (casl_n),
    .We_n                      (rd_wr),
    .Dqm                       (we_n)
);

//Macronix MX29LV320E (32Mbit NOR, 2Mx16, 70ns) on the ordinary pins: area 0
//boot flash for the shared-bus tests. Async device - no clock; CE is gated
//by the flash_en population knob so the legacy area-0 TB devices keep the
//bus in the older tests (a real board populates one or the other).
logic           flash_en = 1'b0;
wire            flash_ce_n = cs0_n | ~flash_en;

MX29LV320E u_flash (
    .A                         (a_pin[21:1]),  //word address = chip A21:A1 (4 MB)
    .Q                         (d_bus[15:0]),  //16-bit port lives on D15-D0
    .Q_in                      (d_o[15:0]),    //write-data view (patched port, see model header)
    .CE_B                      (flash_ce_n),
    .WE_B                      (we_n[1] & we_n[0]), //strobed when either half writes
    .OE_B                      (rd_n),
    .BYTE_B                    (1'b1),         //word mode
    .RESET_B                   (1'b1),
    .WP_B                      (1'b1),
    .RYBY_B                    ()
);

always #5 clk = ~clk;

//RTC crystal surrogate on EXTAL2: 32.768 kHz scaled ~381x so one RTC second
//is 32768 x 80 ns = 262144 core cycles (the CDC only needs a half-period
//over 2 core cycles). 40 ns = 4 cycles keeps every derived tick period an
//exact cycle count, and the 7 ns offset keeps edges off the 10 ns sampling
//grid - constant capture phase, so the period laws below are EXACT.
initial begin
    #7;
    forever #40 extal2 = ~extal2;
end

//hierarchical debug probes (cpu_core keeps its dbg_o_* ports; HS3 leaves them open)
wire            retire_valid = u_dut.u_cpu.dbg_o_RETIRE_VALID;
wire    [31:0]  retire_pc    = u_dut.u_cpu.dbg_o_RETIRE_PC;
wire    [31:0]  sr_o         = u_dut.u_cpu.dbg_o_SR;
wire    [31:0]  expevt_o     = u_dut.u_cpu.dbg_o_EXPEVT;
wire    [31:0]  intevt_o     = u_dut.u_cpu.dbg_o_INTEVT;
wire            exc_valid    = u_dut.u_cpu.dbg_o_EXC_VALID;
wire            entry_valid  = u_dut.u_cpu.o_EXCEPTION_ENTRY_VALID;
wire    [31:0]  entry_pc     = u_dut.u_cpu.o_EXCEPTION_ENTRY_PC;
wire    [31:0]  tea_o        = u_dut.u_cpu.dbg_o_TEA;
wire    [31:0]  tra_o        = u_dut.u_cpu.dbg_o_TRA;
wire            trapa_valid  = u_dut.u_cpu.dbg_o_TRAPA_VALID;
wire    [31:0]  intevt2_reg  = u_dut.u_intc.intevt2;
wire            wdt_rst_hit  = !u_dut.wdt_rst_por_n || !u_dut.wdt_rst_man_n;

//TMU probes + ch0 underflow period monitor: UNF sets are tick-exact hardware
//events, so the rise-to-rise distance IS the (TCOR+1)*ticklen law even while
//interrupt latency jitters (single-writer + clear knob, the TB lesson)
wire            tmu_unf0    = u_dut.u_tmu.unf[0];
logic           unf_mon_clr = 1'b0;
logic           tmu_unf0_z  = 1'b0;
integer         unf0_tlast  = 0;
integer         unf0_dt     = 0;    //cycles between the last two UNF0 rises
always @(posedge clk) begin
    if(unf_mon_clr) begin
        tmu_unf0_z <= 1'b0; unf0_tlast <= 0; unf0_dt <= 0;
    end
    else begin
        tmu_unf0_z <= tmu_unf0;
        if(tmu_unf0 && !tmu_unf0_z) begin
            unf0_dt    <= ($time - unf0_tlast) / 10;
            unf0_tlast <= $time;
        end
    end
end

//RTC probes + PEF period monitor (the UNF monitor pattern): PEF sets are
//divider-grid events, so the rise-to-rise distance IS the PES period law -
//the CDC latency is a constant because the tb EXTAL2 is grid-locked
wire            rtc_pef     = u_dut.u_rtc.pef;
logic           pef_mon_clr = 1'b0;
logic           rtc_pef_z   = 1'b0;
integer         pef_tlast   = 0;
integer         pef_dt      = 0;    //cycles between the last two PEF rises
always @(posedge clk) begin
    if(pef_mon_clr) begin
        rtc_pef_z <= 1'b0; pef_tlast <= 0; pef_dt <= 0;
    end
    else begin
        rtc_pef_z <= rtc_pef;
        if(rtc_pef && !rtc_pef_z) begin
            pef_dt    <= ($time - pef_tlast) / 10;
            pef_tlast <= $time;
        end
    end
end

//system reset view: pins AND WDT-caused internal resets; the memory model and
//scoreboard clear on this so a watchdog reset cannot leave a stale response
wire            sys_rst_n    = u_dut.rst_all_n;


///////////////////////////////////////////////////////////
//////  Memory Models And Fault/Latency Knobs
////

//Instruction memory: word-addressed, 2048 entries cover all test programs.
logic   [15:0]  imem [0:2047];
logic           if_fault_en;
logic   [10:0]  if_fault_widx;

//Data memory: longword-addressed, 256 entries, byte-strobed writes.
logic   [31:0]  dmem [0:255];
logic           d_fault_en;
logic   [7:0]   d_fault_widx;
integer         d_latency;       //extra response-wait cycles

logic           mem_pending;
logic   [31:0]  mem_addr;
logic           mem_is_data;
logic           mem_is_sgw;     //single-address DMA write: device drives the resolved bus
logic           mem_is_dack;    //DACK-tagged cycle (live sideband at the accept edge)
logic           sgdev_en;       //DMAC single-address device attached to DACK0 (test knob)
logic           sgdev_clr;      //latch clear knob (sgdev_latch has ONE writer: the model)
logic   [31:0]  sgdev_data;     //pattern the device drives (dev->mem)
logic   [31:0]  sgdev_latch;    //what the device captured (mem->dev)
logic           mem_is_write;
logic   [3:0]   mem_wstrb_q;
logic           mem_is_fault;
integer         mem_wait_cnt;

//I/D discriminator (test probe into the cache FSM; exact while bypass-only)
//DMAC-mastered accesses are always data (the imem/dmem split is a tb
//artifact keyed off the cache's current-access flag, which a DMA cycle
//never updates)
wire            req_is_data = u_dut.u_arb.own_dma || u_dut.u_cpu.u_cache.cur_is_data;

assign MEM_BUS.req_ready = !mem_pending && !MEM_BUS.rsp_valid;

//RAW BUS MODE: no handshake at all - the model behaves like asynchronous
//ROM/SRAM pins. Reads are served combinationally on i_MEM_RDATA and the BSC
//samples them at its WCR2/i_WAIT_n-timed bus-cycle end; writes commit on
//every held cycle (idempotent). Exercises the ordinary/burst-ROM controller.
integer         raw_mode;
wire    [31:0]  raw_rdata = req_is_data ? dmem[mem_addr_p[9:2]]
                                        : {imem[{mem_addr_p[11:2], 1'b0}],
                                           imem[{mem_addr_p[11:2], 1'b1}]};
//board-level behavior: the raw memory drives the shared D bus like an async
//SRAM output stage - only while the read strobe is asserted (Group B: RD is
//mid-state shaped, fig 23.16); the BSC samples i_D_I at the mid-T2 fall
wire            raw_drv = (raw_mode == 1) && !rd_n && mem_req && !mem_write &&
                          mem_owned && (mem_area != 3'd4) &&
                          !(raw8_en && mem_area == 3'd6);
assign  d_bus = raw_drv ? raw_rdata : 32'hzzzz_zzzz;

//handshake mode: read responses also travel the physical D bus (rsp_valid
//marks the drive window; the BSC samples i_D_I on the completion). The
//controller BACKS OFF while the chip drives D (posted SDRAM engine writes
//share the pins): a real bus device gates its output enable on direction
wire            hsk_drv = (raw_mode == 0) && MEM_BUS.rsp_valid && !mem_is_write && !d_oe;
assign  d_bus = hsk_drv ? MEM_BUS.rsp_rdata : 32'hzzzz_zzzz;

//a 16-bit raw device wired to D15-D0 on area 4 (board wiring for test 37):
//reads serve the half a_pin[1] selects; writes commit per WE1/WE0
wire            raw16     = (raw_mode == 1) && (mem_area == 3'd4);
wire            raw16_drv = raw16 && !rd_n && mem_req && !mem_write;
wire    [31:0]  raw16_w   = dmem[mem_addr_p[9:2]];
assign  d_bus = raw16_drv ? {2{a_pin[1] ? raw16_w[15:0] : raw16_w[31:16]}}
                          : 32'hzzzz_zzzz;
always_ff @(posedge clk) begin
    if(raw16 && mem_req && mem_write) begin
        if(!a_pin[1]) begin
            if(!we_n[1]) dmem[mem_addr_p[9:2]][31:24] <= d_o[15:8];
            if(!we_n[0]) dmem[mem_addr_p[9:2]][23:16] <= d_o[7:0];
        end
        else begin
            if(!we_n[1]) dmem[mem_addr_p[9:2]][15:8]  <= d_o[15:8];
            if(!we_n[0]) dmem[mem_addr_p[9:2]][7:0]   <= d_o[7:0];
        end
    end
end
//writes commit per WE lane while the strobe is low (async SRAM level-latch;
//address/data are held over the WE window by the shape contract, fig 23.16)
always_ff @(posedge clk) begin
    if(raw_mode == 1 && mem_req && mem_write && mem_owned && mem_area != 3'd4 &&
       !(raw8_en && mem_area == 3'd6)) begin
        if(!we_n[0]) dmem[mem_addr_p[9:2]][7:0]   <= d_o[7:0];
        if(!we_n[1]) dmem[mem_addr_p[9:2]][15:8]  <= d_o[15:8];
        if(!we_n[2]) dmem[mem_addr_p[9:2]][23:16] <= d_o[23:16];
        if(!we_n[3]) dmem[mem_addr_p[9:2]][31:24] <= d_o[31:24];
    end
end

//an 8-bit raw device wired to D7-D0 on area 6 (Group C board wiring): reads
//serve the byte a_pin[1:0] selects (big-endian register lane, table 10.9),
//writes latch per WE0 - the only strobe an 8-bit port uses
logic           raw8_en = 1'b0;
wire            raw8     = (raw_mode == 1) && raw8_en && (mem_area == 3'd6);
wire            raw8_drv = raw8 && !rd_n && mem_req && !mem_write;
wire    [31:0]  raw8_w   = dmem[mem_addr_p[9:2]];
logic   [7:0]   raw8_q;
always_comb begin
    case(a_pin[1:0])
        2'd0:    raw8_q = raw8_w[31:24];
        2'd1:    raw8_q = raw8_w[23:16];
        2'd2:    raw8_q = raw8_w[15:8];
        default: raw8_q = raw8_w[7:0];
    endcase
end
assign  d_bus[7:0] = raw8_drv ? raw8_q : 8'hzz;
always_ff @(posedge clk) begin
    if(raw8 && mem_req && mem_write && !we_n[0]) begin
        case(a_pin[1:0])
            2'd0:    dmem[mem_addr_p[9:2]][31:24] <= d_o[7:0];
            2'd1:    dmem[mem_addr_p[9:2]][23:16] <= d_o[7:0];
            2'd2:    dmem[mem_addr_p[9:2]][15:8]  <= d_o[7:0];
            default: dmem[mem_addr_p[9:2]][7:0]   <= d_o[7:0];
        endcase
    end
end

///////////////////////////////////////////////////////////
//////  Board-Bus Shape Monitors (Group B)
////

//D-bus contention: the strobe shapes + WCR1 idles must keep at most one
//driver on the shared D bus at any sample edge; sticky, checked at the end
wire            flash_drv = u_flash.Q_oe_i;
wire            sdram_drv = u_sdram.Dq_oe_i;
wire            tbmem_drv = raw_drv || raw16_drv || raw8_drv || hsk_drv;
wire    [2:0]   dbus_drvs = {2'd0, d_oe} + {2'd0, tbmem_drv} +
                            {2'd0, flash_drv} + {2'd0, sdram_drv};
logic           dbus_viol = 1'b0;
always @(posedge clk) begin
    if(dbus_drvs > 3'd1) begin
        if(!dbus_viol)
            $display("[DBUS] contention at %0t: dut=%b tbmem=%b flash=%b sdram=%b",
                     $time, d_oe, tbmem_drv, flash_drv, sdram_drv);
        dbus_viol <= 1'b1;
    end
end

//WE write-strobe shape: while an ordinary write owns the pins, address and
//data must hold from WE fall to WE rise (async devices latch at the rise -
//catches the held-WE split-16 bug class; tAH/tWDH1 of fig 23.16)
logic           we_low_z      = 1'b0;
logic   [25:0]  we_a_z        = '0;
logic   [31:0]  we_d_z        = '0;
logic           we_shape_viol = 1'b0;
wire            we_low_now    = u_dut.u_bsc.ord_pins && (we_n != 4'b1111);
always @(posedge clk) begin
    if(we_low_now && we_low_z && (a_pin !== we_a_z || d_o !== we_d_z)) begin
        if(!we_shape_viol) $display("[WESHAPE] addr/data moved under WE at %0t", $time);
        we_shape_viol <= 1'b1;
    end
    we_low_z <= we_low_now;
    we_a_z   <= a_pin;
    we_d_z   <= d_o;
end

//SDRAM command/shape monitor (Group C): sampled at the CKIO fall - every
//20 ns command window holds exactly one negedge, so back-to-back beats count
//correctly. Latches the ACTV row pins, the LAST CAS column pins + DQM, and
//counts CAS commands; BS low with EVERY CS idle is an SDRAM Td data cycle
//(p.283 - ordinary cycles assert BS only with their CS). Clear knob per test.
logic           sdm_clr      = 1'b0;
logic   [25:0]  sdm_row_a    = '0;
logic   [25:0]  sdm_col_a    = '0;
logic   [3:0]   sdm_cas_dqm  = '1;
integer         sdm_actv_cnt = 0;
integer         sdm_cas_cnt  = 0;
integer         sdm_td_cnt   = 0;
integer         sdm_cas_t    = 0;   //time of the last READ command sample
integer         sdm_td_t     = 0;   //time of the first Td sample after clear
wire            sdm_sel  = !(cs2_n && cs3_n);
wire            sdm_actv = sdm_sel && !rasl_n &&  casl_n &&  rd_wr;
wire            sdm_cas  = sdm_sel &&  rasl_n && !casl_n;           //READ or WRIT
//read Td cycles: BS is low ONLY for data there (READ commands no longer
//carry it), so a coincident later READ command (fig 10.14 overlap) still
//counts; write data rides its command (rd_wr low) and ordinary cycles are
//excluded by their own CS
wire            sdm_td   = !bs_n && rd_wr && cs0_n && cs4_n && cs5_n && cs6_n;
always @(negedge ckio) begin
    if(sdm_clr) begin
        sdm_row_a <= '0; sdm_col_a <= '0; sdm_cas_dqm <= '1;
        sdm_actv_cnt <= 0; sdm_cas_cnt <= 0; sdm_td_cnt <= 0;
        sdm_cas_t <= 0; sdm_td_t <= 0;
    end
    else begin
        if(sdm_actv) begin
            sdm_row_a    <= a_pin;
            sdm_actv_cnt <= sdm_actv_cnt + 1;
        end
        if(sdm_cas) begin
            sdm_col_a   <= a_pin;
            sdm_cas_dqm <= we_n;
            sdm_cas_cnt <= sdm_cas_cnt + 1;
            if(rd_wr) sdm_cas_t <= $time;
        end
        if(sdm_td) begin
            sdm_td_cnt <= sdm_td_cnt + 1;
            if(sdm_td_t == 0) sdm_td_t <= $time;
        end
    end
end

//MCS pad monitor (Group C): MCS1 rides the PTC1 pad; a low drive outside its
//programmed block is a decode violation, as is a low CS0 pad (= MCS0 after
//the PFC switch) on an out-of-block (A25 = 1) address
logic           mcs_mon_clr  = 1'b0;
logic           mcs1_seen    = 1'b0;
logic           mcs1_viol    = 1'b0;
logic           cs0_mcs_viol = 1'b0;
always @(posedge clk) begin
    if(mcs_mon_clr) begin
        mcs1_seen <= 1'b0; mcs1_viol <= 1'b0; cs0_mcs_viol <= 1'b0;
    end
    else begin
        if(ptc_oe[1] && !ptc_o[1]) begin
            if(a_pin[25:22] == 4'b0001) mcs1_seen <= 1'b1;
            else                        mcs1_viol <= 1'b1;
        end
        if(!cs0_n && a_pin[25]) cs0_mcs_viol <= 1'b1;
    end
end

//PULA window counter (Group C): CKIO cycles with the A pull-up on
logic           apu_clr    = 1'b0;
integer         apu_hi_cnt = 0;
always @(negedge ckio) begin
    if(apu_clr)   apu_hi_cnt <= 0;
    else if(a_pu) apu_hi_cnt <= apu_hi_cnt + 1;
end

//WE0 falling-edge counter (Group C): one strobe per write sub-cycle on a
//narrow port (tables 10.8/10.9 - the strobe RISES between sub-cycles)
logic           we0_clr = 1'b0;
integer         we0_cnt = 0;
logic           we0_z   = 1'b1;
always @(posedge clk) begin
    if(we0_clr) begin
        we0_cnt <= 0; we0_z <= 1'b1;
    end
    else begin
        if(we_n[0] === 1'b0 && we0_z === 1'b1) we0_cnt <= we0_cnt + 1;
        we0_z <= we_n[0];
    end
end

//TEMP debug: trace EXT-leg data writes + retires (remove after the pair bring-up).
logic           dbg_trace = 1'b0;
always @(posedge clk) begin
    if(dbg_trace && raw_mode == 1 && mem_req && mem_write && mem_owned && mem_area != 3'd4)
        $display("        [trace %0t] EXT WR addr=%08h data=%08h strb=%b", $time, mem_addr_p, d_o, mem_wstrb);
    if(dbg_trace && u_dut.u_cpu.dbg_o_RETIRE_VALID)
        $display("        [trace %0t] RET pc=%08h inst=%04h", $time,
                 u_dut.u_cpu.dbg_o_RETIRE_PC, u_dut.u_cpu.dbg_o_RETIRE_INST);
end

//WAIT auto-stretcher: holds i_WAIT_n low for wait_stretch bus cycles at the
//start of every ordinary bus cycle (mem_req rising edge). dackw_arm instead
//arms at the FIRST DACK0 opening after a dackmon clear - a DMAC unit granted
//back-to-back behind a CPU fetch never re-raises mem_req, so the WAIT-ignore
//laws (p.304) need an arming keyed to the unit itself, not the request edge
integer         wait_stretch;
integer         dackw_arm;
integer         ws_cnt = 0;
logic           mem_req_z = 1'b0;
always @(posedge clk) begin
    mem_req_z <= mem_req;
    if(dackw_arm > 0 && dackmon_en && !dack0_z && dack0_win && dack_t0 < 0)
        ws_cnt <= dackw_arm * 2;
    else if(mem_req && !mem_req_z) ws_cnt <= wait_stretch * 2;
    else if(ws_cnt > 0)            ws_cnt <= ws_cnt - 1;
end
always_comb wait_n = (ws_cnt == 0);

//external-controller masking: the port exposes every external area; the
//model answers only what it owns (BSC-owned areas arrive as accept strobes).
//DRAMTP is probed hierarchically - the real SoC controller knows its own map.
wire    [2:0]   mem_area    = MEM_BUS.req_addr[28:26];
wire            mem_a2_sdr  = (u_dut.u_bsc.bcr1[4:2] == 3'b011);
wire            mem_a3_sdr  = !u_dut.u_bsc.bcr1[4] && u_dut.u_bsc.bcr1[3];
wire            mem_owned   = (mem_area == 3'd0) || (mem_area == 3'd4) ||
                              (mem_area == 3'd5) || (mem_area == 3'd6) ||
                              ((mem_area == 3'd2) && !mem_a2_sdr) ||
                              ((mem_area == 3'd3) && !mem_a3_sdr);

always_ff @(posedge clk or negedge sys_rst_n) begin
    integer i;
    if(!sys_rst_n) begin
        mem_pending        <= 1'b0;
        mem_addr           <= 32'd0;
        mem_is_data        <= 1'b0;
        mem_is_fault       <= 1'b0;
        mem_wait_cnt       <= 0;
        MEM_BUS.rsp_valid  <= 1'b0;
        MEM_BUS.rsp_rdata  <= 32'd0;
        MEM_BUS.rsp_fault  <= 1'b0;
    end
    else if(raw_mode != 0) begin        //raw devices (1) or fully silent (2)
        mem_pending       <= 1'b0;
        MEM_BUS.rsp_valid <= 1'b0;
    end
    else begin
        //a write is only taken once the BSC drives the D bus (o_D_OE) - a
        //stalled request may be presented before its bus cycle opens. A
        //single-address DMA write never drives D (fig 11.10a): take it at
        //the accept and delay the sample until the device's DACK window
        if(MEM_BUS.req_valid && MEM_BUS.req_ready && mem_owned &&
           (!MEM_BUS.req_write || d_oe || u_dut.IBUS1_BSC.req_saddr)) begin
            mem_pending  <= 1'b1;
            mem_addr     <= MEM_BUS.req_addr;
            mem_is_data  <= req_is_data;
            mem_is_write <= MEM_BUS.req_write;
            mem_is_sgw   <= MEM_BUS.req_write && u_dut.IBUS1_BSC.req_saddr;
            mem_is_dack  <= u_dut.IBUS1_BSC.req_dack;
            mem_wstrb_q  <= MEM_BUS.req_wstrb;
            if(req_is_data) begin
                mem_is_fault <= d_fault_en && (MEM_BUS.req_addr[9:2] == d_fault_widx);
                mem_wait_cnt <= (MEM_BUS.req_write && u_dut.IBUS1_BSC.req_saddr)
                                ? 6 : d_latency;    //wait for the grid-aligned CS window
            end
            else begin
                mem_is_fault <= if_fault_en && (MEM_BUS.req_addr[11:1] == if_fault_widx);
                mem_wait_cnt <= 0;
            end
        end
        if(mem_pending && !MEM_BUS.rsp_valid) begin
            if(mem_wait_cnt == 0) begin
                mem_pending       <= 1'b0;
                MEM_BUS.rsp_valid <= 1'b1;
                //write data sampled off the physical D bus (held bus cycle);
                //a single-address write reads the RESOLVED bus - the external
                //DACK device is driving, not the chip
                if(mem_is_write && mem_is_data && !mem_is_fault) begin
                    if(mem_wstrb_q[0]) dmem[mem_addr[9:2]][7:0]   <= mem_is_sgw ? d_bus[7:0]   : d_o[7:0];
                    if(mem_wstrb_q[1]) dmem[mem_addr[9:2]][15:8]  <= mem_is_sgw ? d_bus[15:8]  : d_o[15:8];
                    if(mem_wstrb_q[2]) dmem[mem_addr[9:2]][23:16] <= mem_is_sgw ? d_bus[23:16] : d_o[23:16];
                    if(mem_wstrb_q[3]) dmem[mem_addr[9:2]][31:24] <= mem_is_sgw ? d_bus[31:24] : d_o[31:24];
                end
                MEM_BUS.rsp_rdata <= mem_is_data ? dmem[mem_addr[9:2]]
                                                 : {imem[{mem_addr[11:2], 1'b0}],
                                                    imem[{mem_addr[11:2], 1'b1}]};
                MEM_BUS.rsp_fault <= mem_is_fault;
                //DMAC single-address device: take a DACK-tagged read's datum
                //at its completion (dmem reads never ride the physical D pins
                //and early-complete before the pin window opens, so the pin-
                //level model can't see them; single writer - see sgdev_clr)
                if(sgdev_en && mem_is_data && !mem_is_write && mem_is_dack)
                    sgdev_latch <= dmem[mem_addr[9:2]];
            end
            else begin
                mem_wait_cnt <= mem_wait_cnt - 1;
            end
        end
        if(MEM_BUS.rsp_valid && MEM_BUS.rsp_ready) begin
            MEM_BUS.rsp_valid <= 1'b0;
        end
        if(sgdev_clr) sgdev_latch <= 32'd0;     //knob clear (tasks must not write
    end                                         //an always_ff variable directly)
end


///////////////////////////////////////////////////////////
//////  DMAC External Device Model + DACK Monitor (section 11)
////

/*
    Single-address "external device with DACK" (figs 11.9-11.10): on a
    DACK-tagged WRITE cycle the device drives D31-0 (the chip's o_D_OE
    stays low - fig 11.10a) and bumps its pattern at each window close;
    on a DACK-tagged READ it latches the memory's data off the bus.
    The monitor locks the DACK-window laws: the window sits inside the
    CS0 assertion ("same duration as CSn", p.363) and lands on the
    read or write cycle per AM. Counters clear when a test enables it.
*/

logic           dackmon_en;             //DACK-law counters run
logic           dackmon_rst = 1'b0;     //counter clear knob (single-writer law)
integer         dackw_cnt;              //DACK0 window cycles observed
integer         dackf_cnt;              //DACK0 window OPENINGS: envelope framing law
                                        //(fig 11.11: 4 per plain 16-byte unit;
                                        // fig 23.19: 1 per burst-ROM read unit)
integer         dack_naked;             //window cycles with CS0 NEGATED = violation
integer         dack_on_rd, dack_on_wr; //window cycles in read vs write bus cycles
integer         drak0_lo, drak0_hi;     //DRAK0 pad low/high cycles (RL polarity proof)
integer         dackmon_t;              //monitor timebase since clear
integer         dack_t0, dack_t1;       //first opening / last close times: t1 - t0 =
                                        //unit span, proving beats chain with no idle

wire            dack0_win = u_dut.u_bsc.o_DACK_WIN[0];
assign  d_bus = (sgdev_en && dack0_win && !rd_wr) ? sgdev_data : 32'hzzzz_zzzz;

logic           dack0_z, dack0_waswr;
always @(posedge clk) begin
    dack0_z <= dack0_win;
    if(dack0_win) dack0_waswr <= !rd_wr;
    //device pattern advances as each driven (write) window closes
    if(sgdev_en && dack0_z && !dack0_win && dack0_waswr)
        sgdev_data <= sgdev_data + 32'd1;
    //(the device's read-side latch lives in the memory model's completion
    //arm: dmem reads early-complete before the pin-level DACK window opens)

    //knob clear (tasks must not write an always_ff variable directly):
    //every counter has ONE writer - this block
    if(dackmon_rst) begin
        dackw_cnt  <= 0;
        dackf_cnt  <= 0;
        dack_naked <= 0;
        dack_on_rd <= 0;
        dack_on_wr <= 0;
        drak0_lo   <= 0;
        drak0_hi   <= 0;
        dackmon_t  <= 0;
        dack_t0    <= -1;
        dack_t1    <= 0;
    end
    else if(dackmon_en) begin
        if(dack0_win) begin
            dackw_cnt <= dackw_cnt + 1;
            if(cs0_n)   dack_naked <= dack_naked + 1;
            if(rd_wr)   dack_on_rd <= dack_on_rd + 1;
            else        dack_on_wr <= dack_on_wr + 1;
        end
        if(!dack0_z && dack0_win) begin
            dackf_cnt <= dackf_cnt + 1;
            if(dack_t0 < 0) dack_t0 <= dackmon_t;
        end
        if(dack0_z && !dack0_win) dack_t1 <= dackmon_t;
        dackmon_t <= dackmon_t + 1;
        if(!ptd_o[1]) drak0_lo <= drak0_lo + 1;
        if( ptd_o[1]) drak0_hi <= drak0_hi + 1;
    end
end

task automatic dackmon_clear;
    begin
        dackmon_rst = 1'b1;
        run_cycles(2);
        dackmon_rst = 1'b0;
    end
endtask

/*
    DMA write-order log (round-robin/priority proofs): every completed
    DMAC write beat appends its address page nibble addr[11:8]. Tests
    give each channel a distinct destination page, so the accumulated
    hex literal IS the grant order. Single writer; the dmaw_clr knob
    clears (tasks must not write an always_ff variable directly).
*/

logic           dmaw_clr;               //write-order log clear knob
logic   [63:0]  dmaw_log;               //page nibbles, oldest leftmost
integer         dmaw_cnt;               //DMA write beats since clear

always @(posedge clk) begin
    if(dmaw_clr) begin
        dmaw_log <= 64'd0;
        dmaw_cnt <= 0;
    end
    else if(u_dut.IBUS1_DMA.rsp_valid && u_dut.IBUS1_DMA.rsp_ready &&
            u_dut.u_dmac.seq == 3'd4) begin     //S_WR_WAIT completion
        dmaw_log <= {dmaw_log[59:0], u_dut.u_dmac.addr_q[11:8]};
        dmaw_cnt <= dmaw_cnt + 1;
    end
end

/*
    MON match-queue oracle (sh3_sideband.md R1-R3/R8): the pump's
    matching algorithm as a passive whole-run checker. Every o_MON_REQ pulse
    enqueues {addr, wr, size, burst}; every external transaction UNIT start
    (SDRAM engine dispatch / ordinary envelope open, white-box) pops the
    head and compares. Push runs before pop - the registered strobe and
    the earliest grid dispatch land the same cycle. The BSC's own reset
    (WDT flavors included) flushes the queue: a strobed-undispatched op is
    legitimately dropped (spec R10). mon_qmax = the MEASURED R8 depth bound.
    Single writer; results checked in test_bus_monitors.
*/

logic   [32:0]  mon_q [0:3];             //{addr[28:0], wr, size[1:0], burst}
logic   [32:0]  mon_exp;
logic           mon_ordbusy_z = 1'b0;    //ord_busy delay for the rise detect
integer         mon_qn      = 0;         //queue occupancy
integer         mon_qmax    = 0;         //occupancy high-water (measured R8)
integer         mon_pushes  = 0;         //total strobes seen (coverage)
integer         mon_flushed = 0;         //reset-dropped strobes (R10 path)
integer         mon_err     = 0;         //oracle mismatches (must end 0)

always @(posedge clk) begin
    if(!u_dut.u_bsc.i_RST_n) begin              //any reset flavor: front-end dies
        mon_flushed   = mon_flushed + mon_qn;
        mon_qn        = 0;
        mon_ordbusy_z = 1'b0;
    end
    else begin
        if(mon_req) begin                        //push first (same-cycle pop is legal)
            if(mon_qn == 4) begin
                $display("      [FAIL] MON oracle: queue overflow");
                mon_err = mon_err + 1;
                mon_qn  = 0;
            end
            mon_q[mon_qn] = {mon_addr, mon_wr, mon_size, mon_burst};
            mon_qn       = mon_qn + 1;
            mon_pushes   = mon_pushes + 1;
            if(mon_qn > mon_qmax) mon_qmax = mon_qn;
        end
        if((u_dut.u_bsc.eng_start_tk && !u_dut.u_bsc.eng_op_mrs) ||
           (u_dut.u_bsc.ord_busy && !mon_ordbusy_z)) begin
            mon_exp = u_dut.u_bsc.eng_start_tk ?
                {u_dut.u_bsc.eng_addr[28:0], u_dut.u_bsc.eng_op_write,
                 u_dut.u_bsc.eng_op_size,    u_dut.u_bsc.eng_op_burst} :
                {u_dut.u_bsc.ord_addr[28:0], u_dut.u_bsc.ord_write,
                 u_dut.u_bsc.ord_size,       u_dut.u_bsc.ord_burst};
            if(mon_qn == 0) begin
                $display("      [FAIL] MON oracle: unit start with empty queue (exp=%h)", mon_exp);
                mon_err = mon_err + 1;
            end
            else begin
                if(mon_q[0] !== mon_exp) begin
                    $display("      [FAIL] MON oracle: head mismatch got=%h exp=%h", mon_q[0], mon_exp);
                    mon_err = mon_err + 1;
                end
                mon_q[0] = mon_q[1]; mon_q[1] = mon_q[2]; mon_q[2] = mon_q[3];
                mon_qn   = mon_qn - 1;
            end
        end
        mon_ordbusy_z = u_dut.u_bsc.ord_busy;    //updated last: rise detect above
    end
end



///////////////////////////////////////////////////////////
//////  Retirement And Event Scoreboard
////

logic           retired_seen [0:2047];
integer         retire_count [0:2047];
logic           exc_seen;
logic           trapa_seen;
integer         entry_count;
logic   [31:0]  entry_pc_l;

logic           bench_arm = 1'b0;
logic           bench_active, bench_started;
integer         bench_arch_cycles, bench_retires;
integer         bench_cs0f;             //CS0 assertion edges inside the window: a
logic           bench_cs0_z;            //burst-ROM read run frames ONCE (fig 23.19)

always_ff @(posedge clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        retired_seen      <= '{default:1'b0};
        retire_count      <= '{default:0};
        exc_seen          <= 1'b0;
        trapa_seen        <= 1'b0;
        entry_count       <= 0;
        entry_pc_l        <= 32'd0;
        bench_active      <= 1'b0;
        bench_started     <= 1'b0;
        bench_arch_cycles <= 0;
        bench_retires     <= 0;
    end
    else begin
        if(retire_valid) begin
            retired_seen[retire_pc[11:1]] <= 1'b1;
            retire_count[retire_pc[11:1]] <= retire_count[retire_pc[11:1]] + 1;
        end
        if(exc_valid) exc_seen <= 1'b1;
        if(trapa_valid) trapa_seen <= 1'b1;
        if(entry_valid) begin
            entry_count <= entry_count + 1;
            entry_pc_l  <= entry_pc;
        end

        //IPC benchmark window - identical to cpu_core_tb so the numbers compare 1:1
        if(bench_arm && !bench_active) begin
            bench_active      <= 1'b1;
            bench_started     <= 1'b0;
            bench_arch_cycles <= 0;
            bench_retires     <= 0;
            bench_cs0f        <= 0;
        end
        else if(!bench_arm) begin
            bench_active <= 1'b0;
        end
        else if(bench_active) begin
            if(retire_valid && !bench_started) bench_started <= 1'b1;
            if(bench_started) begin
                bench_arch_cycles <= bench_arch_cycles + 1;
                if(retire_valid) bench_retires <= bench_retires + 1;
                //line-burst CS0 assertions only (fills), not the uncached
                //preamble's single fetches - the envelope law counts frames
                if(bench_cs0_z && !cs0_n &&
                   u_dut.u_bsc.ord_busy && u_dut.u_bsc.ord_burst)
                    bench_cs0f <= bench_cs0f + 1;
            end
        end
        bench_cs0_z <= cs0_n;
    end
end


///////////////////////////////////////////////////////////
//////  SoC Boundary Contracts (suite-wide passive checkers)
////

/*
    SoC twins of the cpu_core_tb suite-wide contracts. The lock-pairing law
    watches the CPU's own bus (IBUS1_CORE, the splitter input): every locked
    READ opens an RMW pair that exactly one locked WRITE closes - no nested
    reads, no widowed writes (TAS.B indivisibility, p.320). The ack law is
    the INTC handshake: o_INT_ACK must land ON an exception-entry edge (an
    ack without an entry LOSES the interrupt - the INTC drops the request),
    and one cycle later the core's INTEVT must hold the code presented at
    the ack edge while the INTC's INTEVT2 holds its per-source code2 (the
    two differ for kind=0 sources, where INTEVT gets the level code).
    Coverage counters prove the sweeps land entries mid-machinery.
*/
integer         int_ack_cnt;                    //clocked ack counter (sweep drop key)
integer         locked_rd_cnt, locked_wr_cnt;   //accepted locked beats on IBUS1_CORE
integer         lock_pairs_checked = 0;
integer         lock_pair_viol     = 0;
logic           lock_open_q;
integer         ack_checks = 0, ack_viol = 0;
logic           ack_z;
logic   [11:0]  ack_code_z;                     //o_INT_CODE at the ack edge (-> INTEVT)
logic   [11:0]  ack_code2_z;                    //INTC b_code2_q at the ack edge (-> INTEVT2)
integer         entry_cache_busy = 0;           //entries with the cache FSM mid-excursion
integer         entry_sdram_busy = 0;           //entries with the SDRAM engine mid-cycle
logic   [4:0]   cache_st, bsc_est;
localparam logic [4:0] TB_CS_IDLE = 5'd1;       //cache state_t encoding (drift guard: cpu_core_tb)

wire            core_lock_beat = u_dut.IBUS1_CORE.req_valid && u_dut.IBUS1_CORE.req_ready &&
                                 u_dut.IBUS1_CORE.req_lock;

always @(posedge clk) begin
    cache_st = u_dut.u_cpu.u_cache.state;
    bsc_est  = u_dut.u_bsc.est;
    if(!sys_rst_n) begin
        int_ack_cnt   = 0;
        locked_rd_cnt = 0;
        locked_wr_cnt = 0;
        lock_open_q   = 1'b0;
        ack_z         = 1'b0;
    end
    else begin
        if(u_dut.int_ack) int_ack_cnt = int_ack_cnt + 1;
        //(1) locked read->write pairing, observed as IBUS1_CORE accept beats
        if(core_lock_beat) begin
            if(u_dut.IBUS1_CORE.req_write) begin
                locked_wr_cnt = locked_wr_cnt + 1;
                if(!lock_open_q) begin
                    lock_pair_viol = lock_pair_viol + 1;
                    $display("      [LOCK] locked WRITE with no locked read open");
                end
                else lock_pairs_checked = lock_pairs_checked + 1;
                lock_open_q = 1'b0;
            end
            else begin
                locked_rd_cnt = locked_rd_cnt + 1;
                if(lock_open_q) begin
                    lock_pair_viol = lock_pair_viol + 1;
                    $display("      [LOCK] second locked READ while a pair is open");
                end
                lock_open_q = 1'b1;
            end
        end
        //(2) every ack enters, and both event registers reflect the acked codes
        if(u_dut.int_ack) begin
            ack_checks = ack_checks + 1;
            if(!entry_valid) begin
                ack_viol = ack_viol + 1;
                $display("      [ACK] o_INT_ACK without an interrupt entry");
            end
        end
        if(ack_z) begin
            if(intevt_o[11:0] !== ack_code_z) begin
                ack_viol = ack_viol + 1;
                $display("      [ACK] INTEVT %03h != presented code %03h", intevt_o[11:0], ack_code_z);
            end
            if(intevt2_reg[11:0] !== ack_code2_z) begin
                ack_viol = ack_viol + 1;
                $display("      [ACK] INTEVT2 %03h != presented code2 %03h", intevt2_reg[11:0], ack_code2_z);
            end
        end
        ack_z       = u_dut.int_ack;
        ack_code_z  = u_dut.int_code;
        ack_code2_z = u_dut.u_intc.b_code2_q;
        //(3) entry-vs-machinery coverage (verdicts in the sweep tests)
        if(entry_valid && cache_st != TB_CS_IDLE) entry_cache_busy = entry_cache_busy + 1;
        if(entry_valid && bsc_est  != 5'd0)       entry_sdram_busy = entry_sdram_busy + 1;
    end
end


///////////////////////////////////////////////////////////
//////  Test Harness Helpers
////

integer         errors      = 0;
integer         test_errors = 0;
integer         test_count  = 0;

//Committed register value; valid while reset bank state (MD=RB=1) is active.
function automatic logic [31:0] gpr(input integer n);
    gpr = u_dut.u_cpu.u_int_pipe.u_gpr_bram.ram[8 + n];
endfunction

task automatic chk(input string nm, input logic [31:0] got, input logic [31:0] exp);
    begin
        if(got !== exp) begin
            $display("      [FAIL] %s = %08h (expected %08h)", nm, got, exp);
            test_errors = test_errors + 1;
            errors      = errors + 1;
        end
    end
endtask

task automatic chk_true(input string nm, input logic cond);
    begin
        if(cond !== 1'b1) begin
            $display("      [FAIL] %s", nm);
            test_errors = test_errors + 1;
            errors      = errors + 1;
        end
    end
endtask

task automatic init_knobs;
    begin
        if_fault_en    = 1'b0;
        if_fault_widx  = 11'd0;
        d_fault_en     = 1'b0;
        d_fault_widx   = 8'd0;
        d_latency      = 0;
        raw_mode       = 0;
        wait_stretch   = 0;
        dackw_arm      = 0;
        flash_en       = 1'b0;
        raw8_en        = 1'b0;      //area 6 back to the 32-bit raw device
        sdram_en       = 1'b1;      //Micron model populated
        md4_pin        = 1'b1;      //area 0 back to the 32-bit boot straps
        md3_pin        = 1'b1;
        ptd_pin        = 8'h00;     //port D pads (the historic tie; DREQ tests
        sgdev_en       = 1'b0;      //raise PTD4/6 to the negated-high idle)
        sgdev_clr      = 1'b0;
        dackmon_en     = 1'b0;
        dmaw_clr       = 1'b0;
    end
endtask

task automatic clear_imem;
    integer i;
    begin
        for(i = 0; i < 2048; i = i + 1) imem[i] = 16'h0009; //NOP fill
    end
endtask

task automatic clear_dmem;
    integer i;
    begin
        for(i = 0; i < 256; i = i + 1) dmem[i] = 32'd0;
    end
endtask

task automatic group(input string name);
    begin
        $display("");
        $display("==== %s ====", name);
    end
endtask

task automatic begin_test(input string name);
    begin
        test_errors = 0;
        test_count  = test_count + 1;
        $display("  [%2d] %s", test_count, name);
        init_knobs;
        clear_imem;
        clear_dmem;
        nmi_pin  = 1'b0;
        irq_pin  = '1;              //IRQ/IRL pins idle high (IRL 1111 = no request)
        ptf_pin  = '0;              //PINT idle; IRLS tests raise to '1 BEFORE IRLSEN
        pint_pin = '0;
        tclk_pin = 1'b0;
        pta_pin  = '0;
        ptg_pin  = '0;
    end
endtask

task automatic end_test;
    begin
    end
endtask

//Full power-on reset: both pins. CPG/WDT registers only clear on the POR pin
//(p.215), so tests always start from a clean WDT.
task automatic do_reset;
    begin
        por_n = 1'b0;
        rst_n = 1'b0;
        repeat(4) @(posedge clk);
        por_n = 1'b1;
        rst_n = 1'b1;
        @(posedge clk);
    end
endtask

task automatic run_until_retire(input integer widx, input integer timeout);
    integer c;
    begin
        c = 0;
        while(!retired_seen[widx] && c < timeout) begin
            @(posedge clk);
            c = c + 1;
        end
        repeat(6) @(posedge clk); //pipeline drain tail
    end
endtask

task automatic run_until_entry_count(input integer n, input integer timeout);
    integer c;
    begin
        c = 0;
        while(entry_count < n && c < timeout) begin
            @(posedge clk);
            c = c + 1;
        end
        repeat(2) @(posedge clk); //let INTEVT/INTEVT2 latch
    end
endtask

task automatic run_until_wdt_reset(input integer timeout);
    integer c;
    begin
        c = 0;
        while(!wdt_rst_hit && c < timeout) begin
            @(posedge clk);
            c = c + 1;
        end
    end
endtask

task automatic run_cycles(input integer n);
    begin
        repeat(n) @(posedge clk);
    end
endtask


///////////////////////////////////////////////////////////
//////  Program Emit Helpers (hand-assembled SH-3)
////

//Rn <- 0xA4000000 (INTC-low window base): 0x52 << 25
task automatic emit_a4_base(inout integer idx, input integer rn);
    begin
        imem[idx] = 16'hE052 | (rn << 8); idx = idx + 1; // MOV    #0x52,Rn
        imem[idx] = 16'h4028 | (rn << 8); idx = idx + 1; // SHLL16 Rn
        imem[idx] = 16'h4018 | (rn << 8); idx = idx + 1; // SHLL8  Rn
        imem[idx] = 16'h4000 | (rn << 8); idx = idx + 1; // SHLL   Rn
    end
endtask

//R0 <- 0xFFFFFE00 | off (INTC-high window: ICR0 0xE0 / IPRA 0xE2 / IPRB 0xE4)
task automatic emit_hi_base(inout integer idx, input logic [7:0] off);
    begin
        imem[idx] = 16'hE0FE;        idx = idx + 1;      // MOV   #0xFE,R0    ; 0xFFFFFFFE
        imem[idx] = 16'h4018;        idx = idx + 1;      // SHLL8 R0          ; 0xFFFFFE00
        imem[idx] = 16'hCB00 | off;  idx = idx + 1;      // OR    #off,R0
    end
endtask

//SR <- 0x60000000 | (imask << 4): MD=RB=1, BL=0, IMASK=imask (via R0)
task automatic emit_sr_imask(inout integer idx, input logic [3:0] imask);
    begin
        imem[idx] = 16'hE060;                idx = idx + 1;  // MOV    #0x60,R0
        imem[idx] = 16'h4028;                idx = idx + 1;  // SHLL16 R0
        imem[idx] = 16'h4018;                idx = idx + 1;  // SHLL8  R0        ; 0x60000000
        imem[idx] = 16'hCB00 | (imask << 4); idx = idx + 1;  // OR     #imask<<4,R0
        imem[idx] = 16'h400E;                idx = idx + 1;  // LDC    R0,SR
    end
endtask

//sentinel + self-loop tail: retires forever in place (interruptible, no PC wrap)
task automatic emit_sentinel_loop(inout integer idx, output integer sentinel_idx);
    begin
        sentinel_idx = idx;
        imem[idx] = 16'hE65A; idx = idx + 1;              // MOV #0x5A,R6      ; sentinel
        imem[idx] = 16'hAFFE; idx = idx + 1;              // BRA self
        imem[idx] = 16'h0009; idx = idx + 1;              // NOP (delay slot)
    end
endtask

/*
    Standard interrupt handler at VBR+0x600 (imem 0x300). Mailboxes (dmem):
      0x40 mb0 = INTEVT (L-bus read 0xFFFFFFD8)
      0x44 mb1 = INTEVT2 (bridge read 0xA4000000)
      0x48 mb2 = IRR0 (bridge byte read)
      0x4C mb3 = post-clear IRR0 (clear variant) or WTCSR (wdt-stop variant)
    Options: clear IRR0 bits by writing irr0_mask (write-0-clears, p.138);
    stop the WDT with a keyed WTCSR write 0xA500 (clears IOVF -> drops ITI).
    Grace NOPs give the tb time to release level pins before RTE.
*/
integer handler_grace_idx;      //first grace-NOP imem index of the last emit_handler

task automatic emit_handler(input logic clear_irr0, input logic [7:0] irr0_mask,
                            input logic stop_wdt);
    integer idx, k;
    begin
        idx = 'h300;
        imem[idx] = 16'hE0D8; idx = idx + 1;              // MOV   #0xD8,R0    ; INTEVT
        imem[idx] = 16'h6102; idx = idx + 1;              // MOV.L @R0,R1
        imem[idx] = 16'hE240; idx = idx + 1;              // MOV   #0x40,R2    ; mailbox base
        imem[idx] = 16'h2212; idx = idx + 1;              // MOV.L R1,@R2      ; mb0 = INTEVT
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'h6432; idx = idx + 1;              // MOV.L @R3,R4      ; INTEVT2 via bridge
        imem[idx] = 16'h1241; idx = idx + 1;              // MOV.L R4,@(4,R2)  ; mb1 = INTEVT2
        imem[idx] = 16'h8434; idx = idx + 1;              // MOV.B @(4,R3),R0  ; IRR0
        imem[idx] = 16'h1202; idx = idx + 1;              // MOV.L R0,@(8,R2)  ; mb2 = IRR0
        if(clear_irr0) begin
            imem[idx] = 16'hE000 | irr0_mask; idx = idx + 1; // MOV #mask,R0   ; 0-bits clear
            imem[idx] = 16'h8034; idx = idx + 1;          // MOV.B R0,@(4,R3)  ; IRR0 write
            imem[idx] = 16'h8434; idx = idx + 1;          // MOV.B @(4,R3),R0  ; re-read
            imem[idx] = 16'h1203; idx = idx + 1;          // MOV.L R0,@(12,R2) ; mb3 = post-clear
        end
        if(stop_wdt) begin
            imem[idx] = 16'hE180; idx = idx + 1;          // MOV   #0x80,R1    ; 0xFFFFFF80
            imem[idx] = 16'h8416; idx = idx + 1;          // MOV.B @(6,R1),R0  ; WTCSR
            imem[idx] = 16'h1203; idx = idx + 1;          // MOV.L R0,@(12,R2) ; mb3 = WTCSR
            imem[idx] = 16'hE0A5; idx = idx + 1;          // MOV   #0xA5,R0
            imem[idx] = 16'h4018; idx = idx + 1;          // SHLL8 R0          ; 0xA500
            imem[idx] = 16'h8113; idx = idx + 1;          // MOV.W R0,@(3,R1)  ; WTCSR<=0: stop+clear
        end
        handler_grace_idx = idx;    //tb pin-release key: retire-based, cadence-independent
        for(k = 0; k < 24; k = k + 1) begin
            imem[idx] = 16'h0009; idx = idx + 1;          // grace NOPs (pin release window)
        end
        imem[idx] = 16'h002B; idx = idx + 1;              // RTE
        imem[idx] = 16'h0009; idx = idx + 1;              // NOP (delay slot)
    end
endtask

/*
    TMU interrupt handler at VBR+0x600. Mailboxes: mb0 = INTEVT, mb1 =
    INTEVT2, mb2 = TCPR2. Then TCR(tcr_off) <= tcr_val - the write-0 clears
    UNF/ICPF and drops the level request (p.394); short grace window (the
    request is a flag we cleared, not a pin the tb must release).
*/
task automatic emit_handler_tmu(input logic [7:0] tcr_off, input logic [15:0] tcr_val);
    integer idx, k;
    begin
        idx = 'h300;
        imem[idx] = 16'hE0D8; idx = idx + 1;              // MOV   #0xD8,R0    ; INTEVT (L-bus MMIO)
        imem[idx] = 16'h6102; idx = idx + 1;              // MOV.L @R0,R1
        imem[idx] = 16'hE240; idx = idx + 1;              // MOV   #0x40,R2    ; mailbox base
        imem[idx] = 16'h2212; idx = idx + 1;              // MOV.L R1,@R2      ; mb0 = INTEVT
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'h6432; idx = idx + 1;              // MOV.L @R3,R4      ; INTEVT2 via bridge
        imem[idx] = 16'h1241; idx = idx + 1;              // MOV.L R4,@(4,R2)  ; mb1 = INTEVT2
        imem[idx] = 16'hE0FE; idx = idx + 1;              // MOV   #0xFE,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0xFFFFFE00
        imem[idx] = 16'hCBB8; idx = idx + 1;              // OR    #0xB8,R0    ; TCPR2
        imem[idx] = 16'h6103; idx = idx + 1;              // MOV   R0,R1
        imem[idx] = 16'h6412; idx = idx + 1;              // MOV.L @R1,R4
        imem[idx] = 16'h1242; idx = idx + 1;              // MOV.L R4,@(8,R2)  ; mb2 = TCPR2
        imem[idx] = 16'hE0FE; idx = idx + 1;              // MOV   #0xFE,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB00 | tcr_off; idx = idx + 1;    // OR    #tcr_off,R0
        imem[idx] = 16'h6103; idx = idx + 1;              // MOV   R0,R1       ; TCR address
        imem[idx] = 16'hE000 | tcr_val[15:8]; idx = idx + 1; // MOV #hi,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB00 | tcr_val[7:0]; idx = idx + 1;  // OR  #lo,R0
        imem[idx] = 16'h2101; idx = idx + 1;              // MOV.W R0,@R1      ; clear flags
        for(k = 0; k < 8; k = k + 1) begin
            imem[idx] = 16'h0009; idx = idx + 1;          // grace NOPs (resolver settle)
        end
        imem[idx] = 16'h002B; idx = idx + 1;              // RTE
        imem[idx] = 16'h0009; idx = idx + 1;              // NOP (delay slot)
    end
endtask


///////////////////////////////////////////////////////////
//////  BSC / SDRAM Helpers
////

//module-scope emit index for the helper family below: Verilator miscompiles
//inout-integer copy-back through nested automatic task chains (observed as a
//silently dropped index update), so these helpers share one static index
integer eidx;

//R0 <- any 32-bit constant: byte-build (sign bits shift out; OR imm is R0-only)
task automatic emit_ldr0(input logic [31:0] v);
    begin
        imem[eidx] = 16'hE000 | v[31:24]; eidx = eidx + 1;   // MOV   #b3,R0
        imem[eidx] = 16'h4018;            eidx = eidx + 1;   // SHLL8 R0
        imem[eidx] = 16'hCB00 | v[23:16]; eidx = eidx + 1;   // OR    #b2,R0
        imem[eidx] = 16'h4018;            eidx = eidx + 1;   // SHLL8 R0
        imem[eidx] = 16'hCB00 | v[15:8];  eidx = eidx + 1;   // OR    #b1,R0
        imem[eidx] = 16'h4018;            eidx = eidx + 1;   // SHLL8 R0
        imem[eidx] = 16'hCB00 | v[7:0];   eidx = eidx + 1;   // OR    #b0,R0
    end
endtask

//Rn <- 32-bit constant (through R0)
task automatic emit_ldrn(input integer rn, input logic [31:0] v);
    begin
        emit_ldr0(v);
        imem[eidx] = 16'h6003 | (rn << 8); eidx = eidx + 1;  // MOV R0,Rn
    end
endtask

//16-bit register write: [addr] <- val (word store via R1/R0)
task automatic emit_wreg_w(input logic [31:0] addr, input logic [15:0] val);
    begin
        emit_ldrn(1, addr);
        emit_ldr0({16'd0, val});
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1
    end
endtask

//SDRAM power-on sequence (manual order, p.302): BCR1.DRAMTP -> WCR2(CL) ->
//MCR(AMX/TPC/RCD/TRWL/TRAS) -> SDMR byte write (mode value rides the address)
task automatic emit_sdram_init(input logic [15:0] mcr_v,
                               input logic [15:0] wcr2_v, input logic [31:0] sdmr_a);
    begin
        emit_wreg_w(32'hFFFF_FF60, 16'h0008);             // BCR1: DRAMTP=010 (area 3 SDRAM)
        emit_wreg_w(32'hFFFF_FF66, wcr2_v);               // WCR2: CAS latency
        emit_wreg_w(32'hFFFF_FF68, mcr_v);                // MCR: timing + AMX + refresh
        emit_ldrn(1, sdmr_a);                             // SDMR: PALL+MRS cycle
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1 (data ignored)
    end
endtask

//Micron model backdoor: AMX=0111 mapping, bank=phys[22:21], {row,col}=phys[20:2]
function automatic logic [18:0] sd_index(input logic [31:0] phys);
    sd_index = {phys[20:10], phys[9:2]};
endfunction

task automatic sdram_poke(input logic [31:0] phys, input logic [31:0] data);
    begin
        case(phys[22:21])
            2'd0: u_sdram.Bank0[sd_index(phys)] = data;
            2'd1: u_sdram.Bank1[sd_index(phys)] = data;
            2'd2: u_sdram.Bank2[sd_index(phys)] = data;
            2'd3: u_sdram.Bank3[sd_index(phys)] = data;
        endcase
    end
endtask

function automatic logic [31:0] sdram_peek(input logic [31:0] phys);
    begin
        case(phys[22:21])
            2'd0: sdram_peek = u_sdram.Bank0[sd_index(phys)];
            2'd1: sdram_peek = u_sdram.Bank1[sd_index(phys)];
            2'd2: sdram_peek = u_sdram.Bank2[sd_index(phys)];
            2'd3: sdram_peek = u_sdram.Bank3[sd_index(phys)];
        endcase
    end
endfunction


//flash backdoor: the model packs a 16-bit word as Q = {ARRAY[2n+1], ARRAY[2n]}
//and the big-endian SH bus wants D15-8 = the lower byte address, so the two
//bytes land swapped in the array (a real programmer file does the same)
task automatic flash_poke_h(input logic [20:0] wa, input logic [15:0] v);
    begin
        u_flash.ARRAY[{wa, 1'b0}] = v[7:0];
        u_flash.ARRAY[{wa, 1'b1}] = v[15:8];
    end
endtask

//longword at a long-aligned byte address: MS half at the lower word address
task automatic flash_poke_l(input logic [21:0] ba, input logic [31:0] v);
    begin
        flash_poke_h(ba[21:1],           v[31:16]);
        flash_poke_h(ba[21:1] + 21'd1,   v[15:0]);
    end
endtask

//copy the emitted imem program into the flash array (boot-from-flash tests):
//imem[i] is the halfword at phys 2*i, the same layout the fetch path expects
task automatic flash_load_imem;
    integer i;
    begin
        for(i = 0; i < 2048; i = i + 1) flash_poke_h(i[20:0], imem[i]);
    end
endtask

//pack a 16-bit opcode into the Micron array (big-endian halves, imem-model order)
task automatic emit_sd(input logic [15:0] op);
    logic [31:0] phys, w;
    begin
        phys = 32'h0C00_0000 + {eidx, 1'b0};
        w = sdram_peek({phys[31:2], 2'b00});
        if(phys[1]) w[15:0]  = op;
        else        w[31:16] = op;
        sdram_poke({phys[31:2], 2'b00}, w);
        eidx = eidx + 1;
    end
endtask


///////////////////////////////////////////////////////////
//////  IPC Parity Benches (ported 1:1 from cpu_core_tb)
////

task automatic bench_ipc_straightline(input integer n);
    integer ms_ipc;
    begin
        clear_imem;
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(n, 40000);
        bench_arm = 1'b0;
        @(posedge clk);
        if(bench_arch_cycles > 0) begin
            ms_ipc = (bench_retires * 1000) / bench_arch_cycles;
            $display("  [BENCH] straight-line NOP: %0d retires / %0d arch-cycles -> IPC = %0d.%03d (bypass path)",
                     bench_retires, bench_arch_cycles, ms_ipc/1000, ms_ipc%1000);
        end
        else
            $display("  [BENCH] straight-line NOP: no retirements measured");
        //parity gate: the splitter must add ZERO beats over the cpu_core_tb baseline
        chk("straightline retires (parity)",     bench_retires,     32'd203);  //relocked 2026-07-05 (fetch pair)
        chk("straightline arch-cycles (parity)", bench_arch_cycles, 32'd506);  //relocked 2026-07-05 (fetch pair)
    end
endtask

task automatic cacheable_bootstrap;
    begin
        clear_imem;
        imem[0] = 16'hE0EC; // MOV   #0xEC,R0  ; R0 = 0xFFFFFFEC (CCR)
        imem[1] = 16'hE109; // MOV   #9,R1     ; CCR.CE=1 | CCR.CF=1
        imem[2] = 16'h2012; // MOV.L R1,@R0    ; enable the unified cache + flush
        imem[3] = 16'h0009; // NOP
        imem[4] = 16'h0009; // NOP
        imem[5] = 16'hE240; // MOV   #0x40,R2  ; P0 cacheable entry
        imem[6] = 16'h422B; // JMP   @R2
        imem[7] = 16'h0009; // NOP             ; delay slot
    end
endtask

task automatic bench_ipc_cached(input integer body, input integer iters);
    integer idx, j, loop_idx, bf_idx, sentinel_idx, disp;
    integer guard;
    begin
        cacheable_bootstrap;
        idx = 'h20;
        imem[idx] = 16'hE500 | (iters & 8'hFF);idx = idx + 1; // MOV #iters,R5
        loop_idx = idx;
        imem[idx] = 16'hE300;                  idx = idx + 1; // MOV #0,R3
        for(j = 0; j < body; j = j + 1) begin
            imem[idx] = 16'h7301;              idx = idx + 1; // ADD #1,R3
        end
        imem[idx] = 16'h4510;                  idx = idx + 1; // DT R5
        bf_idx = idx;
        disp   = loop_idx - bf_idx - 2;
        imem[idx] = 16'h8B00 | (disp & 8'hFF); idx = idx + 1; // BF loop_start
        sentinel_idx = idx;
        imem[idx] = 16'hE65A;                  idx = idx + 1; // MOV #0x5A,R6

        do_reset;
        guard = 0;
        while(retire_count[loop_idx] < 2 && guard < 40000) begin
            @(posedge clk); guard = guard + 1;
        end
        bench_arm = 1'b1;
        run_until_retire(sentinel_idx, 60000);
        bench_arm = 1'b0;
        @(posedge clk);

        if(bench_arch_cycles > 0)
            $display("  [BENCH] cached add loop (CCR.CE=%0d): %0d retires / %0d arch-cycles -> IPC = %0d.%03d (cache-hit path)",
                     u_dut.u_cpu.u_cache.ccr_ce, bench_retires, bench_arch_cycles,
                     ((bench_retires * 1000) / bench_arch_cycles) / 1000,
                     ((bench_retires * 1000) / bench_arch_cycles) % 1000);
        else
            $display("  [BENCH] cached add loop: no steady state measured (warm-up guard=%0d)", guard);

        chk("cached loop R3 = body (fetch integrity)", gpr(3), body);
        chk("cached loop R5 = 0 (DT;BF count)",        gpr(5), 32'd0);
        //Relocked 2026-07-05: the wrong-path fetch-leak fix (int_pipe drop_d) removed
        //~1 bogus retire per taken branch; cycle count unchanged (same real work).
        chk("cached retires (parity)",     bench_retires,     32'd1137);
        chk("cached arch-cycles (parity)", bench_arch_cycles, 32'd1167);
        do_reset;
    end
endtask

task automatic bench_ipc_store(input integer nstores, input integer iters);
    integer idx, j, loop_idx, bf_idx, sentinel_idx, disp, guard;
    begin
        cacheable_bootstrap;
        idx = 'h20;
        imem[idx] = 16'hE702;                  idx = idx + 1; // MOV   #2,R7
        imem[idx] = 16'h4718;                  idx = idx + 1; // SHLL8 R7      ; 0x200
        imem[idx] = 16'hE15A;                  idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'hE500 | (iters & 8'hFF);idx = idx + 1; // MOV   #iters,R5
        loop_idx = idx;
        for(j = 0; j < nstores; j = j + 1) begin
            imem[idx] = 16'h2712;              idx = idx + 1; // MOV.L R1,@R7
        end
        imem[idx] = 16'h4510;                  idx = idx + 1; // DT R5
        bf_idx = idx;
        disp   = loop_idx - bf_idx - 2;
        imem[idx] = 16'h8B00 | (disp & 8'hFF); idx = idx + 1; // BF loop_start
        imem[idx] = 16'h6272;                  idx = idx + 1; // MOV.L @R7,R2
        sentinel_idx = idx;
        imem[idx] = 16'h0009;                  idx = idx + 1; // NOP sentinel

        do_reset;
        guard = 0;
        while(retire_count[loop_idx] < 2 && guard < 40000) begin
            @(posedge clk); guard = guard + 1;
        end
        bench_arm = 1'b1;
        run_until_retire(sentinel_idx, 60000);
        bench_arm = 1'b0;
        @(posedge clk);

        if(bench_arch_cycles > 0)
            $display("  [BENCH] cached store loop (%0d stores/iter): %0d retires / %0d arch-cycles -> IPC = %0d.%03d",
                     nstores, bench_retires, bench_arch_cycles,
                     ((bench_retires * 1000) / bench_arch_cycles) / 1000,
                     ((bench_retires * 1000) / bench_arch_cycles) % 1000);
        chk("store loop verify load -> R2", gpr(2), 32'h0000_005A);
        chk("store loop R5 = 0 (DT;BF count)", gpr(5), 32'd0);
        //Relocked 2026-07-05 (fetch-leak fix): the old 827-cycle figure was TRUNCATED -
        //a leaked wrong-path sentinel retire ended the measurement window early.
        chk("store retires (parity)",     bench_retires,     32'd415);
        chk("store arch-cycles (parity)", bench_arch_cycles, 32'd748);
        do_reset;
    end
endtask


///////////////////////////////////////////////////////////
//////  Tests - Bridge And Register Access
////

task automatic test_boot_smoke;
    integer idx, sent;
    begin
        begin_test("Boot smoke: program runs through the splitter EXT leg and retires");
        idx = 0;
        imem[idx] = 16'hE107; idx = idx + 1;              // MOV #7,R1
        imem[idx] = 16'h7103; idx = idx + 1;              // ADD #3,R1
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 5000);
        chk("R1 = 10", gpr(1), 32'd10);
        chk("sentinel R6", gpr(6), 32'h0000_005A);
        end_test;
    end
endtask

task automatic test_bridge_word_rw;
    integer idx, sent;
    begin
        begin_test("Bridge word R/W: IPRA/IPRB lanes, ICR1 reset 0x4000, ICR0.NMIE");
        idx = 0;
        //IPRA (0xFFFFFEE2, odd halfword lane) write/readback
        emit_hi_base(idx, 8'hE2);                         // R0 = 0xFFFFFEE2
        imem[idx] = 16'hE112; idx = idx + 1;              // MOV   #0x12,R1
        imem[idx] = 16'h4118; idx = idx + 1;              // SHLL8 R1          ; 0x1200
        imem[idx] = 16'h7134; idx = idx + 1;              // ADD   #0x34,R1    ; 0x1234
        imem[idx] = 16'h2011; idx = idx + 1;              // MOV.W R1,@R0      ; IPRA
        imem[idx] = 16'h6201; idx = idx + 1;              // MOV.W @R0,R2      ; readback
        //IPRB (0xFFFFFEE4, even halfword lane): [3:0] reserved-zero mask check
        imem[idx] = 16'h7002; idx = idx + 1;              // ADD   #2,R0       ; 0xFFFFFEE4
        imem[idx] = 16'hE3FF; idx = idx + 1;              // MOV   #0xFF,R3    ; 0xFFFFFFFF
        imem[idx] = 16'h2031; idx = idx + 1;              // MOV.W R3,@R0      ; IPRB <= 0xFFFF
        imem[idx] = 16'h6401; idx = idx + 1;              // MOV.W @R0,R4      ; -> 0xFFF0 (sign-ext)
        //ICR1 reset value through the P2 alias window
        emit_a4_base(idx, 5);                             // R5 = 0xA4000000
        imem[idx] = 16'h8558; idx = idx + 1;              // MOV.W @(16,R5),R0 ; ICR1 -> 0x4000
        imem[idx] = 16'h6503; idx = idx + 1;              // MOV   R0,R5       ; (R6 is the sentinel)
        //ICR0.NMIE write/read (NMI pin held low -> NMIL=0)
        emit_hi_base(idx, 8'hE0);                         // R0 = 0xFFFFFEE0
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        imem[idx] = 16'h4718; idx = idx + 1;              // SHLL8 R7          ; 0x0100 = NMIE
        imem[idx] = 16'h2071; idx = idx + 1;              // MOV.W R7,@R0
        imem[idx] = 16'h6101; idx = idx + 1;              // MOV.W @R0,R1      ; -> 0x0100
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 8000);
        chk("IPRA readback",       gpr(2), 32'h0000_1234);
        chk("IPRB reserved mask",  gpr(4), 32'hFFFF_FFF0);
        chk("ICR1 reset = 0x4000", gpr(5), 32'h0000_4000);
        chk("ICR0.NMIE readback",  gpr(1), 32'h0000_0100);
        end_test;
    end
endtask

task automatic test_bridge_byte_rw;
    integer idx, sent;
    begin
        begin_test("Bridge byte R/W: WTCNT/STBCR/WTCSR/IRR0 lanes, MOV.B sign-extension");
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1    ; 0xFFFFFF80
        //keyed word write WTCNT <= 0xF0, then byte read (lane addr[1:0]=00)
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x5A00
        imem[idx] = 16'hCBF0; idx = idx + 1;              // OR    #0xF0,R0    ; 0x5AF0
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)  ; WTCNT (0x84)
        imem[idx] = 16'h8414; idx = idx + 1;              // MOV.B @(4,R1),R0  ; -> sign-ext 0xF0
        imem[idx] = 16'h6203; idx = idx + 1;              // MOV   R0,R2
        //STBCR byte write/read (lane addr[1:0]=10)
        imem[idx] = 16'hE015; idx = idx + 1;              // MOV   #0x15,R0
        imem[idx] = 16'h8012; idx = idx + 1;              // MOV.B R0,@(2,R1)  ; STBCR (0x82)
        imem[idx] = 16'h8412; idx = idx + 1;              // MOV.B @(2,R1),R0
        imem[idx] = 16'h6303; idx = idx + 1;              // MOV   R0,R3
        //WTCSR byte read (0 after reset)
        imem[idx] = 16'h8416; idx = idx + 1;              // MOV.B @(6,R1),R0  ; WTCSR (0x86)
        imem[idx] = 16'h6403; idx = idx + 1;              // MOV   R0,R4
        //IRR0 byte read through the P2 alias (0 - no requests)
        emit_a4_base(idx, 5);                             // R5 = 0xA4000000
        imem[idx] = 16'h8454; idx = idx + 1;              // MOV.B @(4,R5),R0  ; IRR0
        imem[idx] = 16'h6703; idx = idx + 1;              // MOV   R0,R7
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 8000);
        chk("WTCNT byte read, sign-extended", gpr(2), 32'hFFFF_FFF0);
        chk("STBCR byte readback",            gpr(3), 32'h0000_0015);
        chk("WTCSR reset read",               gpr(4), 32'd0);
        chk("IRR0 idle read",                 gpr(7), 32'd0);
        end_test;
    end
endtask

task automatic test_bridge_long;
    integer idx, sent;
    begin
        begin_test("Bridge long read: INTEVT2 (0xA4000000) via MOV.L");
        idx = 0;
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'h6432; idx = idx + 1;              // MOV.L @R3,R4      ; INTEVT2 = 0 at reset
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 5000);
        chk("INTEVT2 reset read", gpr(4), 32'd0);
        end_test;
    end
endtask

task automatic test_cpg_regs;
    integer idx, sent;
    begin
        begin_test("CPG registers: FRQCR word-only W/R (bit8 stuck 1), STBCR2 byte R/W");
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1    ; 0xFFFFFF80
        //FRQCR word write 0x0112 / readback
        imem[idx] = 16'hE001; idx = idx + 1;              // MOV   #1,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x0100
        imem[idx] = 16'hCB12; idx = idx + 1;              // OR    #0x12,R0    ; 0x0112
        imem[idx] = 16'h2101; idx = idx + 1;              // MOV.W R0,@R1      ; FRQCR
        imem[idx] = 16'h6211; idx = idx + 1;              // MOV.W @R1,R2      ; -> 0x0112
        //byte write must be ignored (word-access-only register, p.211)
        imem[idx] = 16'hE055; idx = idx + 1;              // MOV   #0x55,R0
        imem[idx] = 16'h8010; idx = idx + 1;              // MOV.B R0,@(0,R1)
        imem[idx] = 16'h6311; idx = idx + 1;              // MOV.W @R1,R3      ; still 0x0112
        //STBCR2 byte write/read at 0x88
        imem[idx] = 16'hE07F; idx = idx + 1;              // MOV   #0x7F,R0
        imem[idx] = 16'h8018; idx = idx + 1;              // MOV.B R0,@(8,R1)  ; STBCR2
        imem[idx] = 16'h8418; idx = idx + 1;              // MOV.B @(8,R1),R0
        imem[idx] = 16'h6403; idx = idx + 1;              // MOV   R0,R4
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 8000);
        chk("FRQCR word readback",       gpr(2), 32'h0000_0112);
        chk("FRQCR byte write ignored",  gpr(3), 32'h0000_0112);
        chk("STBCR2 byte readback",      gpr(4), 32'h0000_007F);
        end_test;
    end
endtask

task automatic test_wdt_keyed_write;
    integer idx, sent;
    begin
        begin_test("WDT keyed writes: 0x5A/0xA5 keys honored; wrong key/byte/long ignored");
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1
        //good key: WTCNT <= 0x5A
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB5A; idx = idx + 1;              // OR    #0x5A,R0    ; 0x5A5A
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)
        imem[idx] = 16'h8414; idx = idx + 1;              // MOV.B @(4,R1),R0
        imem[idx] = 16'h6203; idx = idx + 1;              // MOV   R0,R2       ; 0x5A
        //wrong key 0x005A: ignored
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0    ; upper byte 0x00
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)
        imem[idx] = 16'h8414; idx = idx + 1;              // MOV.B @(4,R1),R0
        imem[idx] = 16'h6303; idx = idx + 1;              // MOV   R0,R3       ; still 0x5A
        //byte write: ignored
        imem[idx] = 16'hE077; idx = idx + 1;              // MOV   #0x77,R0
        imem[idx] = 16'h8014; idx = idx + 1;              // MOV.B R0,@(4,R1)
        imem[idx] = 16'h8414; idx = idx + 1;              // MOV.B @(4,R1),R0
        imem[idx] = 16'h6403; idx = idx + 1;              // MOV   R0,R4       ; still 0x5A
        //long write (even with a good-looking key): ignored
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB33; idx = idx + 1;              // OR    #0x33,R0    ; 0x5A33
        imem[idx] = 16'h1101; idx = idx + 1;              // MOV.L R0,@(4,R1)
        imem[idx] = 16'h8414; idx = idx + 1;              // MOV.B @(4,R1),R0
        imem[idx] = 16'h6503; idx = idx + 1;              // MOV   R0,R5       ; still 0x5A
        //WTCSR keyed write (TME left 0)
        imem[idx] = 16'hE0A5; idx = idx + 1;              // MOV   #0xA5,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB25; idx = idx + 1;              // OR    #0x25,R0    ; 0xA525
        imem[idx] = 16'h8113; idx = idx + 1;              // MOV.W R0,@(6,R1)
        imem[idx] = 16'h8416; idx = idx + 1;              // MOV.B @(6,R1),R0
        imem[idx] = 16'h6703; idx = idx + 1;              // MOV   R0,R7       ; 0x25
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 10000);
        chk("WTCNT keyed write",        gpr(2), 32'h0000_005A);
        chk("WTCNT wrong key ignored",  gpr(3), 32'h0000_005A);
        chk("WTCNT byte write ignored", gpr(4), 32'h0000_005A);
        chk("WTCNT long write ignored", gpr(5), 32'h0000_005A);
        chk("WTCSR keyed write",        gpr(7), 32'h0000_0025);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Tests - WDT
////

//shared program: FRQCR /1, IPRB[15:12]=iprb_lvl, SR.IMASK=imask, WDT interval
//from WTCNT=0xF8 (8 taps to overflow); handler mailboxes + stops the WDT
task automatic wdt_interval_program(input logic [3:0] iprb_lvl, input logic [3:0] imask,
                                    output integer sent);
    integer idx;
    begin
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1    ; CPG base
        imem[idx] = 16'hE001; idx = idx + 1;              // MOV   #1,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x0100
        imem[idx] = 16'h2101; idx = idx + 1;              // MOV.W R0,@R1      ; FRQCR: P-phi /1
        emit_hi_base(idx, 8'hE4);                         // R0 = IPRB
        imem[idx] = 16'hE200 | {4'd0, iprb_lvl}; idx = idx + 1; // MOV #lvl,R2
        imem[idx] = 16'h4228; idx = idx + 1;              // SHLL16 R2
        imem[idx] = 16'h4218; idx = idx + 1;              // SHLL8  R2         ; lvl<<24? no: lvl<<12
        //NOTE: lvl<<12 built as (lvl<<16)>>4 is wasteful; rebuild via two shifts:
        //undo: use SHLR? keep simple - rebuild below
        idx = idx - 2;                                    //scrap the two shifts above
        imem[idx] = 16'h4218; idx = idx + 1;              // SHLL8 R2          ; lvl<<8
        imem[idx] = 16'h4208; idx = idx + 1;              // SHLL2 R2          ; lvl<<10
        imem[idx] = 16'h4208; idx = idx + 1;              // SHLL2 R2          ; lvl<<12
        imem[idx] = 16'h2021; idx = idx + 1;              // MOV.W R2,@R0      ; IPRB
        emit_sr_imask(idx, imask);                        // unmask BEFORE starting the WDT
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCBF8; idx = idx + 1;              // OR    #0xF8,R0    ; 0x5AF8
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)  ; WTCNT = 0xF8
        imem[idx] = 16'hE0A5; idx = idx + 1;              // MOV   #0xA5,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB80; idx = idx + 1;              // OR    #0x80,R0    ; 0xA580: TME, interval
        imem[idx] = 16'h8113; idx = idx + 1;              // MOV.W R0,@(6,R1)  ; WTCSR - WDT runs
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b1);                  // wdt-stop handler
    end
endtask

task automatic test_wdt_interval_e2e;
    integer sent;
    begin
        begin_test("WDT interval end-to-end: ITI -> VBR+0x600, INTEVT=INTEVT2=0x560, IOVF");
        wdt_interval_program(4'hF, 4'h0, sent);
        do_reset;
        run_until_retire(sent, 10000);
        run_until_entry_count(1, 10000);
        run_cycles(200);                                  //handler runs, stops the WDT
        chk("entry PC = VBR+0x600",   entry_pc_l, 32'h0000_0600);
        chk("INTEVT  (L bus)",        dmem[16'h10], 32'h0000_0560);
        chk("INTEVT2 (bridge read)",  dmem[16'h11], 32'h0000_0560);
        chk("WTCSR in handler: TME|IOVF", dmem[16'h13], 32'hFFFF_FF88);
        chk("INTEVT probe",           intevt_o, 32'h0000_0560);
        chk("single entry",           entry_count, 32'd1);
        chk_true("ITI level dropped after keyed IOVF clear", u_dut.u_intc.i_ITI_REQ === 1'b0);
        end_test;
    end
endtask

task automatic test_wdt_iprb_mask;
    integer sent;
    begin
        begin_test("WDT masking: IPRB=0 never accepted; IMASK>=level blocks, lower accepts");
        //phase A: IPRB level 0 -> never accepted
        wdt_interval_program(4'h0, 4'h0, sent);
        do_reset;
        run_until_retire(sent, 10000);
        run_cycles(300);
        chk("IPRB=0: no entry", entry_count, 32'd0);
        //phase B: level 15, IMASK=15 -> blocked (15 > 15 is false)
        clear_imem; clear_dmem;
        wdt_interval_program(4'hF, 4'hF, sent);
        do_reset;
        run_until_retire(sent, 10000);
        run_cycles(300);
        chk("IMASK=15 blocks level 15", entry_count, 32'd0);
        //phase C: level 15, IMASK=14 -> accepted
        clear_imem; clear_dmem;
        wdt_interval_program(4'hF, 4'hE, sent);
        do_reset;
        run_until_retire(sent, 10000);
        run_until_entry_count(1, 10000);
        run_cycles(200);
        chk("IMASK=14 accepts level 15", entry_count, 32'd1);
        chk("INTEVT2 = 0x560", dmem[16'h11], 32'h0000_0560);
        end_test;
    end
endtask

task automatic wdt_watchdog_program(input logic rsts);
    integer idx;
    begin
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCBFC; idx = idx + 1;              // OR    #0xFC,R0    ; 0x5AFC
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)  ; WTCNT = 0xFC
        imem[idx] = 16'hE0A5; idx = idx + 1;              // MOV   #0xA5,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCBC0 | {7'd0, rsts, 5'd0};        // OR #0xC0|rsts<<5  ; TME|WT/IT(|RSTS)
        idx = idx + 1;
        imem[idx] = 16'h8113; idx = idx + 1;              // MOV.W R0,@(6,R1)  ; watchdog armed
        //spin: the watchdog reset arrives ~16 P-phi(/4) ticks later
    end
endtask

task automatic test_wdt_watchdog_por;
    begin
        begin_test("WDT watchdog reset (RSTS=0): EXPEVT=0x000, restart, WTCSR retained");
        wdt_watchdog_program(1'b0);
        do_reset;
        run_until_wdt_reset(20000);
        chk_true("watchdog reset request fired", wdt_rst_hit === 1'b1);
        clear_imem;                                       //the restarted run must not re-arm
        run_cycles(40);                                   //ride out the 16-cycle pulse + restart
        run_until_retire(0, 5000);                        //fresh program (NOPs) retires from idx 0
        chk("EXPEVT after WDT POR", expevt_o, 32'h0000_0000);
        chk_true("WOVF retained through the reset", u_dut.u_cpg_wdt.wt_wovf === 1'b1);
        chk_true("WT/IT retained",                  u_dut.u_cpg_wdt.wt_it   === 1'b1);
        end_test;
    end
endtask

task automatic test_wdt_watchdog_manual;
    begin
        begin_test("WDT watchdog reset (RSTS=1): EXPEVT=0x020 (manual-reset class)");
        wdt_watchdog_program(1'b1);
        do_reset;
        run_until_wdt_reset(20000);
        chk_true("watchdog reset request fired", wdt_rst_hit === 1'b1);
        clear_imem;
        run_cycles(40);
        run_until_retire(0, 5000);
        chk("EXPEVT after WDT manual reset", expevt_o, 32'h0000_0020);
        chk_true("WOVF retained", u_dut.u_cpg_wdt.wt_wovf === 1'b1);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Tests - INTC
////

task automatic test_irq_level;
    integer idx, sent, marker;
    begin
        begin_test("IRQ0 level mode: entry, INTEVT=levelcode, INTEVT2=0x600, no re-entry");
        idx = 0;
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'hE002; idx = idx + 1;              // MOV   #2,R0       ; IRQ0 sense = low level
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1 (IRQLVL=0)
        imem[idx] = 16'hE00C; idx = idx + 1;              // MOV   #0x0C,R0    ; IRQ0 level 12
        imem[idx] = 16'h813B; idx = idx + 1;              // MOV.W R0,@(22,R3) ; IPRC
        emit_sr_imask(idx, 4'h0);                         // unmask
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7       ; setup-done marker
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[0] = 1'b0;                                //assert (low level active)
        run_until_entry_count(1, 8000);
        //Retire-keyed release: the IRR0 mailbox read has committed once the grace
        //window starts, whatever the fetch cadence (the pair sped the handler up).
        run_until_retire(handler_grace_idx, 8000);
        irq_pin[0] = 1'b1;                                //release inside the handler grace
        run_cycles(300);
        chk("entry PC = VBR+0x600", entry_pc_l, 32'h0000_0600);
        chk("INTEVT = levelcode(12) = 0x260", dmem[16'h10], 32'h0000_0260);
        chk("INTEVT2 = 0x600",                dmem[16'h11], 32'h0000_0600);
        chk("IRR0 in handler shows IRQ0R",    dmem[16'h12], 32'h0000_0001);
        chk("single entry (level dropped)",   entry_count, 32'd1);
        end_test;
    end
endtask

task automatic test_irq_edge_irr0;
    integer idx, sent, marker;
    begin
        begin_test("IRQ0 falling edge: IRR0 pend, write-0-clear protocol, re-pend on 2nd pulse");
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE000; idx = idx + 1;              // MOV   #0,R0       ; IRQ0 sense = falling
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1 (IRQLVL=0)
        imem[idx] = 16'hE00C; idx = idx + 1;              // MOV   #0x0C,R0
        imem[idx] = 16'h813B; idx = idx + 1;              // MOV.W R0,@(22,R3) ; IPRC: IRQ0 lvl 12
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b1, 8'hFE, 1'b0);                  //clears IRR0[0] by writing 0
        do_reset;
        run_until_retire(marker, 8000);
        run_cycles(20);                                   //let the P-phi sampler see idle high
        irq_pin[0] = 1'b0;                                //>= 2 P-phi pulse (P-phi = /4 here)
        run_cycles(16);
        irq_pin[0] = 1'b1;
        run_until_entry_count(1, 8000);
        run_cycles(400);                                  //handler: mailbox, clear, re-read, RTE
        chk("INTEVT2 = 0x600",              dmem[16'h11], 32'h0000_0600);
        chk("IRR0 pend before clear",       dmem[16'h12], 32'h0000_0001);
        chk("IRR0 clear by write-0",        dmem[16'h13], 32'd0);
        chk("single entry after clear",     entry_count, 32'd1);
        //second pulse re-pends and re-enters
        irq_pin[0] = 1'b0;
        run_cycles(16);
        irq_pin[0] = 1'b1;
        run_until_entry_count(2, 8000);
        run_cycles(400);
        chk("second edge re-enters", entry_count, 32'd2);
        end_test;
    end
endtask

task automatic test_irl_mode;
    integer idx, sent, marker;
    begin
        begin_test("IRL mode (reset default): encoded level 13, codes 0x240, IMASK gate");
        idx = 0;
        //ICR1 stays at its reset value 0x4000 (IRQLVL=1) - nothing to program
        emit_sr_imask(idx, 4'hD);                         // IMASK=13 blocks level 13
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[3:0] = 4'b0010;                           //IRL code 2 -> level 13 (table 6.3)
        run_cycles(300);
        chk("IMASK=13 blocks level 13", entry_count, 32'd0);
        //reprogram the mask from the spin loop is impossible; use probes + a second run
        irq_pin[3:0] = 4'b1111;
        clear_imem; clear_dmem;
        idx = 0;
        emit_sr_imask(idx, 4'hC);                         // IMASK=12 accepts level 13
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[3:0] = 4'b0010;
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed: hold through the body
        irq_pin[3:0] = 4'b1111;                           //release in the grace window
        run_cycles(300);
        chk("INTEVT  = 0x240 (IRL level 13)", dmem[16'h10], 32'h0000_0240);
        chk("INTEVT2 = 0x240",                dmem[16'h11], 32'h0000_0240);
        chk("single entry", entry_count, 32'd1);
        end_test;
    end
endtask

task automatic test_irls;
    integer idx, sent, marker;
    begin
        begin_test("IRLS pins: IRLSEN=1, the higher of IRL/IRLS levels wins");
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE050; idx = idx + 1;              // MOV   #0x50,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x5000: IRQLVL|IRLSEN
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        ptf_pin = 8'hFF;                                  //IRLS idle HIGH before IRLSEN turns on
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[3:0] = 4'b0111;                           //IRL  code 7 -> level 8
        ptf_pin[3:0] = 4'b0010;                           //IRLS code 2 -> level 13 (wins)
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed release
        irq_pin[3:0] = 4'b1111;
        ptf_pin[3:0] = 4'b1111;
        run_cycles(300);
        chk("INTEVT2 = 0x240 (IRLS level 13 wins)", dmem[16'h11], 32'h0000_0240);
        ptf_pin = '0;                                     //back to the PINT idle level
        end_test;
    end
endtask

task automatic test_nmi;
    integer idx, sent, marker;
    begin
        begin_test("NMI: falling edge, IMASK-independent, NMIL readback, BL/BLMSK, ack clear");
        //phase A: NMIE=0 (falling), IMASK=15 - NMI is accepted anyway
        idx = 0;
        emit_sr_imask(idx, 4'hF);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        nmi_pin = 1'b1;                                   //pre-charge high for a falling edge
        run_until_retire(marker, 8000);
        nmi_pin = 1'b0;                                   //falling edge -> NMI
        run_until_entry_count(1, 8000);
        run_cycles(300);
        chk("INTEVT  = 0x1C0", dmem[16'h10], 32'h0000_01C0);
        chk("INTEVT2 = 0x1C0", dmem[16'h11], 32'h0000_01C0);
        chk("ack cleared pending: one entry per edge", entry_count, 32'd1);
        //phase B: ICR0.NMIL tracks the pin level
        clear_imem; clear_dmem;
        idx = 0;
        emit_hi_base(idx, 8'hE0);                         // R0 = ICR0
        imem[idx] = 16'h6101; idx = idx + 1;              // MOV.W @R0,R1      ; pin high -> 0x8000
        emit_sentinel_loop(idx, sent);
        do_reset;
        nmi_pin = 1'b1;
        run_until_retire(sent, 8000);
        chk("ICR0.NMIL high", gpr(1), 32'hFFFF_8000);     //sign-extended word read
        //phase C: SR.BL=1 blocks NMI (BLMSK=0)
        clear_imem; clear_dmem;
        idx = 0;
        imem[idx] = 16'hE070; idx = idx + 1;              // MOV   #0x70,R0
        imem[idx] = 16'h4028; idx = idx + 1;              // SHLL16 R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8  R0         ; 0x70000000: BL=1
        imem[idx] = 16'h400E; idx = idx + 1;              // LDC   R0,SR
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        do_reset;
        nmi_pin = 1'b1;
        run_until_retire(marker, 8000);
        nmi_pin = 1'b0;                                   //falling edge while BL=1
        run_cycles(300);
        chk("BL=1 blocks NMI (BLMSK=0)", entry_count, 32'd0);
        //phase D: ICR1.BLMSK=1 - NMI accepted regardless of BL
        clear_imem; clear_dmem;
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE020; idx = idx + 1;              // MOV   #0x20,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x2000: BLMSK
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1 (IRQLVL=0 too)
        imem[idx] = 16'hE070; idx = idx + 1;              // MOV   #0x70,R0
        imem[idx] = 16'h4028; idx = idx + 1;              // SHLL16 R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8  R0
        imem[idx] = 16'h400E; idx = idx + 1;              // LDC   R0,SR       ; BL=1
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        nmi_pin = 1'b1;
        run_until_retire(marker, 8000);
        nmi_pin = 1'b0;
        run_until_entry_count(1, 8000);
        run_cycles(300);
        chk("BLMSK=1 accepts NMI under BL", entry_count, 32'd1);
        chk("INTEVT2 = 0x1C0", dmem[16'h11], 32'h0000_01C0);
        end_test;
    end
endtask

task automatic test_pint;
    integer idx, sent, marker;
    begin
        begin_test("PINT: ICR2 polarity + PINTER enable, group codes 0x700/0x720, IRR0 flags");
        //phase A: PINT3 high-level detect -> code 0x700, IRR0[7]
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE000; idx = idx + 1;              // MOV   #0,R0       ; IRQLVL=0
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1
        imem[idx] = 16'hE008; idx = idx + 1;              // MOV   #8,R0       ; bit3
        imem[idx] = 16'h8139; idx = idx + 1;              // MOV.W R0,@(18,R3) ; ICR2: PINT3 high
        imem[idx] = 16'h813A; idx = idx + 1;              // MOV.W R0,@(20,R3) ; PINTER: PINT3 en
        imem[idx] = 16'hE0C0; idx = idx + 1;              // MOV   #0xC0,R0    ; 0xFFFFFFC0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0xFFFFC000
        imem[idx] = 16'h813C; idx = idx + 1;              // MOV.W R0,@(24,R3) ; IPRD: PINT07 lvl 12
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        pint_pin[3] = 1'b1;                               //high level = request
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed: IRR0 read committed
        pint_pin[3] = 1'b0;
        run_cycles(300);
        chk("INTEVT2 = 0x700 (PINT0-7)", dmem[16'h11], 32'h0000_0700);
        chk("IRR0[7] group flag",        dmem[16'h12], 32'hFFFF_FF80);   //MOV.B sign-extends bit 7
        //phase B: PINT9 -> 0x720, IRR0[6]; PINTER gate checked by pre-masked pin 3
        clear_imem; clear_dmem;
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE000; idx = idx + 1;              // MOV   #0,R0
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1
        imem[idx] = 16'hE002; idx = idx + 1;              // MOV   #2,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x0200: bit9
        imem[idx] = 16'h8139; idx = idx + 1;              // MOV.W R0,@(18,R3) ; ICR2: PINT9 high
        imem[idx] = 16'h813A; idx = idx + 1;              // MOV.W R0,@(20,R3) ; PINTER: PINT9 only
        imem[idx] = 16'hE00C; idx = idx + 1;              // MOV   #0x0C,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0          ; 0x0C00: PINT815 lvl 12
        imem[idx] = 16'h813C; idx = idx + 1;              // MOV.W R0,@(24,R3) ; IPRD
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        pint_pin[3] = 1'b1;                               //enabled? NO (PINTER=bit9 only) - masked
        run_cycles(100);
        chk("PINTER=0 masks PINT3", entry_count, 32'd0);
        ptf_pin[1]  = 1'b1;                               //PINT9 rides the PTF1 pad (table 18.1)
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed release
        pint_pin    = '0;
        ptf_pin     = '0;
        run_cycles(300);
        chk("INTEVT2 = 0x720 (PINT8-15)", dmem[16'h11], 32'h0000_0720);
        chk("IRR0[6] group flag",         dmem[16'h12], 32'h0000_0040);
        end_test;
    end
endtask

task automatic test_priority_tiebreak;
    integer idx, sent, marker;
    begin
        begin_test("Priority: same-level default order (IRQ0<IRQ1, IRQ<ITI), higher level wins");
        //phase A: IRQ0 + IRQ1 pending, same level -> IRQ0 (earlier in table 6.4)
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE00A; idx = idx + 1;              // MOV   #0x0A,R0    ; IRQ0/IRQ1 low level
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1: sense 10,10
        imem[idx] = 16'hE0CC; idx = idx + 1;              // MOV   #0xCC,R0    ; 0xFFFFFFCC
        imem[idx] = 16'h813B; idx = idx + 1;              // MOV.W R0,@(22,R3) ; IPRC: lvl 12/12 (0xFFCC masked to nibbles 0,1)
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[1:0] = 2'b00;                             //both request
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed release
        irq_pin[1:0] = 2'b11;                             //release both in the grace window
        run_cycles(300);
        chk("same level: IRQ0 first (0x600)", dmem[16'h11], 32'h0000_0600);
        chk("one entry", entry_count, 32'd1);
        //phase B: IRQ1 at a higher level wins
        clear_imem; clear_dmem;
        idx = 0;
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE00A; idx = idx + 1;              // MOV   #0x0A,R0
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1
        imem[idx] = 16'hE0DC; idx = idx + 1;              // MOV   #0xDC,R0    ; IRQ1=13, IRQ0=12
        imem[idx] = 16'h813B; idx = idx + 1;              // MOV.W R0,@(22,R3) ; IPRC
        emit_sr_imask(idx, 4'h0);
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b0);
        do_reset;
        run_until_retire(marker, 8000);
        irq_pin[1:0] = 2'b00;
        run_until_entry_count(1, 8000);
        run_until_retire(handler_grace_idx, 8000);        //retire-keyed release
        irq_pin[1:0] = 2'b11;
        run_cycles(300);
        chk("higher level: IRQ1 first (0x620)", dmem[16'h11], 32'h0000_0620);
        //phase C: ITI vs IRQ0 at the same level -> IRQ0 (earlier in table 6.4)
        clear_imem; clear_dmem;
        idx = 0;
        imem[idx] = 16'hE180; idx = idx + 1;              // MOV   #0x80,R1
        imem[idx] = 16'hE001; idx = idx + 1;              // MOV   #1,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'h2101; idx = idx + 1;              // MOV.W R0,@R1      ; FRQCR: P-phi /1
        emit_a4_base(idx, 3);
        imem[idx] = 16'hE002; idx = idx + 1;              // MOV   #2,R0       ; IRQ0 low level
        imem[idx] = 16'h8138; idx = idx + 1;              // MOV.W R0,@(16,R3) ; ICR1
        imem[idx] = 16'hE00C; idx = idx + 1;              // MOV   #0x0C,R0
        imem[idx] = 16'h813B; idx = idx + 1;              // MOV.W R0,@(22,R3) ; IPRC: IRQ0 lvl 12
        emit_hi_base(idx, 8'hE4);                         // R0 = IPRB
        imem[idx] = 16'hE2C0; idx = idx + 1;              // MOV   #0xC0,R2    ; 0xFFFFFFC0
        imem[idx] = 16'h4218; idx = idx + 1;              // SHLL8 R2          ; [15:12]=0xC: WDT lvl 12
        imem[idx] = 16'h2021; idx = idx + 1;              // MOV.W R2,@R0      ; IPRB (0xC000)
        imem[idx] = 16'hE05A; idx = idx + 1;              // MOV   #0x5A,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCBF8; idx = idx + 1;              // OR    #0xF8,R0
        imem[idx] = 16'h8112; idx = idx + 1;              // MOV.W R0,@(4,R1)  ; WTCNT = 0xF8
        imem[idx] = 16'hE0A5; idx = idx + 1;              // MOV   #0xA5,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB80; idx = idx + 1;              // OR    #0x80,R0
        imem[idx] = 16'h8113; idx = idx + 1;              // MOV.W R0,@(6,R1)  ; WDT runs (masked: BL=1)
        marker = idx;
        imem[idx] = 16'hE701; idx = idx + 1;              // MOV   #1,R7       ; both sources pend now
        //32 NOPs: overflow (8 P-phi) happens well inside this window
        idx = idx + 32;
        emit_sr_imask(idx, 4'h0);                         // unmask LAST: both pending -> tiebreak
        emit_sentinel_loop(idx, sent);
        emit_handler(1'b0, 8'h00, 1'b1);                  //stop the WDT inside the handler
        do_reset;
        run_until_retire(marker, 10000);
        irq_pin[0] = 1'b0;                                //IRQ0 requests while SR.BL (reset) still blocks
        run_until_entry_count(1, 10000);
        run_until_retire(handler_grace_idx, 10000);       //retire-keyed release
        irq_pin[0] = 1'b1;
        run_cycles(400);
        chk("tie ITI/IRQ0: IRQ0 first (0x600)", dmem[16'h11], 32'h0000_0600);
        chk("INTEVT = levelcode(12) = 0x260",   dmem[16'h10], 32'h0000_0260);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Tests - External Leg
////

task automatic test_ext_leg_regression;
    integer idx, sent;
    begin
        begin_test("EXT leg: latency knob + interleaved bridge/EXT transactions (owner routing)");
        d_latency = 3;
        idx = 0;
        //dmem store/load with latency
        imem[idx] = 16'hE101; idx = idx + 1;              // MOV   #1,R1
        imem[idx] = 16'h4118; idx = idx + 1;              // SHLL8 R1          ; 0x100 (dmem 0x40)
        imem[idx] = 16'hE277; idx = idx + 1;              // MOV   #0x77,R2
        imem[idx] = 16'h2122; idx = idx + 1;              // MOV.L R2,@R1      ; EXT write
        //bridge write right behind the EXT write
        emit_hi_base(idx, 8'hE2);                         // R0 = IPRA
        imem[idx] = 16'hE355; idx = idx + 1;              // MOV   #0x55,R3
        imem[idx] = 16'h2031; idx = idx + 1;              // MOV.W R3,@R0      ; bridge write
        //EXT load immediately after a bridge access, then a bridge read after an EXT load
        imem[idx] = 16'h6412; idx = idx + 1;              // MOV.L @R1,R4      ; EXT read (latency 3)
        imem[idx] = 16'h6501; idx = idx + 1;              // MOV.W @R0,R5      ; bridge read (IPRA)
        imem[idx] = 16'h6312; idx = idx + 1;              // MOV.L @R1,R3      ; EXT read again
        emit_sentinel_loop(idx, sent);
        do_reset;
        run_until_retire(sent, 10000);
        chk("EXT store/load round trip", gpr(4), 32'h0000_0077);
        chk("bridge read between EXT",   gpr(5), 32'h0000_0055);
        chk("EXT read after bridge",     gpr(3), 32'h0000_0077);
        chk("dmem content",              dmem[8'h40], 32'h0000_0077);  //addr 0x100 -> dmem[0x40]
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Tests - BSC Registers And SDRAM
////

task automatic test_bsc_reg_rw;
    integer idx, sent;
    begin
        begin_test("BSC regs vs section 10.2: POR values, reserved-bit masks, ENDIAN RO, byte write ignored");
        eidx = 0;
        emit_ldrn(8, 32'h0000_0100);                 //mailbox base (MOV.L @(disp,R8))
        //BCR1: POR value, all-ones mask probe (ENDIAN stays 0 = big), DRAMTP write
        emit_ldrn(1, 32'hFFFF_FF60);
        imem[eidx] = 16'h6311; eidx = eidx + 1;              // MOV.W @R1,R3      ; POR = 0x0000
        emit_ldr0(32'h0000_FFFF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; mask probe
        imem[eidx] = 16'h6211; eidx = eidx + 1;              // MOV.W @R1,R2
        imem[eidx] = 16'h1822; eidx = eidx + 1;              // MOV.L R2,@(8,R8)  ; mb2 = 0xF7FF
        emit_ldr0(32'h0000_0008);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; DRAMTP=010
        imem[eidx] = 16'h6411; eidx = eidx + 1;              // MOV.W @R1,R4
        //BCR2 POR value
        emit_ldrn(1, 32'hFFFF_FF62);
        imem[eidx] = 16'h6511; eidx = eidx + 1;              // MOV.W @R1,R5      ; POR = 0x3FF0
        //WCR1: POR value, mask probe, restore
        emit_ldrn(1, 32'hFFFF_FF64);
        imem[eidx] = 16'h6211; eidx = eidx + 1;              // MOV.W @R1,R2
        imem[eidx] = 16'h1823; eidx = eidx + 1;              // MOV.L R2,@(12,R8) ; mb3 = 0x3FF3
        emit_ldr0(32'h0000_FFFF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1824; eidx = eidx + 1;              // MOV.L R2,@(16,R8) ; mb4 = 0xBFF3
        emit_ldr0(32'h0000_3FF3);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // restore POR value
        //WCR2: POR value first, then the CL2 write + byte-write-ignored law
        emit_ldrn(1, 32'hFFFF_FF66);
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1825; eidx = eidx + 1;              // MOV.L R2,@(20,R8) ; mb5 = 0xFFFF
        emit_ldr0(32'h0000_FFDF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; WCR2 = CL2
        imem[eidx] = 16'h6711; eidx = eidx + 1;              // MOV.W @R1,R7
        emit_ldr0(32'h0000_0000);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; byte: ignored
        imem[eidx] = 16'h6211; eidx = eidx + 1;              // MOV.W @R1,R2
        imem[eidx] = 16'h1820; eidx = eidx + 1;              // MOV.L R2,@(0,R8)  ; mb0 = 0xFFDF
        //MCR: POR value, mask probe (RFSH/RMODE kept 0), restore
        emit_ldrn(1, 32'hFFFF_FF68);
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1826; eidx = eidx + 1;              // MOV.L R2,@(24,R8) ; mb6 = 0x0000
        emit_ldr0(32'h0000_FFF9);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1827; eidx = eidx + 1;              // MOV.L R2,@(28,R8) ; mb7 = 0xFFF8
        emit_ldr0(32'h0000_0000);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // restore
        //PCR: POR value, mask probe, restore
        emit_ldrn(1, 32'hFFFF_FF6C);
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1828; eidx = eidx + 1;              // MOV.L R2,@(32,R8) ; mb8 = 0x0000
        emit_ldr0(32'h0000_FFFF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1829; eidx = eidx + 1;              // MOV.L R2,@(36,R8) ; mb9 = 0xCFFF
        emit_ldr0(32'h0000_0000);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        //MCSCR3 mask probe (bits 15-7 reserved)
        emit_ldrn(1, 32'hFFFF_FF56);
        emit_ldr0(32'h0000_FFFF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        imem[eidx] = 16'h1821; eidx = eidx + 1;              // MOV.L R2,@(4,R8)  ; mb1 = 0x007F
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 20000);
        chk("BCR1 POR value (ENDIAN=0: big)",  gpr(3), 32'h0000_0000);
        chk("BCR1 mask (ENDIAN read-only)",    dmem[8'h42], 32'hFFFF_F7FF);
        chk("BCR1 after DRAMTP write",         gpr(4), 32'h0000_0008);
        chk("BCR2 POR value",                  gpr(5), 32'h0000_3FF0);
        chk("WCR1 POR value",                  dmem[8'h43], 32'h0000_3FF3);
        chk("WCR1 mask (14,3,2 reserved)",     dmem[8'h44], 32'hFFFF_BFF3);
        chk("WCR2 POR value",                  dmem[8'h45], 32'hFFFF_FFFF);
        chk("WCR2 after write (sign-ext)",     gpr(7), 32'hFFFF_FFDF);
        chk("WCR2 after byte write (kept)",    dmem[8'h40], 32'hFFFF_FFDF);
        chk("MCR POR value",                   dmem[8'h46], 32'h0000_0000);
        chk("MCR mask (bit 0 reserved)",       dmem[8'h47], 32'hFFFF_FFF8);
        chk("PCR POR value",                   dmem[8'h48], 32'h0000_0000);
        chk("PCR mask (13,12 reserved)",       dmem[8'h49], 32'hFFFF_CFFF);
        chk("MCSCR mask (15-7 reserved)",      dmem[8'h41], 32'h0000_007F);
        end_test;
    end
endtask

task automatic test_refresh_reg_keys;
    integer idx, sent;
    begin
        begin_test("Refresh reg keys: RTCSR/RTCNT/RTCOR need 0xA5, RFCR needs 101001 (fig 10.5)");
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FF72);                 // RTCOR
        emit_ldr0(32'h0000_A512);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // keyed write 0x12
        imem[eidx] = 16'h6311; eidx = eidx + 1;              // MOV.W @R1,R3      ; 0x12
        emit_ldr0(32'h0000_5534);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // wrong key: ignored
        imem[eidx] = 16'h6411; eidx = eidx + 1;              // MOV.W @R1,R4      ; still 0x12
        emit_ldr0(32'h0000_A577);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // byte write: ignored
        imem[eidx] = 16'h6511; eidx = eidx + 1;              // MOV.W @R1,R5      ; still 0x12
        emit_ldrn(1, 32'hFFFF_FF70);                 // RTCNT
        emit_ldr0(32'h0000_A544);
        imem[eidx] = 16'h2101; eidx = eidx + 1;
        imem[eidx] = 16'h6711; eidx = eidx + 1;              // MOV.W @R1,R7      ; 0x44 (CKS=0: static)
        emit_ldrn(1, 32'hFFFF_FF74);                 // RFCR
        emit_ldr0(32'h0000_A455);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // key 101001, data 0x055
        imem[eidx] = 16'h6211; eidx = eidx + 1;              // MOV.W @R1,R2
        emit_ldrn(1, 32'h0000_0100);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb0 = RFCR
        emit_ldrn(1, 32'hFFFF_FF74);
        emit_ldr0(32'h0000_A855);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // wrong key: ignored
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        emit_ldrn(1, 32'h0000_0104);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb1 = RFCR unchanged
        emit_ldrn(1, 32'hFFFF_FF6E);                 // RTCSR
        emit_ldr0(32'h0000_A542);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // CMIE|OVIE
        imem[eidx] = 16'h6211; eidx = eidx + 1;
        emit_ldrn(1, 32'h0000_0108);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb2 = RTCSR
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 20000);
        chk("RTCOR keyed write",       gpr(3), 32'h0000_0012);
        chk("RTCOR wrong key ignored", gpr(4), 32'h0000_0012);
        chk("RTCOR byte ignored",      gpr(5), 32'h0000_0012);
        chk("RTCNT keyed write",       gpr(7), 32'h0000_0044);
        chk("RFCR keyed write",        dmem[8'h40], 32'h0000_0055);
        chk("RFCR wrong key ignored",  dmem[8'h41], 32'h0000_0055);
        chk("RTCSR keyed write",       dmem[8'h42], 32'h0000_0042);
        end_test;
    end
endtask

task automatic test_sdmr_mrs;
    integer idx, sent;
    begin
        begin_test("SDMR: PALL+MRS pin cycle programs the device (CL2/BL1/single-write)");
        eidx = 0;
        //MCR 0x5038: TPC=2cyc, RCD=2cyc, TRWL=1, TRAS=2, AMX=0111
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 20000);
        run_cycles(32);                                   //let the engine finish tMRD
        chk("device Mode_reg = 0x220",  {21'd0, u_sdram.Mode_reg}, 32'h0000_0220);
        chk_true("CKE held high",       cke === 1'b1);
        chk_true("CS3 deselected idle", cs3_n === 1'b1);
        end_test;
    end
endtask

task automatic test_sdram_single_rw;
    integer idx, sent, m0;
    begin
        begin_test("SDRAM single R/W: long/word/byte lanes via DQM, data lands in the device");
        m0 = mirror_cnt;
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_ldrn(1, 32'hAC00_0010);
        emit_ldr0(32'hDEAD_BEEF);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        emit_ldrn(1, 32'hAC00_0014);
        emit_ldr0(32'h0123_4567);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        emit_ldrn(1, 32'hAC00_0011);
        emit_ldr0(32'h0000_0055);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; lane [23:16]
        emit_ldrn(1, 32'hAC00_0016);
        emit_ldr0(32'h0000_ABCD);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; lane [15:0]
        emit_ldrn(1, 32'hAC00_0010);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // MOV.L @R1,R3      ; 0xDE55BEEF
        emit_ldrn(1, 32'hAC00_0014);
        imem[eidx] = 16'h6412; eidx = eidx + 1;              // MOV.L @R1,R4      ; 0x0123ABCD
        emit_ldrn(1, 32'hAC00_0011);
        imem[eidx] = 16'h6510; eidx = eidx + 1;              // MOV.B @R1,R5      ; 0x55
        emit_ldrn(1, 32'hAC00_0016);
        imem[eidx] = 16'h6711; eidx = eidx + 1;              // MOV.W @R1,R7      ; 0xFFFFABCD
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 40000);
        chk("long readback + byte lane", gpr(3), 32'hDE55_BEEF);
        chk("long readback + word lane", gpr(4), 32'h0123_ABCD);
        chk("byte read (DQM lane)",      gpr(5), 32'h0000_0055);
        chk("word read (sign-extended)", gpr(7), 32'hFFFF_ABCD);
        chk("device word 0 content",     sdram_peek(32'h0C00_0010), 32'hDE55_BEEF);
        chk("device word 1 content",     sdram_peek(32'h0C00_0014), 32'h0123_ABCD);
        chk("mirror strobes on the generic port (4W+4R, CS3)", mirror_cnt - m0, 32'd8);
        end_test;
    end
endtask

task automatic test_sdram_fill_drain;
    integer idx, sent;
    begin
        begin_test("SDRAM cached: fill burst, dirty write-back drain burst, refill integrity");
        //five same-set lines (index 0x10) - the 4-way set forces an LRU eviction
        sdram_poke(32'h0C00_0100, 32'h1111_1111);
        sdram_poke(32'h0C00_0104, 32'h2222_2222);
        sdram_poke(32'h0C00_0108, 32'h3333_3333);
        sdram_poke(32'h0C00_010C, 32'h4444_4444);
        sdram_poke(32'h0C00_1100, 32'hB0B0_0001);
        sdram_poke(32'h0C00_2100, 32'hB0B0_0002);
        sdram_poke(32'h0C00_3100, 32'hB0B0_0003);
        sdram_poke(32'h0C00_4100, 32'hB0B0_0004);
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        imem[eidx] = 16'hE0EC; eidx = eidx + 1;              // MOV   #0xEC,R0    ; CCR
        imem[eidx] = 16'hE10F; eidx = eidx + 1;              // MOV   #0x0F,R1    ; CE|WT(P0 thru)|CB(P1 WB)|CF
        imem[eidx] = 16'h2012; eidx = eidx + 1;              // MOV.L R1,@R0
        emit_ldrn(1, 32'h8C00_0100);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // MOV.L @R1,R3      ; fill A
        emit_ldr0(32'hA5A5_A5A5);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1      ; dirty A
        emit_ldrn(1, 32'h8C00_1100);
        imem[eidx] = 16'h6412; eidx = eidx + 1;              // MOV.L @R1,R4      ; fill B
        emit_ldrn(1, 32'h8C00_2100);
        imem[eidx] = 16'h6512; eidx = eidx + 1;              // MOV.L @R1,R5      ; fill C
        emit_ldrn(1, 32'h8C00_3100);
        imem[eidx] = 16'h6712; eidx = eidx + 1;              // MOV.L @R1,R7      ; fill D
        emit_ldrn(1, 32'h8C00_4100);
        imem[eidx] = 16'h6212; eidx = eidx + 1;              // MOV.L @R1,R2      ; fill E: evict+drain A
        emit_ldrn(1, 32'hAC00_0100);
        imem[eidx] = 16'h6212; eidx = eidx + 1;              // MOV.L @R1,R2      ; uncached: drained?
        emit_ldrn(1, 32'h0000_0100);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb0
        emit_ldrn(1, 32'h8C00_0100);
        imem[eidx] = 16'h6212; eidx = eidx + 1;              // MOV.L @R1,R2      ; cached refill of A
        emit_ldrn(1, 32'h0000_0104);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb1
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 60000);
        chk("fill A word 0",             gpr(3), 32'h1111_1111);
        chk("fill B word 0",             gpr(4), 32'hB0B0_0001);
        chk("fill C word 0",             gpr(5), 32'hB0B0_0002);
        chk("fill D word 0",             gpr(7), 32'hB0B0_0003);
        chk("uncached read after drain", dmem[8'h40], 32'hA5A5_A5A5);
        chk("cached refill of A",        dmem[8'h41], 32'hA5A5_A5A5);
        chk("drained line word 0",       sdram_peek(32'h0C00_0100), 32'hA5A5_A5A5);
        chk("drained line word 1",       sdram_peek(32'h0C00_0104), 32'h2222_2222);
        chk("drained line word 3",       sdram_peek(32'h0C00_010C), 32'h4444_4444);
        end_test;
    end
endtask

task automatic test_sdram_refresh;
    integer idx, sent;
    begin
        begin_test("Auto-refresh: RTCNT/RTCOR pacing, RFCR counts, CMF sets, data retained");
        sdram_poke(32'h0C00_0020, 32'h600D_BEEF);
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);    //RFSH=1
        emit_wreg_w(32'hFFFF_FF72, 16'hA504);        // RTCOR = 4
        emit_wreg_w(32'hFFFF_FF6E, 16'hA508);        // RTCSR: CKS=001 (bus/4)
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;              // MOV   R0,R2       ; delay counter
        imem[eidx] = 16'h4210; eidx = eidx + 1;              // DT    R2
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF    back to the DT
        emit_ldrn(1, 32'hFFFF_FF74);
        imem[eidx] = 16'h6311; eidx = eidx + 1;              // MOV.W @R1,R3      ; RFCR
        emit_ldrn(1, 32'hFFFF_FF6E);
        imem[eidx] = 16'h6411; eidx = eidx + 1;              // MOV.W @R1,R4      ; RTCSR (CMF?)
        emit_ldrn(1, 32'hAC00_0020);
        imem[eidx] = 16'h6512; eidx = eidx + 1;              // MOV.L @R1,R5      ; retained data
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 40000);
        chk_true("RFCR advanced (refreshes ran)", gpr(3) >= 32'd10 && gpr(3) <= 32'd500);
        chk_true("RTCSR.CMF set",                 gpr(4)[7] === 1'b1);
        chk("data retained across refreshes",     gpr(5), 32'h600D_BEEF);
        end_test;
    end
endtask

//shared handler for the two refresh-interrupt tests: mailbox INTEVT/INTEVT2,
//then a keyed RTCSR<=0 write (stops CKS, clears CMF/OVF -> level drops)
task automatic emit_ref_handler;
    integer idx, k;
    begin
        eidx = 'h300;
        imem[eidx] = 16'hE0D8; eidx = eidx + 1;              // MOV   #0xD8,R0    ; INTEVT
        imem[eidx] = 16'h6102; eidx = eidx + 1;              // MOV.L @R0,R1
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2
        imem[eidx] = 16'h2212; eidx = eidx + 1;              // MOV.L R1,@R2      ; mb0 = INTEVT
        emit_a4_base(eidx, 3);                             // R3 = 0xA4000000
        imem[eidx] = 16'h6432; eidx = eidx + 1;              // MOV.L @R3,R4      ; INTEVT2
        imem[eidx] = 16'h1241; eidx = eidx + 1;              // MOV.L R4,@(4,R2)  ; mb1
        emit_ldrn(1, 32'hFFFF_FF6E);
        emit_ldr0(32'h0000_A500);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // RTCSR<=0: stop + clear flags
        for(k = 0; k < 8; k = k + 1) begin
            imem[eidx] = 16'h0009; eidx = eidx + 1;          // grace NOPs
        end
        imem[eidx] = 16'h002B; eidx = eidx + 1;              // RTE
        imem[eidx] = 16'h0009; eidx = eidx + 1;              // NOP (delay slot)
    end
endtask

task automatic test_sdram_rcmi;
    integer idx, sent;
    begin
        begin_test("RCMI e2e: compare-match refresh interrupt, INTEVT=INTEVT2=0x580");
        emit_ref_handler;
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);
        emit_hi_base(eidx, 8'hE4);                         // R0 = IPRB
        imem[eidx] = 16'hE10F; eidx = eidx + 1;              // MOV   #0x0F,R1
        imem[eidx] = 16'h4118; eidx = eidx + 1;              // SHLL8 R1          ; 0x0F00 = REF level 15
        imem[eidx] = 16'h2011; eidx = eidx + 1;              // MOV.W R1,@R0
        emit_sr_imask(eidx, 4'h0);
        emit_wreg_w(32'hFFFF_FF72, 16'hA504);        // RTCOR = 4
        emit_wreg_w(32'hFFFF_FF6E, 16'hA548);        // RTCSR: CMIE | CKS=001
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_entry_count(1, 40000);
        chk("INTEVT = 0x580 (RCMI)",  intevt_o,    32'h0000_0580);
        run_cycles(400);
        chk("mb0 INTEVT via handler", dmem[8'h10], 32'h0000_0580);
        chk("mb1 INTEVT2 = 0x580",    dmem[8'h11], 32'h0000_0580);
        run_cycles(400);
        chk_true("no re-entry after keyed clear", entry_count === 1);
        end_test;
    end
endtask

task automatic test_sdram_rovi;
    integer idx, sent;
    begin
        begin_test("ROVI e2e: RFCR overflow past LMTS=512, INTEVT=INTEVT2=0x5A0");
        emit_ref_handler;
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);
        emit_hi_base(eidx, 8'hE4);                         // R0 = IPRB
        imem[eidx] = 16'hE10F; eidx = eidx + 1;              // MOV   #0x0F,R1
        imem[eidx] = 16'h4118; eidx = eidx + 1;              // SHLL8 R1
        imem[eidx] = 16'h2011; eidx = eidx + 1;              // MOV.W R1,@R0
        emit_sr_imask(eidx, 4'h0);
        emit_wreg_w(32'hFFFF_FF74, 16'hA5FE);        // RFCR preload = 510
        emit_wreg_w(32'hFFFF_FF72, 16'hA504);        // RTCOR = 4
        emit_wreg_w(32'hFFFF_FF6E, 16'hA50B);        // RTCSR: OVIE | CKS=001 | LMTS
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_entry_count(1, 40000);
        chk("INTEVT = 0x5A0 (ROVI)",  intevt_o,    32'h0000_05A0);
        run_cycles(400);
        chk("mb0 INTEVT via handler", dmem[8'h10], 32'h0000_05A0);
        chk("mb1 INTEVT2 = 0x5A0",    dmem[8'h11], 32'h0000_05A0);
        end_test;
    end
endtask

task automatic test_sdram_selfrefresh;
    integer idx, sent;
    begin
        begin_test("Self-refresh: CKE low entry (RMODE&RFSH), exit on RMODE clear, data retained");
        sdram_poke(32'h0C00_0030, 32'h600D_5E1F);
        cke_mon_clr = 1'b1;
        run_cycles(2);
        cke_mon_clr = 1'b0;
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_wreg_w(32'hFFFF_FF68, 16'h503E);        // MCR: RFSH|RMODE -> enter self
        emit_ldr0(32'd40);
        imem[eidx] = 16'h6203; eidx = eidx + 1;              // MOV   R0,R2
        imem[eidx] = 16'h4210; eidx = eidx + 1;              // DT    R2
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF    back to the DT (hold in self)
        emit_wreg_w(32'hFFFF_FF68, 16'h5038);        // RMODE=0 -> exit
        emit_ldrn(1, 32'hAC00_0030);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // MOV.L @R1,R3      ; read after exit
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 40000);
        chk_true("CKE went low (self-refresh entered)", cke_low_seen === 1'b1);
        chk_true("CKE high again after exit",           cke === 1'b1);
        chk("data retained through self-refresh",       gpr(3), 32'h600D_5E1F);
        end_test;
    end
endtask

task automatic test_sdram_rst_survival;
    integer idx, sent;
    integer rfcr_pre;
    begin
        begin_test("Manual reset: BSC regs + SDRAM data retained, refresh keeps running (p.297)");
        eidx = 0;
        //phase A: init + refresh on + marker store, then arm a watchdog manual reset
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);
        emit_wreg_w(32'hFFFF_FF72, 16'hA504);        // RTCOR = 4
        emit_wreg_w(32'hFFFF_FF6E, 16'hA508);        // RTCSR: CKS=001
        emit_ldrn(1, 32'hAC00_0040);
        emit_ldr0(32'hCAFE_BABE);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1      ; marker into SDRAM
        imem[eidx] = 16'hE180; eidx = eidx + 1;              // MOV   #0x80,R1    ; 0xFFFFFF80 (WDT)
        imem[eidx] = 16'hE05A; eidx = eidx + 1;              // MOV   #0x5A,R0
        imem[eidx] = 16'h4018; eidx = eidx + 1;              // SHLL8 R0
        imem[eidx] = 16'hCBFC; eidx = eidx + 1;              // OR    #0xFC,R0    ; 0x5AFC
        imem[eidx] = 16'h8112; eidx = eidx + 1;              // MOV.W R0,@(4,R1)  ; WTCNT = 0xFC
        imem[eidx] = 16'hE0A5; eidx = eidx + 1;              // MOV   #0xA5,R0
        imem[eidx] = 16'h4018; eidx = eidx + 1;              // SHLL8 R0
        imem[eidx] = 16'hCBE0; eidx = eidx + 1;              // OR    #0xE0,R0    ; TME|WT/IT|RSTS
        imem[eidx] = 16'h8113; eidx = eidx + 1;              // MOV.W R0,@(6,R1)  ; watchdog armed
        do_reset;
        run_until_wdt_reset(40000);
        chk_true("watchdog manual reset fired", wdt_rst_hit === 1'b1);
        rfcr_pre = u_dut.u_bsc.rfcr;
        //phase B: fresh program after the reset - no re-init, straight readback
        clear_imem;
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FF60);
        imem[eidx] = 16'h6311; eidx = eidx + 1;              // MOV.W @R1,R3      ; BCR1 retained
        emit_ldrn(1, 32'hFFFF_FF68);
        imem[eidx] = 16'h6411; eidx = eidx + 1;              // MOV.W @R1,R4      ; MCR retained
        emit_ldrn(1, 32'hFFFF_FF74);
        imem[eidx] = 16'h6511; eidx = eidx + 1;              // MOV.W @R1,R5      ; RFCR still counting
        emit_ldrn(1, 32'hAC00_0040);
        imem[eidx] = 16'h6712; eidx = eidx + 1;              // MOV.L @R1,R7      ; marker readback
        emit_sentinel_loop(eidx, sent);
        run_cycles(40);
        run_until_retire(sent, 40000);
        chk("EXPEVT after manual reset",   expevt_o, 32'h0000_0020);
        chk("BCR1 retained",               gpr(3), 32'h0000_0008);
        chk("MCR retained",                gpr(4), 32'h0000_503C);
        chk_true("refresh continued through reset", gpr(5) > rfcr_pre);
        chk("SDRAM marker survived",       gpr(7), 32'hCAFE_BABE);
        end_test;
    end
endtask


task automatic test_sdram_bank_active;
    integer idx, sent;
    begin
        begin_test("Bank active (RASD=1): row-hit fast path, row conflict PRE, per-bank rows, CL1 Tnop");
        sdram_poke(32'h0C00_1010, 32'hC0DE_0004);         //row 4, bank 0
        sdram_poke(32'h0C20_0010, 32'hB1B1_B1B1);         //row 0, bank 1
        eidx = 0;
        emit_sdram_init(16'h50B8, 16'hFFDF, 32'hFFFF_E880);    //RASD=1, CL2
        emit_ldrn(1, 32'hAC00_0010);
        emit_ldr0(32'hA110_0001);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // W A: ACTV row0 + WRIT (no AP)
        emit_ldrn(1, 32'hAC00_0014);
        emit_ldr0(32'hB220_0002);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // W B: row hit, WRIT only
        emit_ldrn(1, 32'hAC00_0010);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // R A: row hit, READ only
        emit_ldrn(1, 32'hAC00_1010);
        imem[eidx] = 16'h6412; eidx = eidx + 1;              // R C: row 4 conflict -> PRE+ACTV+READ
        emit_ldrn(1, 32'hAC20_0010);
        imem[eidx] = 16'h6512; eidx = eidx + 1;              // R D: bank 1 ACTV (bank 0 stays open)
        emit_ldrn(1, 32'hAC00_0010);
        imem[eidx] = 16'h6712; eidx = eidx + 1;              // R A: bank0 row0 again (conflict path)
        //CL1 phase: WCR2 A3W=01 + MRS the device to CL1, then a row-hit read (Tnop)
        emit_wreg_w(32'hFFFF_FF66, 16'hFFBF);        // WCR2: A3W=01 -> CL1
        emit_ldrn(1, 32'hFFFF_E840);                 // SDMR: CL1
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1
        emit_ldrn(1, 32'hAC00_0014);
        imem[eidx] = 16'h6212; eidx = eidx + 1;              // R B at CL1 (row hit + Tnop)
        emit_ldrn(1, 32'h0000_0100);
        imem[eidx] = 16'h2122; eidx = eidx + 1;              // mb0 = R2
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 60000);
        chk("row-hit read of A",        gpr(3), 32'hA110_0001);
        chk("row-conflict read (row4)", gpr(4), 32'hC0DE_0004);
        chk("bank-1 read",              gpr(5), 32'hB1B1_B1B1);
        chk("reopened row-0 read of A", gpr(7), 32'hA110_0001);
        chk("CL1 row-hit read of B",    dmem[8'h40], 32'hB220_0002);
        end_test;
    end
endtask

task automatic test_sdram_latency;
    integer idx, sent, j;
    begin
        begin_test("Natural latency: cycle counts for auto-precharge vs bank-active reads");
        sdram_poke(32'h0C00_0050, 32'h1A7E_2C00);
        //phase 1: auto-precharge, CL2/RCD2/TPC2 - 16 identical uncached loads
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_ldrn(1, 32'hAC00_0050);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // warm-up load (aligns the engine)
        for(j = 0; j < 16; j = j + 1) begin
            imem[eidx] = 16'h6312; eidx = eidx + 1;          // MOV.L @R1,R3 x16
        end
        emit_sentinel_loop(eidx, sent);
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(sent, 60000);
        bench_arm = 1'b0;
        @(posedge clk);
        $display("      [LAT] auto-precharge: %0d retires / %0d cycles", bench_retires, bench_arch_cycles);
        chk("read data (AP phase)", gpr(3), 32'h1A7E_2C00);
        chk("AP 16-load cycles", bench_arch_cycles, 32'd455);  //relocked 2026-07-07 (BSC Group A)
        //phase 2: bank-active, same row - 16 row-hit loads
        eidx = 0;
        emit_sdram_init(16'h50B8, 16'hFFDF, 32'hFFFF_E880);
        emit_ldrn(1, 32'hAC00_0050);
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // warm-up (opens the row)
        for(j = 0; j < 16; j = j + 1) begin
            imem[eidx] = 16'h6312; eidx = eidx + 1;
        end
        emit_sentinel_loop(eidx, sent);
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(sent, 60000);
        bench_arm = 1'b0;
        @(posedge clk);
        $display("      [LAT] bank-active row-hit: %0d retires / %0d cycles", bench_retires, bench_arch_cycles);
        chk("read data (BA phase)", gpr(3), 32'h1A7E_2C00);
        chk("BA 16-load cycles", bench_arch_cycles, 32'd423);  //relocked 2026-07-07 (BSC Group A)
        end_test;
    end
endtask

task automatic bench_ipc_sdram;
    integer idx, sent, j, hidx, loop_h, bf_h, disp;
    begin
        begin_test("IPC from SDRAM: uncached straightline + cached add-loop baselines");
        //uncached phase: 64 NOPs + sentinel at phys 0x800 (P2 0xAC000800)
        eidx = 'h400;                                     //halfword index of phys 0x800
        for(j = 0; j < 64; j = j + 1) emit_sd(16'h0009);
        emit_sd(16'hE65A);                          // MOV #0x5A,R6 (sentinel)
        emit_sd(16'hAFFE);                          // BRA self
        emit_sd(16'h0009);                          // NOP (delay slot)
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_ldrn(2, 32'hAC00_0800);
        imem[eidx] = 16'h422B; eidx = eidx + 1;              // JMP @R2
        imem[eidx] = 16'h0009; eidx = eidx + 1;              // (delay slot)
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(('h800 + 64*2) >> 1, 120000);    //sentinel widx = pc[11:1]
        bench_arm = 1'b0;
        @(posedge clk);
        if(bench_arch_cycles > 0)
            $display("  [BENCH] SDRAM uncached straightline: %0d retires / %0d cycles -> IPC = %0d.%03d",
                     bench_retires, bench_arch_cycles,
                     ((bench_retires * 1000) / bench_arch_cycles) / 1000,
                     ((bench_retires * 1000) / bench_arch_cycles) % 1000);
        chk("SDRAM uncached retires (incl. boot)",     bench_retires,     32'd131);
        chk("SDRAM uncached arch-cycles (incl. boot)", bench_arch_cycles, 32'd655);  //relocked 2026-07-07 (BSC Group A)
        //cached phase: add-loop (body 100, iters 12) at P1 0x8C000800, CCR on
        eidx = 'h400;
        emit_sd(16'hE50C);                          // MOV #12,R5
        loop_h = eidx;
        emit_sd(16'hE300);                          // MOV #0,R3
        for(j = 0; j < 100; j = j + 1) emit_sd(16'h7301);   // ADD #1,R3
        emit_sd(16'h4510);                          // DT R5
        bf_h = eidx;
        disp = loop_h - bf_h - 2;
        emit_sd(16'h8B00 | (disp & 8'hFF));         // BF loop
        emit_sd(16'hE65A);                          // MOV #0x5A,R6 (sentinel)
        emit_sd(16'hAFFE);                          // BRA self
        emit_sd(16'h0009);                          // NOP
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        imem[eidx] = 16'hE0EC; eidx = eidx + 1;              // MOV   #0xEC,R0    ; CCR
        imem[eidx] = 16'hE10F; eidx = eidx + 1;              // MOV   #0x0F,R1    ; CE|WT|CB|CF
        imem[eidx] = 16'h2012; eidx = eidx + 1;              // MOV.L R1,@R0
        emit_ldrn(2, 32'h8C00_0800);
        imem[eidx] = 16'h422B; eidx = eidx + 1;              // JMP @R2 (cached SDRAM code)
        imem[eidx] = 16'h0009; eidx = eidx + 1;
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(('h800 + (2 + 100 + 2)*2) >> 1, 120000);
        bench_arm = 1'b0;
        @(posedge clk);
        if(bench_arch_cycles > 0)
            $display("  [BENCH] SDRAM cached add loop: %0d retires / %0d cycles -> IPC = %0d.%03d",
                     bench_retires, bench_arch_cycles,
                     ((bench_retires * 1000) / bench_arch_cycles) / 1000,
                     ((bench_retires * 1000) / bench_arch_cycles) % 1000);
        chk("SDRAM cached loop R3", gpr(3), 32'd100);
        chk("SDRAM cached retires (incl. boot)",     bench_retires,     32'd1311); //relocked 2026-07-05 (fetch-leak fix)
        chk("SDRAM cached arch-cycles (incl. boot)", bench_arch_cycles, 32'd1969);  //relocked 2026-07-07 (Group A + fill-forward)
        end_test;
    end
endtask


//shared program for the ordinary-bus WAIT phases: store + 4 loads, benched
task automatic ord_wait_bench(input logic [15:0] wcr2_v, input integer stretch,
                              output integer cycles,
                              input logic [15:0] wcr1_v = 16'h3FF3);
    integer sent;
    begin
        init_knobs;
        clear_imem;
        clear_dmem;
        raw_mode     = 1;
        wait_stretch = stretch;
        eidx = 0;
        emit_wreg_w(32'hFFFF_FF66, wcr2_v);               //program area-0 timing
        emit_wreg_w(32'hFFFF_FF64, wcr1_v);               //WAITSEL + inter-access idles
        emit_ldrn(1, 32'h0000_0180);
        emit_ldr0(32'h5EED_0001);
        imem[eidx] = 16'h2102; eidx = eidx + 1;           // MOV.L R0,@R1 (raw write)
        imem[eidx] = 16'h6312; eidx = eidx + 1;           // MOV.L @R1,R3
        imem[eidx] = 16'h6212; eidx = eidx + 1;           // MOV.L @R1,R2 x3
        imem[eidx] = 16'h6212; eidx = eidx + 1;
        imem[eidx] = 16'h6212; eidx = eidx + 1;
        emit_sentinel_loop(eidx, sent);
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(sent, 120000);
        bench_arm = 1'b0;
        @(posedge clk);
        cycles = bench_arch_cycles;
        chk("raw write/read integrity", gpr(3), 32'h5EED_0001);
    end
endtask

task automatic test_ordinary_wait;
    integer c_1, c_2, c_3, c_4, c_5, c_6;
    begin
        begin_test("Ordinary bus: WCR2 waits, i_WAIT_n + WAITSEL, 0-wait pin ignore, WCR1 idles");
        ord_wait_bench(16'hFFF9, 0, c_1);                 //A0W=001: 1 wait, pin sampled
        ord_wait_bench(16'hFFF9, 6, c_2);                 //i_WAIT_n held low 6 bus cycles
        ord_wait_bench(16'hFFF8, 6, c_3);                 //A0W=000: 0 waits, pin IGNORED
        ord_wait_bench(16'hFFF8, 0, c_4);
        ord_wait_bench(16'hFFF9, 6, c_5, 16'hBFF3);       //WAITSEL=1: mid-state sample (fig 10.11)
        ord_wait_bench(16'hFFF9, 0, c_6, 16'h8000);       //AnIW=00 everywhere: 1-idle minimum
        $display("      [ORD] 1w/ws0: %0d   1w/ws6: %0d   0w/ws6: %0d   0w/ws0: %0d   wsel/ws6: %0d   1-idle: %0d",
                 c_1, c_2, c_3, c_4, c_5, c_6);
        chk("1-wait baseline cycles",  c_1, 32'd464);  //relocked 2026-07-07 (BSC Group A)
        chk("WAIT-stretched cycles",   c_2, 32'd684);
        chk("0-wait cycles",           c_4, 32'd418);
        chk_true("i_WAIT_n stretched the bus",    c_2 > c_1);
        chk_true("0-wait area ignores i_WAIT_n",  c_3 === c_4);
        chk_true("WAITSEL=1 stretches too",       c_5 > c_1);
        chk_true("1-idle WCR1 is not slower",     c_6 <= c_1);
        end_test;
    end
endtask

task automatic test_burst_rom;
    integer sent, j, c_nb, c_bst, f_nb, f_bst, loop_e, bf_e, disp, ph;
    begin
        begin_test("Burst ROM: BCR1-enabled line fills use the WCR2 burst pitch (p.304)");
        for(ph = 0; ph < 2; ph = ph + 1) begin
            init_knobs;
            clear_imem;
            clear_dmem;
            raw_mode = 1;
            eidx = 0;
            emit_wreg_w(32'hFFFF_FF66, 16'hFFFC);         //A0W=100: first=4w, pitch=4 states
            emit_wreg_w(32'hFFFF_FF60, (ph == 1) ? 16'h0200 : 16'h0000);  //A0BST on/off
            imem[eidx] = 16'hE0EC; eidx = eidx + 1;       // MOV   #0xEC,R0    ; CCR
            imem[eidx] = 16'hE109; eidx = eidx + 1;       // MOV   #9,R1       ; CE|CF
            imem[eidx] = 16'h2012; eidx = eidx + 1;       // MOV.L R1,@R0
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            emit_ldrn(2, 32'h0000_0400);
            imem[eidx] = 16'h422B; eidx = eidx + 1;       // JMP @R2 (P0 cached, line fills)
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            eidx = 'h200;                                 //cached code at address 0x400
            imem[eidx] = 16'hE504; eidx = eidx + 1;       // MOV #4,R5
            loop_e = eidx;
            imem[eidx] = 16'hE300; eidx = eidx + 1;       // MOV #0,R3
            for(j = 0; j < 16; j = j + 1) begin
                imem[eidx] = 16'h7301; eidx = eidx + 1;   // ADD #1,R3
            end
            imem[eidx] = 16'h4510; eidx = eidx + 1;       // DT R5
            bf_e = eidx;
            disp = loop_e - bf_e - 2;
            imem[eidx] = 16'h8B00 | (disp & 8'hFF); eidx = eidx + 1;
            imem[eidx] = 16'hE65A; eidx = eidx + 1;       // sentinel
            sent = eidx - 1;
            imem[eidx] = 16'hAFFE; eidx = eidx + 1;       // BRA self
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            do_reset;
            bench_arm = 1'b1;
            run_until_retire(sent, 120000);
            bench_arm = 1'b0;
            @(posedge clk);
            if(ph == 0) begin c_nb  = bench_arch_cycles; f_nb  = bench_cs0f; end
            else        begin c_bst = bench_arch_cycles; f_bst = bench_cs0f; end
            chk("cached loop result", gpr(3), 32'd16);
            chk("loop count consumed", gpr(5), 32'd0);
        end
        $display("      [ROM] no-burst: %0d (%0d CS falls)   burst pitch: %0d (%0d CS falls)",
                 c_nb, f_nb, c_bst, f_bst);
        chk("no-burst fill cycles", c_nb,  32'd987);  //relocked 2026-07-09 (burst envelope:
        chk("burst-ROM fill cycles", c_bst, 32'd951); //beats chain w/ no idle state, fig 10.30 -
                                                      //9 beat transitions x 2 cycles saved)
        chk_true("burst pitch is faster", c_bst < c_nb);
        //envelope law (p.304/fig 23.19): a burst-ROM line fill asserts CS0 ONCE
        //("CS0 is not negated"); a plain-area fill re-frames all 4 beats
        chk_true("burst-ROM fills hold CS0 low", f_bst * 4 == f_nb);
        end_test;
    end
endtask


task automatic test_width16;
    integer sent;
    begin
        begin_test("16-bit port (BCR2): longword = two D15-D0 bus cycles, lanes/WE per half");
        init_knobs;
        raw_mode = 1;
        eidx = 0;
        emit_wreg_w(32'hFFFF_FF62, 16'h3EF0);             // BCR2: A4SZ=10 (area 4 = 16-bit)
        emit_wreg_w(32'hFFFF_FF66, 16'hFCFF);             // WCR2: A4W=001 (1 wait, pin on)
        emit_ldrn(1, 32'hB000_0200);
        emit_ldr0(32'hCAFE_F00D);
        imem[eidx] = 16'h2102; eidx = eidx + 1;           // MOV.L R0,@R1 (split write)
        imem[eidx] = 16'h6312; eidx = eidx + 1;           // MOV.L @R1,R3 (split read)
        imem[eidx] = 16'h6411; eidx = eidx + 1;           // MOV.W @R1,R4 (MS half)
        emit_ldrn(1, 32'hB000_0202);
        imem[eidx] = 16'h6511; eidx = eidx + 1;           // MOV.W @R1,R5 (LS half)
        emit_ldrn(1, 32'hB000_0201);
        imem[eidx] = 16'h6710; eidx = eidx + 1;           // MOV.B @R1,R7 (odd byte, MS half)
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 60000);
        chk("split long write/read",   gpr(3), 32'hCAFE_F00D);
        chk("MS word (sign-extended)", gpr(4), 32'hFFFF_CAFE);
        chk("LS word (sign-extended)", gpr(5), 32'hFFFF_F00D);
        chk("odd byte of MS half",     gpr(7), 32'hFFFF_FFFE);
        chk("device content",          dmem[8'h80], 32'hCAFE_F00D);
        end_test;
    end
endtask

task automatic test_bus_release;
    integer sent, marker;
    begin
        begin_test("BREQ/BACK: bus drained + banks PALLed, o_BUS_OE released, clean resume");
        sdram_poke(32'h0C00_0060, 32'h0DDB_A115);
        eidx = 0;
        emit_sdram_init(16'h50B8, 16'hFFDF, 32'hFFFF_E880);   //bank-active mode
        emit_ldrn(1, 32'hAC00_0060);
        imem[eidx] = 16'h6312; eidx = eidx + 1;           // MOV.L @R1,R3 (opens bank 0)
        marker = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;           // MOV #1,R7 (BREQ window marker)
        emit_ldr0(32'd60);
        imem[eidx] = 16'h6203; eidx = eidx + 1;           // MOV  R0,R2
        imem[eidx] = 16'h4210; eidx = eidx + 1;           // DT   R2
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;           // BF   back to the DT
        emit_ldrn(1, 32'hAC00_0060);
        imem[eidx] = 16'h6412; eidx = eidx + 1;           // MOV.L @R1,R4 (re-ACTV after PALL)
        emit_ldrn(1, 32'h0000_0100);
        imem[eidx] = 16'h2142; eidx = eidx + 1;           // MOV.L R4,@R1 -> mb0
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(marker, 60000);
        breq_n = 1'b0;
        run_cycles(60);
        chk_true("BACK asserted (bus granted)", back_n === 1'b0);
        chk_true("shared pads released",        bus_oe === 1'b0);
        breq_n = 1'b1;
        run_cycles(20);
        chk_true("BACK negated on release",     back_n === 1'b1);
        chk_true("pads re-driven",              bus_oe === 1'b1);
        run_until_retire(sent, 60000);
        chk("read before release",  gpr(3), 32'h0DDB_A115);
        chk("read after regrant",   gpr(4), 32'h0DDB_A115);
        chk("mailbox round trip",   dmem[8'h40], 32'h0DDB_A115);
        end_test;
    end
endtask



task automatic test_tas_atomic_breq;
    integer sent, c;
    begin
        begin_test("TAS atomicity: BREQ inside the locked pair defers until the write completes");
        //phase A: TAS.B on SDRAM - the engine's locked pair (e_lock_hold)
        sdram_poke(32'h0C00_0070, 32'h00FF_00FF);    //byte@0x70 = 0x00 -> T=1
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
        emit_ldrn(1, 32'hAC00_0070);
        imem[eidx] = 16'h401B; eidx = eidx + 1;      // TAS.B @R1 (locked RMW pair)
        imem[eidx] = 16'h0329; eidx = eidx + 1;      // MOVT  R3
        imem[eidx] = 16'h6412; eidx = eidx + 1;      // MOV.L @R1,R4 (readback)
        emit_sentinel_loop(eidx, sent);
        do_reset;
        tas_mon_clr = 1'b1; @(posedge clk); tas_mon_clr = 1'b0;
        c = 0;                                       //BREQ lands inside the pair:
        while(u_dut.u_bsc.e_lock_hold !== 1'b1 && c < 60000) begin @(posedge clk); c = c + 1; end
        chk_true("locked read dispatched (probe)", u_dut.u_bsc.e_lock_hold === 1'b1);
        breq_n = 1'b0;
        c = 0;
        while(back_n !== 1'b0 && c < 60000) begin @(posedge clk); c = c + 1; end
        run_cycles(2);                              //let the sticky monitor settle
        chk_true("bus granted after the pair (SDRAM)",     back_n   === 1'b0);
        chk_true("no grant inside the locked pair (SDRAM)", tas_viol === 1'b0);
        breq_n = 1'b1;
        run_until_retire(sent, 120000);
        run_cycles(200);
        chk("T set (semaphore was clear)", gpr(3), 32'd1);
        chk("TAS readback has bit7",       gpr(4), 32'h80FF_00FF);
        chk("device byte set", sdram_peek(32'h0C00_0070), 32'h80FF_00FF);
        //phase B: TAS.B on an ordinary/handshake area - the front-end pair
        init_knobs;
        clear_imem;
        clear_dmem;
        d_latency = 8;                  //slow the locked read so BREQ (2FF sync)
                                        //lands inside the pair deterministically
        dmem[8'h1C] = 32'h00FF_00FF;
        eidx = 0;
        emit_ldrn(1, 32'hA000_0070);
        imem[eidx] = 16'h401B; eidx = eidx + 1;      // TAS.B @R1
        imem[eidx] = 16'h0429; eidx = eidx + 1;      // MOVT  R4
        emit_sentinel_loop(eidx, sent);
        do_reset;
        tas_mon_clr = 1'b1; @(posedge clk); tas_mon_clr = 1'b0;
        c = 0;
        while(u_dut.u_bsc.fe_lock_hold !== 1'b1 && c < 60000) begin @(posedge clk); c = c + 1; end
        chk_true("front-end lock opened (probe)", u_dut.u_bsc.fe_lock_hold === 1'b1);
        breq_n = 1'b0;
        c = 0;
        while(back_n !== 1'b0 && c < 60000) begin @(posedge clk); c = c + 1; end
        run_cycles(2);
        chk_true("bus granted after the pair (ord)",     back_n   === 1'b0);
        chk_true("no grant inside the locked pair (ord)", tas_viol === 1'b0);
        breq_n = 1'b1;
        run_until_retire(sent, 60000);
        chk("T set (ord)",       gpr(4), 32'd1);
        chk("dmem byte set", dmem[8'h1C], 32'h80FF_00FF);
        end_test;
    end
endtask

task automatic test_flash_boot;
    integer sent;
    begin
        begin_test("NOR flash boot (area 0, 16-bit): fetch + reads on the pins, SDRAM/refresh interleave");
        raw_mode = 2;                       //TB memory silent: flash + SDRAM own the board
        flash_en = 1'b1;
        md4_pin  = 1'b1;                    //MD4:MD3 = 10 -> area 0 is a 16-bit port
        md3_pin  = 1'b0;
        while(!u_flash.Power_Up) run_cycles(1000);   //Tvcs = 200us model gate
        flash_poke_l(22'h00_1000, 32'hC0DE_F1A5);    //data pattern in the array
        //boot program, assembled into imem then copied into the flash array;
        //every fetch below is a two-sub-cycle longword on D15-D0
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFCB, 32'hFFFF_E880); //AP CL2 + RFSH; A0W=011 (3 waits)
        emit_wreg_w(32'hFFFF_FF72, 16'hA508);        // RTCOR = 8
        emit_wreg_w(32'hFFFF_FF6E, 16'hA508);        // RTCSR: CKS=001 (bus/4) -> REF every ~32 bus cyc
        emit_ldrn(1, 32'hA000_1000);                 //flash data window (P2 area 0)
        imem[eidx] = 16'h6312; eidx = eidx + 1;      // MOV.L @R1,R3
        imem[eidx] = 16'h6411; eidx = eidx + 1;      // MOV.W @R1,R4      (MS half)
        imem[eidx] = 16'h8411; eidx = eidx + 1;      // MOV.B @(1,R1),R0  (odd byte)
        imem[eidx] = 16'h6703; eidx = eidx + 1;      // MOV   R0,R7
        emit_ldrn(2, 32'hAC00_0100);                 //SDRAM mailbox base (P2 area 3)
        imem[eidx] = 16'h2232; eidx = eidx + 1;      // MOV.L R3,@R2
        imem[eidx] = 16'h1241; eidx = eidx + 1;      // MOV.L R4,@(4,R2)
        imem[eidx] = 16'h1272; eidx = eidx + 1;      // MOV.L R7,@(8,R2)
        emit_ldr0(32'd40);                           //flash-read loop, refresh interleaving
        imem[eidx] = 16'h6503; eidx = eidx + 1;      // MOV   R0,R5
        imem[eidx] = 16'h6912; eidx = eidx + 1;      // MOV.L @R1,R9      <- loop
        imem[eidx] = 16'h4510; eidx = eidx + 1;      // DT    R5
        imem[eidx] = 16'h8BFC; eidx = eidx + 1;      // BF    loop (disp -4)
        imem[eidx] = 16'h1293; eidx = eidx + 1;      // MOV.L R9,@(12,R2)
        emit_sentinel_loop(eidx, sent);
        flash_load_imem;
        do_reset;
        bench_arm = 1'b1;
        run_until_retire(sent, 300000);
        bench_arm = 1'b0;
        run_cycles(200);                    //posted engine write needs its bus cycles
        $display("      [FLASH] boot-to-sentinel: %0d retires / %0d cycles", bench_retires, bench_arch_cycles);
        chk("flash long read",            gpr(3), 32'hC0DE_F1A5);
        chk("flash word read (MS half)",  gpr(4), 32'hFFFF_C0DE);
        chk("flash byte read (offset 1)", gpr(7), 32'hFFFF_FFDE);
        chk("loop read intact",           gpr(9), 32'hC0DE_F1A5);
        chk("loop count consumed",        gpr(5), 32'd0);
        chk("mailbox long", sdram_peek(32'h0C00_0100), 32'hC0DE_F1A5);
        chk("mailbox word", sdram_peek(32'h0C00_0104), 32'hFFFF_C0DE);
        chk("mailbox byte", sdram_peek(32'h0C00_0108), 32'hFFFF_FFDE);
        chk("mailbox loop", sdram_peek(32'h0C00_010C), 32'hC0DE_F1A5);
        chk_true("refreshes interleaved the flash loop", u_dut.u_bsc.rfcr >= 16'd10);
        //Relocked 2026-07-05 (fetch pair): every OTHER cycle law got faster, but this
        //slow-flash BRANCH loop pays ~16 cyc/iteration more - the redirect's wrong-path
        //drop-wait and the refresh cadence interleave differently against 15-cycle
        //fetches. Correctness intact; candidate for a later fetch-path DSE.
        chk("flash boot cycle law", bench_arch_cycles, 32'd6668);   //16-bit boot + 3-wait, T1..T2+idles
                                                        //flash reads + refresh interleave
        end_test;
    end
endtask

task automatic test_flash_autoselect;
    integer sent_h;
    begin
        begin_test("NOR autoselect from SDRAM-resident code: command writes + ID reads on shared pins");
        raw_mode = 2;
        flash_en = 1'b1;
        md4_pin  = 1'b1;
        md3_pin  = 1'b0;
        while(!u_flash.Power_Up) run_cycles(1000);
        //routine in SDRAM (phys 0x800): every fetch is an SDRAM read cycle
        //BETWEEN the flash command write cycles - shared-pin interleave
        eidx = 'h400;
        emit_sd(16'hE1A0);      // MOV  #0xA0,R1  ; flash base 0xA0000000
        emit_sd(16'h4128);      // SHLL16 R1      ; (sign junk shifts out)
        emit_sd(16'h4118);      // SHLL8  R1
        emit_sd(16'hE00A);      // MOV  #0x0A,R0  ; unlock addr 1: word 0x555
        emit_sd(16'h4018);      // SHLL8 R0       ; (byte offset 0xAAA)
        emit_sd(16'hCBAA);      // OR   #0xAA,R0
        emit_sd(16'h6203);      // MOV  R0,R2
        emit_sd(16'h321C);      // ADD  R1,R2
        emit_sd(16'hE005);      // MOV  #0x05,R0  ; unlock addr 2: word 0x2AA
        emit_sd(16'h4018);      // SHLL8 R0       ; (byte offset 0x554)
        emit_sd(16'hCB54);      // OR   #0x54,R0
        emit_sd(16'h6303);      // MOV  R0,R3
        emit_sd(16'h331C);      // ADD  R1,R3
        emit_sd(16'hE0AA);      // MOV  #0xAA,R0  ; cmd cycle 1: AA -> 555
        emit_sd(16'h2201);      // MOV.W R0,@R2
        emit_sd(16'hE055);      // MOV  #0x55,R0  ; cmd cycle 2: 55 -> 2AA
        emit_sd(16'h2301);      // MOV.W R0,@R3
        emit_sd(16'hE090);      // MOV  #0x90,R0  ; cmd cycle 3: 90 -> 555 (autoselect)
        emit_sd(16'h2201);      // MOV.W R0,@R2
        emit_sd(16'h6411);      // MOV.W @R1,R4       ; manufacturer (0x00C2)
        emit_sd(16'h8511);      // MOV.W @(2,R1),R0   ; device ID
        emit_sd(16'h6503);      // MOV  R0,R5         ; (0x22A7 = MX29LV320ET)
        emit_sd(16'hE0F0);      // MOV  #0xF0,R0  ; reset command
        emit_sd(16'h2101);      // MOV.W R0,@R1
        emit_sd(16'h8518);      // MOV.W @(16,R1),R0  ; array read after reset
        emit_sd(16'h6603);      // MOV  R0,R6         ; (0x5AC3)
        emit_sd(16'hE7AC);      // MOV  #0xAC,R7  ; SDRAM mailbox base 0xAC000000
        emit_sd(16'h4728);      // SHLL16 R7
        emit_sd(16'h4718);      // SHLL8  R7
        emit_sd(16'h2742);      // MOV.L R4,@R7
        emit_sd(16'h1751);      // MOV.L R5,@(4,R7)
        emit_sd(16'h1762);      // MOV.L R6,@(8,R7)
        emit_sd(16'hE85A);      // MOV  #0x5A,R8  ; sentinel
        sent_h = eidx - 1;
        emit_sd(16'hAFFE);      // BRA  self
        emit_sd(16'h0009);      // NOP  (delay slot)
        //boot stub in flash: SDRAM init, jump into the SDRAM routine
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFCB, 32'hFFFF_E880);
        emit_ldrn(2, 32'hAC00_0800);
        imem[eidx] = 16'h422B; eidx = eidx + 1;      // JMP  @R2
        imem[eidx] = 16'h0009; eidx = eidx + 1;      // (delay slot)
        flash_load_imem;
        flash_poke_h(21'h8, 16'h5AC3);      //array halfword at byte 0x10 (after the
                                            //program copy - load_imem covers word 8)
        do_reset;
        run_until_retire(sent_h, 300000);
        run_cycles(200);                    //posted engine write needs its bus cycles
        chk("manufacturer code",          gpr(4), 32'h0000_00C2);
        chk("device ID (MX29LV320ET)",    gpr(5), 32'h0000_22A7);
        chk("array read after F0 reset",  gpr(6), 32'h0000_5AC3);
        chk("mailbox manufacturer", sdram_peek(32'h0C00_0000), 32'h0000_00C2);
        chk("mailbox device ID",    sdram_peek(32'h0C00_0004), 32'h0000_22A7);
        chk("mailbox array",        sdram_peek(32'h0C00_0008), 32'h0000_5AC3);
        end_test;
    end
endtask

task automatic test_bus_monitors;
    begin
        begin_test("Board-bus shape monitors: D-bus contention, addr/data hold under WE (whole run)");
        chk_true("no D-bus driver overlap",         !dbus_viol);
        chk_true("no addr/data movement under WE",  !we_shape_viol);
        end_test;
        begin_test("MON match-queue oracle: strobe==unit 1:1, in order, fields exact (whole run)");
        $display("      (info) %0d strobes matched, measured R8 depth = %0d, reset-flushed = %0d",
                 mon_pushes, mon_qmax, mon_flushed);
        chk_true("no oracle mismatches",            mon_err == 0);
        chk_true("strobe coverage nonzero",         mon_pushes > 1000);
        chk_true("R8 outstanding depth within 2",   mon_qmax <= 2);
        end_test;
    end
endtask



///////////////////////////////////////////////////////////
//////  Tests - BSC Group C (datasheet breadth)
////

task automatic test_mcs_pins;
    integer sent;
    begin
        begin_test("MCS0-7: MCSCR block decode on the PTC pads + the CS0 pad switch (table 10.15)");
        raw_mode = 1;
        dmem[0]  = 32'h4D43_5331;
        eidx = 0;
        //MCSCR1: area 0, 32-Mbit block 0x0400000-0x07FFFFF (A25:22 = 0001);
        //MCSCR0: area 0, 256-Mbit block at 0 - covers the boot fetches, so
        //the CS0 pad keeps selecting the raw device after the PFC switch
        emit_wreg_w(32'hFFFF_FF52, 16'h0001);
        emit_wreg_w(32'hFFFF_FF50, 16'h0030);
        emit_wreg_w(32'hA400_0104, 16'hAAA0);            //PCCR: PTC1/PTC0 -> MCS
        emit_ldrn(1, 32'hA040_0000);
        imem[eidx] = 16'h6312; eidx = eidx + 1;          // MOV.L @R1,R3 (in-block)
        emit_ldrn(1, 32'hA200_0000);
        imem[eidx] = 16'h6412; eidx = eidx + 1;          // MOV.L @R1,R4 (A25=1: out)
        emit_wreg_w(32'hA400_0104, 16'hAAAA);            //restore PCCR + MCSCRs
        emit_wreg_w(32'hFFFF_FF50, 16'h0000);
        emit_wreg_w(32'hFFFF_FF52, 16'h0000);
        emit_sentinel_loop(eidx, sent);
        mcs_mon_clr = 1'b1; @(posedge clk); mcs_mon_clr = 1'b0;
        do_reset;
        run_until_retire(sent, 60000);
        chk("in-block read data (MCS0 kept CS0 alive)", gpr(3), 32'h4D43_5331);
        chk_true("MCS1 asserted on the PTC1 pad in-block",  mcs1_seen    === 1'b1);
        chk_true("MCS1 never asserted out-of-block",        mcs1_viol    === 1'b0);
        chk_true("CS0 pad (= MCS0) high on A25=1 accesses", cs0_mcs_viol === 1'b0);
        end_test;
    end
endtask

task automatic test_width8;
    integer sent;
    begin
        begin_test("8-bit port (BCR2): longword = four D7-D0/WE0 sub-cycles, data round-trip");
        raw_mode = 1;
        raw8_en  = 1'b1;
        eidx = 0;
        emit_wreg_w(32'hFFFF_FF62, 16'h1FF0);            // BCR2: A6SZ=01 (area 6 = 8-bit)
        emit_ldrn(1, 32'hB800_0200);
        emit_ldr0(32'hCAFE_F00D);
        imem[eidx] = 16'h2102; eidx = eidx + 1;          // MOV.L R0,@R1 (4 sub-cycles)
        emit_ldrn(1, 32'hB800_0202);
        emit_ldr0(32'h0000_BEEF);
        imem[eidx] = 16'h2101; eidx = eidx + 1;          // MOV.W R0,@R1 (2 sub-cycles)
        emit_ldrn(1, 32'hB800_0201);
        emit_ldr0(32'h0000_0077);
        imem[eidx] = 16'h2100; eidx = eidx + 1;          // MOV.B R0,@R1 (1 sub-cycle)
        emit_ldrn(1, 32'hB800_0200);
        imem[eidx] = 16'h6312; eidx = eidx + 1;          // MOV.L @R1,R3 (4 sub-cycles)
        emit_ldrn(1, 32'hB800_0202);
        imem[eidx] = 16'h6411; eidx = eidx + 1;          // MOV.W @R1,R4
        emit_ldrn(1, 32'hB800_0203);
        imem[eidx] = 16'h6510; eidx = eidx + 1;          // MOV.B @R1,R5
        emit_sentinel_loop(eidx, sent);
        we0_clr = 1'b1; @(posedge clk); we0_clr = 1'b0;
        do_reset;
        run_until_retire(sent, 60000);
        chk("byte-walked longword readback", gpr(3), 32'hCA77_BEEF);
        chk("word read (sign-extended)",     gpr(4), 32'hFFFF_BEEF);
        chk("byte read (sign-extended)",     gpr(5), 32'hFFFF_FFEF);
        chk("device content",                dmem[8'h80], 32'hCA77_BEEF);
        chk("WE0 strobes: 4 + 2 + 1 write sub-cycles", we0_cnt, 32'd7);
        end_test;
    end
endtask

//table-10.13 pin expectations are transcribed as tb constants: row = the
//A(shift+1)-first pattern on A16-A1, column = original address with the
//held/bank pins and the A12 precharge flag substituted
task automatic test_amx_shapes;
    integer sent, m1, m2;
    logic [25:0] A;
    begin
        begin_test("AMX address multiplex: row shift + column bank/hold pins (table 10.13, 32-bit)");
        sdram_en = 1'b0;                    //shape probe: the 0111-wired device is depopulated
        A = 26'h0AB_A984;                   //probe address (area 3 offset, A25 = 0)
        eidx = 0;
        emit_sdram_init(16'h5028, 16'hFFDF, 32'hFFFF_E880);   //AMX = 0101 (2Mx16x4)
        emit_ldrn(1, {6'b1010_11, A});                        //P2 + area 3 + offset
        emit_ldr0(32'hA5A5_0101);
        imem[eidx] = 16'h2102; eidx = eidx + 1;               // MOV.L R0,@R1 (WRITA)
        m1 = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;               // marker 1
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;               // MOV  R0,R2
        imem[eidx] = 16'h4210; eidx = eidx + 1;               // DT   R2
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;               // BF   (delay: probe window)
        emit_wreg_w(32'hFFFF_FF68, 16'h5020);                 //AMX = 0100 (1Mx16x4)
        emit_ldrn(1, {6'b1010_11, A});
        emit_ldr0(32'hA5A5_0100);
        imem[eidx] = 16'h2102; eidx = eidx + 1;               // MOV.L R0,@R1 (WRITA)
        m2 = eidx;
        imem[eidx] = 16'hE702; eidx = eidx + 1;               // marker 2
        emit_sentinel_loop(eidx, sent);
        do_reset;
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m1, 60000);
        run_cycles(120);
        chk("0101: one ACTV, one WRIT", sdm_actv_cnt * 16 + sdm_cas_cnt, 32'd17);
        chk("0101 row pins: A10-first shift, banks A15/A14",
            {6'd0, sdm_row_a}, {6'd0, A[25:17], A[25:10], A[0]});
        chk("0101 col pins: banks held, AP at A12",
            {6'd0, sdm_col_a}, {6'd0, A[25:17], A[16], A[24], A[23], A[13], 1'b1, A[11:1], A[0]});
        chk("0101 write DQM all lanes", {28'd0, sdm_cas_dqm}, 32'd0);
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m2, 60000);
        run_cycles(120);
        chk("0100: one ACTV, one WRIT", sdm_actv_cnt * 16 + sdm_cas_cnt, 32'd17);
        chk("0100 row pins: A9-first shift, banks A15/A14",
            {6'd0, sdm_row_a}, {6'd0, A[25:17], A[24:9], A[0]});
        chk("0100 col pins: banks held, AP at A12",
            {6'd0, sdm_col_a}, {6'd0, A[25:17], A[16], A[23], A[22], A[13], 1'b1, A[11:1], A[0]});
        run_until_retire(sent, 60000);
        end_test;
    end
endtask

task automatic test_sdram16_shapes;
    integer sent, m1, m2, m3, m4;
    begin
        begin_test("16-bit SDRAM bus: half-word beats on A3:A1, DQM rails, AP at A11, 8-beat fill");
        sdram_en = 1'b0;                    //shape probe: 16-bit wiring differs from the model
        eidx = 0;
        emit_wreg_w(32'hFFFF_FF62, 16'h3FB0);                 //BCR2: A3SZ=10 (word) FIRST
        emit_sdram_init(16'h5020, 16'hFFDF, 32'hFFFF_E440);   //AMX=0100; 16-bit SDMR window (p.302)
        emit_ldrn(1, 32'hAC00_0018);
        emit_ldr0(32'hFEED_C0DE);
        imem[eidx] = 16'h2102; eidx = eidx + 1;               // MOV.L R0,@R1: 2 WRIT halves
        m1 = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;               // marker 1
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;               // DT/BF delay
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        emit_ldrn(1, 32'hAC00_0012);
        imem[eidx] = 16'h6411; eidx = eidx + 1;               // MOV.W @R1,R4: 1 READ beat
        m2 = eidx;
        imem[eidx] = 16'hE702; eidx = eidx + 1;               // marker 2
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        emit_ldrn(1, 32'hAC00_0011);
        imem[eidx] = 16'h6510; eidx = eidx + 1;               // MOV.B @R1,R5 (odd byte)
        m3 = eidx;
        imem[eidx] = 16'hE703; eidx = eidx + 1;               // marker 3
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        imem[eidx] = 16'hE0EC; eidx = eidx + 1;               // MOV #0xEC,R0 (CCR)
        imem[eidx] = 16'hE10F; eidx = eidx + 1;               // MOV #0x0F,R1
        imem[eidx] = 16'h2012; eidx = eidx + 1;               // MOV.L R1,@R0: cache on
        emit_ldrn(1, 32'h8C00_0040);
        imem[eidx] = 16'h6712; eidx = eidx + 1;               // MOV.L @R1,R7: 8-beat fill
        m4 = eidx;
        imem[eidx] = 16'hE704; eidx = eidx + 1;               // marker 4
        emit_sentinel_loop(eidx, sent);
        do_reset;
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m1, 60000);
        run_cycles(120);
        chk("long write: 2 WRIT half-beats",   sdm_cas_cnt, 32'd2);
        chk("last half col: A3:A1 walked, AP at A11",
            {6'd0, sdm_col_a}, 32'h0000_081A);
        chk("write DQM on the low rails",      {28'd0, sdm_cas_dqm}, {28'd0, 4'b1100});
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m2, 60000);
        run_cycles(120);
        chk("word read: one beat",             sdm_cas_cnt, 32'd1);
        chk("word read col + AP",              {6'd0, sdm_col_a}, 32'h0000_0812);
        chk("word read DQM: DQMLU+DQMLL",      {28'd0, sdm_cas_dqm}, {28'd0, 4'b1100});
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m3, 60000);
        run_cycles(120);
        chk("odd byte read: one beat",         sdm_cas_cnt, 32'd1);
        chk("odd byte DQM: DQMLL only",        {28'd0, sdm_cas_dqm}, {28'd0, 4'b1110});
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m4, 60000);
        run_cycles(150);
        chk("cache fill: 8 READ half-beats",   sdm_cas_cnt, 32'd8);
        chk("cache fill: 8 BS Td data cycles", sdm_td_cnt,  32'd8);
        run_until_retire(sent, 60000);
        end_test;
    end
endtask

task automatic test_bs_td_dqm;
    integer sent, m0, m1, m2, m3;
    begin
        begin_test("BS on Td data cycles + per-byte read DQM against the live device (p.283)");
        sdram_poke(32'h0C00_0030, 32'h1122_3344);
        eidx = 0;
        emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);   //CL2, AMX=0111
        m0 = eidx;
        imem[eidx] = 16'hE700; eidx = eidx + 1;               // marker 0 (init done)
        emit_ldr0(32'd60);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;               // DT/BF delay
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        emit_ldrn(1, 32'hAC00_0030);
        imem[eidx] = 16'h6312; eidx = eidx + 1;               // MOV.L @R1,R3
        m1 = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;               // marker 1
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        emit_ldrn(1, 32'hAC00_0031);
        imem[eidx] = 16'h6410; eidx = eidx + 1;               // MOV.B @R1,R4 (byte 1)
        m2 = eidx;
        imem[eidx] = 16'hE702; eidx = eidx + 1;               // marker 2
        emit_ldr0(32'd200);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        imem[eidx] = 16'hE0EC; eidx = eidx + 1;               // MOV #0xEC,R0 (CCR)
        imem[eidx] = 16'hE10F; eidx = eidx + 1;               // MOV #0x0F,R1
        imem[eidx] = 16'h2012; eidx = eidx + 1;               // MOV.L R1,@R0: cache on
        emit_ldrn(1, 32'h8C00_0030);
        imem[eidx] = 16'h6512; eidx = eidx + 1;               // MOV.L @R1,R5: 4-beat fill
        m3 = eidx;
        imem[eidx] = 16'hE703; eidx = eidx + 1;               // marker 3
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(m0, 60000);
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m1, 60000);
        run_cycles(120);
        chk("single read: 1 READ, 1 Td",   sdm_cas_cnt * 16 + sdm_td_cnt, 32'd17);
        //data lands CL device clocks after registration = CL-1 command
        //windows later in the controller frame (the rd_lat capture pipeline)
        chk("BS marks the CL-2 data window", (sdm_td_t - sdm_cas_t), 32'd20);
        chk("long read DQM all lanes",     {28'd0, sdm_cas_dqm}, 32'd0);
        chk("read data (device answered)", gpr(3), 32'h1122_3344);
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m2, 60000);
        run_cycles(120);
        chk("byte read DQM: DQMUL lane only", {28'd0, sdm_cas_dqm}, {28'd0, 4'b1011});
        chk("byte read data through its lane", gpr(4), 32'h0000_0022);
        sdm_clr = 1'b1; run_cycles(4); sdm_clr = 1'b0;
        run_until_retire(m3, 60000);
        run_cycles(150);
        chk("burst fill: 4 READ, 4 Td",    sdm_cas_cnt * 16 + sdm_td_cnt, 32'd68);
        chk("fill word 0 data",            gpr(5), 32'h1122_3344);
        run_until_retire(sent, 60000);
        end_test;
    end
endtask

task automatic test_release_pads;
    integer sent, m1, m2, c;
    begin
        begin_test("Bus-release pads: PULA 4-cycle window, PULD, HIZCNT, IRQOUT on pending refresh");
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);   //RFSH=1
        emit_wreg_w(32'hFFFF_FF72, 16'hA520);                 //RTCOR = 0x20
        emit_wreg_w(32'hFFFF_FF6E, 16'hA508);                 //RTCSR: CKS=001 (bus/4)
        emit_wreg_w(32'hFFFF_FF60, 16'hD008);                 //BCR1: PULA|PULD|HIZCNT
        m1 = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;               // marker 1
        emit_ldr0(32'd60);
        imem[eidx] = 16'h6203; eidx = eidx + 1;
        imem[eidx] = 16'h4210; eidx = eidx + 1;               // DT/BF delay
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;
        emit_wreg_w(32'hFFFF_FF60, 16'hC008);                 //BCR1: HIZCNT off
        m2 = eidx;
        imem[eidx] = 16'hE702; eidx = eidx + 1;               // marker 2
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(m1, 60000);
        apu_clr = 1'b1; run_cycles(4); apu_clr = 1'b0;
        breq_n = 1'b0;
        c = 0;
        while(back_n !== 1'b0 && c < 60000) begin @(posedge clk); c = c + 1; end
        run_cycles(40);                     //the 4-cycle pull window has passed
        chk("PULA: A pins pulled exactly 4 CKIO cycles", apu_hi_cnt, 32'd4);
        chk_true("A pull-up released after the window",  a_pu      === 1'b0);
        chk_true("HIZCNT=1: RAS/CAS pads stay driven",   rascas_oe === 1'b1);
        chk_true("other shared pads released",           bus_oe    === 1'b0);
        chk_true("PULD: D pins pulled while idle",       d_pu      === 1'b1);
        c = 0;                              //a refresh request must pend -> IRQOUT
        while(irqout_n !== 1'b0 && c < 4000) begin @(posedge clk); c = c + 1; end
        chk_true("IRQOUT asserted on pending refresh",   irqout_n  === 1'b0);
        breq_n = 1'b1;
        c = 0;                              //bus regained: the refresh cycle runs
        while(irqout_n !== 1'b1 && c < 4000) begin @(posedge clk); c = c + 1; end
        chk_true("IRQOUT negated once the refresh ran",  irqout_n  === 1'b1);
        run_until_retire(m2, 60000);
        breq_n = 1'b0;
        c = 0;
        while(back_n !== 1'b0 && c < 60000) begin @(posedge clk); c = c + 1; end
        run_cycles(10);
        chk_true("HIZCNT=0: RAS/CAS pads released too",  rascas_oe === 1'b0);
        breq_n = 1'b1;
        run_until_retire(sent, 60000);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  P Bus: TMU and I/O Ports (session 3)
////

task automatic test_pbus_tmu_regs;
    integer idx, sent;
    begin
        begin_test("P bus TMU regs: reset values, R/W lanes, neighbors stay dummy, TCLK pad grant");
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FE94);                 // TCOR0
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // MOV.L @R1,R3      ; reset 0xFFFFFFFF
        emit_ldr0(32'h1234_5678);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6412; eidx = eidx + 1;              // MOV.L @R1,R4      ; readback
        emit_wreg_w(32'hFFFF_FE9C, 16'hFFFF);        // TCR0 <- all ones
        emit_ldrn(1, 32'hFFFF_FE9C);
        imem[eidx] = 16'h6511; eidx = eidx + 1;              // MOV.W @R1,R5      ; implemented bits only
        emit_ldrn(1, 32'hFFFF_FE92);                 // TSTR
        emit_ldr0(32'h0000_00FF);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1
        imem[eidx] = 16'h6710; eidx = eidx + 1;              // MOV.B @R1,R7      ; STR2-0 only
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2    ; mailbox base
        emit_ldrn(1, 32'hFFFF_FE90);                 // TOCR
        emit_ldr0(32'h0000_00FF);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; TCOE only
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0
        emit_ldrn(1, 32'hFFFF_FE80);                 // SCI window: unmapped
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // MOV.L R0,@(4,R2)  ; mb1 = 0
        emit_ldrn(1, 32'hA400_0104);                 // PCCR via the area-1 P window
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0      ; 0xAAAA sign-ext
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // MOV.L R0,@(8,R2)  ; mb2
        emit_ldrn(1, 32'hA400_0140);                 // just past the port window
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // MOV.L R0,@(12,R2) ; mb3 = 0
        emit_ldrn(1, 32'hFFFF_FEB8);                 // TCPR2
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // MOV.L R0,@(16,R2) ; mb4
        emit_wreg_w(32'hA400_010E, 16'h2AAA);        // PHCR: PH7 mode 00 -> TCLK pad grant
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("TCOR0 reset 0xFFFFFFFF",           gpr(3), 32'hFFFF_FFFF);
        chk("TCOR0 long R/W",                   gpr(4), 32'h1234_5678);
        chk("TCR0 implemented bits only",       gpr(5), 32'h0000_003F);
        chk("TSTR reserved bits read 0",        gpr(7), 32'h0000_0007);
        chk("TOCR.TCOE only",                   dmem[16'h10], 32'h0000_0001);
        chk("SCI neighbor stays dummy",         dmem[16'h11], 32'h0000_0000);
        chk("PCCR reset 0xAAAA (P window)",     dmem[16'h12], 32'hFFFF_AAAA);
        chk("port-window fringe stays dummy",   dmem[16'h13], 32'h0000_0000);
        chk("TCPR2 never initialized (sim 0)",  dmem[16'h14], 32'h0000_0000);
        //Drive grant only: the pad VALUE is the toggling RTC output clock (phase depends
        //on the sample instant; the pad==RTCCLK value law is locked by the RTC-clock test).
        chk_true("PTH7 drives as TCLK (TCOE & PH7 mode 00)", pth_oe[7]);
        chk_true("PTH6-0 stay inputs",          pth_oe[6:0] == 7'd0);
        end_test;
    end
endtask

task automatic test_tmu_underflow_e2e;
    integer idx, sent, marker;
    begin
        begin_test("TMU0 underflow e2e: period = (TCOR+1) ticks, auto-reload, INTEVT 0x400, UNF clear");
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FE94);
        emit_ldr0(32'd16);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // TCOR0 = 16
        emit_ldrn(1, 32'hFFFF_FE98);
        emit_ldr0(32'd16);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // TCNT0 = 16
        emit_wreg_w(32'hFFFF_FE9C, 16'h0020);        // TCR0: UNIE, P-phi/4
        emit_wreg_w(32'hFFFF_FEE2, 16'hD000);        // IPRA: TMU0 level 13
        emit_sr_imask(eidx, 4'h0);
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: STR0
        marker = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;              // MOV #1,R7
        emit_sentinel_loop(eidx, sent);
        emit_handler_tmu(8'h9C, 16'h0020);           // clear UNF, keep UNIE/TPSC
        unf_mon_clr = 1'b1;
        do_reset;
        unf_mon_clr = 1'b0;
        run_until_retire(marker, 30000);
        run_until_entry_count(2, 30000);             //2nd entry = reload + clear protocol work
        run_cycles(300);
        chk("INTEVT = 0x400 (TUNI0)",  dmem[16'h10], 32'h0000_0400);
        chk("INTEVT2 = 0x400 (TUNI0)", dmem[16'h11], 32'h0000_0400);
        //P-phi/4 tick = 16 core cycles at the FRQCR reset ratio: (16+1)*16
        chk("underflow period = (TCOR+1)*16 cycles", unf0_dt, 32'd272);
        end_test;
    end
endtask

task automatic test_tmu_prescaler_tstr;
    integer idx, sent;
    integer c0a, c1a, c2a;
    begin
        begin_test("TMU prescaler taps P/4 P/16 P/256 exact + TSTR halt/freeze/resume");
        //phase A: three channels on three taps, deltas over a 4096-cycle window
        eidx = 0;
        emit_wreg_w(32'hFFFF_FE9C, 16'h0000);        // TCR0: P-phi/4   (tick / 16 cycles)
        emit_wreg_w(32'hFFFF_FEA8, 16'h0001);        // TCR1: P-phi/16  (tick / 64 cycles)
        emit_wreg_w(32'hFFFF_FEB4, 16'h0003);        // TCR2: P-phi/256 (tick / 1024 cycles)
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd7);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: all three
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        c0a = u_dut.u_tmu.tcnt[0];
        c1a = u_dut.u_tmu.tcnt[1];
        c2a = u_dut.u_tmu.tcnt[2];
        run_cycles(4096);                            //multiple of every tick period: deltas exact
        chk("P/4 tap: 256 ticks / 4096 cycles",  c0a - u_dut.u_tmu.tcnt[0], 32'd256);
        chk("P/16 tap: 64 ticks / 4096 cycles",  c1a - u_dut.u_tmu.tcnt[1], 32'd64);
        chk("P/256 tap: 4 ticks / 4096 cycles",  c2a - u_dut.u_tmu.tcnt[2], 32'd4);
        //phase B: STR0 stops the count dead and resumes where it left off
        clear_imem; clear_dmem;
        eidx = 0;
        emit_wreg_w(32'hFFFF_FE9C, 16'h0000);        // TCR0: P-phi/4
        emit_ldrn(1, 32'hFFFF_FE98);
        emit_ldr0(32'd1000);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // TCNT0 = 1000
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: run
        for(idx = 0; idx < 16; idx = idx + 1) begin
            imem[eidx] = 16'h0009; eidx = eidx + 1;          // let some ticks pass
        end
        emit_ldr0(32'd0);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: halt (R1 still TSTR)
        emit_ldrn(2, 32'hFFFF_FE98);
        imem[eidx] = 16'h6422; eidx = eidx + 1;              // MOV.L @R2,R4      ; snapshot
        for(idx = 0; idx < 12; idx = idx + 1) begin
            imem[eidx] = 16'h0009; eidx = eidx + 1;          // frozen window
        end
        imem[eidx] = 16'h6522; eidx = eidx + 1;              // MOV.L @R2,R5      ; still equal?
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: resume
        for(idx = 0; idx < 16; idx = idx + 1) begin
            imem[eidx] = 16'h0009; eidx = eidx + 1;
        end
        imem[eidx] = 16'h6722; eidx = eidx + 1;              // MOV.L @R2,R7      ; counting again
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk_true("counted while STR0=1",   gpr(4) < 32'd1000);
        chk    ("frozen while STR0=0",     gpr(5), gpr(4));
        chk_true("resumed on STR0 re-set", gpr(7) < gpr(5));
        end_test;
    end
endtask

task automatic test_tmu_capture;
    integer idx, sent, marker;
    integer c2a, cap1;
    begin
        begin_test("TMU2 input capture: TCPR2 copy, ICPF clear protocol, INTEVT 0x460, edge select");
        eidx = 0;
        emit_wreg_w(32'hFFFF_FEB4, 16'h00C0);        // TCR2: ICPE=11, CKEG=00 rising, P-phi/4
        emit_wreg_w(32'hFFFF_FEE2, 16'h00D0);        // IPRA: TMU2/TICPI2 level 13
        emit_sr_imask(eidx, 4'h0);
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd4);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: STR2
        marker = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;              // MOV #1,R7
        emit_sentinel_loop(eidx, sent);
        emit_handler_tmu(8'hB4, 16'h00C0);           // write-0 clears ICPF, keeps ICPE
        do_reset;
        run_until_retire(marker, 30000);
        c2a = u_dut.u_tmu.tcnt[2];
        tclk_pin = 1'b1;                             //rising edge: capture + TICPI2
        run_until_entry_count(1, 20000);
        run_cycles(300);
        cap1 = u_dut.u_tmu.tcpr2;
        chk("INTEVT2 = 0x460 (TICPI2)",          dmem[16'h11], 32'h0000_0460);
        chk("handler saw the captured TCPR2",    dmem[16'h12], cap1);
        chk_true("TCPR2 = TCNT2 at the edge",    cap1 <= c2a && (c2a - cap1) < 4);
        tclk_pin = 1'b0;                             //falling edge: CKEG=00 must ignore
        run_cycles(300);
        chk("falling edge ignored (CKEG=00)",    entry_count, 32'd1);
        chk("TCPR2 unchanged on falling edge",   u_dut.u_tmu.tcpr2, cap1);
        tclk_pin = 1'b1;                             //second rising edge: capture again
        run_until_entry_count(2, 20000);
        run_cycles(300);
        tclk_pin = 1'b0;
        chk_true("second capture lower (down count)", u_dut.u_tmu.tcpr2 < cap1);
        end_test;
    end
endtask

task automatic test_tmu_external_clock;
    integer idx, sent, k;
    begin
        begin_test("TMU external clock (TPSC=101): TCLK pin edges count TCNT, both-edge mode");
        eidx = 0;
        emit_wreg_w(32'hFFFF_FE9C, 16'h0015);        // TCR0: CKEG=1x both edges, TPSC=101 TCLK
        emit_ldrn(1, 32'hFFFF_FE98);
        emit_ldr0(32'd100);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // TCNT0 = 100
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: STR0
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        run_cycles(50);
        chk("no edges, no count", u_dut.u_tmu.tcnt[0], 32'd100);
        for(k = 0; k < 5; k = k + 1) begin           //5 pulses = 10 edges (>2.5 Pcyc widths)
            tclk_pin = 1'b1; run_cycles(20);
            tclk_pin = 1'b0; run_cycles(20);
        end
        run_cycles(50);
        chk("10 edges = 10 ticks", u_dut.u_tmu.tcnt[0], 32'd90);
        end_test;
    end
endtask

task automatic test_ioport_modes;
    integer idx, sent;
    begin
        begin_test("Ports: mode matrix, pad ring O/OE/PU, RO bits, PGCR quirk, PINT/IRQ5 pad shares");
        //pads set before reset; all interrupt consumers of these pads are off
        pta_pin  = 8'h5A;
        pint_pin = 8'h33;                            //PTC pads (PINTER=0: no requests)
        ptf_pin  = 8'h0F;                            //PTF pads (IRLSEN=0, PINTER=0)
        ptg_pin  = 8'h01;
        eidx = 0;
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2      ; mailbox base
        emit_ldrn(1, 32'hA400_0124);                 // PCDR (reset: input, pull-up)
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; = PTC pads
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0
        emit_wreg_w(32'hA400_0100, 16'hAAAA);        // PACR: all input, pull-up on
        emit_ldrn(1, 32'hA400_0120);                 // PADR
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; = PTA pads
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // MOV.L R0,@(4,R2)  ; mb1
        emit_ldr0(32'h0000_00A5);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; stores, pad wins read
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // MOV.L R0,@(8,R2)  ; mb2 = pads still
        emit_wreg_w(32'hA400_0100, 16'h0005);        // PACR: PTA1/PTA0 output, rest other
        emit_ldrn(1, 32'hA400_0120);
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; = register now
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // MOV.L R0,@(12,R2) ; mb3
        emit_ldrn(1, 32'hA400_012A);                 // PFDR (read-only port)
        emit_ldr0(32'h0000_00FF);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; must be ignored
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; = PTF pads
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // MOV.L R0,@(16,R2) ; mb4
        emit_wreg_w(32'hA400_010A, 16'h0000);        // PFCR: all other-function
        emit_ldrn(1, 32'hA400_012A);
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; reads low
        imem[eidx] = 16'h1205; eidx = eidx + 1;              // MOV.L R0,@(20,R2) ; mb5
        emit_wreg_w(32'hA400_0106, 16'h0000);        // PDCR: all other-function
        emit_ldrn(1, 32'hA400_0126);                 // PDDR
        emit_ldr0(32'h0000_00FF);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; bits 4,6 are RO
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; 0xAF
        imem[eidx] = 16'h1206; eidx = eidx + 1;              // MOV.L R0,@(24,R2) ; mb6
        emit_wreg_w(32'hA400_010C, 16'h0008);        // PGCR: bit3 controls PTG0 (quirk, p.577)
        emit_ldrn(1, 32'hA400_012C);                 // PGDR
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; PTG0 pad = 1
        imem[eidx] = 16'h1207; eidx = eidx + 1;              // MOV.L R0,@(28,R2) ; mb7
        emit_wreg_w(32'hA400_010C, 16'h0002);        // PGCR: bit1 must NOT enable PTG0
        emit_ldrn(1, 32'hA400_012C);
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; 0
        imem[eidx] = 16'h1208; eidx = eidx + 1;              // MOV.L R0,@(32,R2) ; mb8
        emit_ldrn(1, 32'hA400_0136);                 // SCPDR (SCP7 input at reset)
        imem[eidx] = 16'h6010; eidx = eidx + 1;              // MOV.B @R1,R0      ; bit7 = IRQ5 pad
        imem[eidx] = 16'h1209; eidx = eidx + 1;              // MOV.L R0,@(36,R2) ; mb9
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 40000);
        chk("PCDR = PTC pads (PINT share)",             dmem[16'h10], 32'h0000_0033);
        chk("PADR input mode reads the pads",           dmem[16'h11], 32'h0000_005A);
        chk("input-mode write stored, pads still read", dmem[16'h12], 32'h0000_005A);
        chk("PADR output/other mode reads the register",dmem[16'h13], 32'hFFFF_FFA5);
        chk("PFDR write ignored, pads read",            dmem[16'h14], 32'h0000_000F);
        chk("PFDR other-function reads low",            dmem[16'h15], 32'h0000_0000);
        chk("PDDR RO bits 4,6 read low",                dmem[16'h16], 32'hFFFF_FFAF);
        chk("PGCR quirk: PTG0 mode from bit 3",         dmem[16'h17], 32'h0000_0001);
        chk("PGCR quirk: bit 1 has no effect",          dmem[16'h18], 32'h0000_0000);
        chk("SCPDR bit7 = SCPT7 pad (IRQ5 share)",      dmem[16'h19], 32'hFFFF_FF80);
        chk("PTA pad ring: O = PADR register",          pta_o,  32'h0000_00A5);
        chk("PTA pad ring: OE only on mode-01 pins",    pta_oe, 32'h0000_0003);
        chk("PTA pad ring: pull-ups off outside mode 10", pta_pu, 32'h0000_0000);
        end_test;
    end
endtask

task automatic test_ckio_pin;
    integer k, tog;
    logic v;
    begin
        begin_test("CKIO pin: bus clock output at core/2 (the SDRAM device clock source)");
        do_reset;
        run_cycles(8);
        tog = 0;
        v = ckio;
        for(k = 0; k < 16; k = k + 1) begin
            @(negedge clk);                      //CKIO changes at posedge; stable here
            if(ckio !== v) tog = tog + 1;
            v = ckio;
        end
        chk("CKIO toggles on every core cycle", tog, 32'd16);
        end_test;
    end
endtask

task automatic test_rtc_regs;
    integer sent;
    begin
        begin_test("P bus RTC regs: POR values, counter/alarm R/W + masks, ADJ/RESET read 0, CF write law");
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FEC0);                 // counter base
        emit_ldrn(3, 32'hFFFF_FED0);                 // alarm/control base
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2    ; mailbox base
        imem[eidx] = 16'h843E; eidx = eidx + 1;              // MOV.B @(14,R3),R0 ; RCR2
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0 = 0x09 (RTCEN|START)
        imem[eidx] = 16'h843C; eidx = eidx + 1;              // MOV.B @(12,R3),R0 ; RCR1
        imem[eidx] = 16'hC97F; eidx = eidx + 1;              // AND   #0x7F,R0    ; CF may set at 128 Hz
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1 = 0x00
        imem[eidx] = 16'hE008; eidx = eidx + 1;              // MOV   #0x08,R0
        imem[eidx] = 16'h803E; eidx = eidx + 1;              // RCR2: START=0 (halt for writes, 13.4.1)
        imem[eidx] = 16'hE059; eidx = eidx + 1;              // MOV   #0x59,R0
        imem[eidx] = 16'h8012; eidx = eidx + 1;              // RSECCNT
        imem[eidx] = 16'h8412; eidx = eidx + 1;              // readback
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // mb2 = 0x59
        imem[eidx] = 16'hE023; eidx = eidx + 1;              // MOV   #0x23,R0
        imem[eidx] = 16'h8016; eidx = eidx + 1;              // RHRCNT
        imem[eidx] = 16'h8416; eidx = eidx + 1;
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // mb3 = 0x23
        imem[eidx] = 16'hE006; eidx = eidx + 1;              // MOV   #0x06,R0
        imem[eidx] = 16'h8018; eidx = eidx + 1;              // RWKCNT
        imem[eidx] = 16'h8418; eidx = eidx + 1;
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // mb4 = 0x06
        imem[eidx] = 16'hE099; eidx = eidx + 1;              // MOV   #0x99,R0
        imem[eidx] = 16'h801E; eidx = eidx + 1;              // RYRCNT (BCD 99)
        imem[eidx] = 16'h841E; eidx = eidx + 1;
        imem[eidx] = 16'h1205; eidx = eidx + 1;              // mb5 = 0xFFFFFF99 (MOV.B sign-ext)
        imem[eidx] = 16'hE0D9; eidx = eidx + 1;              // MOV   #0xD9,R0
        imem[eidx] = 16'h8030; eidx = eidx + 1;              // RSECAR = ENB|0x59
        imem[eidx] = 16'h8430; eidx = eidx + 1;
        imem[eidx] = 16'h1206; eidx = eidx + 1;              // mb6 = 0xFFFFFFD9
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #0xFF,R0
        imem[eidx] = 16'h8034; eidx = eidx + 1;              // RHRAR
        imem[eidx] = 16'h8434; eidx = eidx + 1;
        imem[eidx] = 16'h1207; eidx = eidx + 1;              // mb7 = 0xFFFFFFBF (bit 6 reads 0)
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;
        imem[eidx] = 16'h8036; eidx = eidx + 1;              // RWKAR
        imem[eidx] = 16'h8436; eidx = eidx + 1;
        imem[eidx] = 16'h1208; eidx = eidx + 1;              // mb8 = 0xFFFFFF87 (bits 6:3 read 0)
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;
        imem[eidx] = 16'h803A; eidx = eidx + 1;              // RMONAR
        imem[eidx] = 16'h843A; eidx = eidx + 1;
        imem[eidx] = 16'h1209; eidx = eidx + 1;              // mb9 = 0xFFFFFF9F (bits 6:5 read 0)
        imem[eidx] = 16'hE00F; eidx = eidx + 1;              // MOV   #0x0F,R0
        imem[eidx] = 16'h803E; eidx = eidx + 1;              // RCR2 <= RTCEN|ADJ|RESET|START
        imem[eidx] = 16'h843E; eidx = eidx + 1;
        imem[eidx] = 16'h120A; eidx = eidx + 1;              // mb10 = 0x09 (action bits read 0)
        imem[eidx] = 16'h8410; eidx = eidx + 1;              // MOV.B @(0,R1),R0  ; R64CNT
        imem[eidx] = 16'h120B; eidx = eidx + 1;              // mb11 = 0 (divider-reset shadow)
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0x00,R0
        imem[eidx] = 16'h803C; eidx = eidx + 1;              // RCR1 <= 0: CF write-0 clear
        imem[eidx] = 16'h843C; eidx = eidx + 1;
        imem[eidx] = 16'h120C; eidx = eidx + 1;              // mb12 = 0 (no 128 Hz set in the shadow)
        imem[eidx] = 16'hE080; eidx = eidx + 1;              // MOV   #0x80,R0
        imem[eidx] = 16'h803C; eidx = eidx + 1;              // RCR1 <= CF: write-1 sets (13.2.15)
        imem[eidx] = 16'h843C; eidx = eidx + 1;
        imem[eidx] = 16'h120D; eidx = eidx + 1;              // mb13 = 0xFFFFFF80
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 40000);
        chk("RCR2 POR value 0x09 (RTCEN|START)", dmem[16'h10], 32'h0000_0009);
        chk("RCR1 POR value 0x00 (CF masked)",   dmem[16'h11], 32'h0000_0000);
        chk("RSECCNT byte R/W",                  dmem[16'h12], 32'h0000_0059);
        chk("RHRCNT byte R/W",                   dmem[16'h13], 32'h0000_0023);
        chk("RWKCNT byte R/W",                   dmem[16'h14], 32'h0000_0006);
        chk("RYRCNT byte R/W (BCD 99)",          dmem[16'h15], 32'hFFFF_FF99);
        chk("RSECAR ENB + value",                dmem[16'h16], 32'hFFFF_FFD9);
        chk("RHRAR reserved bit 6 reads 0",      dmem[16'h17], 32'hFFFF_FFBF);
        chk("RWKAR bits 6:3 read 0",             dmem[16'h18], 32'hFFFF_FF87);
        chk("RMONAR bits 6:5 read 0",            dmem[16'h19], 32'hFFFF_FF9F);
        chk("ADJ/RESET read 0 (RCR2 = 0x09)",    dmem[16'h1A], 32'h0000_0009);
        chk("R64CNT = 0 after divider RESET",    dmem[16'h1B], 32'h0000_0000);
        chk("CF write-0 clears",                 dmem[16'h1C], 32'h0000_0000);
        chk("CF write-1 sets",                   dmem[16'h1D], 32'hFFFF_FF80);
        end_test;
    end
endtask

/*
    Calendar cascade: Feb 28 23:59:59 + one second exercises EVERY carry in
    one 1 Hz wrap (sec->min->hr->wk/day->mon), with the leap-year rule
    picking the February length. RCR2.RESET aligns the divider so the wrap
    lands exactly 1.000 RTC s after the start - reading anywhere inside
    (1.0 s, 2.0 s) is deterministic. The POR between set and read proves the
    time survives resets (table 13.2: counters have no pin reset).
*/
task automatic test_rtc_time_carry;
    integer sent;
    begin
        begin_test("RTC calendar: full carry cascade, leap year, time survives a POR");
        //phase A: Feb 28 2003 (non-leap) 23:59:59, divider reset, run 1.3 s
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FEC0);
        emit_ldrn(3, 32'hFFFF_FED0);
        imem[eidx] = 16'hE008; eidx = eidx + 1;              // RCR2: stop the clock
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        imem[eidx] = 16'hE059; eidx = eidx + 1;
        imem[eidx] = 16'h8012; eidx = eidx + 1;              // sec  59
        imem[eidx] = 16'hE059; eidx = eidx + 1;
        imem[eidx] = 16'h8014; eidx = eidx + 1;              // min  59
        imem[eidx] = 16'hE023; eidx = eidx + 1;
        imem[eidx] = 16'h8016; eidx = eidx + 1;              // hour 23
        imem[eidx] = 16'hE002; eidx = eidx + 1;
        imem[eidx] = 16'h8018; eidx = eidx + 1;              // week 2 (Tuesday)
        imem[eidx] = 16'hE028; eidx = eidx + 1;
        imem[eidx] = 16'h801A; eidx = eidx + 1;              // date 28
        imem[eidx] = 16'hE002; eidx = eidx + 1;
        imem[eidx] = 16'h801C; eidx = eidx + 1;              // month 02
        imem[eidx] = 16'hE003; eidx = eidx + 1;
        imem[eidx] = 16'h801E; eidx = eidx + 1;              // year 03: 03 % 4 != 0, non-leap
        imem[eidx] = 16'hE00B; eidx = eidx + 1;              // RCR2: RESET | START
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        run_cycles(340000);                          //1.30 RTC s: exactly one 1 Hz wrap
        //phase A2: POR between set and read - the time must ride through it
        clear_imem;
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FEC0);
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2
        imem[eidx] = 16'h8412; eidx = eidx + 1;              // sec
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0
        imem[eidx] = 16'h8414; eidx = eidx + 1;              // min
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1
        imem[eidx] = 16'h8416; eidx = eidx + 1;              // hour
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // mb2
        imem[eidx] = 16'h8418; eidx = eidx + 1;              // week
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // mb3
        imem[eidx] = 16'h841A; eidx = eidx + 1;              // date
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // mb4
        imem[eidx] = 16'h841C; eidx = eidx + 1;              // month
        imem[eidx] = 16'h1205; eidx = eidx + 1;              // mb5
        imem[eidx] = 16'h841E; eidx = eidx + 1;              // year
        imem[eidx] = 16'h1206; eidx = eidx + 1;              // mb6
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("Feb 28 23:59:59 + 1 s: seconds 00",  dmem[16'h10], 32'h0000_0000);
        chk("minutes 00",                         dmem[16'h11], 32'h0000_0000);
        chk("hours 00",                           dmem[16'h12], 32'h0000_0000);
        chk("day of week Tue -> Wed",             dmem[16'h13], 32'h0000_0003);
        chk("date = 01 (non-leap Feb rolls over)",dmem[16'h14], 32'h0000_0001);
        chk("month = 03",                         dmem[16'h15], 32'h0000_0003);
        chk("year rides through the POR (03)",    dmem[16'h16], 32'h0000_0003);
        //phase B: leap year - Feb 28 2004 must roll to Feb 29, same month
        clear_imem; clear_dmem;
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FEC0);
        emit_ldrn(3, 32'hFFFF_FED0);
        imem[eidx] = 16'hE008; eidx = eidx + 1;              // RCR2: stop
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        imem[eidx] = 16'hE059; eidx = eidx + 1;
        imem[eidx] = 16'h8012; eidx = eidx + 1;              // sec  59
        imem[eidx] = 16'hE059; eidx = eidx + 1;
        imem[eidx] = 16'h8014; eidx = eidx + 1;              // min  59
        imem[eidx] = 16'hE023; eidx = eidx + 1;
        imem[eidx] = 16'h8016; eidx = eidx + 1;              // hour 23
        imem[eidx] = 16'hE028; eidx = eidx + 1;
        imem[eidx] = 16'h801A; eidx = eidx + 1;              // date 28
        imem[eidx] = 16'hE002; eidx = eidx + 1;
        imem[eidx] = 16'h801C; eidx = eidx + 1;              // month 02
        imem[eidx] = 16'hE004; eidx = eidx + 1;
        imem[eidx] = 16'h801E; eidx = eidx + 1;              // year 04: leap
        imem[eidx] = 16'hE00B; eidx = eidx + 1;              // RCR2: RESET | START
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        run_cycles(340000);
        clear_imem;
        eidx = 0;
        emit_ldrn(1, 32'hFFFF_FEC0);
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2
        imem[eidx] = 16'h841A; eidx = eidx + 1;              // date
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0
        imem[eidx] = 16'h841C; eidx = eidx + 1;              // month
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("leap year: Feb 28 -> Feb 29",        dmem[16'h10], 32'h0000_0029);
        chk("month stays 02",                     dmem[16'h11], 32'h0000_0002);
        end_test;
    end
endtask

task automatic test_rtc_periodic;
    integer sent, marker;
    begin
        begin_test("RTC periodic interrupt: PES=1/256 s period law, INTEVT 0x4A0, CF count-up");
        eidx = 0;
        emit_wreg_w(32'hFFFF_FEE2, 16'h000D);        // IPRA: RTC level 13
        emit_ldrn(3, 32'hFFFF_FED0);
        emit_sr_imask(eidx, 4'h0);
        imem[eidx] = 16'hE019; eidx = eidx + 1;              // MOV #0x19,R0
        imem[eidx] = 16'h803E; eidx = eidx + 1;              // RCR2: PES=001 | RTCEN | START
        marker = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;              // MOV #1,R7
        emit_sentinel_loop(eidx, sent);
        emit_handler_tmu(8'hDE, 16'h0019);           // RCR2 rewrite: PEF write-0 clear, PES kept
        pef_mon_clr = 1'b1;
        do_reset;
        pef_mon_clr = 1'b0;
        run_until_retire(marker, 30000);
        run_until_entry_count(3, 40000);
        run_cycles(300);
        chk("INTEVT = 0x4A0 (PRI)",  dmem[16'h10], 32'h0000_04A0);
        chk("INTEVT2 = 0x4A0 (PRI)", dmem[16'h11], 32'h0000_04A0);
        //1/256 s on the scaled crystal = 128 EXTAL2 periods = 1024 cycles
        chk("PEF period = 1/256 s = 1024 cycles", pef_dt, 32'd1024);
        chk_true("CF set by the 128 Hz count-up", u_dut.u_rtc.cf === 1'b1);
        end_test;
    end
endtask

task automatic test_rtc_alarm;
    integer sent, marker;
    begin
        begin_test("RTC alarm: ENB-gated frame compare (sec+min), ATI 0x480, AF edge discipline");
        eidx = 0;
        emit_wreg_w(32'hFFFF_FEE2, 16'h000D);        // IPRA: RTC level 13
        emit_ldrn(1, 32'hFFFF_FEC0);
        emit_ldrn(3, 32'hFFFF_FED0);
        imem[eidx] = 16'hE008; eidx = eidx + 1;              // RCR2: stop
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        imem[eidx] = 16'hE000; eidx = eidx + 1;
        imem[eidx] = 16'h8012; eidx = eidx + 1;              // sec = 00
        imem[eidx] = 16'hE005; eidx = eidx + 1;
        imem[eidx] = 16'h8014; eidx = eidx + 1;              // min = 05
        imem[eidx] = 16'hE082; eidx = eidx + 1;
        imem[eidx] = 16'h8030; eidx = eidx + 1;              // RSECAR = ENB|02
        imem[eidx] = 16'hE085; eidx = eidx + 1;
        imem[eidx] = 16'h8032; eidx = eidx + 1;              // RMINAR = ENB|05 (AND across ENB set)
        imem[eidx] = 16'hE008; eidx = eidx + 1;
        imem[eidx] = 16'h803C; eidx = eidx + 1;              // RCR1 = AIE
        emit_sr_imask(eidx, 4'h0);
        imem[eidx] = 16'hE00B; eidx = eidx + 1;              // RCR2: RESET | START
        imem[eidx] = 16'h803E; eidx = eidx + 1;
        marker = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;              // MOV #1,R7
        emit_sentinel_loop(eidx, sent);
        emit_handler_tmu(8'hDC, 16'h0008);           // RCR1: AF write-0 clear, AIE kept
        do_reset;
        run_until_retire(marker, 30000);
        chk("no alarm before the match second", entry_count, 32'd0);
        run_until_entry_count(1, 700000);            //sec reaches 02 at 2.0 RTC s
        run_cycles(800);                             //let the handler finish
        chk("INTEVT = 0x480 (ATI)",  dmem[16'h10], 32'h0000_0480);
        chk("INTEVT2 = 0x480 (ATI)", dmem[16'h11], 32'h0000_0480);
        chk_true("AF cleared by the handler",    u_dut.u_rtc.af === 1'b0);
        chk_true("alarm hit at seconds == 02",   u_dut.u_rtc.rseccnt == 7'h02);
        run_cycles(150000);                          //match level persists a full second
        chk("AF is edge-set: no refire inside the matching second", entry_count, 32'd1);
        end_test;
    end
endtask

task automatic test_tmu_rtc_tick;
    integer sent, markerA, markerB, k, tog;
    integer c0;
    logic v;
    begin
        begin_test("TMU on the RTC clock (TPSC=100): tick law, TCLK pad = RTCCLK, RTCEN freeze");
        eidx = 0;
        emit_wreg_w(32'hFFFF_FE9C, 16'h0004);        // TCR0: TPSC=100 (RTC output clock)
        emit_ldrn(1, 32'hFFFF_FE98);
        emit_ldr0(32'd100000);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // TCNT0: large, no underflow
        emit_ldrn(1, 32'hFFFF_FE90);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TOCR: TCOE (RTCCLK -> TCLK pad)
        emit_wreg_w(32'hA400_010E, 16'h2AAA);        // PHCR: PH7 mode 00 (pad grant)
        emit_ldrn(1, 32'hFFFF_FE92);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // TSTR: STR0
        markerA = eidx;
        imem[eidx] = 16'hE701; eidx = eidx + 1;              // MOV #1,R7
        emit_ldrn(4, 32'd2000);                      // ~25k-cycle DT delay (bypass fetch ~12 cyc/iter):
        imem[eidx] = 16'h4410; eidx = eidx + 1;              // DT  R4       the tb law window fits under it
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF  . -3 (loop)
        emit_ldrn(3, 32'hFFFF_FEDE);
        emit_ldr0(32'd1);
        imem[eidx] = 16'h2300; eidx = eidx + 1;              // RCR2: RTCEN=0, START=1 (halt crystal)
        markerB = eidx;
        imem[eidx] = 16'hE702; eidx = eidx + 1;              // MOV #2,R7
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(markerA, 30000);
        c0 = u_dut.u_tmu.tcnt[0];
        run_cycles(4096);                            //multiple of the 16-cycle tick: exact
        chk("RTC tick: 256 ticks / 4096 cycles", c0 - u_dut.u_tmu.tcnt[0], 32'd256);
        chk_true("PTH7 drives (TCOE + PH7 mode 00)", pth_oe[7] === 1'b1);
        tog = 0;
        v = pth_o[7];
        for(k = 0; k < 64; k = k + 1) begin
            @(negedge clk);
            if(pth_o[7] !== v) begin tog = tog + 1; v = pth_o[7]; end
        end
        chk("TCLK pad = RTCCLK (8 toggles / 64 cycles)", tog, 32'd8);
        run_until_retire(markerB, 200000);
        run_cycles(200);                             //RTCEN level crosses in ~2 EXTAL2 edges
        c0 = u_dut.u_tmu.tcnt[0];
        run_cycles(1024);
        chk("RTCEN=0 freezes the divider: TCNT0 holds", u_dut.u_tmu.tcnt[0], c0);
        tog = 0;
        v = pth_o[7];
        for(k = 0; k < 64; k = k + 1) begin
            @(negedge clk);
            if(pth_o[7] !== v) tog = tog + 1;
        end
        chk("RTCCLK pad frozen with RTCEN=0", tog, 32'd0);
        end_test;
    end
endtask



///////////////////////////////////////////////////////////
//////  Interrupt x Machinery Collision (SoC twins of cpu_core_tb 87-89)
////

/*
    The core-level sweeps drove i_INT_VALID directly; here the interrupt is a
    real IRL request: pins -> 2FF sync -> P-phi noise cancel (two consecutive
    equal samples, p.122-123) -> resolver -> INTC/core handshake. The tests
    program FRQCR for P-phi = /1 so the pin-offset sweep keeps a ~2-cycle
    acceptance grain. Pin protocol per the manual: the IRL level must be HELD
    until the interrupt is accepted and handling starts (p.123) - the tb keys
    the release on the clocked ack counter, and the handler carries grace NOPs
    so the released level has drained out of the resolver before RTE re-opens
    the acceptance boundary (a held level across RTE legally RE-ENTERS).
*/

//interrupt handler at VBR+0x600: count the entry in R13 and return. The 10
//grace NOPs cover the pin-release path: 2FF + 2 P-phi cancel + winner reg.
task automatic emit_count_handler;
    integer k;
    begin
        imem['h300] = 16'h7D01;                           // ADD   #1,R13
        for(k = 0; k < 10; k = k + 1) begin
            imem['h301 + k] = 16'h0009;                   // grace NOPs
        end
        imem['h30B] = 16'h002B;                           // RTE
        imem['h30C] = 16'h0009;                           //   delay slot
    end
endtask

/*
    Twin of cpu_core_tb test 87 on the real BSC: a P0 write-back loop walks
    8 same-set SDRAM lines (write-allocate fill + dirty-victim drain bursts)
    plus a cold-miss load walk, with the LOOP CODE lines sharing sets 0x20/
    0x21 so the data walks evict them (I-misses too) - all while auto-refresh
    (RTCOR=4) churns the SDRAM engine. An IRL-13 request lands at every
    offset; acceptance mid-fill/drain/refresh must be transparent.
*/
task automatic test_int_sdram_sweep;
    integer off, w, k, e0, cb0, sb0;
    begin
        begin_test("Interrupt vs SDRAM fill/drain/refresh: acceptance mid-machinery is transparent");
        cb0 = entry_cache_busy;
        sb0 = entry_sdram_busy;
        eidx = 0;
        emit_sdram_init(16'h503C, 16'hFFDF, 32'hFFFF_E880);  //AP CL2 + RFSH=1
        emit_wreg_w(32'hFFFF_FF72, 16'hA504);        // RTCOR = 4: refresh every ~16 bus cyc
        emit_wreg_w(32'hFFFF_FF6E, 16'hA508);        // RTCSR: CKS=001 (bus/4)
        emit_wreg_w(32'hFFFF_FF80, 16'h0100);        // FRQCR: P-phi /1 (IRL sampling grain)
        imem[eidx] = 16'hE0EC; eidx = eidx + 1;      // MOV   #0xEC,R0    ; CCR
        imem[eidx] = 16'hE109; eidx = eidx + 1;      // MOV   #9,R1       ; CE|CF (WT=0: P0 WB)
        imem[eidx] = 16'h2012; eidx = eidx + 1;      // MOV.L R1,@R0
        emit_sr_imask(eidx, 4'h0);                   // BL=0, IMASK=0
        imem[eidx] = 16'hED00; eidx = eidx + 1;      // MOV   #0,R13      ; handler counter
        emit_ldrn(1, 32'h0C00_1200);                 // dirty walk base (P0 SDRAM, set 0x20)
        imem[eidx] = 16'h6713; eidx = eidx + 1;      // MOV   R1,R7
        imem[eidx] = 16'h7710; eidx = eidx + 1;      // ADD   #0x10,R7    ; cold walk (set 0x21)
        imem[eidx] = 16'hE310; eidx = eidx + 1;      // MOV   #0x10,R3
        imem[eidx] = 16'h4318; eidx = eidx + 1;      // SHLL8 R3          ; 0x1000 same-set stride
        imem[eidx] = 16'hE508; eidx = eidx + 1;      // MOV   #8,R5       ; 8 lines > 4 ways
        emit_ldrn(2, 32'h0000_0200);
        imem[eidx] = 16'h422B; eidx = eidx + 1;      // JMP   @R2         ; P0 cacheable loop
        imem[eidx] = 16'h0009; eidx = eidx + 1;      //   delay slot
        chk_true("setup fits below the P0 loop", eidx <= 'h100);
        //loop at P0 0x200 (imem 'h100): its own lines live in sets 0x20/0x21
        imem['h100] = 16'hE200;  // MOV   #0,R2    ; anchor (loop is starting)
        imem['h101] = 16'hE600;  // MOV   #0,R6
        imem['h102] = 16'h7201;  // ADD   #1,R2    ; loop head
        imem['h103] = 16'h2122;  // MOV.L R2,@R1   ; write-allocate miss (dirty)
        imem['h104] = 16'h6412;  // MOV.L @R1,R4   ; load-back on the fresh line
        imem['h105] = 16'h313C;  // ADD   R3,R1    ; next same-set line
        imem['h106] = 16'h6972;  // MOV.L @R7,R9   ; COLD-MISS load (set 0x21)
        imem['h107] = 16'h373C;  // ADD   R3,R7
        imem['h108] = 16'h4510;  // DT    R5
        imem['h109] = 16'h8FF7;  // BF/S  loop head ; delayed
        imem['h10A] = 16'h7601;  // ADD   #1,R6    ;   delay slot
        imem['h10B] = 16'h0009;  // sentinel
        imem['h10C] = 16'hAFFE;  // guard
        imem['h10D] = 16'h0009;
        emit_count_handler;
        for(off = 0; off < 120; off = off + 1) begin
            //fresh SDRAM content: zeroed store lines, signed cold-load lines
            for(k = 0; k < 8; k = k + 1) begin
                sdram_poke(32'h0C00_1200 + k*32'h1000, 32'd0);
                sdram_poke(32'h0C00_1210 + k*32'h1000, 32'hC01D_0000 + k);
            end
            do_reset;
            e0 = test_errors;
            run_until_retire('h100, 60000);          //loop is starting
            repeat(off) @(posedge clk);
            irq_pin[3:0] = 4'b0010;                  //IRL level 13 (table 6.3, p.122)
            w = 0;
            while(int_ack_cnt == 0 && w < 60000) begin @(posedge clk); w = w + 1; end
            @(posedge clk);
            irq_pin[3:0] = 4'b1111;                  //release after acceptance (p.123)
            run_until_retire('h30B, 60000);          //handler RTE retired
            run_until_retire('h10B, 60000);          //main line completed
            chk($sformatf("off=%0d: exactly one ack", off), int_ack_cnt, 32'd1);
            chk($sformatf("off=%0d: exactly one entry", off), entry_count, 32'd1);
            chk($sformatf("off=%0d: handler ran once", off), gpr(13), 32'd1);
            chk($sformatf("off=%0d: accumulator transparent", off), gpr(2), 32'd8);
            chk($sformatf("off=%0d: load-back transparent", off), gpr(4), 32'd8);
            chk($sformatf("off=%0d: cold-miss data transparent", off), gpr(9), 32'hC01D_0007);
            chk($sformatf("off=%0d: slot count transparent", off), gpr(6), 32'd8);
            chk($sformatf("off=%0d: loop count consumed", off), gpr(5), 32'd0);
            chk_true($sformatf("off=%0d: no spurious exception", off), !exc_seen);
            chk($sformatf("off=%0d: INTEVT = IRL code", off), intevt_o, 32'h0000_0240);
            chk($sformatf("off=%0d: INTEVT2 = IRL code", off), intevt2_reg, 32'h0000_0240);
            if(test_errors != e0)
                $display("      [dbg] EXPEVT=%08h INTEVT=%08h entries=%0d ack=%0d R13=%0d R2=%0d R5=%0d R6=%0d cst=%0d est=%0d",
                         expevt_o, intevt_o, entry_count, int_ack_cnt,
                         gpr(13), gpr(2), gpr(5), gpr(6), cache_st, bsc_est);
        end
        //coverage verdict: entries really landed inside machinery windows. The
        //cache count is intrinsically small at the SoC - excursions stall the
        //whole pipe (no retire boundaries inside), so only the excursion-entry
        //edge can coincide with an acceptance, and the IRL resolver quantizes
        //arrivals to the P-phi grid. The SDRAM-engine count is the rich gate:
        //wb-buffer drains and refreshes run under a retiring pipe.
        $display("      entry-vs-machinery coverage: %0d cache-busy / %0d SDRAM-engine-busy entries",
                 entry_cache_busy - cb0, entry_sdram_busy - sb0);
        chk_true("entries landed at excursion edges (cache)", (entry_cache_busy - cb0) > 2);
        chk_true("entries landed mid-cycle (SDRAM engine)",   (entry_sdram_busy - sb0) > 30);
        end_test;
    end
endtask

/*
    Twin of cpu_core_tb test 88 on the real BSC: two back-to-back TAS.B locked
    RMW pairs, phase 0 on SDRAM (engine lock path), phase 1 on an ordinary
    handshake area stretched by d_latency (front-end lock path). An IRL-13
    request lands at every offset: acceptance must never split a pair (the
    MA-inflight defer), T flows exactly once, and the IBUS1_CORE lock-pairing
    counters must show exactly 2 reads / 2 writes per run.
*/
task automatic test_int_tas_sweep;
    integer ph, off, w, e0, marker, sent;
    begin
        begin_test("Interrupt vs TAS.B: locked RMW indivisible on both BSC paths (offset sweep)");
        for(ph = 0; ph < 2; ph = ph + 1) begin
            init_knobs;
            clear_imem;
            clear_dmem;
            if(ph == 1) d_latency = 8;               //stretch the ordinary locked pair
            eidx = 0;
            if(ph == 0) emit_sdram_init(16'h5038, 16'hFFDF, 32'hFFFF_E880);
            emit_wreg_w(32'hFFFF_FF80, 16'h0100);    // FRQCR: P-phi /1 (IRL sampling grain)
            emit_sr_imask(eidx, 4'h0);               // BL=0, IMASK=0
            imem[eidx] = 16'hED00; eidx = eidx + 1;  // MOV   #0,R13
            emit_ldrn(1, (ph == 0) ? 32'hAC00_0070 : 32'hA000_0070);  //P2 uncached byte
            marker = eidx;
            imem[eidx] = 16'hE400; eidx = eidx + 1;  // MOV   #0,R4       ; anchor
            imem[eidx] = 16'hE600; eidx = eidx + 1;  // MOV   #0,R6
            imem[eidx] = 16'h0009; eidx = eidx + 1;  // offset-window NOPs
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            imem[eidx] = 16'h411B; eidx = eidx + 1;  // TAS.B @R1: pair #1 (byte 0 -> T=1, set 0x80)
            imem[eidx] = 16'h0429; eidx = eidx + 1;  // MOVT  R4
            imem[eidx] = 16'h411B; eidx = eidx + 1;  // TAS.B @R1: pair #2 (byte 0x80 -> T=0)
            imem[eidx] = 16'h0629; eidx = eidx + 1;  // MOVT  R6
            sent = eidx;
            imem[eidx] = 16'h0009; eidx = eidx + 1;  // sentinel
            imem[eidx] = 16'hAFFE; eidx = eidx + 1;  // guard
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            emit_count_handler;
            for(off = 0; off < 45; off = off + 1) begin
                do_reset;
                e0 = test_errors;
                if(ph == 0) sdram_poke(32'h0C00_0070, 32'h00FF_00FF);  //byte@0x70 = 0x00
                else        dmem['h1C] = 32'h00FF_00FF;
                run_until_retire(marker, 60000);
                repeat(off) @(posedge clk);
                irq_pin[3:0] = 4'b0010;              //IRL level 13
                w = 0;
                while(int_ack_cnt == 0 && w < 60000) begin @(posedge clk); w = w + 1; end
                @(posedge clk);
                irq_pin[3:0] = 4'b1111;              //release after acceptance (p.123)
                run_until_retire('h30B, 60000);      //handler RTE retired
                run_until_retire(sent, 60000);
                chk($sformatf("ph=%0d off=%0d: TAS#1 saw the pre-RMW byte once (T=1)", ph, off), gpr(4), 32'd1);
                chk($sformatf("ph=%0d off=%0d: TAS#2 saw bit7 already set (T=0)", ph, off), gpr(6), 32'd0);
                if(ph == 0)
                    chk($sformatf("ph=%0d off=%0d: byte mutated exactly once", ph, off),
                        sdram_peek(32'h0C00_0070), 32'h80FF_00FF);
                else
                    chk($sformatf("ph=%0d off=%0d: byte mutated exactly once", ph, off),
                        dmem['h1C], 32'h80FF_00FF);
                chk($sformatf("ph=%0d off=%0d: locked reads paired", ph, off), locked_rd_cnt, 32'd2);
                chk($sformatf("ph=%0d off=%0d: locked writes paired", ph, off), locked_wr_cnt, 32'd2);
                chk($sformatf("ph=%0d off=%0d: exactly one entry", ph, off), entry_count, 32'd1);
                chk($sformatf("ph=%0d off=%0d: handler ran once", ph, off), gpr(13), 32'd1);
                chk_true($sformatf("ph=%0d off=%0d: no spurious exception", ph, off), !exc_seen);
                if(test_errors != e0)
                    $display("      [dbg] EXPEVT=%08h INTEVT=%08h entries=%0d ack=%0d lockR=%0d lockW=%0d cst=%0d est=%0d",
                             expevt_o, intevt_o, entry_count, int_ack_cnt,
                             locked_rd_cnt, locked_wr_cnt, cache_st, bsc_est);
            end
        end
        end_test;
    end
endtask

/*
    Twin of cpu_core_tb test 89 (minus the bus-fault flavor: no pin-level
    mechanism can fault a beat on this board - that flavor stays core-only).
    A synchronous event at F (illegal / misaligned address / TRAPA) collides
    with a pending IRL-13 request at every offset, over two ordinary-bus
    latencies (i_WAIT_n stretch). Whatever the order: one ack, one exception,
    EXPEVT/INTEVT never mix, and the pre-decrement store runs exactly once.
*/
task automatic test_exc_int_collision_soc;
    integer f, ws, off, w, e0, marker, sent;
    logic [15:0] f_op;
    logic [31:0] exp_ev;
    logic        skip;
    string       fname;
    begin
        begin_test("Exception x interrupt collision: both once, never mixed (3 flavors x offsets)");
        for(f = 0; f < 3; f = f + 1) begin
            case(f)
                0:       begin fname = "illegal"; f_op = 16'hF000; exp_ev = 32'h0000_0180; skip = 1'b1; end
                1:       begin fname = "address"; f_op = 16'h6402; exp_ev = 32'h0000_00E0; skip = 1'b1; end
                default: begin fname = "trapa";   f_op = 16'hC342; exp_ev = 32'h0000_0160; skip = 1'b0; end
            endcase
        for(ws = 0; ws <= 5; ws = ws + 5) begin
            init_knobs;
            clear_imem;
            clear_dmem;
            wait_stretch = ws;                       //i_WAIT_n latency axis
            eidx = 0;
            emit_wreg_w(32'hFFFF_FF66, 16'hFFF9);    // WCR2: A0W=001 (1 wait, pin sampled)
            emit_wreg_w(32'hFFFF_FF80, 16'h0100);    // FRQCR: P-phi /1 (IRL sampling grain)
            emit_sr_imask(eidx, 4'h0);               // BL=0, IMASK=0
            imem[eidx] = 16'hED00; eidx = eidx + 1;  // MOV   #0,R13   ; interrupt counter
            imem[eidx] = 16'hEB00; eidx = eidx + 1;  // MOV   #0,R11   ; exception counter
            imem[eidx] = 16'hE200; eidx = eidx + 1;  // MOV   #0,R2
            imem[eidx] = 16'hE001; eidx = eidx + 1;  // MOV   #1,R0    ; odd EA (ADDRESS flavor)
            imem[eidx] = 16'hE304; eidx = eidx + 1;  // MOV   #4,R3
            marker = eidx;
            imem[eidx] = 16'h4318; eidx = eidx + 1;  // SHLL8 R3       ; 0x400 + anchor
            imem[eidx] = 16'h7201; eidx = eidx + 1;  // ADD   #1,R2
            imem[eidx] = 16'h7201; eidx = eidx + 1;  // ADD   #1,R2
            imem[eidx] = f_op;     eidx = eidx + 1;  // F: the flavor's faulting/trapping op
            imem[eidx] = 16'h7201; eidx = eidx + 1;  // G: younger ADD
            imem[eidx] = 16'h2326; eidx = eidx + 1;  // MOV.L R2,@-R3  ; double-execution detector
            imem[eidx] = 16'h6432; eidx = eidx + 1;  // MOV.L @R3,R4   ; load-back of the store
            sent = eidx;
            imem[eidx] = 16'h0009; eidx = eidx + 1;  // sentinel
            imem[eidx] = 16'hAFFE; eidx = eidx + 1;  // guard
            imem[eidx] = 16'h0009; eidx = eidx + 1;
            //general-exception handler (VBR+0x100): count, skip F when it faulted
            if(skip) begin
                imem['h80] = 16'h0942;  // STC SPC,R9
                imem['h81] = 16'h7902;  // ADD #2,R9
                imem['h82] = 16'h494E;  // LDC R9,SPC   ; resume past F
                imem['h83] = 16'h7B01;  // ADD #1,R11
                imem['h84] = 16'h002B;  // RTE
                imem['h85] = 16'h0009;  //   delay slot
            end
            else begin
                imem['h80] = 16'h7B01;  // ADD #1,R11   ; TRAPA: SPC is already F+2
                imem['h81] = 16'h002B;  // RTE
                imem['h82] = 16'h0009;  //   delay slot
            end
            emit_count_handler;
            for(off = 0; off < 25; off = off + 1) begin
                dmem['hFF] = 32'd0;                  //pre-dec target (0x3FC) fresh per run
                do_reset;
                e0 = test_errors;
                run_until_retire(marker, 60000);
                repeat(off) @(posedge clk);
                irq_pin[3:0] = 4'b0010;              //IRL level 13
                w = 0;
                while(int_ack_cnt == 0 && w < 60000) begin @(posedge clk); w = w + 1; end
                @(posedge clk);
                irq_pin[3:0] = 4'b1111;              //release after acceptance (p.123)
                w = 0;                               //the synchronous event must also fire
                while(!exc_seen && !trapa_seen && w < 60000) begin @(posedge clk); w = w + 1; end
                run_until_retire('h30B, 60000);      //interrupt handler returned
                run_until_retire(sent, 60000);       //main line completed
                run_cycles(30);
                chk($sformatf("%s ws=%0d off=%0d: interrupt ack'd exactly once", fname, ws, off), int_ack_cnt, 32'd1);
                chk($sformatf("%s ws=%0d off=%0d: interrupt handler ran once", fname, ws, off), gpr(13), 32'd1);
                chk($sformatf("%s ws=%0d off=%0d: exception handler ran once", fname, ws, off), gpr(11), 32'd1);
                chk($sformatf("%s ws=%0d off=%0d: exactly two entries", fname, ws, off), entry_count, 32'd2);
                chk($sformatf("%s ws=%0d off=%0d: INTEVT = IRL code", fname, ws, off), intevt_o, 32'h0000_0240);
                chk($sformatf("%s ws=%0d off=%0d: INTEVT2 = IRL code", fname, ws, off), intevt2_reg, 32'h0000_0240);
                chk($sformatf("%s ws=%0d off=%0d: EXPEVT", fname, ws, off), expevt_o, exp_ev);
                chk($sformatf("%s ws=%0d off=%0d: mainline result transparent", fname, ws, off), gpr(2), 32'd3);
                chk($sformatf("%s ws=%0d off=%0d: pre-dec store executed once", fname, ws, off), gpr(3), 32'h0000_03FC);
                chk($sformatf("%s ws=%0d off=%0d: stored word load-back", fname, ws, off), gpr(4), 32'd3);
                chk($sformatf("%s ws=%0d off=%0d: stored word in memory", fname, ws, off), dmem['hFF], 32'd3);
                if(f == 1) chk($sformatf("%s ws=%0d off=%0d: TEA = misaligned EA", fname, ws, off), tea_o, 32'd1);
                if(f == 2) begin
                    chk_true($sformatf("%s ws=%0d off=%0d: TRAPA pulsed", fname, ws, off), trapa_seen);
                    chk($sformatf("%s ws=%0d off=%0d: TRA = imm<<2", fname, ws, off), tra_o, 32'h0000_0108);
                end
                if(test_errors != e0)
                    $display("      [dbg] EXPEVT=%08h INTEVT=%08h TEA=%08h entries=%0d ack=%0d R2=%0d R11=%0d R13=%0d cst=%0d est=%0d",
                             expevt_o, intevt_o, tea_o, entry_count, int_ack_cnt,
                             gpr(2), gpr(11), gpr(13), cache_st, bsc_est);
            end
        end
        end
        end_test;
    end
endtask

//suite-wide verdicts on the passive checkers - exercised gates make a silent
//(vacuous) pass fail loudly, the cpu_core_tb pattern.
task automatic test_boundary_summary;
    begin
        begin_test("SoC boundary contracts: lock pairing + ack law exercised, zero violations");
        $display("      locked pairs checked: %0d   acks checked: %0d", lock_pairs_checked, ack_checks);
        chk_true("locked-pair law exercised (>100 pairs)", lock_pairs_checked > 100);
        chk("locked-pair violations",       lock_pair_viol[31:0], 32'd0);
        chk_true("interrupt-ack law exercised (>200 acks)", ack_checks > 200);
        chk("ack-without-entry / INTEVT violations", ack_viol[31:0], 32'd0);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  DMAC Register Block + CMT (session 5, phase 1)
////

/*
    Section 11 register laws (tables 11.2/11.7, pp.331-344, 377-380).
    Phase 1 covers only the register face + CMT counter; transfers, DEI
    interrupts (need a hardware TE set) and DREQ pins arrive with the
    engine phases. All accesses ride the P2 window 0xA4000020-77 through
    the BSC's P-bus bridge (shadow-invariant area-1 decode).
*/

task automatic test_dmac_channel_regs;
    integer idx, sent;
    begin
        begin_test("DMAC channel quads: lanes, DMATCR mask, per-channel CHCR bits, decode holes");
        eidx = 0;
        emit_ldrn(1, 32'hA400_0020);                 // SAR0
        imem[eidx] = 16'h6312; eidx = eidx + 1;              // MOV.L @R1,R3      ; reset 0 (sim; arch undefined)
        emit_ldr0(32'h1234_5678);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6412; eidx = eidx + 1;              // MOV.L @R1,R4      ; long readback
        emit_ldr0(32'h0000_AAAA);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; upper half (big-endian)
        imem[eidx] = 16'h6512; eidx = eidx + 1;              // MOV.L @R1,R5      ; lower half retained
        emit_ldrn(1, 32'hA400_0022);                 // SAR0 lower half
        emit_ldr0(32'h0000_BBBB);
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1
        emit_ldrn(1, 32'hA400_0020);
        imem[eidx] = 16'h6712; eidx = eidx + 1;              // MOV.L @R1,R7      ; both halves written
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2    ; mailbox base
        emit_ldrn(1, 32'hA400_0028);                 // DMATCR0
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0      ; 0xFFFFFFFF
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0: bits 31:24 masked
        emit_ldrn(1, 32'hA400_002C);                 // CHCR0
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // MOV.L R0,@(4,R2)  ; mb1: RL/AM/AL/DS live, no TE
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0,R0
        imem[eidx] = 16'h8013; eidx = eidx + 1;              // MOV.B R0,@(3,R1)  ; byte lane clears CHCR0[7:0]
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // MOV.L R0,@(8,R2)  ; mb2: upper bytes retained
        emit_ldrn(1, 32'hA400_003C);                 // CHCR1
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // MOV.L R0,@(12,R2) ; mb3
        emit_ldrn(1, 32'hA400_004C);                 // CHCR2
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // MOV.L R0,@(16,R2) ; mb4: RO only
        emit_ldrn(1, 32'hA400_005C);                 // CHCR3
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1205; eidx = eidx + 1;              // MOV.L R0,@(20,R2) ; mb5: DI only
        emit_ldrn(1, 32'hA400_0068);                 // 0x62-6F hole: undecoded (11.6 note 11)
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1206; eidx = eidx + 1;              // MOV.L R0,@(24,R2) ; mb6 = 0
        emit_ldrn(1, 32'hA400_0078);                 // fringe just past the CMT
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1207; eidx = eidx + 1;              // MOV.L R0,@(28,R2) ; mb7 = 0
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("SAR0 reset 0 (sim; arch undefined)",  gpr(3), 32'h0000_0000);
        chk("SAR0 long R/W",                       gpr(4), 32'h1234_5678);
        chk("SAR0 word @+0: lower half retained",  gpr(5), 32'hAAAA_5678);
        chk("SAR0 word @+2: upper half retained",  gpr(7), 32'hAAAA_BBBB);
        chk("DMATCR0 bits 31:24 read 0 / WI",      dmem[16'h10], 32'h00FF_FFFF);
        chk("CHCR0 all-ones: RL/AM/AL/DS, no TE",  dmem[16'h11], 32'h0007_FF7D);
        chk("CHCR0 byte lane: 7:0 clear only",     dmem[16'h12], 32'h0007_FF00);
        chk("CHCR1 all-ones mask == CHCR0",        dmem[16'h13], 32'h0007_FF7D);
        chk("CHCR2 all-ones: RO only",             dmem[16'h14], 32'h0008_FF3D);
        chk("CHCR3 all-ones: DI only",             dmem[16'h15], 32'h0010_FF3D);
        chk("0x62-6F hole reads 0",                dmem[16'h16], 32'h0000_0000);
        chk("0x78 fringe reads 0",                 dmem[16'h17], 32'h0000_0000);
        end_test;
    end
endtask

task automatic test_dmac_dmaor_cmt_regs;
    integer idx, sent, c0;
    begin
        begin_test("DMAOR flag/lane laws + CMT reset values + CMCNT0 tick rate (P-phi/8)");
        eidx = 0;
        emit_ldrn(1, 32'hA400_0060);                 // DMAOR
        imem[eidx] = 16'h6311; eidx = eidx + 1;              // MOV.W @R1,R3      ; reset 0x0000
        imem[eidx] = 16'hE0FF; eidx = eidx + 1;              // MOV   #-1,R0
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; all-ones word
        imem[eidx] = 16'h6411; eidx = eidx + 1;              // MOV.W @R1,R4      ; AE/NMIF refuse write-1
        emit_ldrn(1, 32'hA400_0061);
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0,R0
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; low byte: DME clears
        emit_ldrn(1, 32'hA400_0060);
        imem[eidx] = 16'h6511; eidx = eidx + 1;              // MOV.W @R1,R5      ; PR retained
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0,R0
        imem[eidx] = 16'h2100; eidx = eidx + 1;              // MOV.B R0,@R1      ; high byte: PR clears
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2    ; mailbox base
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0 = 0
        emit_ldrn(1, 32'hA400_0076);                 // CMCOR0
        imem[eidx] = 16'h6711; eidx = eidx + 1;              // MOV.W @R1,R7      ; reset 0xFFFF (sign-ext)
        emit_wreg_w(32'hA400_0072, 16'h0041);        // CMCSR0: spare bit 6 + CKS=01 (P-phi/8)
        emit_ldrn(1, 32'hA400_0072);
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // MOV.L R0,@(4,R2)  ; mb1 = 0x0041
        emit_wreg_w(32'hA400_0070, 16'h0001);        // CMSTR: STR0
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("DMAOR reset 0x0000",                  gpr(3), 32'h0000_0000);
        chk("DMAOR all-ones: PR+DME only stick",   gpr(4), 32'h0000_0301);
        chk("DMAOR byte @+1: DME clears, PR held", gpr(5), 32'h0000_0300);
        chk("DMAOR byte @+0: PR clears",           dmem[16'h10], 32'h0000_0000);
        chk("CMCOR0 reset 0xFFFF",                 gpr(7), 32'hFFFF_FFFF);
        chk("CMCSR0 spare+CKS readback",           dmem[16'h11], 32'h0000_0041);
        //tick-rate law: P-phi/8 = one count / 32 core cycles at the FRQCR
        //reset ratio; 4096 is a multiple, so the delta is exact
        c0 = u_dut.u_dmac.cmcnt;
        run_cycles(4096);
        chk("CMCNT0 P-phi/8: 128 counts / 4096 cycles", u_dut.u_dmac.cmcnt - c0, 32'd128);
        end_test;
    end
endtask

task automatic test_dmac_cmt_match;
    integer idx, sent, c0;
    begin
        begin_test("CMT compare match: CMCNT0 wrap, CMF set + write-0 clear protocol, STR0 halt");
        eidx = 0;
        emit_wreg_w(32'hA400_0076, 16'h001F);        // CMCOR0 = 31: match / 512 cycles (P-phi/4)
        emit_wreg_w(32'hA400_0072, 16'h0000);        // CMCSR0: CKS=00 (P-phi/4)
        emit_wreg_w(32'hA400_0070, 16'h0001);        // CMSTR: STR0
        emit_ldrn(3, 32'd400);                       // delay > one match period (~4 cyc/iter)
        imem[eidx] = 16'h4310; eidx = eidx + 1;              // DT    R3
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF    .-1 (loop)
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2    ; mailbox base
        emit_ldrn(1, 32'hA400_0072);                 // CMCSR0
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0 = CMF set
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0,R0
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; CMF write-0 clear
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // MOV.L R0,@(4,R2)  ; mb1 = cleared
        emit_ldrn(1, 32'hA400_0074);                 // CMCNT0
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // MOV.L R0,@(8,R2)  ; mb2 = wrapped count
        emit_wreg_w(32'hA400_0070, 16'h0000);        // CMSTR: STR0 off
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        chk("CMF set after a match period",     dmem[16'h10], 32'h0000_0080);
        chk("CMF write-0 clears",               dmem[16'h11], 32'h0000_0000);
        chk_true("CMCNT0 wrapped below CMCOR0", dmem[16'h12] <= 32'h0000_001F);
        //STR0 = 0 freezes the counter dead
        c0 = u_dut.u_dmac.cmcnt;
        run_cycles(1024);
        chk("STR0 off: CMCNT0 frozen", u_dut.u_dmac.cmcnt, c0[15:0]);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  DMAC Transfers (session 5, phase 3)
////

/*
    Auto-request dual-direct engine (section 11.3): unit = read at SAR
    then write at DAR through the on-chip arbiter and the BSC ordinary
    bus (dmem region). DMA source/destination addresses are PHYSICAL
    area-0 values; the CPU seeds/verifies through P2. Mailboxes: the
    shared DEI handler uses mb0 (INTEVT2) and mb1 (CHCR readback).
*/

//DEI handler at VBR+0x600: mb0 = INTEVT2, mb1 = CHCR readback (TE visible),
//then CHCR <- {16'd0, clr_val} to drop the DE/IE level (write-1 keeps TE)
task automatic emit_handler_dmac(input logic [7:0] chcr_off, input logic [15:0] clr_val);
    integer idx, k;
    begin
        idx = 'h300;
        imem[idx] = 16'hE240; idx = idx + 1;              // MOV   #0x40,R2    ; mailbox base
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'h6432; idx = idx + 1;              // MOV.L @R3,R4      ; INTEVT2
        imem[idx] = 16'h2242; idx = idx + 1;              // MOV.L R4,@R2      ; mb0
        imem[idx] = 16'h6033; idx = idx + 1;              // MOV   R3,R0
        imem[idx] = 16'hCB00 | chcr_off; idx = idx + 1;   // OR    #off,R0     ; CHCRn address
        imem[idx] = 16'h6103; idx = idx + 1;              // MOV   R0,R1
        imem[idx] = 16'h6512; idx = idx + 1;              // MOV.L @R1,R5
        imem[idx] = 16'h1251; idx = idx + 1;              // MOV.L R5,@(4,R2)  ; mb1 = CHCR (TE set)
        imem[idx] = 16'hE000 | clr_val[15:8]; idx = idx + 1; // MOV #hi,R0
        imem[idx] = 16'h4018; idx = idx + 1;              // SHLL8 R0
        imem[idx] = 16'hCB00 | clr_val[7:0]; idx = idx + 1;  // OR  #lo,R0
        imem[idx] = 16'h2102; idx = idx + 1;              // MOV.L R0,@R1      ; drop DE/IE level
        for(k = 0; k < 8; k = k + 1) begin
            imem[idx] = 16'h0009; idx = idx + 1;          // grace NOPs (resolver settle)
        end
        imem[idx] = 16'h002B; idx = idx + 1;              // RTE
        imem[idx] = 16'h0009; idx = idx + 1;              // NOP (delay slot)
    end
endtask

//seed one longword at a P2 address (CPU store; DMA later reads it raw)
task automatic emit_poke_l(input logic [31:0] addr, input logic [31:0] v);
    begin
        emit_ldrn(1, addr);
        emit_ldr0(v);
        imem[eidx] = 16'h2102; eidx = eidx + 1;              // MOV.L R0,@R1
    end
endtask

//poll CHCRn (address in R1) until TE (bit 1) sets
task automatic emit_poll_te(input logic [31:0] chcr_addr);
    begin
        emit_ldrn(1, chcr_addr);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'hC802; eidx = eidx + 1;              // TST   #2,R0       ; T = !TE
        imem[eidx] = 16'h89FC; eidx = eidx + 1;              // BT    .-2 (poll)
    end
endtask

task automatic test_dmac_auto_long;
    integer idx, sent;
    begin
        begin_test("DMAC auto-request: 4 longs mem->mem cycle-steal, end regs, TE, DEI INTEVT2=0x800");
        eidx = 0;
        emit_poke_l(32'hA000_0100, 32'hC0FF_EE01);   //source block
        emit_poke_l(32'hA000_0104, 32'hC0FF_EE02);
        emit_poke_l(32'hA000_0108, 32'hC0FF_EE03);
        emit_poke_l(32'hA000_010C, 32'hC0FF_EE04);
        emit_wreg_w(32'hA400_001A, 16'hD000);        // IPRE: DMAC level 13
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0 (physical)
        emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0004);   // DMATCR0 = 4
        emit_poke_l(32'hA400_002C, 32'h0000_5415);   // CHCR0: inc/inc auto long cs IE DE
        emit_sr_imask(eidx, 4'h0);
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME last (11.6 note 6)
        emit_sentinel_loop(eidx, sent);
        emit_handler_dmac(8'h2C, 16'h0000);          // mb0/mb1 + CHCR0 <- 0
        do_reset;
        run_until_entry_count(1, 30000);
        run_cycles(300);
        chk("dst[0]",                       dmem[16'h80], 32'hC0FF_EE01);
        chk("dst[1]",                       dmem[16'h81], 32'hC0FF_EE02);
        chk("dst[2]",                       dmem[16'h82], 32'hC0FF_EE03);
        chk("dst[3]",                       dmem[16'h83], 32'hC0FF_EE04);
        chk("DEI0 INTEVT2 = 0x800",         dmem[16'h10], 32'h0000_0800);
        chk("CHCR0 in handler: TE set",     dmem[16'h11], 32'h0000_5417);
        chk("SAR0 end",  u_dut.u_dmac.u_ch0.sar, 32'h0000_0110);
        chk("DAR0 end",  u_dut.u_dmac.u_ch0.dar, 32'h0000_0210);
        chk("DMATCR0 end", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'h0000_0000);
        end_test;
    end
endtask

task automatic test_dmac_sizes_lanes;
    integer idx, sent;
    begin
        begin_test("DMAC sizes/lanes: byte inc->dec scramble + word dec->fixed, end registers");
        eidx = 0;
        emit_poke_l(32'hA000_0140, 32'h0123_4567);   //source bytes/words
        emit_poke_l(32'hA000_0144, 32'h89AB_CDEF);
        emit_poke_l(32'hA400_0030, 32'h0000_0141);   // SAR1: byte offset 1
        emit_poke_l(32'hA400_0034, 32'h0000_01F3);   // DAR1: byte offset 3, decrementing
        emit_poke_l(32'hA400_0038, 32'h0000_0004);   // DMATCR1 = 4 bytes
        emit_poke_l(32'hA400_003C, 32'h0000_9401);   // CHCR1: dec/inc auto byte cs DE
        emit_poke_l(32'hA400_0040, 32'h0000_0146);   // SAR2: word offset 2
        emit_poke_l(32'hA400_0044, 32'h0000_01E0);   // DAR2: fixed word
        emit_poke_l(32'hA400_0048, 32'h0000_0002);   // DMATCR2 = 2 words
        emit_poke_l(32'hA400_004C, 32'h0000_2409);   // CHCR2: fixed/dec auto word cs DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME (starts both)
        emit_poll_te(32'hA400_003C);                 // wait ch1
        emit_poll_te(32'hA400_004C);                 // wait ch2
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2      ; mailbox base
        emit_ldrn(1, 32'hA400_003C);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // MOV.L R0,@R2      ; mb0 = CHCR1
        emit_ldrn(1, 32'hA400_0030);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1 = SAR1
        emit_ldrn(1, 32'hA400_0034);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // mb2 = DAR1
        emit_ldrn(1, 32'hA400_0040);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // mb3 = SAR2
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        //bytes 0x23,0x45,0x67,0x89 land at 0x1F3,0x1F2,0x1F1,0x1F0 (dec)
        chk("byte scramble dst",            dmem[16'h7C], 32'h8967_4523);
        //words 0xCDEF then 0x89AB both hit fixed 0x1E0 (upper lane): last wins
        chk("word fixed-dst last wins",     dmem[16'h78], 32'h89AB_0000);
        chk("CHCR1 TE readback",            dmem[16'h10], 32'h0000_9403);
        chk("SAR1 end (+4 bytes)",          dmem[16'h11], 32'h0000_0145);
        chk("DAR1 end (-4 bytes)",          dmem[16'h12], 32'h0000_01EF);
        chk("SAR2 end (-2 words)",          dmem[16'h13], 32'h0000_0142);
        end_test;
    end
endtask

task automatic test_dmac_priority;
    integer idx, sent, c;
    begin
        begin_test("DMAC fixed priority PR=10: ch2 strictly before ch1, DMATCR1 untouched at te2");
        //the CPU only configures and parks in the sentinel; strict order is
        //observed at the te2-set EDGE from the tb (a polled read is far too
        //slow relative to a transfer unit under bus contention)
        eidx = 0;
        emit_poke_l(32'hA000_0100, 32'hAA00_0001);
        emit_poke_l(32'hA000_0104, 32'hAA00_0002);
        emit_poke_l(32'hA000_0108, 32'hAA00_0003);
        emit_poke_l(32'hA000_010C, 32'hAA00_0004);
        emit_poke_l(32'hA400_0040, 32'h0000_0100);   // SAR2
        emit_poke_l(32'hA400_0044, 32'h0000_0300);   // DAR2
        emit_poke_l(32'hA400_0048, 32'h0000_0004);   // DMATCR2 = 4
        emit_poke_l(32'hA400_004C, 32'h0000_5411);   // CHCR2: inc/inc auto long cs DE
        emit_poke_l(32'hA400_0030, 32'h0000_0180);   // SAR1
        emit_poke_l(32'hA400_0034, 32'h0000_0340);   // DAR1
        emit_poke_l(32'hA400_0038, 32'h0000_0020);   // DMATCR1 = 32
        emit_poke_l(32'hA400_003C, 32'h0000_5411);   // CHCR1: same, lower priority at PR=10
        emit_wreg_w(32'hA400_0060, 16'h0201);        // DMAOR: PR=10 (2>0>1>3), DME
        emit_sentinel_loop(eidx, sent);
        do_reset;
        c = 0;
        while(!u_dut.u_dmac.u_ch2.te && c < 30000) begin @(posedge clk); c = c + 1; end
        //strict order law: ch1 has moved NOTHING when ch2's TE sets (ch1's
        //first unit needs >=6 cycles after its first-ever grant)
        chk("PR=10: DMATCR1 untouched at te2", {8'd0, u_dut.u_dmac.u_ch1.tcr}, 32'h0000_0020);
        c = 0;
        while(!u_dut.u_dmac.u_ch1.te && c < 60000) begin @(posedge clk); c = c + 1; end
        run_cycles(50);
        chk("ch2 image [0]",                dmem[16'hC0], 32'hAA00_0001);
        chk("ch2 image [3]",                dmem[16'hC3], 32'hAA00_0004);
        chk("ch1 SAR end (+128)",  u_dut.u_dmac.u_ch1.sar, 32'h0000_0200);
        chk("ch1 DAR end (+128)",  u_dut.u_dmac.u_ch1.dar, 32'h0000_03C0);
        chk("ch1 DMATCR end",      {8'd0, u_dut.u_dmac.u_ch1.tcr}, 32'h0000_0000);
        end_test;
    end
endtask

task automatic test_dmac_gating;
    integer idx, sent;
    begin
        begin_test("DMAC gating: DE-clear stops without TE + TE blocks re-enable, DEI3 INTEVT2=0x860");
        //phase A: DE-clear mid-run stops (unit completes, count freezes, no TE)
        eidx = 0;
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0 (clear of every checked image)
        emit_poke_l(32'hA400_0028, 32'h0000_0040);   // DMATCR0 = 64
        emit_poke_l(32'hA400_002C, 32'h0000_5411);   // CHCR0: inc/inc auto long cs
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME (starts ch0)
        emit_ldrn(3, 32'd12);
        imem[eidx] = 16'h4310; eidx = eidx + 1;              // DT R3             ; brief run
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF .-1
        emit_poke_l(32'hA400_002C, 32'h0000_5410);   // CHCR0: DE clear (in-flight unit completes)
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2      ; mailbox base
        emit_ldrn(1, 32'hA400_0028);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // mb4 = DMATCR0 after halt
        emit_ldrn(3, 32'd40);
        imem[eidx] = 16'h4310; eidx = eidx + 1;              // DT R3             ; long delay
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF .-1
        emit_ldrn(1, 32'hA400_0028);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1205; eidx = eidx + 1;              // mb5 = DMATCR0 later (frozen)
        emit_ldrn(1, 32'hA400_002C);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1206; eidx = eidx + 1;              // mb6 = CHCR0 (TE must be 0)
        //phase B: ch3 completes w/ DEI (0x860); handler holds TE; DE=1 with TE=1 stays halted
        emit_wreg_w(32'hA400_001A, 16'hD000);        // IPRE: DMAC level 13
        emit_poke_l(32'hA400_0050, 32'h0000_0100);   // SAR3
        emit_poke_l(32'hA400_0054, 32'h0000_01C0);   // DAR3
        emit_poke_l(32'hA400_0058, 32'h0000_0002);   // DMATCR3 = 2
        emit_sr_imask(eidx, 4'h0);
        emit_poke_l(32'hA400_005C, 32'h0000_5415);   // CHCR3: IE DE (starts; DME already 1)
        emit_poll_te(32'hA400_005C);                 // TE holds through the handler's clear
        emit_poke_l(32'hA400_005C, 32'h0000_0403);   // DE=1 again, TE write-1 held -> no restart
        emit_ldrn(3, 32'd40);
        imem[eidx] = 16'h4310; eidx = eidx + 1;              // DT R3             ; settle window
        imem[eidx] = 16'h8BFD; eidx = eidx + 1;              // BF .-1
        emit_sentinel_loop(eidx, sent);
        emit_handler_dmac(8'h5C, 16'h0002);          // CHCR3 <- 2: DE/IE off, TE write-1 held
        do_reset;
        run_until_retire(sent, 60000);
        chk_true("DE-clear: stopped mid-count", dmem[16'h14] > 32'd0 && dmem[16'h14] < 32'd64);
        chk("DE-clear: count frozen",       dmem[16'h15], dmem[16'h14]);
        chk("DE-clear: TE stays 0",         dmem[16'h16], 32'h0000_5410);
        chk("DEI3 INTEVT2 = 0x860",         dmem[16'h10], 32'h0000_0860);
        chk("CHCR3 in handler: TE set",     dmem[16'h11], 32'h0000_5417);
        chk("TE blocks re-enable: SAR3 froze", u_dut.u_dmac.u_ch3.sar, 32'h0000_0108);
        chk("ch3 TE still set",             {31'd0, u_dut.u_dmac.u_ch3.te}, 32'd1);
        end_test;
    end
endtask

task automatic test_dmac_bus_modes;
    integer idx, sent, c;
    begin
        begin_test("DMAC bus modes: DME->TE duration laws, burst locks the CPU out vs cycle-steal");
        //phase A: cycle-steal, 8 longs, sentinel fetches interleave
        eidx = 0;
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0380);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0008);   // DMATCR0 = 8
        emit_poke_l(32'hA400_002C, 32'h0000_5411);   // CHCR0: cycle-steal
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_sentinel_loop(eidx, sent);
        do_reset;
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        c = 0;
        while(!u_dut.u_dmac.u_ch0.te && c < 30000) begin @(posedge clk); c = c + 1; end
        chk("cycle-steal 8-long DME->TE law", c[31:0], 32'd94);
        //phase B: same transfer in burst - CPU locked out, much shorter
        clear_imem; clear_dmem;
        eidx = 0;
        emit_poke_l(32'hA400_0020, 32'h0000_0100);
        emit_poke_l(32'hA400_0024, 32'h0000_0380);
        emit_poke_l(32'hA400_0028, 32'h0000_0008);
        emit_poke_l(32'hA400_002C, 32'h0000_5431);   // CHCR0: burst
        emit_wreg_w(32'hA400_0060, 16'h0001);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        c = 0;
        while(!u_dut.u_dmac.u_ch0.te && c < 30000) begin @(posedge clk); c = c + 1; end
        chk("burst 8-long DME->TE law", c[31:0], 32'd66);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  DMAC External Request + Single Address (session 5, phase 4)
////

/*
    Section 11.3.5 laws on the pins: DREQ sampled on the CKIO falling
    edge, DS level/edge detection, DACK framed on CSn in the AM-selected
    cycle with AL polarity, DRAK request-accepted pulse with RL polarity,
    burst-edge runs to DMATCR=0 from ONE edge (fig 11.21), level stop/
    resume. Port D pads granted by PDCR mode 00 (write 0xA080: PTD5/4/1
    to their functions). DREQ tests idle PTD4 HIGH before enabling the
    channel (11.6 note 10: keep the pin high while setting up).
*/

task automatic test_dmac_dreq_level_cs;
    integer idx, sent, c, frozen;
    begin
        begin_test("DREQ0 level cycle-steal: latency law, stop/resume, DACK=CS read window, DRAK");
        eidx = 0;
        emit_wreg_w(32'hA400_0106, 16'hA080);        // PDCR: PTD5/4/1/0 mode 00 (fn grant)
        emit_poke_l(32'hA000_0100, 32'hD0D0_0001);
        emit_poke_l(32'hA000_0104, 32'hD0D0_0002);
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0240);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0004);   // DMATCR0 = 4
        emit_poke_l(32'hA400_002C, 32'h0000_5011);   // CHCR0: inc/inc ext-dual DS=0 AM=0 cs DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_sentinel_loop(eidx, sent);
        do_reset;
        ptd_pin[4] = 1'b1;                           //DREQ0 negated during setup (11.6 note 10)
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        run_cycles(8);
        dackmon_clear; dackmon_en = 1'b1;
        ptd_pin[4] = 1'b0;                           //request
        c = 0;
        while(u_dut.u_dmac.seq == 3'd0 && c < 1000) begin @(posedge clk); c = c + 1; end
        //DREQ assert -> grant law: 5 core cycles (2FF sync + CKIO-fall sample
        //+ request resolve); the first PIN cycle lands several CKIO later,
        //beyond the >=3-state minimum of p.363
        chk("DREQ->grant latency law", c[31:0], 32'd5);
        //level stop: negate after the first unit lands; the one-step-ahead
        //sample may admit one more unit, then the count freezes
        c = 0;
        while(u_dut.u_dmac.u_ch0.tcr > 24'd3 && c < 5000) begin @(posedge clk); c = c + 1; end
        ptd_pin[4] = 1'b1;
        run_cycles(120);
        frozen = {8'd0, u_dut.u_dmac.u_ch0.tcr};
        run_cycles(256);
        chk_true("level stop: mid-count",   frozen > 0 && frozen < 4);
        chk("level stop: count frozen",     {8'd0, u_dut.u_dmac.u_ch0.tcr}, frozen[31:0]);
        ptd_pin[4] = 1'b0;                           //resume to completion
        c = 0;
        while(!u_dut.u_dmac.u_ch0.te && c < 30000) begin @(posedge clk); c = c + 1; end
        ptd_pin[4] = 1'b1;
        run_cycles(20);
        dackmon_en = 1'b0;
        chk("dst[0]",                       dmem[16'h90], 32'hD0D0_0001);
        chk("dst[1]",                       dmem[16'h91], 32'hD0D0_0002);
        chk("SAR0 end",  u_dut.u_dmac.u_ch0.sar, 32'h0000_0110);
        chk("DAR0 end",  u_dut.u_dmac.u_ch0.dar, 32'h0000_0250);
        chk_true("DACK0 windows observed",  dackw_cnt > 0);
        chk("DACK window inside CS0",       dack_naked[31:0], 32'd0);
        chk("AM=0: no DACK on write cycles", dack_on_wr[31:0], 32'd0);
        chk_true("DRAK0 pulses (RL=0: low)", drak0_lo > 0);
        end_test;
    end
endtask

task automatic test_dmac_dreq_edge_burst;
    integer idx, sent, c;
    begin
        begin_test("DREQ0 edge burst: one edge runs to DMATCR=0 (fig 11.21), AM=1/AL=1 DACK, RL=1 DRAK");
        eidx = 0;
        emit_wreg_w(32'hA400_0106, 16'hA080);        // PDCR fn grant
        emit_poke_l(32'hA000_0100, 32'hE0E0_0001);
        emit_poke_l(32'hA000_0104, 32'hE0E0_0002);
        emit_poke_l(32'hA000_0108, 32'hE0E0_0003);
        emit_poke_l(32'hA000_010C, 32'hE0E0_0004);
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0240);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0004);   // DMATCR0 = 4
        emit_poke_l(32'hA400_002C, 32'h0007_5071);   // CHCR0: RL AM AL, DS=1 edge, burst
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_sentinel_loop(eidx, sent);
        do_reset;
        ptd_pin[4] = 1'b1;
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        run_cycles(8);
        dackmon_clear; dackmon_en = 1'b1;
        ptd_pin[4] = 1'b0;                           //ONE falling edge...
        run_cycles(16);
        ptd_pin[4] = 1'b1;                           //...negated again mid-run
        c = 0;
        while(!u_dut.u_dmac.u_ch0.te && c < 30000) begin @(posedge clk); c = c + 1; end
        run_cycles(20);
        dackmon_en = 1'b0;
        chk("dst[0]",                       dmem[16'h90], 32'hE0E0_0001);
        chk("dst[3]",                       dmem[16'h93], 32'hE0E0_0004);
        chk("DMATCR0 ran to 0 off one edge", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
        chk_true("DACK0 windows observed",  dackw_cnt > 0);
        chk("DACK window inside CS0",       dack_naked[31:0], 32'd0);
        chk("AM=1: no DACK on read cycles", dack_on_rd[31:0], 32'd0);
        //AL=1 pad polarity: PTD5 is active-HIGH = high exactly during windows
        chk_true("DRAK0 pulses (RL=1: high)", drak0_hi > 0 && drak0_hi < 20);
        end_test;
    end
endtask

task automatic test_dmac_single_addr;
    integer idx, sent, c;
    begin
        begin_test("Single-address: dev->DACK->mem write w/ external drive + mem->dev latch, side masks");
        eidx = 0;
        emit_wreg_w(32'hA400_0106, 16'hA080);        // PDCR fn grant
        //phase A: device -> memory (RS=0011): lone WRITE cycles, device drives D
        emit_poke_l(32'hA400_0020, 32'h0000_0000);   // SAR0 = 0: must NOT step (mask)
        emit_poke_l(32'hA400_0024, 32'h0000_0260);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0002);   // DMATCR0 = 2
        emit_poke_l(32'hA400_002C, 32'h0000_5311);   // CHCR0: dm/sm inc, RS=0011, cs, long
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_002C);
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV #0x40,R2      ; mailbox base
        emit_ldrn(1, 32'hA400_0020);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0 = SAR0 (stayed 0)
        emit_poke_l(32'hA400_002C, 32'h0000_0000);   // CHCR0 off (TE read-1 then write-0)
        //phase B: memory -> device (RS=0010): lone READ cycles, device latches
        emit_poke_l(32'hA000_0140, 32'hBEEF_00AA);
        emit_poke_l(32'hA400_0020, 32'h0000_0140);   // SAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0001);   // DMATCR0 = 1
        emit_poke_l(32'hA400_002C, 32'h0000_5211);   // CHCR0: dm/sm inc, RS=0010, cs, long
        emit_poll_te(32'hA400_002C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        ptd_pin[4] = 1'b1;
        sgdev_data = 32'hFEED_0001;
        sgdev_clr  = 1'b1;                           //latch cleared by its own writer
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        run_cycles(8);
        sgdev_clr  = 1'b0;
        sgdev_en   = 1'b1;
        ptd_pin[4] = 1'b0;                           //level request through both phases
        run_until_retire(sent, 60000);
        ptd_pin[4] = 1'b1;
        sgdev_en   = 1'b0;
        chk("dev->mem [0] (device pattern)", dmem[16'h98], 32'hFEED_0001);
        chk("dev->mem [1] (pattern +1)",     dmem[16'h99], 32'hFEED_0002);
        chk("SAR0 frozen in dev->mem",       dmem[16'h10], 32'h0000_0000);
        chk("mem->dev: device latched",      sgdev_latch, 32'hBEEF_00AA);
        chk("DAR0 frozen in mem->dev",       u_dut.u_dmac.u_ch0.dar, 32'h0000_0268);
        chk("SAR0 end (mem->dev +4)",        u_dut.u_dmac.u_ch0.sar, 32'h0000_0144);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  DMAC Specials (session 5, phase 5)
////

/*
    16-byte units (fig 11.11), ch2 source reload (section 11.3.6), ch3
    indirect gather (figs 11.7-11.8) and round-robin priority (figs
    11.3-11.4). Round-robin laws come from the DMA write-order log: the
    chip reset pins rr_head to 0, so every phase's grant sequence is a
    static hex literal. Buffer pages: dst 0x1xx/0x2xx/0x3xx per channel
    (= the log nibble), sources parked in the 0x0xx page.
*/

task automatic test_dmac_16byte;
    integer idx, sent, c;
    begin
        begin_test("16-byte units: dual 4R->4W +16 steps, single mem->dev 4-read DACK unit");
        //phase A: dual-direct 16-byte auto burst, 3 units = 12 longs
        eidx = 0;
        for(idx = 0; idx < 12; idx = idx + 1)
            emit_poke_l(32'hA000_0100 + 32'(idx*4), 32'h16B0_0001 + 32'(idx));
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0 (16n boundary, p.333)
        emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0 (16n)
        emit_poke_l(32'hA400_0028, 32'h0000_0003);   // DMATCR0 = 3 units (one per 16 bytes)
        emit_poke_l(32'hA400_002C, 32'h0000_5439);   // CHCR0: inc/inc auto 16-byte burst DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_002C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        for(idx = 0; idx < 12; idx = idx + 1)
            chk("16-byte dual image", dmem[16'h80 + 16'(idx)], 32'h16B0_0001 + 32'(idx));
        chk("SAR0 end (+48)", u_dut.u_dmac.u_ch0.sar, 32'h0000_0130);
        chk("DAR0 end (+48)", u_dut.u_dmac.u_ch0.dar, 32'h0000_0230);
        chk("DMATCR0 end",    {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
        //phase B: single-address mem->dev 16-byte = 4 lone DACK reads per unit
        eidx = 0;
        emit_wreg_w(32'hA400_0106, 16'hA080);        // PDCR: DACK/DRAK pads fn grant
        for(idx = 0; idx < 8; idx = idx + 1)
            emit_poke_l(32'hA000_0140 + 32'(idx*4), 32'h16B1_0001 + 32'(idx));
        emit_poke_l(32'hA400_0020, 32'h0000_0140);   // SAR0 (16n)
        emit_poke_l(32'hA400_0028, 32'h0000_0002);   // DMATCR0 = 2 units
        emit_poke_l(32'hA400_002C, 32'h0000_5219);   // CHCR0: RS=0010 mem->dev, cs, 16-byte
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_002C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        ptd_pin[4] = 1'b1;                           //DREQ0 negated during setup
        c = 0;
        while(!u_dut.u_dmac.dme && c < 30000) begin @(posedge clk); c = c + 1; end
        run_cycles(8);
        dackmon_clear; dackmon_en = 1'b1;
        sgdev_en   = 1'b1;
        ptd_pin[4] = 1'b0;                           //level request to completion
        run_until_retire(sent, 60000);
        ptd_pin[4] = 1'b1;
        sgdev_en   = 1'b0;
        dackmon_en = 1'b0;
        chk("device latched the LAST beat",  sgdev_latch, 32'h16B1_0008);
        chk("SAR0 end (2 x +16)", u_dut.u_dmac.u_ch0.sar, 32'h0000_0160);
        chk("DMATCR0 end",        {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
        chk_true("DACK windows on all 8 reads", dackw_cnt >= 8);
        chk("DACK window inside CS0",        dack_naked[31:0], 32'd0);
        chk("single: no DACK on write cycles", dack_on_wr[31:0], 32'd0);
        //phase C: RAW pins - the fig 11.11 / fig 23.19 framing laws. Same
        //single mem->dev 16-byte unit, 0-wait 32-bit area 0, one unit per
        //sub-phase. Plain area: 4 back-to-back basic cycles, DACK re-framed
        //per beat (4 openings x 3 cycles, T1 -> mid-T2, zero idle states).
        //Burst ROM (A0BST=01): ONE envelope - DACK/CS0 low across all 4
        //beats (1 opening x 15 cycles = 8 CKIO states minus the final half)
        for(idx = 0; idx < 2; idx = idx + 1) begin
            init_knobs;
            clear_imem;
            clear_dmem;
            raw_mode = 1;
            eidx = 0;
            emit_wreg_w(32'hFFFF_FF66, 16'hFFF8);    // WCR2: A0W=000, 0-wait area 0
            if(idx == 1) emit_wreg_w(32'hFFFF_FF60, 16'h0200);  // BCR1: A0BST=01
            emit_wreg_w(32'hA400_0106, 16'hA080);    // PDCR: DACK/DRAK pads fn grant
            emit_poke_l(32'hA400_0020, 32'h0000_0140);   // SAR0 (16n)
            emit_poke_l(32'hA400_0028, 32'h0000_0001);   // DMATCR0 = 1 unit
            emit_poke_l(32'hA400_002C, 32'h0000_5219);   // CHCR0: RS=0010 mem->dev, cs, 16-byte
            emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
            emit_poll_te(32'hA400_002C);
            emit_sentinel_loop(eidx, sent);
            do_reset;
            ptd_pin[4] = 1'b1;                           //DREQ0 negated during setup
            c = 0;
            while(!u_dut.u_dmac.dme && c < 60000) begin @(posedge clk); c = c + 1; end
            run_cycles(8);
            dackmon_clear; dackmon_en = 1'b1;
            ptd_pin[4] = 1'b0;                           //level request to completion
            run_until_retire(sent, 90000);
            ptd_pin[4] = 1'b1;
            dackmon_en = 1'b0;
            $display("      [DACK16] %s: falls=%0d win=%0d t0=%0d t1=%0d",
                     (idx == 0) ? "plain" : "burst", dackf_cnt, dackw_cnt, dack_t0, dack_t1);
            chk("raw unit completed (TE path)", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
            chk("DACK window inside CS0", dack_naked[31:0], 32'd0);
            if(idx == 0) begin
                chk("plain 16-byte: DACK framed per beat (fig 11.11)", dackf_cnt[31:0], 32'd4);
                chk("plain 16-byte: window cycles",                    dackw_cnt[31:0], 32'd12);
            end
            else begin
                chk("burst ROM 16-byte: ONE DACK envelope (fig 23.19)", dackf_cnt[31:0], 32'd1);
                chk("burst ROM 16-byte: CS0 held across the run",       dackw_cnt[31:0], 32'd15);
            end
            //fig 11.11 contiguity: first assertion to last release spans the
            //unit's 8 CKIO states minus the final half - zero idle states
            chk("16-byte unit span (no idle states)", (dack_t1 - dack_t0), 32'd15);
        end
        //phase D: the p.304 / 11.6-note-12 WAIT-ignore law on RAW pins. Area 0
        //wait-coded (A0W=001: 1 programmed wait, pin ENABLED); the tb stretcher
        //holds i_WAIT_n low 12 cycles from the unit's head. A single dev->mem
        //16-byte unit is a WRITE run: the pin is IGNORED, so its DACK span is
        //the fixed 4 x (T1+Tw+T2) shape regardless of the stretch. The mem->dev
        //twin is a READ run - NOT exempted - so the same stretch grows it.
        for(idx = 0; idx < 3; idx = idx + 1) begin
            init_knobs;
            clear_imem;
            clear_dmem;
            raw_mode = 1;
            eidx = 0;
            emit_wreg_w(32'hFFFF_FF66, 16'hFFF9);        // WCR2: A0W=001, pin sampled
            emit_wreg_w(32'hA400_0106, 16'hA080);        // PDCR: DACK/DRAK pads fn grant
            if(idx < 2) begin                            //dev->mem: DAR only (fig 11.10a)
                emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0 (16n)
                emit_poke_l(32'hA400_002C, 32'h0000_5319);   // CHCR0: RS=0011 dev->mem, cs, 16-byte
            end
            else begin                                   //mem->dev control: SAR only
                emit_poke_l(32'hA400_0020, 32'h0000_0140);   // SAR0 (16n)
                emit_poke_l(32'hA400_002C, 32'h0000_5219);   // CHCR0: RS=0010 mem->dev, cs, 16-byte
            end
            emit_poke_l(32'hA400_0028, 32'h0000_0001);   // DMATCR0 = 1 unit
            emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
            emit_poll_te(32'hA400_002C);
            emit_sentinel_loop(eidx, sent);
            do_reset;
            ptd_pin[4] = 1'b1;                           //DREQ0 negated during setup
            c = 0;
            while(!u_dut.u_dmac.dme && c < 60000) begin @(posedge clk); c = c + 1; end
            run_cycles(8);
            dackw_arm = (idx == 0) ? 0 : 6;              //12 cycles of WAIT from the
            dackmon_clear; dackmon_en = 1'b1;            //unit's first DACK opening
            sgdev_en   = 1'b1;                           //device drives D in the windows
            ptd_pin[4] = 1'b0;                           //level request to completion
            run_until_retire(sent, 120000);
            ptd_pin[4] = 1'b1;
            sgdev_en   = 1'b0;
            dackmon_en = 1'b0;
            dackw_arm  = 0;
            chk("raw unit completed (TE path)", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
            case(idx)
                //4 write beats x (T1+Tw+T2): DACK low 5 of each 6 cycles
                0: chk("dev->mem 16-byte baseline span",        (dack_t1 - dack_t0), 32'd23);
                1: chk("dev->mem WRITE run ignores WAIT (p.304)", (dack_t1 - dack_t0), 32'd23);
                default: chk_true("mem->dev READ run honors WAIT", (dack_t1 - dack_t0) > 23);
            endcase
        end
        end_test;
    end
endtask

task automatic test_dmac_ch2_reload;
    integer idx, sent;
    begin
        begin_test("ch2 source reload: SAR2 returns to base every 4 transfers (fig 11.23)");
        eidx = 0;
        emit_poke_l(32'hA000_0100, 32'hAAAA_BBBB);   //4 source words, fetched twice
        emit_poke_l(32'hA000_0104, 32'hCCCC_DDDD);
        emit_poke_l(32'hA400_0040, 32'h0000_0100);   // SAR2 = reload image
        emit_poke_l(32'hA400_0044, 32'h0000_0200);   // DAR2 increments through both rounds
        emit_poke_l(32'hA400_0048, 32'h0000_0008);   // DMATCR2 = 8: multiple of 4 (p.373)
        emit_poke_l(32'hA400_004C, 32'h0008_5429);   // CHCR2: RO, inc/inc auto word burst DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_004C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        //words SAR2+0/2/4/6, reload, then the same four again (fig 11.23)
        chk("round 1 words [0]", dmem[16'h80], 32'hAAAA_BBBB);
        chk("round 1 words [1]", dmem[16'h81], 32'hCCCC_DDDD);
        chk("round 2 words [2]", dmem[16'h82], 32'hAAAA_BBBB);
        chk("round 2 words [3]", dmem[16'h83], 32'hCCCC_DDDD);
        chk("SAR2 end = reloaded base", u_dut.u_dmac.u_ch2.sar, 32'h0000_0100);
        chk("DAR2 end (+16)",  u_dut.u_dmac.u_ch2.dar, 32'h0000_0210);
        chk("DMATCR2 end",     {8'd0, u_dut.u_dmac.u_ch2.tcr}, 32'd0);
        chk("reload 4-counter cleared by TE", {30'd0, u_dut.u_dmac.u_ch2.ro_cnt}, 32'd0);
        end_test;
    end
endtask

task automatic test_dmac_ch3_indirect;
    integer idx, sent;
    begin
        begin_test("ch3 indirect: scattered byte gather, ptr fetch always LONG, SAR3 +4 at TS=byte");
        eidx = 0;
        emit_poke_l(32'hA000_0140, 32'h0000_0181);   //pointer table: three byte addresses
        emit_poke_l(32'hA000_0144, 32'h0000_0186);   //with different lane picks [1:0]
        emit_poke_l(32'hA000_0148, 32'h0000_018B);
        emit_poke_l(32'hA000_0180, 32'h1122_3344);   //data pool: 0x181 -> 0x22
        emit_poke_l(32'hA000_0184, 32'h5566_7788);   //           0x186 -> 0x77
        emit_poke_l(32'hA000_0188, 32'h99AA_BBCC);   //           0x18B -> 0xCC
        emit_poke_l(32'hA000_01C0, 32'h0000_0000);   //dest long cleared
        emit_poke_l(32'hA400_0050, 32'h0000_0140);   // SAR3 = pointer table base
        emit_poke_l(32'hA400_0054, 32'h0000_01C0);   // DAR3, byte increments
        emit_poke_l(32'hA400_0058, 32'h0000_0003);   // DMATCR3 = 3
        emit_poke_l(32'hA400_005C, 32'h0010_5401);   // CHCR3: DI, inc/inc auto byte cs DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_005C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        //a byte-size ptr fetch would read garbage addresses: correctness of the
        //gathered bytes proves the pointer read is LONG regardless of TS
        chk("gathered bytes at DAR3",  dmem[16'h70], 32'h2277_CC00);
        chk("SAR3 end (3 x +4 at TS=byte, p.339)", u_dut.u_dmac.u_ch3.sar, 32'h0000_014C);
        chk("DAR3 end (+3 bytes)", u_dut.u_dmac.u_ch3.dar, 32'h0000_01C3);
        chk("DMATCR3 end",         {8'd0, u_dut.u_dmac.u_ch3.tcr}, 32'd0);
        end_test;
    end
endtask

task automatic test_dmac_round_robin;
    integer idx, sent;
    begin
        begin_test("Round-robin PR=11: burst pair alternates, 3-channel rotation law (figs 11.3-11.4)");
        //phase A: two burst auto channels, 4 units each - RR breaks the burst
        //into alternating units (fig 11.14 note: the bus still never returns
        //to the CPU while a burst channel is requesting)
        eidx = 0;
        for(idx = 0; idx < 4; idx = idx + 1)
            emit_poke_l(32'hA000_0080 + 32'(idx*4), 32'hD0D0_0001 + 32'(idx));
        for(idx = 0; idx < 4; idx = idx + 1)
            emit_poke_l(32'hA000_00C0 + 32'(idx*4), 32'hD1D1_0001 + 32'(idx));
        emit_poke_l(32'hA400_0020, 32'h0000_0080);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0100);   // DAR0: log page 1
        emit_poke_l(32'hA400_0028, 32'h0000_0004);   // DMATCR0 = 4
        emit_poke_l(32'hA400_002C, 32'h0000_5431);   // CHCR0: inc/inc auto long BURST DE
        emit_poke_l(32'hA400_0030, 32'h0000_00C0);   // SAR1
        emit_poke_l(32'hA400_0034, 32'h0000_0200);   // DAR1: log page 2
        emit_poke_l(32'hA400_0038, 32'h0000_0004);   // DMATCR1 = 4
        emit_poke_l(32'hA400_003C, 32'h0000_5431);   // CHCR1: same
        emit_wreg_w(32'hA400_0060, 16'h0301);        // DMAOR: PR=11 round-robin + DME
        emit_poll_te(32'hA400_002C);
        emit_poll_te(32'hA400_003C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        dmaw_clr = 1'b1;                             //reset pins rr_head to 0
        run_cycles(4);
        dmaw_clr = 1'b0;
        run_until_retire(sent, 60000);
        chk("burst pair alternates 0/1", dmaw_log[31:0], 32'h1212_1212);
        chk("8 write beats logged",      dmaw_cnt[31:0], 32'd8);
        chk("ch0 image [3]", dmem[16'h43], 32'hD0D0_0004);
        chk("ch1 image [3]", dmem[16'h83], 32'hD1D1_0004);
        //phase B: fig 11.3 rotation - counts 1/2/3 on ch0/1/2 cycle-steal
        //from reset order 0>1>2>3: serve 0,1,2 then 1,2 then 2 = 1,2,3,2,3,3
        eidx = 0;
        emit_poke_l(32'hA000_0088, 32'hE0E0_0001);   //ch0 source (1 long)
        emit_poke_l(32'hA000_00C8, 32'hE1E1_0001);   //ch1 source (2 longs)
        emit_poke_l(32'hA000_00CC, 32'hE1E1_0002);
        emit_poke_l(32'hA000_00E0, 32'hE2E2_0001);   //ch2 source (3 longs)
        emit_poke_l(32'hA000_00E4, 32'hE2E2_0002);
        emit_poke_l(32'hA000_00E8, 32'hE2E2_0003);
        emit_poke_l(32'hA400_0020, 32'h0000_0088);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0100);   // DAR0: page 1
        emit_poke_l(32'hA400_0028, 32'h0000_0001);   // DMATCR0 = 1
        emit_poke_l(32'hA400_002C, 32'h0000_5411);   // CHCR0: inc/inc auto long cs DE
        emit_poke_l(32'hA400_0030, 32'h0000_00C8);   // SAR1
        emit_poke_l(32'hA400_0034, 32'h0000_0200);   // DAR1: page 2
        emit_poke_l(32'hA400_0038, 32'h0000_0002);   // DMATCR1 = 2
        emit_poke_l(32'hA400_003C, 32'h0000_5411);
        emit_poke_l(32'hA400_0040, 32'h0000_00E0);   // SAR2
        emit_poke_l(32'hA400_0044, 32'h0000_0300);   // DAR2: page 3
        emit_poke_l(32'hA400_0048, 32'h0000_0003);   // DMATCR2 = 3
        emit_poke_l(32'hA400_004C, 32'h0000_5411);
        emit_wreg_w(32'hA400_0060, 16'h0301);        // DMAOR: PR=11 + DME (starts all)
        emit_poll_te(32'hA400_002C);
        emit_poll_te(32'hA400_003C);
        emit_poll_te(32'hA400_004C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        dmaw_clr = 1'b1;
        run_cycles(4);
        dmaw_clr = 1'b0;
        run_until_retire(sent, 60000);
        chk("rotation law 1,2,3,2,3,3", {8'd0, dmaw_log[23:0]}, 32'h0012_3233);
        chk("6 write beats logged",     dmaw_cnt[31:0], 32'd6);
        chk("ch2 image [0]", dmem[16'hC0], 32'hE2E2_0001);
        chk("ch2 image [2]", dmem[16'hC2], 32'hE2E2_0003);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  DMAC Aborts + Hardening (session 5, phase 6)
////

/*
    NMIF (INTC NMI edge -> DMAOR, 11.6 note 3 + p.375 resume protocol),
    AE (alignment at grant + bus fault in flight, p.343), a randomized
    legal-config differential against a tb golden model under random
    bus latency, and BREQ cutting a burst at the bus-cycle tier.
*/

task automatic test_dmac_nmi_abort;
    integer idx, sent, c;
    begin
        begin_test("NMI->NMIF: sets while DMAC idle (11.6 note 3), aborts a burst w/o TE, resumes");
        //phase A: DMAC fully idle AND the CPU vector blocked (reset SR.BL=1
        //is left in place) - the NMI edge must still set NMIF, proving the
        //hook is independent of both the engine and the CPU accept
        eidx = 0;
        emit_ldrn(1, 32'hA400_0060);                 // DMAOR
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'hC802; eidx = eidx + 1;              // TST   #2,R0       ; T = !NMIF
        imem[eidx] = 16'h89FC; eidx = eidx + 1;              // BT    .-2 (poll)
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // mb3 = DMAOR (NMIF, DME=0)
        imem[eidx] = 16'hE000; eidx = eidx + 1;              // MOV   #0,R0
        imem[eidx] = 16'h2101; eidx = eidx + 1;              // MOV.W R0,@R1      ; write-0 clear
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1204; eidx = eidx + 1;              // mb4 = cleared readback
        emit_sentinel_loop(eidx, sent);
        do_reset;
        nmi_pin = 1'b1;                              //pre-charge for the falling edge
        run_cycles(400);
        nmi_pin = 1'b0;                              //NMI while the DMAC is idle
        run_until_retire(sent, 30000);
        chk("NMIF set while idle",          dmem[16'h13], 32'h0000_0002);
        chk("NMIF write-0 cleared",         dmem[16'h14], 32'h0000_0000);
        chk("BL held the CPU out entirely", entry_count, 32'd0);
        //phase B: NMI mid-burst - all channels suspend at the unit boundary
        //(registers stay stepped, TE unset); handler clears NMIF -> resume
        eidx = 0;
        emit_sr_imask(eidx, 4'h0);
        for(idx = 0; idx < 16; idx = idx + 1)
            emit_poke_l(32'hA000_0100 + 32'(idx*4), 32'hB6B6_0001 + 32'(idx));
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0010);   // DMATCR0 = 16
        emit_poke_l(32'hA400_002C, 32'h0000_5431);   // CHCR0: inc/inc auto long BURST DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_002C);
        emit_sentinel_loop(eidx, sent);
        //dedicated handler: mb0 = INTEVT2, mb1 = DMAOR (NMIF|DME), clear NMIF
        idx = 'h300;
        imem[idx] = 16'hE240; idx = idx + 1;              // MOV   #0x40,R2
        emit_a4_base(idx, 3);                             // R3 = 0xA4000000
        imem[idx] = 16'h6432; idx = idx + 1;              // MOV.L @R3,R4      ; INTEVT2
        imem[idx] = 16'h2242; idx = idx + 1;              // mb0
        imem[idx] = 16'h6033; idx = idx + 1;              // MOV   R3,R0
        imem[idx] = 16'hCB60; idx = idx + 1;              // OR    #0x60,R0    ; DMAOR
        imem[idx] = 16'h6103; idx = idx + 1;              // MOV   R0,R1
        imem[idx] = 16'h6011; idx = idx + 1;              // MOV.W @R1,R0
        imem[idx] = 16'h1201; idx = idx + 1;              // mb1 = DMAOR (0x0003)
        imem[idx] = 16'hE001; idx = idx + 1;              // MOV   #1,R0
        imem[idx] = 16'h2101; idx = idx + 1;              // DMAOR = 1: NMIF clear, DME kept
        for(c = 0; c < 8; c = c + 1) begin
            imem[idx] = 16'h0009; idx = idx + 1;          // grace NOPs
        end
        imem[idx] = 16'h002B; idx = idx + 1;              // RTE
        imem[idx] = 16'h0009; idx = idx + 1;              // NOP (delay slot)
        do_reset;
        c = 0;                                       //config lands, burst begins...
        while(u_dut.u_dmac.u_ch0.tcr != 24'd16 && c < 30000) begin @(posedge clk); c = c + 1; end
        c = 0;
        while(u_dut.u_dmac.u_ch0.tcr > 24'd10 && c < 30000) begin @(posedge clk); c = c + 1; end
        nmi_pin = 1'b1;                              //...NMI lands mid-count
        run_cycles(8);
        nmi_pin = 1'b0;
        run_until_retire(sent, 60000);
        chk("handler INTEVT2 = 0x1C0",      dmem[16'h10], 32'h0000_01C0);
        chk("DMAOR in handler: NMIF|DME",   dmem[16'h11], 32'h0000_0003);
        chk("resumed image [0]",            dmem[16'h80], 32'hB6B6_0001);
        chk("resumed image [15]",           dmem[16'h8F], 32'hB6B6_0010);
        chk("SAR0 end", u_dut.u_dmac.u_ch0.sar, 32'h0000_0140);
        chk("DAR0 end", u_dut.u_dmac.u_ch0.dar, 32'h0000_0240);
        chk("DMATCR0 end", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
        chk_true("TE set only at true completion", u_dut.u_dmac.u_ch0.te === 1'b1);
        end_test;
    end
endtask

task automatic test_dmac_addr_error;
    integer idx, sent;
    begin
        begin_test("AE: misaligned SAR at grant (re-arms until fixed) + bus fault abandons in flight");
        //phase A: long transfer with SAR = 4n+2 - AE before any bus cycle
        eidx = 0;
        emit_poke_l(32'hA000_0100, 32'hAE00_0001);   //src seed (used after the fix)
        emit_poke_l(32'hA000_0104, 32'hAE00_0002);
        emit_poke_l(32'hA400_0030, 32'h0000_0102);   // SAR1: misaligned for long
        emit_poke_l(32'hA400_0034, 32'h0000_0200);   // DAR1
        emit_poke_l(32'hA400_0038, 32'h0000_0002);   // DMATCR1 = 2
        emit_poke_l(32'hA400_003C, 32'h0000_5411);   // CHCR1: inc/inc auto long cs DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_ldrn(1, 32'hA400_0060);
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'hC804; eidx = eidx + 1;              // TST   #4,R0       ; T = !AE
        imem[eidx] = 16'h89FC; eidx = eidx + 1;              // BT    .-2 (poll AE)
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0 = DMAOR (AE|DME)
        emit_ldrn(1, 32'hA400_0038);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1 = DMATCR1 untouched
        emit_wreg_w(32'hA400_0060, 16'h0001);        // clear AE, config still bad...
        emit_ldrn(1, 32'hA400_0060);
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'hC804; eidx = eidx + 1;              // TST   #4,R0
        imem[eidx] = 16'h89FC; eidx = eidx + 1;              // BT    .-2 (...AE re-arms)
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // mb2 = DMAOR (AE|DME again)
        emit_poke_l(32'hA400_003C, 32'h0000_0000);   // fix: DE off first
        emit_poke_l(32'hA400_0030, 32'h0000_0100);   // SAR1 aligned
        emit_poke_l(32'hA400_003C, 32'h0000_5411);   // DE on
        emit_wreg_w(32'hA400_0060, 16'h0001);        // AE clear LAST (11.6 note 6)
        emit_poll_te(32'hA400_003C);
        emit_ldrn(1, 32'hA400_0030);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1203; eidx = eidx + 1;              // mb3 = SAR1 end
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 60000);
        chk("AE set, no transfer ran",      dmem[16'h10], 32'h0000_0005);
        chk("DMATCR1 untouched under AE",   dmem[16'h11], 32'h0000_0002);
        chk("AE re-arms while config bad",  dmem[16'h12], 32'h0000_0005);
        chk("SAR1 end after fix",           dmem[16'h13], 32'h0000_0108);
        chk("post-fix image [0]",           dmem[16'h80], 32'hAE00_0001);
        chk("post-fix image [1]",           dmem[16'h81], 32'hAE00_0002);
        //phase B: aligned addresses, but the bus faults the read - the unit
        //is abandoned in flight: no write, no register step, TE unset
        eidx = 0;
        emit_poke_l(32'hA000_0280, 32'hDEAD_BEEF);   //dst sentinel: must survive
        emit_poke_l(32'hA400_0020, 32'h0000_0180);   // SAR0 = the faulted word
        emit_poke_l(32'hA400_0024, 32'h0000_0280);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0002);   // DMATCR0 = 2
        emit_poke_l(32'hA400_002C, 32'h0000_5411);   // CHCR0: inc/inc auto long cs DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_ldrn(1, 32'hA400_0060);
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'hC804; eidx = eidx + 1;              // TST   #4,R0
        imem[eidx] = 16'h89FC; eidx = eidx + 1;              // BT    .-2 (poll AE)
        imem[eidx] = 16'hE240; eidx = eidx + 1;              // MOV   #0x40,R2
        imem[eidx] = 16'h6011; eidx = eidx + 1;              // MOV.W @R1,R0
        imem[eidx] = 16'h2202; eidx = eidx + 1;              // mb0 = DMAOR (AE|DME)
        emit_ldrn(1, 32'hA400_0028);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1201; eidx = eidx + 1;              // mb1 = DMATCR0 (no step)
        emit_ldrn(1, 32'hA400_0020);
        imem[eidx] = 16'h6012; eidx = eidx + 1;              // MOV.L @R1,R0
        imem[eidx] = 16'h1202; eidx = eidx + 1;              // mb2 = SAR0 (no step)
        emit_poke_l(32'hA400_002C, 32'h0000_0000);   // DE off
        emit_wreg_w(32'hA400_0060, 16'h0000);        // AE clear + DME off (leave idle)
        emit_sentinel_loop(eidx, sent);
        d_fault_en   = 1'b1;                         //fault the DMA read target only
        d_fault_widx = 8'h60;                        //word 0x60 = byte address 0x180
        do_reset;
        run_until_retire(sent, 60000);
        d_fault_en   = 1'b0;
        d_fault_widx = 8'd0;
        chk("bus fault set AE",             dmem[16'h10], 32'h0000_0005);
        chk("DMATCR0 not stepped",          dmem[16'h11], 32'h0000_0002);
        chk("SAR0 not stepped",             dmem[16'h12], 32'h0000_0180);
        chk("abandoned unit wrote nothing", dmem[16'hA0], 32'hDEAD_BEEF);
        chk_true("TE unset on AE abort",    u_dut.u_dmac.u_ch0.te === 1'b0);
        end_test;
    end
endtask

//big-endian lane helpers for the randomized differential's golden model
function automatic logic [31:0] lane_get(input logic [31:0] l, input logic [1:0] a,
                                         input logic [1:0] ts);
    begin
        case(ts)
            2'd0:    lane_get = {24'd0, l[(3 - a)*8 +: 8]};
            2'd1:    lane_get = {16'd0, a[1] ? l[15:0] : l[31:16]};
            default: lane_get = l;
        endcase
    end
endfunction

function automatic logic [31:0] lane_put(input logic [31:0] old, input logic [31:0] d,
                                         input logic [1:0] a, input logic [1:0] ts);
    begin
        lane_put = old;
        case(ts)
            2'd0:    lane_put[(3 - a)*8 +: 8] = d[7:0];
            2'd1:    if(a[1]) lane_put[15:0] = d[15:0]; else lane_put[31:16] = d[15:0];
            default: lane_put = d;
        endcase
    end
endfunction

task automatic test_dmac_random_diff;
    integer r, i, k, sent, mism, ch, ts, sm, dm, tm, cnt, s, maxu, slot_s, slot_d;
    logic   [31:0]  sar0, dar0, sa, da, datum, regbase;
    logic   [31:0]  exp_dst [0:15];
    logic   [31:0]  pool_l  [0:15];
    begin
        begin_test("Randomized differential: 12 legal configs vs golden model, random bus latency");
        //seed a 16-long source pool once; every round re-snapshots it and the
        //destination window from dmem, so rounds compose without re-seeding
        eidx = 0;
        for(i = 0; i < 16; i = i + 1)
            emit_poke_l(32'hA000_0080 + 32'(i*4), $urandom);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        run_until_retire(sent, 30000);
        for(r = 0; r < 12; r = r + 1) begin
            //draw a legal config: every 4th round is 16-byte (inc/fixed only,
            //16n addresses); dec walks start at the window's high end
            ch = $urandom % 4;
            ts = (r % 4 == 3) ? 3 : ($urandom % 3);
            if(ts == 3) begin
                sm = $urandom % 2;   dm = $urandom % 2;
                cnt = 1 + ($urandom % 2);
                s = 16;
            end
            else begin
                sm = $urandom % 3;   dm = $urandom % 3;
                cnt = 1 + ($urandom % 6);
                s = 1 << ts;
            end
            maxu   = 64 / s;
            slot_s = $urandom % (maxu - cnt + 1);
            slot_d = $urandom % (maxu - cnt + 1);
            sar0 = 32'h0000_0080 + 32'(((sm == 2) ? (slot_s + cnt - 1) : slot_s) * s);
            dar0 = 32'h0000_0200 + 32'(((dm == 2) ? (slot_d + cnt - 1) : slot_d) * s);
            tm = $urandom % 2;
            d_latency = $urandom % 4;                //the invariance oracle: the golden
                                                     //model never sees the latency
            //golden model over tb snapshots
            for(i = 0; i < 16; i = i + 1) pool_l[i]  = dmem[16'h20 + 16'(i)];
            for(i = 0; i < 16; i = i + 1) exp_dst[i] = dmem[16'h80 + 16'(i)];
            sa = sar0;  da = dar0;
            for(i = 0; i < cnt; i = i + 1) begin
                if(ts == 3) begin
                    for(k = 0; k < 4; k = k + 1)
                        exp_dst[integer'(da[5:2]) + k] = pool_l[integer'(sa[5:2]) + k];
                end
                else begin
                    datum = lane_get(pool_l[sa[5:2]], sa[1:0], ts[1:0]);
                    exp_dst[da[5:2]] = lane_put(exp_dst[da[5:2]], datum, da[1:0], ts[1:0]);
                end
                if(sm == 1) sa = sa + 32'(s); else if(sm == 2) sa = sa - 32'(s);
                if(dm == 1) da = da + 32'(s); else if(dm == 2) da = da - 32'(s);
            end
            //program the drawn channel and run to TE
            eidx = 0;
            regbase = 32'hA400_0020 + 32'(ch * 16);
            emit_poke_l(regbase + 32'd0,  sar0);
            emit_poke_l(regbase + 32'd4,  dar0);
            emit_poke_l(regbase + 32'd8,  32'(cnt));
            emit_poke_l(regbase + 32'd12, (32'(dm) << 14) | (32'(sm) << 12) | 32'h0000_0400 |
                                          (32'(tm) << 5)  | (32'(ts) << 3)  | 32'h0000_0001);
            emit_wreg_w(32'hA400_0060, 16'h0001);    // DMAOR: DME
            emit_poll_te(regbase + 32'd12);
            emit_poke_l(regbase + 32'd12, 32'd0);    // DE off + TE clear for the next round
            emit_sentinel_loop(eidx, sent);
            do_reset;
            run_until_retire(sent, 60000);
            mism = 0;
            for(i = 0; i < 16; i = i + 1)
                if(dmem[16'h80 + 16'(i)] !== exp_dst[i]) mism = mism + 1;
            if(mism != 0)
                $display("      [rnd %0d] ch%0d ts%0d sm%0d dm%0d tm%0d cnt%0d sar %08x dar %08x lat %0d",
                         r, ch, ts, sm, dm, tm, cnt, sar0, dar0, d_latency);
            chk_true($sformatf("round %0d image (ch%0d ts%0d cnt%0d)", r, ch, ts, cnt), mism == 0);
            chk_true($sformatf("round %0d SAR end", r), u_dut.u_dmac.ch_sar[ch] === sa);
            chk_true($sformatf("round %0d DAR end", r), u_dut.u_dmac.ch_dar[ch] === da);
            chk_true($sformatf("round %0d TCR end", r), u_dut.u_dmac.ch_tcr[ch] === 24'd0);
        end
        d_latency = 0;
        end_test;
    end
endtask

task automatic test_dmac_breq_cut;
    integer idx, sent, c, saw_back;
    begin
        begin_test("BREQ cuts a burst: pair split at the bus-cycle tier, unit results intact");
        eidx = 0;
        for(idx = 0; idx < 8; idx = idx + 1)
            emit_poke_l(32'hA000_0100 + 32'(idx*4), 32'hB4B4_0001 + 32'(idx));
        emit_poke_l(32'hA400_0020, 32'h0000_0100);   // SAR0
        emit_poke_l(32'hA400_0024, 32'h0000_0200);   // DAR0
        emit_poke_l(32'hA400_0028, 32'h0000_0008);   // DMATCR0 = 8
        emit_poke_l(32'hA400_002C, 32'h0000_5431);   // CHCR0: inc/inc auto long BURST DE
        emit_wreg_w(32'hA400_0060, 16'h0001);        // DMAOR: DME
        emit_poll_te(32'hA400_002C);
        emit_sentinel_loop(eidx, sent);
        do_reset;
        c = 0;                                       //burst begins...
        while(u_dut.u_dmac.u_ch0.tcr != 24'd8 && c < 30000) begin @(posedge clk); c = c + 1; end
        c = 0;
        while(u_dut.u_dmac.u_ch0.tcr > 24'd6 && c < 30000) begin @(posedge clk); c = c + 1; end
        saw_back = 0;                                //...six board-bus grabs walk across it
        for(idx = 0; idx < 6; idx = idx + 1) begin
            breq_n = 1'b0;
            c = 0;
            while(back_n !== 1'b0 && c < 2000) begin @(posedge clk); c = c + 1; end
            if(back_n === 1'b0) saw_back = saw_back + 1;
            run_cycles(4);
            breq_n = 1'b1;
            run_cycles(9);                           //odd spacing: pulses walk the R/W pair
        end
        run_until_retire(sent, 60000);
        chk_true("bus granted 6x during the burst", saw_back == 6);
        for(idx = 0; idx < 8; idx = idx + 1)
            chk("image long", dmem[16'h80 + 16'(idx)], 32'hB4B4_0001 + 32'(idx));
        chk("SAR0 end", u_dut.u_dmac.u_ch0.sar, 32'h0000_0120);
        chk("DAR0 end", u_dut.u_dmac.u_ch0.dar, 32'h0000_0220);
        chk("DMATCR0 end", {8'd0, u_dut.u_dmac.u_ch0.tcr}, 32'd0);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Main Sequence
////

initial begin
    clk      = 1'b0;
    por_n    = 1'b1;
    rst_n    = 1'b1;
    nmi_pin  = 1'b0;
    irq_pin  = '1;
    ptf_pin  = '0;
    pint_pin = '0;
    tclk_pin = 1'b0;
    pta_pin  = '0;
    ptg_pin  = '0;
    init_knobs;
    clear_imem;
    clear_dmem;

    group("1. Bring-up and IPC parity (splitter must add zero beats)");
    test_boot_smoke;
    begin_test("IPC parity: straight-line bypass bench == cpu_core_tb baseline");
    bench_ipc_straightline(200);
    end_test;
    begin_test("IPC parity: cached add-loop bench == cpu_core_tb baseline");
    bench_ipc_cached(100, 12);
    end_test;
    begin_test("IPC parity: cached store-loop bench == cpu_core_tb baseline");
    bench_ipc_store(80, 6);
    end_test;

    group("2. Bridge and register access (word/byte/long lanes)");
    test_bridge_word_rw;
    test_bridge_byte_rw;
    test_bridge_long;
    test_cpg_regs;
    test_wdt_keyed_write;

    group("3. WDT: interval interrupt and watchdog resets");
    test_wdt_interval_e2e;
    test_wdt_iprb_mask;
    test_wdt_watchdog_por;
    test_wdt_watchdog_manual;

    group("4. INTC: IRQ/IRL/NMI/PINT, priority and masking");
    test_irq_level;
    test_irq_edge_irr0;
    test_irl_mode;
    test_irls;
    test_nmi;
    test_pint;
    test_priority_tiebreak;

    group("5. External leg regression");
    test_ext_leg_regression;

    group("6. BSC registers and the SDRAM interface");
    test_bsc_reg_rw;
    test_refresh_reg_keys;
    test_sdmr_mrs;
    test_sdram_single_rw;
    test_sdram_fill_drain;

    group("7. SDRAM refresh, interrupts, self-refresh, reset retention");
    test_sdram_refresh;
    test_sdram_rcmi;
    test_sdram_rovi;
    test_sdram_selfrefresh;
    test_sdram_rst_survival;

    group("8. Bank-active mode and natural-latency baselines");
    test_sdram_bank_active;
    test_sdram_latency;
    bench_ipc_sdram;

    group("9. Ordinary memory / burst ROM: WCR2 waits, i_WAIT_n, burst pitch");
    test_ordinary_wait;
    test_burst_rom;

    group("10. Bus width and bus arbitration");
    test_width16;
    test_bus_release;
    test_tas_atomic_breq;

    group("BSC - NOR flash on the shared board bus");
    test_flash_boot;
    test_flash_autoselect;

    group("11. P bus: TMU and I/O ports");
    test_pbus_tmu_regs;
    test_tmu_underflow_e2e;
    test_tmu_prescaler_tstr;
    test_tmu_capture;
    test_tmu_external_clock;
    test_ioport_modes;

    group("12. RTC (EXTAL2 clock domain) and the CKIO pin");
    test_ckio_pin;
    test_rtc_regs;
    test_rtc_time_carry;
    test_rtc_periodic;
    test_rtc_alarm;
    test_tmu_rtc_tick;

    group("13. Interrupt x machinery collision (SoC twins of the core-level sweeps)");
    test_int_sdram_sweep;
    test_int_tas_sweep;
    test_exc_int_collision_soc;
    test_boundary_summary;

    group("14. BSC Group C: MCS pins, bus widths, AMX, BS/DQM shapes, release pads");
    test_mcs_pins;
    test_width8;
    test_amx_shapes;
    test_sdram16_shapes;
    test_bs_td_dqm;
    test_release_pads;

    group("15. DMAC register block + CMT (session 5, phase 1)");
    test_dmac_channel_regs;
    test_dmac_dmaor_cmt_regs;
    test_dmac_cmt_match;

    group("16. DMAC transfers: auto-request dual-direct engine (session 5, phase 3)");
    test_dmac_auto_long;
    test_dmac_sizes_lanes;
    test_dmac_priority;
    test_dmac_gating;
    test_dmac_bus_modes;

    group("17. DMAC external request + single address (session 5, phase 4)");
    test_dmac_dreq_level_cs;
    test_dmac_dreq_edge_burst;
    test_dmac_single_addr;

    group("18. DMAC specials: 16-byte, reload, indirect, round-robin (session 5, phase 5)");
    test_dmac_16byte;
    test_dmac_ch2_reload;
    test_dmac_ch3_indirect;
    test_dmac_round_robin;

    group("19. DMAC aborts + hardening (session 5, phase 6)");
    test_dmac_nmi_abort;
    test_dmac_addr_error;
    test_dmac_random_diff;
    test_dmac_breq_cut;

    group("20. Board-bus shape monitors (whole run)");
    test_bus_monitors;

    $display("");
    $display("################################");
    if(errors == 0) $display("HS3_tb: PASS (%0d tests)", test_count);
    else            $display("HS3_tb: FAIL (%0d errors over %0d tests)", errors, test_count);
    if(errors != 0) $fatal(1, "test failures: %0d", errors);
    $finish;
end

endmodule

`default_nettype wire
