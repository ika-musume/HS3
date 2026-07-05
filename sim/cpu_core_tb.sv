`timescale 1ns/1ps
`default_nettype none

/*
    Structured testbench for cpu_core (SH7709S integer CPU wrapper).

    The bench is organized as independent, self-checking test cases. Each case:
      1. fills the instruction memory with a short annotated program,
      2. pulses reset (which also clears the scoreboard and data memory),
      3. runs until the program's sentinel retires or an exception fires,
      4. checks architectural results (register file, SR, MACL/GBR, events).

    Every program line is annotated with the pipeline element it exercises:
    forwarding into ID/EX, the load-use interlock, the indexed-R0 third
    read, pre/post address-update lanes, EX-resolved branches and delay slots,
    state-register serialization, and each precise exception cause.

    Memory is modeled behind the same ready/valid ports the core drives.
    Instruction memory is word-addressed; data memory is longword-addressed and
    honors byte strobes. Fault injection and extra data latency are knobs so the
    fetch-fault, data-abort, and memory-wait paths can be exercised on demand.

    Register read note: while SR.MD=SR.RB=1 (reset state) logical Rn maps to BRAM
    word 8+n, so the helper gpr(n) reads the committed value directly.
*/

module cpu_core_tb;

///////////////////////////////////////////////////////////
//////  DUT Connections
////

logic           clk;
logic           por_n;
logic           rst_n;

IBus_1          MEM_BUS();       //I bus 1 out of the core; tb memory model slaves it

//Interrupt drive (INTC stand-in): the tb asserts a level+code and drops it at
//the core's acknowledge pulse, the same protocol the SoC INTC uses.
logic           int_valid_q = 1'b0;
logic   [3:0]   int_level_q = 4'd0;
logic   [11:0]  int_code_q  = 12'd0;
wire            int_ack_w;

logic           exc_valid;
logic   [2:0]   exc_cause;
logic   [31:0]  exc_pc;
logic           exc_in_delay_slot;
logic           exc_access_write;
logic   [31:0]  exc_access_addr;
logic           trapa_valid;
logic   [7:0]   trapa_imm;
logic           rte_valid;
logic           sleep_valid;
logic           ldtlb_valid;
logic           exception_entry_valid;
logic   [31:0]  exception_entry_pc;

logic           retire_valid;
logic   [31:0]  retire_pc;
logic   [15:0]  retire_inst;
logic           retire_gpr_we;
logic   [4:0]   retire_gpr;
logic   [31:0]  retire_gpr_data;

logic   [31:0]  sr;
logic   [31:0]  gbr_o;
logic   [31:0]  ssr_o;
logic   [31:0]  spc_o;
logic   [31:0]  vbr_o;
logic   [31:0]  mach_o;
logic   [31:0]  macl_o;
logic   [31:0]  pr_o;
logic   [31:0]  tra_o;
logic   [31:0]  expevt_o;
logic   [31:0]  intevt_o;
logic   [31:0]  tea_o;

//Architectural exception encodings; mirror localparams inside the DUT.
localparam logic [2:0] EXC_ILLEGAL   = 3'd1;
localparam logic [2:0] EXC_PRIVILEGE = 3'd2;
localparam logic [2:0] EXC_IFETCH    = 3'd3;
localparam logic [2:0] EXC_DATA      = 3'd4;
localparam logic [2:0] EXC_ADDRESS   = 3'd5;

//Single architectural clock: one posedge per architectural cycle (10 ns period).
wire            clk_p = clk;    //alias kept for the memory-model process below

cpu_core u_dut (
    .i_POR_n                   (por_n),
    .i_RST_n                   (rst_n),
    .i_CLK                     (clk),
    .i_CEN                     (1'b1),

    .I_BUS                     (MEM_BUS),

    .i_NMI_VALID               (1'b0),
    .i_NMI_BLMSK               (1'b0),
    .i_INT_VALID               (int_valid_q),
    .i_INT_LEVEL               (int_level_q),
    .i_INT_CODE                (int_code_q),
    .o_INT_ACK                 (int_ack_w),
    .o_NMI_ACK                 (),

    .dbg_o_RETIRE_VALID        (retire_valid),
    .dbg_o_RETIRE_PC           (retire_pc),
    .dbg_o_RETIRE_INST         (retire_inst),
    .dbg_o_RETIRE_GPR_WE       (retire_gpr_we),
    .dbg_o_RETIRE_GPR          (retire_gpr),
    .dbg_o_RETIRE_GPR_DATA     (retire_gpr_data),

    .dbg_o_FETCH_PC            (),
    .dbg_o_SR                  (sr),
    .dbg_o_GBR                 (gbr_o),
    .dbg_o_SSR                 (ssr_o),
    .dbg_o_SPC                 (spc_o),
    .dbg_o_VBR                 (vbr_o),
    .dbg_o_MACH                (mach_o),
    .dbg_o_MACL                (macl_o),
    .dbg_o_PR                  (pr_o),

    .dbg_o_TRA                 (tra_o),
    .dbg_o_EXPEVT              (expevt_o),
    .dbg_o_INTEVT              (intevt_o),
    .dbg_o_TEA                 (tea_o),

    .dbg_o_EXC_VALID           (exc_valid),
    .dbg_o_EXC_CAUSE           (exc_cause),
    .dbg_o_EXC_PC              (exc_pc),
    .dbg_o_EXC_IN_DELAY_SLOT   (exc_in_delay_slot),
    .dbg_o_EXC_ACCESS_WRITE    (exc_access_write),
    .dbg_o_EXC_ACCESS_ADDR     (exc_access_addr),
    .dbg_o_TRAPA_VALID         (trapa_valid),
    .dbg_o_TRAPA_IMM           (trapa_imm),
    .dbg_o_RTE_VALID           (rte_valid),
    .o_EXCEPTION_ENTRY_VALID   (exception_entry_valid),
    .o_EXCEPTION_ENTRY_PC      (exception_entry_pc),
    .o_SLEEP_VALID             (sleep_valid),
    .o_LDTLB_VALID             (ldtlb_valid)
);

always #5 clk = ~clk;


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
integer         d_latency;       //extra response-wait cycles for MA-wait tests
integer         i_latency;       //extra response-wait cycles on INSTRUCTION reads (fill-stretch knob)
integer         data_request_count;

//Unified external memory model. The cache now drives ONE bus, so instruction
//fetches and data accesses are serialised here. They are told apart with a
//test-only probe of the cache's cur_is_data: the cache is bypass-only in this
//bench (CCR.CE stays 0, so there are no line fills or write-back drains that
//could mislabel a transaction), which keeps the probe exact. Harvard test
//memories are preserved - imem (instructions) and dmem (data) stay separate,
//even where a PC-relative load reads data from a P2 address that also holds code.
//
//Instruction reads return a packed 32-bit longword (cache picks the addressed
//halfword); data reads/writes hit dmem with the configurable latency and fault
//injection. Only data requests advance data_request_count.
logic           mem_pending;
logic   [31:0]  mem_addr;
logic           mem_is_data;     //captured: serviced transaction is a data access
logic           mem_is_fault;
integer         mem_wait_cnt;

//I/D discriminator for the shared bus (test probe into the DUT's cache FSM). A WRITE
//is always data-side: a background wb-buffer drain dispatches from an idle edge where
//cur_is_data holds junk (speculative capture), and a mislabeled drain write would be
//dropped here, masking DUT coherency bugs. Reads keep the cur_is_data label (fills).
wire            req_is_data = u_dut.u_cache.cur_is_data || MEM_BUS.req_write;

assign MEM_BUS.req_ready = !mem_pending && !MEM_BUS.rsp_valid;

//Memory model state advances every clock, matching the architectural bus cycle.
always_ff @(posedge clk_p or negedge rst_n) begin
    integer i;
    if(!rst_n) begin
        mem_pending        <= 1'b0;
        mem_addr           <= 32'd0;
        mem_is_data        <= 1'b0;
        mem_is_fault       <= 1'b0;
        mem_wait_cnt       <= 0;
        MEM_BUS.rsp_valid  <= 1'b0;
        MEM_BUS.rsp_rdata  <= 32'd0;
        MEM_BUS.rsp_fault  <= 1'b0;
        data_request_count <= 0;
        for(i = 0; i < 256; i = i + 1) dmem[i] <= 32'd0;
    end
    else begin if(1'b1) begin
        if(MEM_BUS.req_valid && MEM_BUS.req_ready) begin
            mem_pending <= 1'b1;
            mem_addr    <= MEM_BUS.req_addr;
            mem_is_data <= req_is_data;
            if(req_is_data) begin
                data_request_count <= data_request_count + 1;
                mem_is_fault <= d_fault_en && (MEM_BUS.req_addr[9:2] == d_fault_widx);
                mem_wait_cnt <= d_latency;
                //A faulting access leaves memory unchanged, matching no-commit behavior.
                if(MEM_BUS.req_write && !(d_fault_en && (MEM_BUS.req_addr[9:2] == d_fault_widx))) begin
                    if(MEM_BUS.req_wstrb[0]) dmem[MEM_BUS.req_addr[9:2]][7:0]   <= MEM_BUS.req_wdata[7:0];
                    if(MEM_BUS.req_wstrb[1]) dmem[MEM_BUS.req_addr[9:2]][15:8]  <= MEM_BUS.req_wdata[15:8];
                    if(MEM_BUS.req_wstrb[2]) dmem[MEM_BUS.req_addr[9:2]][23:16] <= MEM_BUS.req_wdata[23:16];
                    if(MEM_BUS.req_wstrb[3]) dmem[MEM_BUS.req_addr[9:2]][31:24] <= MEM_BUS.req_wdata[31:24];
                end
            end
            else begin
                //Instruction fetch: fault tracks the exact requested halfword.
                mem_is_fault <= if_fault_en && (MEM_BUS.req_addr[11:1] == if_fault_widx);
                mem_wait_cnt <= i_latency;
            end
        end
        if(mem_pending && !MEM_BUS.rsp_valid) begin
            if(mem_wait_cnt == 0) begin
                mem_pending       <= 1'b0;
                MEM_BUS.rsp_valid <= 1'b1;
                //Data -> dmem longword; instruction -> two halfwords packed
                //big-endian (even halfword in [31:16]) for the cache to pick from.
                MEM_BUS.rsp_rdata <= mem_is_data ? dmem[mem_addr[9:2]]
                                                 : {imem[{mem_addr[11:2], 1'b0}],
                                                    imem[{mem_addr[11:2], 1'b1}]};
                MEM_BUS.rsp_fault <= mem_is_fault;
            end
            else begin
                mem_wait_cnt <= mem_wait_cnt - 1;
            end
        end
        if(MEM_BUS.rsp_valid && MEM_BUS.rsp_ready) begin
            MEM_BUS.rsp_valid <= 1'b0;
        end
    end end
end


///////////////////////////////////////////////////////////
//////  Retirement And Event Scoreboard
////

/*
    The scoreboard records architectural commit and event pulses. It clears on
    reset, so each test starts with a clean record. Pulse outputs are sampled on
    their rising edge to count each commit or event exactly once.
*/

logic           retired_seen [0:2047];
integer         retire_count [0:2047];
logic           retire_valid_z, exc_valid_z, trapa_valid_z, rte_valid_z, sleep_valid_z, ldtlb_valid_z;

logic           exc_seen;
logic   [2:0]   exc_cause_l;
logic   [31:0]  exc_pc_l;
logic           exc_delay_l;
logic   [31:0]  exc_aaddr_l;     //latched faulting access address (fill-fault goldens)
logic           exc_awrite_l;
logic           trapa_seen;
logic   [7:0]   trapa_imm_l;
logic           rte_seen;
logic           sleep_seen;
logic           ldtlb_seen;
logic           exception_entry_seen;
logic   [31:0]  exception_entry_pc_l;
integer         entry_count;         //exception/interrupt entries since reset
logic           entry_z;             //one-cycle-delayed entry pulse (SPC settle)
logic   [31:0]  entry_spc_l;         //SPC captured the cycle after each entry
integer         int_ack_count;       //o_INT_ACK pulses since reset (INTC drop key)
logic           gpr_phase_write_seen;
logic           gpr_phase_read_seen;
logic           gpr_phase_capture_seen;
logic           mac_latency_active;
integer         mac_latency_cycles;
integer         mac_last_latency;
integer         locked_read_count;
integer         locked_write_count;

//IPC benchmark instrumentation. bench_arm is driven ONLY by the test sequence;
//bench_active and the counters are owned by the scoreboard always_ff below, so
//each signal keeps a single driver (no procedural race).
logic           bench_arm = 1'b0;
logic           bench_active, bench_started;
integer         bench_arch_cycles, bench_retires;

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        retired_seen   <= '{default:1'b0};
        retire_count   <= '{default:0};
        retire_valid_z <= 1'b0;
        exc_valid_z    <= 1'b0;
        trapa_valid_z  <= 1'b0;
        rte_valid_z    <= 1'b0;
        sleep_valid_z  <= 1'b0;
        ldtlb_valid_z  <= 1'b0;
        exc_seen       <= 1'b0;
        exc_cause_l    <= 3'd0;
        exc_pc_l       <= 32'd0;
        exc_delay_l    <= 1'b0;
        exc_aaddr_l    <= 32'd0;
        exc_awrite_l   <= 1'b0;
        trapa_seen     <= 1'b0;
        trapa_imm_l    <= 8'd0;
        rte_seen       <= 1'b0;
        sleep_seen     <= 1'b0;
        ldtlb_seen     <= 1'b0;
        exception_entry_seen <= 1'b0;
        exception_entry_pc_l <= 32'd0;
        entry_count          <= 0;
        entry_z              <= 1'b0;
        entry_spc_l          <= 32'd0;
        int_ack_count        <= 0;
        gpr_phase_write_seen   <= 1'b0;
        gpr_phase_read_seen    <= 1'b0;
        gpr_phase_capture_seen <= 1'b0;
        mac_latency_active     <= 1'b0;
        mac_latency_cycles     <= 0;
        mac_last_latency       <= 0;
        locked_read_count      <= 0;
        locked_write_count     <= 0;
        bench_active           <= 1'b0;
        bench_started          <= 1'b0;
        bench_arch_cycles      <= 0;
        bench_retires          <= 0;
    end
    else begin
        retire_valid_z <= retire_valid;
        exc_valid_z    <= exc_valid;
        trapa_valid_z  <= trapa_valid;
        rte_valid_z    <= rte_valid;
        sleep_valid_z  <= sleep_valid;
        ldtlb_valid_z  <= ldtlb_valid;

        //Sample once per architectural cycle (every posedge now). The pulse outputs
        //are set for exactly one cycle, so a run of back-to-back retirements is
        //captured one PC at a time.
        if(1'b1) begin
            if(retire_valid) begin
                retired_seen[retire_pc[11:1]] <= 1'b1;
                retire_count[retire_pc[11:1]] <= retire_count[retire_pc[11:1]] + 1;
            end
            if(exc_valid) begin
                exc_seen     <= 1'b1;
                exc_cause_l  <= exc_cause;
                exc_pc_l     <= exc_pc;
                exc_delay_l  <= exc_in_delay_slot;
                exc_aaddr_l  <= exc_access_addr;
                exc_awrite_l <= exc_access_write;
            end
            if(trapa_valid) begin trapa_seen <= 1'b1; trapa_imm_l <= trapa_imm; end
            if(rte_valid)   rte_seen   <= 1'b1;
            if(sleep_valid) sleep_seen <= 1'b1;
            if(ldtlb_valid) ldtlb_seen <= 1'b1;
            if(exception_entry_valid) begin
                exception_entry_seen <= 1'b1;
                exception_entry_pc_l <= exception_entry_pc;
                entry_count          <= entry_count + 1;
            end
            entry_z <= exception_entry_valid;
            if(entry_z) entry_spc_l <= spc_o;   //SPC is committed by the cycle after entry
            //Clocked ack sampling: the SoC INTC drops its request on this pulse, so an
            //ack WITHOUT a matching interrupt entry is a lost interrupt (collision law).
            if(int_ack_w) int_ack_count <= int_ack_count + 1;
        end

        //IPC benchmark window. One architectural cycle per posedge; counting starts
        //at the FIRST retirement so pipeline fill latency is excluded from the ratio.
        if(1'b1) begin
            if(bench_arm && !bench_active) begin
                bench_active      <= 1'b1;
                bench_started     <= 1'b0;
                bench_arch_cycles <= 0;
                bench_retires     <= 0;
            end
            else if(!bench_arm) begin
                bench_active <= 1'b0;
            end
            else if(bench_active) begin
                if(retire_valid && !bench_started) bench_started <= 1'b1;
                if(bench_started) begin
                    bench_arch_cycles <= bench_arch_cycles + 1;
                    if(retire_valid) bench_retires <= bench_retires + 1;
                end
            end
        end

        //The phase test uses R1 as WB data and MOV R1,R2 as the read/capture probe.
        if(u_dut.u_int_pipe.gpr_wb0_we && u_dut.u_int_pipe.gpr_wb0_dst == 5'd9 &&
           u_dut.u_int_pipe.gpr_wb0_data == 32'h0000_002A) begin
            gpr_phase_write_seen <= 1'b1;
        end
        if(u_dut.u_int_pipe.ifid.valid && u_dut.u_int_pipe.ifid.inst == 16'h6213 &&
           u_dut.u_int_pipe.gpr_read1_address == 5'd9) begin
            gpr_phase_read_seen <= 1'b1;
        end
        if(u_dut.u_int_pipe.idex.valid && u_dut.u_int_pipe.idex.inst == 16'h6213 &&
           u_dut.u_int_pipe.idex.src_a_value == 32'h0000_002A) begin
            gpr_phase_capture_seen <= 1'b1;
        end

        //Count enabled clocks from DSP launch until its pending result is visible.
        if(1'b1) begin
            if(u_dut.u_int_pipe.mac_start) begin
                mac_latency_active <= 1'b1;
                mac_latency_cycles <= 0;
            end
            else if(mac_latency_active) begin
                if(u_dut.u_int_pipe.mac_dsp_done) begin
                    mac_latency_active <= 1'b0;
                    mac_last_latency   <= mac_latency_cycles + 1;
                end
                else begin
                    mac_latency_cycles <= mac_latency_cycles + 1;
                end
            end
        end

        //Count accepted atomic phases; TST.B intentionally never asserts lock.
        if(MEM_BUS.req_valid && MEM_BUS.req_ready && MEM_BUS.req_lock) begin
            if(MEM_BUS.req_write) locked_write_count <= locked_write_count + 1;
            else                  locked_read_count  <= locked_read_count + 1;
        end
    end
end


///////////////////////////////////////////////////////////
//////  Cache Property Checkers (passive, suite-wide)
////

/*
    LRU-divergence mirror: an INDEPENDENT true-LRU model kept as a per-set
    recency queue (slot 0 = most recent, slot 3 = oldest = the victim),
    cross-checked against the DUT's 6-pairwise-bit machinery (Table 5.2,
    p.104) at every LRU RAM write and every miss-dispatch victim choice.
    A divergence means a stale LRU read, a missed MRU update, or a broken
    same-edge RDW bypass in the LRU RAM. A software LRU load (non-assoc mm
    tag write) is not a total order in general: a zero value is the known
    reset order, anything else UNTRACKS that set until the next flush walk.

    The pair-bit semantic (bit = 1 means the pair's FIRST way is OLDER) and
    the state encodings mirror cache.sv; the "unknown lru_we site" guard
    below fails loudly if the state_t declaration order ever drifts.
*/

//cache.sv state_t encodings (declaration order). CS = "cache state".
localparam logic [4:0] CS_FLUSH      = 5'd0;
localparam logic [4:0] CS_IDLE       = 5'd1;
localparam logic [4:0] CS_IFILL_REQ  = 5'd8;
localparam logic [4:0] CS_IFILL_WAIT = 5'd9;
localparam logic [4:0] CS_DFILL_WAIT = 5'd11;
localparam logic [4:0] CS_MMTAG_WR   = 5'd19;

logic   [1:0]   lru_order [0:255][0:3];     //mirror recency queue per set
logic           lru_untracked [0:255];      //set holds a software-written non-order
integer         lru_upd_checks   = 0;       //MRU-update writes cross-checked
integer         lru_vic_checks   = 0;       //victim choices cross-checked
integer         lru_mismatches   = 0;

integer         squash_fill_hits = 0;       //cycles with I_SQUASH high during an I-fill run
integer         entry_cache_busy = 0;       //interrupt/exception entries with the cache FSM mid-excursion

integer         lbus_dreq_pends  = 0;       //unaccepted-and-held D-request cycles observed
integer         lbus_dreq_viol   = 0;       //D-request field mutated while pending
integer         lbus_drsp_pends  = 0;
integer         lbus_drsp_viol   = 0;       //D-response mutated/lost while unconsumed
integer         mbus_req_pends   = 0;
integer         mbus_req_viol    = 0;       //external request mutated/withdrawn while pending

integer         lbus_ifetch_acc  = 0;       //accepted L-bus FETCH requests (pair-law probe)

task automatic mirror_reset_set(input integer set);
    begin
        lru_order[set][0] = 2'd0;   //all-zero LRU bits decode to the order 0,1,2,3
        lru_order[set][1] = 2'd1;   //(victim = way 3), matching lru_victim's reset row
        lru_order[set][2] = 2'd2;
        lru_order[set][3] = 2'd3;
        lru_untracked[set] = 1'b0;
    end
endtask

task automatic mirror_mru(input integer set, input logic [1:0] way);
    integer i, j;
    begin
        for(i = 1; i < 4; i = i + 1) begin
            if(lru_order[set][i] == way) begin
                for(j = i; j > 0; j = j - 1) lru_order[set][j] = lru_order[set][j-1];
                lru_order[set][0] = way;
            end
        end
    end
endtask

//Expected pairwise-older bits off the mirror queue. Bit map matches cache_pkg:
//lru[5]=(0,1) lru[4]=(0,2) lru[3]=(0,3) lru[2]=(1,2) lru[1]=(1,3) lru[0]=(2,3).
function automatic logic [5:0] mirror_bits(input integer set);
    integer pos [0:3];
    integer i;
    for(i = 0; i < 4; i = i + 1) pos[lru_order[set][i]] = i;
    mirror_bits[5] = pos[0] > pos[1];
    mirror_bits[4] = pos[0] > pos[2];
    mirror_bits[3] = pos[0] > pos[3];
    mirror_bits[2] = pos[1] > pos[2];
    mirror_bits[1] = pos[1] > pos[3];
    mirror_bits[0] = pos[2] > pos[3];
endfunction

initial begin
    integer s;
    for(s = 0; s < 256; s = s + 1) mirror_reset_set(s);  //BRAM powers up all-zero
end

//The mirror persists across do_reset like the RAMs (p.104: reset keeps V/U/LRU);
//only the CCR.CF flush walk re-baselines it. Blocking assigns: tb-only state.
always @(posedge clk) begin
    logic [4:0]  cst;
    integer      cset;
    logic [1:0]  cway;
    if(rst_n) begin
        cst = u_dut.u_cache.state;

        //(1) Miss-dispatch victim choice vs the mirror's oldest way.
        if(cst == CS_IDLE && u_dut.u_cache.vic_ld) begin
            cset = u_dut.u_cache.bram_addr[11:4];
            if(!lru_untracked[cset]) begin
                lru_vic_checks = lru_vic_checks + 1;
                if(u_dut.u_cache.victim !== lru_order[cset][3]) begin
                    lru_mismatches = lru_mismatches + 1;
                    $display("      [LRU] victim mismatch set %02h: dut way %0d, mirror way %0d",
                             cset, u_dut.u_cache.victim, lru_order[cset][3]);
                end
            end
        end

        //(2) Every LRU RAM write: cross-check the written bits, then track it.
        if(u_dut.u_cache.lru_we) begin
            cset = u_dut.u_cache.lru_waddr;
            if(cst == CS_FLUSH)
                mirror_reset_set(cset);
            else if(cst == CS_IDLE || cst == CS_IFILL_WAIT || cst == CS_DFILL_WAIT) begin
                cway = (cst == CS_IDLE) ? u_dut.u_cache.hit_way : u_dut.u_cache.cur_way;
                mirror_mru(cset, cway);
                if(!lru_untracked[cset]) begin
                    lru_upd_checks = lru_upd_checks + 1;
                    if(u_dut.u_cache.lru_wdata !== mirror_bits(cset)) begin
                        lru_mismatches = lru_mismatches + 1;
                        $display("      [LRU] update mismatch set %02h way %0d: dut %06b, mirror %06b",
                                 cset, cway, u_dut.u_cache.lru_wdata, mirror_bits(cset));
                    end
                end
            end
            else if(cst == CS_MMTAG_WR) begin
                if(u_dut.u_cache.cur_wdata[9:4] == 6'd0) mirror_reset_set(cset);
                else                                     lru_untracked[cset] = 1'b1;
            end
            else begin
                lru_mismatches = lru_mismatches + 1;    //state encoding drift guard
                $display("      [LRU] lru_we from unexpected cache state %0d", cst);
            end
        end

        //(3) Squash-during-fill coverage: proves the latency sweep really lands
        //redirects inside I-fill excursions (consume-and-drop path exercised).
        if(u_dut.pipe_i_squash && (cst == CS_IFILL_REQ || cst == CS_IFILL_WAIT))
            squash_fill_hits = squash_fill_hits + 1;

        //(4) Entry-vs-machinery coverage: proves the interrupt sweeps really land
        //acceptance edges while the cache FSM is mid-fill/drain/bypass (not IDLE).
        if(exception_entry_valid && cst != CS_IDLE)
            entry_cache_busy = entry_cache_busy + 1;
    end
end

//Debug trace, off by default: a test may pulse these around a window of interest
//to print retirements and/or accepted external bus requests.
logic           dbg_trace     = 1'b0;
logic           dbg_trace_mem = 1'b0;
always @(posedge clk) begin
    if(dbg_trace && retire_valid)
        $display("        [trace %0t] RET pc=%08h inst=%04h", $time, retire_pc, retire_inst);
    if(dbg_trace && retire_gpr_we)
        $display("        [trace %0t] RETW R%0d <= %08h", $time, retire_gpr, retire_gpr_data);
    if(dbg_trace_mem && MEM_BUS.req_valid && MEM_BUS.req_ready)
        $display("        [trace %0t] MEM %s addr=%08h %s", $time,
                 MEM_BUS.req_write ? "WR" : "RD", MEM_BUS.req_addr, req_is_data ? "(D)" : "(I)");
    if(dbg_trace && exc_valid)
        $display("        [trace %0t] EXC cause=%0d pc=%08h aaddr=%08h wr=%b slot=%b inst_at_pc=%04h",
                 $time, exc_cause, exc_pc, exc_access_addr, exc_access_write,
                 exc_in_delay_slot, imem[exc_pc[11:1]]);
    if(dbg_trace && u_dut.u_int_pipe.ex_advance)
        $display("        [trace %0t] EX  pc=%08h inst=%04h ea=%08h a=%08h b=%08h", $time,
                 u_dut.u_int_pipe.idex.pc, u_dut.u_int_pipe.idex.inst,
                 u_dut.u_int_pipe.effective_addr,
                 u_dut.u_int_pipe.idex.src_a_value, u_dut.u_int_pipe.idex.src_b_value);
    if(dbg_trace && u_dut.u_int_pipe.wb_valid && !u_dut.u_int_pipe.mawb.fault)
        $display("        [trace %0t] WB  pc=%08h inst=%04h g0=%b/%0d/%08h g1=%b/%0d/%08h", $time,
                 u_dut.u_int_pipe.mawb.pc, u_dut.u_int_pipe.mawb.inst,
                 u_dut.u_int_pipe.mawb.gpr0_we, u_dut.u_int_pipe.mawb.gpr0_dst, u_dut.u_int_pipe.mawb.gpr0_data,
                 u_dut.u_int_pipe.mawb.gpr1_we, u_dut.u_int_pipe.mawb.gpr1_dst, u_dut.u_int_pipe.mawb.gpr1_data);
    if(dbg_trace && exception_entry_valid)
        $display("        [trace %0t] ENTRY nextpc=%08h stages mawb=%b/%08h exma=%b/%08h idex=%b/%08h ifid=%b/%08h pair=%b/%08h fpend=%b/%08h fpc=%08h",
                 $time, u_dut.u_int_pipe.o_INT_NEXT_PC,
                 u_dut.u_int_pipe.mawb.valid, u_dut.u_int_pipe.mawb.pc,
                 u_dut.u_int_pipe.exma.valid, u_dut.u_int_pipe.exma.pc,
                 u_dut.u_int_pipe.idex.valid, u_dut.u_int_pipe.idex.pc,
                 u_dut.u_int_pipe.ifid.valid, u_dut.u_int_pipe.ifid.pc,
                 u_dut.u_int_pipe.pair_ready, u_dut.u_int_pipe.pair_pc,
                 u_dut.u_int_pipe.fetch_pending, u_dut.u_int_pipe.fetch_pending_pc,
                 u_dut.u_int_pipe.fetch_pc);
end

/*
    L-bus D-request stability contract: the cache captures a request descriptor
    at its accept edge and commits stores from that capture, so a data request
    held while unaccepted MUST re-present identical fields. Withdrawal (valid
    dropping, or the slot turning into a fetch) is a pipeline kill - allowed.
    The D-response hold contract is the loss-free retirement rule: an unconsumed
    D response re-presents identically (D priority keeps rsp_fetch low).
    The external MEM bus adds the no-withdrawal rule the BSC depends on.
*/
logic           lb_dpend_z;
logic   [31:0]  lb_addr_z, lb_wdata_z;
logic           lb_write_z, lb_lock_z;
logic   [1:0]   lb_size_z;
logic           lb_rpend_z;
logic   [31:0]  lb_rdata_z;
logic           lb_dfault_z;
logic           mb_pend_z;
logic   [31:0]  mb_addr_z, mb_wdata_z;
logic   [3:0]   mb_wstrb_z;
logic   [1:0]   mb_size_z;
logic           mb_write_z, mb_burst_z, mb_lock_z;

wire            lb_d_now = u_dut.PIPE_L_BUS.req_valid && !u_dut.PIPE_L_BUS.req_fetch;
wire            lb_r_now = u_dut.PIPE_L_BUS.rsp_valid && !u_dut.PIPE_L_BUS.rsp_fetch;

always @(posedge clk) begin
    if(!rst_n) begin
        lb_dpend_z = 1'b0;
        lb_rpend_z = 1'b0;
        mb_pend_z  = 1'b0;
    end
    else begin
        //D-request stability while pending (presented last cycle, unaccepted).
        if(lb_dpend_z && lb_d_now) begin
            lbus_dreq_pends = lbus_dreq_pends + 1;
            if(u_dut.PIPE_L_BUS.req_addr  !== lb_addr_z  ||
               u_dut.PIPE_L_BUS.req_write !== lb_write_z ||
               u_dut.PIPE_L_BUS.req_size  !== lb_size_z  ||
               u_dut.PIPE_L_BUS.req_lock  !== lb_lock_z  ||
               (lb_write_z && u_dut.PIPE_L_BUS.req_wdata !== lb_wdata_z)) begin
                lbus_dreq_viol = lbus_dreq_viol + 1;
                $display("      [LBUS] D-request mutated while unaccepted @%0t", $time);
            end
        end
        //D-response hold until consumed (identical data/fault re-presentation).
        if(lb_rpend_z) begin
            lbus_drsp_pends = lbus_drsp_pends + 1;
            if(!lb_r_now ||
               u_dut.PIPE_L_BUS.rsp_rdata  !== lb_rdata_z ||
               u_dut.PIPE_L_BUS.rsp_dfault !== lb_dfault_z) begin
                lbus_drsp_viol = lbus_drsp_viol + 1;
                $display("      [LBUS] D-response mutated/lost while unconsumed @%0t", $time);
            end
        end
        //External request: no withdrawal, no mutation, until accepted.
        if(mb_pend_z) begin
            mbus_req_pends = mbus_req_pends + 1;
            if(!MEM_BUS.req_valid            ||
               MEM_BUS.req_addr  !== mb_addr_z  ||
               MEM_BUS.req_write !== mb_write_z ||
               MEM_BUS.req_size  !== mb_size_z  ||
               MEM_BUS.req_burst !== mb_burst_z ||
               MEM_BUS.req_lock  !== mb_lock_z  ||
               (mb_write_z && (MEM_BUS.req_wdata !== mb_wdata_z ||
                               MEM_BUS.req_wstrb !== mb_wstrb_z))) begin
                mbus_req_viol = mbus_req_viol + 1;
                $display("      [MBUS] external request mutated/withdrawn while pending @%0t", $time);
            end
        end

        lb_dpend_z = lb_d_now && !u_dut.PIPE_L_BUS.req_ready;
        lb_addr_z  = u_dut.PIPE_L_BUS.req_addr;
        lb_write_z = u_dut.PIPE_L_BUS.req_write;
        lb_size_z  = u_dut.PIPE_L_BUS.req_size;
        lb_lock_z  = u_dut.PIPE_L_BUS.req_lock;
        lb_wdata_z = u_dut.PIPE_L_BUS.req_wdata;

        lb_rpend_z  = lb_r_now && !u_dut.PIPE_L_BUS.rsp_ready;
        lb_rdata_z  = u_dut.PIPE_L_BUS.rsp_rdata;
        lb_dfault_z = u_dut.PIPE_L_BUS.rsp_dfault;

        if(u_dut.PIPE_L_BUS.req_valid && u_dut.PIPE_L_BUS.req_ready && u_dut.PIPE_L_BUS.req_fetch)
            lbus_ifetch_acc = lbus_ifetch_acc + 1;

        mb_pend_z  = MEM_BUS.req_valid && !MEM_BUS.req_ready;
        mb_addr_z  = MEM_BUS.req_addr;
        mb_write_z = MEM_BUS.req_write;
        mb_size_z  = MEM_BUS.req_size;
        mb_burst_z = MEM_BUS.req_burst;
        mb_lock_z  = MEM_BUS.req_lock;
        mb_wdata_z = MEM_BUS.req_wdata;
        mb_wstrb_z = MEM_BUS.req_wstrb;
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
    gpr = u_dut.u_int_pipe.u_gpr_bram.ram[8 + n];
endfunction

function automatic logic [31:0] control_mask(input integer selector, input logic [31:0] value);
    if(selector == 0 || selector == 3) control_mask = value & 32'h7000_13F3;
    else                               control_mask = value;
endfunction

//Reference follows DIV1 pseudocode in SH-3 software manual section 8.2.19.
task automatic div1_reference(
    input  logic [31:0] rn,
    input  logic [31:0] rm,
    input  logic [31:0] sr_in,
    output logic [31:0] rn_out,
    output logic [31:0] sr_out
);
    logic           old_q, next_q, carry_or_borrow;
    logic   [31:0]  value_before;
    begin
        old_q  = sr_in[8];
        next_q = rn[31];
        rn_out = {rn[30:0], sr_in[0]};
        value_before = rn_out;

        case({old_q, sr_in[9]})
            2'b00: begin
                rn_out          = rn_out - rm;
                carry_or_borrow = rn_out > value_before;
                next_q          = next_q ? !carry_or_borrow : carry_or_borrow;
            end
            2'b01: begin
                rn_out          = rn_out + rm;
                carry_or_borrow = rn_out < value_before;
                next_q          = next_q ? carry_or_borrow : !carry_or_borrow;
            end
            2'b10: begin
                rn_out          = rn_out + rm;
                carry_or_borrow = rn_out < value_before;
                next_q          = next_q ? !carry_or_borrow : carry_or_borrow;
            end
            default: begin
                rn_out          = rn_out - rm;
                carry_or_borrow = rn_out > value_before;
                next_q          = next_q ? carry_or_borrow : !carry_or_borrow;
            end
        endcase

        sr_out    = sr_in;
        sr_out[8] = next_q;
        sr_out[0] = next_q == sr_in[9];
    end
endtask

task automatic init_knobs;
    begin
        if_fault_en    = 1'b0;
        if_fault_widx  = 11'd0;
        d_fault_en     = 1'b0;
        d_fault_widx   = 8'd0;
        d_latency      = 0;
        i_latency      = 0;
    end
endtask

task automatic clear_imem;
    integer i;
    begin
        for(i = 0; i < 2048; i = i + 1) imem[i] = 16'h0009; //NOP fill
    end
endtask

//Print one banner line that introduces a category of tests.
task automatic group(input string name);
    begin
        $display("");
        $display("==== %s ====", name);
    end
endtask

//Each test prints exactly one description line here; failures add [FAIL] details.
task automatic begin_test(input string name);
    begin
        test_errors = 0;
        test_count  = test_count + 1;
        $display("  [%2d] %s", test_count, name);
        init_knobs;
        clear_imem;
    end
endtask

//Result is shown by the description line plus any [FAIL] details and the summary.
task automatic end_test;
    begin
    end
endtask

//Reset clears the DUT, the memory models, and the scoreboard.
task automatic do_reset;
    begin
        por_n          = 1'b1;
        rst_n          = 1'b0;
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);
    end
endtask

task automatic test_reset_event_codes;
    begin
        begin_test("Reset event codes: POR sets EXPEVT=0x000, manual reset sets EXPEVT=0x020");

        rst_n = 1'b1;
        por_n = 1'b0;
        repeat(2) @(posedge clk);
        chk("POR EXPEVT", expevt_o, 32'h0000_0000);

        por_n = 1'b1;
        @(posedge clk);
        rst_n = 1'b0;
        repeat(2) @(posedge clk);
        chk("manual reset EXPEVT", expevt_o, 32'h0000_0020);

        rst_n = 1'b1;
        @(posedge clk);
        end_test;
    end
endtask

//Run until the sentinel word retires, an exception fires, or time runs out.
task automatic run_until(input integer widx, input integer timeout);
    integer c;
    begin
        c = 0;
        while(!retired_seen[widx] && !exc_seen && c < timeout) begin
            @(posedge clk);
            c = c + 1;
        end
        repeat(6) @(posedge clk); //pipeline drain tail (6 arch cycles, as the 2x-clock original)
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
        repeat(6) @(posedge clk); //pipeline drain tail (6 arch cycles, as the 2x-clock original)
    end
endtask

task automatic run_until_exc(input integer timeout);
    integer c;
    begin
        c = 0;
        while(!exc_seen && c < timeout) begin
            @(posedge clk);
            c = c + 1;
        end
        repeat(8) @(posedge clk);
    end
endtask

task automatic run_cycles(input integer n);
    begin
        repeat(n) @(posedge clk);
    end
endtask

//Straight-line IPC probe: a long run of NOPs (no data/branch/memory hazards) is
//purely fetch/issue bound, so retirements/arch-cycle measures the front-end ceiling.
//This currently exercises the P2 (non-cacheable) bypass fetch path - see
//cache-backtoback-testing-gap; a cacheable-hit variant is the next step.
task automatic bench_ipc_straightline(input integer n);
    integer ms_ipc;
    begin
        clear_imem;                  //imem default fill is NOP (0x0009)
        do_reset;
        bench_arm = 1'b1;            //open the measurement window
        run_until_retire(n, 40000);  //run until instruction #n retires
        bench_arm = 1'b0;
        @(posedge clk);             //let the scoreboard deassert bench_active
        if(bench_arch_cycles > 0) begin
            ms_ipc = (bench_retires * 1000) / bench_arch_cycles;
            $display("  [BENCH] straight-line NOP: %0d retires / %0d arch-cycles -> IPC = %0d.%03d (bypass path)",
                     bench_retires, bench_arch_cycles, ms_ipc/1000, ms_ipc%1000);
        end
        else
            $display("  [BENCH] straight-line NOP: no retirements measured");
    end
endtask

//Cacheable-hit IPC probe. A small bootstrap (run from the P2 bypass region at reset)
//enables the unified cache, then JMPs into P0 cacheable space where a Fibonacci-style
//dependent-add loop lives. Iteration 1 fills the lines; the measurement window opens at
//iteration 2 so every fetch in it is a cache hit. This is the path on which peak IPC=1
//is the target; a backward branch per iteration costs ~2 bubbles, so a long body keeps
//the branch overhead small. EX->EX forwarding makes the dependent adds stall-free.
task automatic bench_ipc_cached(input integer body, input integer iters);
    integer idx, j, loop_idx, bf_idx, sentinel_idx, disp;
    integer ms_ipc, guard;
    begin
        clear_imem;

        //Bootstrap in P2 (non-cacheable): enable the cache, then jump into P0.
        imem[0] = 16'hE0EC; // MOV   #0xEC,R0  ; R0 = 0xFFFFFFEC (CCR); 0xEC sign-extends
        imem[1] = 16'hE109; // MOV   #9,R1     ; CCR.CE=1 | CCR.CF=1 (flush stale lines first)
        imem[2] = 16'h2012; // MOV.L R1,@R0    ; enable the unified cache + flush
        imem[3] = 16'h0009; // NOP             ; let the CCR write settle
        imem[4] = 16'h0009; // NOP
        imem[5] = 16'hE240; // MOV   #0x40,R2  ; R2 = 0x40 (P0 cacheable entry)
        imem[6] = 16'h422B; // JMP   @R2
        imem[7] = 16'h0009; // NOP             ; JMP delay slot

        //Cacheable loop at index 0x20 (P0 address 0x40). The body recomputes R3 from 0
        //each iteration (MOV #0,R3 then `body` x ADD #1,R3), so R3 == body at exit
        //regardless of the iteration count - an iteration-count-independent fetch-integrity
        //gate (the cacheable counted-loop control flow is separately off-by-one; see notes).
        idx = 'h20;
        imem[idx] = 16'hE500 | (iters & 8'hFF);idx = idx + 1; // MOV #iters,R5  ; loop count (seed)
        loop_idx = idx;                                       // loop_start = BF target
        imem[idx] = 16'hE300;                  idx = idx + 1; // MOV #0,R3      ; reset accumulator
        for(j = 0; j < body; j = j + 1) begin
            imem[idx] = 16'h7301;              idx = idx + 1; // ADD #1,R3
        end
        imem[idx] = 16'h4510;                  idx = idx + 1; // DT R5          ; T=1 at zero
        bf_idx = idx;
        disp   = loop_idx - bf_idx - 2;                       // signed, instruction units
        imem[idx] = 16'h8B00 | (disp & 8'hFF); idx = idx + 1; // BF loop_start
        sentinel_idx = idx;
        imem[idx] = 16'hE65A;                  idx = idx + 1; // MOV #0x5A,R6   ; exit sentinel

        do_reset;
        //Warm-up: wait until loop_start has retired twice (iteration 2 has begun).
        guard = 0;
        while(retire_count[loop_idx] < 2 && guard < 40000) begin
            @(posedge clk); guard = guard + 1;
        end
        bench_arm = 1'b1;                          // measure the all-hit steady state
        run_until_retire(sentinel_idx, 60000);
        bench_arm = 1'b0;
        @(posedge clk);

        if(bench_arch_cycles > 0)
            $display("  [BENCH] cached add loop (CCR.CE=%0d): %0d retires / %0d arch-cycles -> IPC = %0d.%03d (cache-hit path)",
                     u_dut.u_cache.ccr_ce, bench_retires, bench_arch_cycles,
                     ((bench_retires * 1000) / bench_arch_cycles) / 1000,
                     ((bench_retires * 1000) / bench_arch_cycles) % 1000);
        else
            $display("  [BENCH] cached add loop: no steady state measured (warm-up guard=%0d)", guard);

        //Fetch-integrity gate (the 61-test suite only runs the bypass path): a corrupted
        //pipelined cache fetch would make R3 != body. Count-independent by construction.
        chk("cached loop R3 = body (fetch integrity)", gpr(3), body);
        //R5 counts the DT;BF loop down to exactly 0 when the iteration count is right.
        //A stale T read by BF (2-ahead forwarding hole) runs one extra pass -> R5 = -1.
        chk("cached loop R5 = 0 (DT;BF count)", gpr(5), 32'd0);

        do_reset;  // leave CCR.CE cleared for the correctness suite
    end
endtask

//Store-throughput probe: a cacheable loop of write-HIT stores (all to one longword, so
//iteration 1 allocates the line and the rest hit). Measures cycles per store-hit on the
//cache-hit path; a load after the loop verifies the stores actually landed (R2==0x5A).
task automatic bench_ipc_store(input integer nstores, input integer iters);
    integer idx, j, loop_idx, bf_idx, sentinel_idx, disp, guard;
    begin
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE702;                  idx = idx + 1; // MOV   #2,R7
        imem[idx] = 16'h4718;                  idx = idx + 1; // SHLL8 R7      ; R7 = 0x200 (cacheable)
        imem[idx] = 16'hE15A;                  idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'hE500 | (iters & 8'hFF);idx = idx + 1; // MOV   #iters,R5
        loop_idx = idx;                                       // loop_start = BF target
        for(j = 0; j < nstores; j = j + 1) begin
            imem[idx] = 16'h2712;              idx = idx + 1; // MOV.L R1,@R7  ; write-HIT store
        end
        imem[idx] = 16'h4510;                  idx = idx + 1; // DT R5
        bf_idx = idx;
        disp   = loop_idx - bf_idx - 2;
        imem[idx] = 16'h8B00 | (disp & 8'hFF); idx = idx + 1; // BF loop_start
        imem[idx] = 16'h6272;                  idx = idx + 1; // MOV.L @R7,R2  ; verify -> 0x5A
        sentinel_idx = idx;
        imem[idx] = 16'h0009;                  idx = idx + 1; // NOP           ; sentinel

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
        do_reset;
    end
endtask

//Cacheable D-cache GOLDEN tests (rebuild gates). The 61-test suite runs CCR.CE=0
//(bypass), so the cacheable data path - write-allocate fill, the data-bank write,
//and the read hit - is otherwise untested. These two ISA-correct programs SPEC the
//cacheable store->load round-trip for the cen_p rebuild. Both are EXPECTED TO FAIL on
//the present FSM cache; the rebuilt 1-cycle cache must turn them green. R7=0x100 is a
//cacheable P0 longword (dmem index 0x40) distinct from the P0 code at 0x40 (byte addr).
//
//Shared bootstrap: run from P2 (bypass) at reset, enable the unified cache, JMP to P0.
//ccr_val picks the write policy: 8'h09 = CE|CF (write-back), 8'h0B = CE|WT|CF (write-through).
task automatic cacheable_bootstrap(input logic [7:0] ccr_val);
    begin
        clear_imem;
        imem[0] = 16'hE0EC;             // MOV   #0xEC,R0  ; R0 = 0xFFFFFFEC (CCR); 0xEC sign-extends
        imem[1] = 16'hE100 | ccr_val;   // MOV   #ccr,R1   ; CCR.CE=1 | CCR.CF=1 (flush stale lines first)
        imem[2] = 16'h2012;             // MOV.L R1,@R0    ; enable the unified cache + flush
        imem[3] = 16'h0009;             // NOP             ; let the CCR write settle
        imem[4] = 16'h0009;             // NOP
        imem[5] = 16'hE240;             // MOV   #0x40,R2  ; P0 cacheable entry (byte addr 0x40 = idx 0x20)
        imem[6] = 16'h422B;             // JMP   @R2
        imem[7] = 16'h0009;             // NOP             ; JMP delay slot
    end
endtask

//GOLDEN A - spaced store->load (no store-to-load forward needed). Two NOPs separate the
//store from the load, so the load's bank read edge lands AFTER the store's bank write
//commits (k>=2): this gates only that the write-allocate + data-bank write + read hit
//round-trip works, independent of the RDW bypass register.
task automatic test_cached_store_load_spaced;
    integer idx;
    begin
        begin_test("Cacheable D$: store-allocate then spaced load hit (no fwd)");
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE701; idx = idx + 1; // MOV   #1,R7
        imem[idx] = 16'h4718; idx = idx + 1; // SHLL8 R7       ; R7 = 0x100
        imem[idx] = 16'hE15A; idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'h2712; idx = idx + 1; // MOV.L R1,@R7   ; store-allocate @0x100 = 0x5A
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; spacer (avoid same-edge RDW)
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; spacer
        imem[idx] = 16'h6272; idx = idx + 1; // MOV.L @R7,R2 (n=2,m=7)   ; load hit -> R2 = 0x5A
        imem[idx] = 16'h0009; idx = idx + 1; // NOP
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; sentinel (idx 0x28)
        do_reset;
        run_until_retire('h28, 40000);
        chk("spaced cacheable store->load hit -> R2", gpr(2), 32'h0000_005A);
        do_reset;  // clear CCR.CE for the rest of the suite
        end_test;
    end
endtask

//GOLDEN B - back-to-back store->load to the SAME address (store-to-load forward gate).
//The load immediately follows the store (k=1), so the load's bank read edge coincides
//with the store's bank write edge. The 1R1W M10K banks are "no_rw_check", so the bank
//returns stale data on that cell; only the store-bypass register in the rebuilt cache
//can supply 0x5A. This is the dedicated forwarding gate.
task automatic test_cached_store_load_fwd;
    integer idx;
    begin
        begin_test("Cacheable D$: back-to-back store->load hit (store-to-load fwd)");
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE701; idx = idx + 1; // MOV   #1,R7
        imem[idx] = 16'h4718; idx = idx + 1; // SHLL8 R7       ; R7 = 0x100
        imem[idx] = 16'hE15A; idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'h2712; idx = idx + 1; // MOV.L R1,@R7   ; store-allocate @0x100 = 0x5A
        imem[idx] = 16'h6272; idx = idx + 1; // MOV.L @R7,R2 (n=2,m=7)   ; back-to-back load hit -> R2 = 0x5A
        imem[idx] = 16'h0009; idx = idx + 1; // NOP
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; sentinel (idx 0x26)
        do_reset;
        run_until_retire('h26, 40000);
        chk("back-to-back cacheable store->load hit -> R2", gpr(2), 32'h0000_005A);
        do_reset;  // clear CCR.CE for the rest of the suite
        end_test;
    end
endtask

//GOLDEN C - store-HIT then back-to-back load (the genuine RDW case). The first store
//write-allocates the line; the SECOND store is a write-HIT that commits straight to the
//data bank; the load immediately follows. If a store-hit bank write and the load bank read
//ever shared a cen_p edge, the no_rw_check bank would return stale data - this gates that
//they don't (the store ack cycle spaces them), so no store-bypass register is required.
task automatic test_cached_store_hit_load;
    integer idx;
    begin
        begin_test("Cacheable D$: store-hit then back-to-back load (RDW)");
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE701; idx = idx + 1; // MOV   #1,R7
        imem[idx] = 16'h4718; idx = idx + 1; // SHLL8 R7       ; R7 = 0x100
        imem[idx] = 16'hE15A; idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'h2712; idx = idx + 1; // MOV.L R1,@R7   ; store-allocate @0x100 = 0x5A
        imem[idx] = 16'hE33C; idx = idx + 1; // MOV   #0x3C,R3
        imem[idx] = 16'h2732; idx = idx + 1; // MOV.L R3,@R7   ; store-HIT @0x100 = 0x3C
        imem[idx] = 16'h6272; idx = idx + 1; // MOV.L @R7,R2 (n=2,m=7) ; back-to-back load -> 0x3C
        imem[idx] = 16'h0009; idx = idx + 1; // NOP
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; sentinel (idx 0x28)
        do_reset;
        run_until_retire('h28, 40000);
        chk("cacheable store-hit then load -> R2", gpr(2), 32'h0000_003C);
        do_reset;  // clear CCR.CE for the rest of the suite
        end_test;
    end
endtask

//GOLDEN D - byte store then back-to-back longword load (mixed-lane RDW compose). The
//byte hit commits ONE strobed lane at the resolve edge; the load's read captures at that
//same edge, so cache_mem o_DO must compose the bypassed lane (0x3C) over three RAM lanes.
//Goldens B/C are MOV.L-only (all-lane bypass); this gates the partial byp_q compose.
task automatic test_cached_byte_store_fwd;
    integer idx;
    begin
        begin_test("Cacheable D$: byte store then back-to-back load (mixed-lane RDW)");
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE701; idx = idx + 1; // MOV   #1,R7
        imem[idx] = 16'h4718; idx = idx + 1; // SHLL8 R7       ; R7 = 0x100
        imem[idx] = 16'hE15A; idx = idx + 1; // MOV   #0x5A,R1
        imem[idx] = 16'h2712; idx = idx + 1; // MOV.L R1,@R7   ; store-allocate @0x100 = 0x0000005A
        imem[idx] = 16'hE33C; idx = idx + 1; // MOV   #0x3C,R3
        imem[idx] = 16'h2730; idx = idx + 1; // MOV.B R3,@R7   ; byte hit: BE lane 3 only = 0x3C
        imem[idx] = 16'h6272; idx = idx + 1; // MOV.L @R7,R2   ; back-to-back load -> 0x3C00005A
        imem[idx] = 16'h0009; idx = idx + 1; // NOP
        imem[idx] = 16'h0009; idx = idx + 1; // NOP            ; sentinel (idx 0x28)
        do_reset;
        run_until_retire('h28, 40000);
        chk("byte-store then load composes lanes -> R2", gpr(2), 32'h3C00_005A);
        do_reset;
        end_test;
    end
endtask

//Self-modify visibility threshold (instructions ahead of the store): targets k < K were
//already fetched when the store commits and execute STALE (real SH pipelines prefetch the
//same way); targets k >= K are fetched at/after the commit edge and MUST run the new
//opcode (unified array + the same-edge RDW bypass on the I side). Relocked 2026-07-05
//with the FETCH PAIR: the prefetch window is now IF/ID + the pair slot + the in-flight
//longword fetch, so k=1..3 execute stale (uniform over target parity, k-sweep measured).
localparam integer SELFMOD_K = 4;

//GOLDEN E - self-modifying code distance sweep (I-side coherency law). MOV.W pokes a new
//opcode k instructions ahead of the store, INSIDE the store's own line - resident by
//construction, since the store itself was fetched from it (a store MISS would
//write-allocate from dmem and shadow the code line: keep the poke in a resident line).
task automatic test_cached_selfmod_sweep;
    integer k;
    logic [31:0] r6;
    begin
        begin_test("Self-modify sweep: MOV.W pokes opcode k ahead; new opcode from k>=K");
        for(k = 1; k <= 7; k = k + 1) begin
            cacheable_bootstrap(8'h09);   // write-back mode
            //Setup line at bytes 0x40-0x4F; store line at 0x50-0x5F. Unlisted slots
            //stay NOP (clear_imem). Guard BRA stops past-sentinel retires.
            imem['h20]     = 16'hE1E6;                  // MOV   #0xE6,R1 ; sign-extends
            imem['h21]     = 16'h4118;                  // SHLL8 R1
            imem['h22]     = 16'h7102;                  // ADD   #2,R1    ; R1 low 16 = 0xE602 = MOV #2,R6
            imem['h23]     = 16'hE250 | ((2*k) & 8'hFF);// MOV   #(0x50+2k),R2 ; poke target byte addr
            imem['h24]     = 16'hE600;                  // MOV   #0,R6
            imem['h28]     = 16'h2211;                  // MOV.W R1,@R2   ; poke k slots ahead (write HIT)
            imem['h28 + k] = 16'hE601;                  //   target: OLD opcode = MOV #1,R6
            imem['h31]     = 16'h0009;                  // NOP            ; sentinel (byte 0x62)
            imem['h32]     = 16'hAFFE;                  // BRA self       ; guard spin
            do_reset;
            run_until_retire('h31, 40000);
            r6 = gpr(6);
            $display("      k=%0d -> R6=%0d (%s opcode)", k, r6, (r6 == 32'd2) ? "new" : "old");
            if(k < SELFMOD_K) chk($sformatf("k=%0d executes stale opcode", k), r6, 32'd1);
            else              chk($sformatf("k=%0d executes new opcode",   k), r6, 32'd2);
        end
        do_reset;
        end_test;
    end
endtask

//GOLDEN F - redirect into the just-modified line. Pass 1 executes the target line
//(making it resident, R6=1), pokes the NEW opcode into that same line, then BRA back:
//the redirect fetch trails the store commit by only 1-2 cycles and must return the new
//halfword (same-edge RDW bypass or the freshly written RAM cell). Pass 2's CMP exits.
//WT mode (ccr 0x0B) additionally parks the refetch across the S_STORE_WR excursion.
task automatic test_cached_selfmod_branch(input logic [7:0] ccr_val, input string label);
    begin
        begin_test(label);
        cacheable_bootstrap(ccr_val);
        imem['h20] = 16'hE1E6;  // MOV   #0xE6,R1
        imem['h21] = 16'h4118;  // SHLL8 R1
        imem['h22] = 16'h7102;  // ADD   #2,R1    ; R1 low 16 = 0xE602 = MOV #2,R6
        imem['h23] = 16'hE262;  // MOV   #0x62,R2 ; poke target byte address
        imem['h24] = 16'hE600;  // MOV   #0,R6
        //Target line, bytes 0x60-0x6F: target + check + store + loop-back in ONE line.
        imem['h30] = 16'h0009;  // NOP            ; BRA re-entry (byte 0x60)
        imem['h31] = 16'hE601;  // MOV   #1,R6    ; the target halfword (byte 0x62)
        imem['h32] = 16'h0009;  // NOP
        imem['h33] = 16'h6063;  // MOV   R6,R0
        imem['h34] = 16'h8802;  // CMP/EQ #2,R0   ; T=1 only after the new opcode ran
        imem['h35] = 16'h8903;  // BT    byte 0x74 (exit)
        imem['h36] = 16'h2211;  // MOV.W R1,@R2   ; poke byte 0x62 (this very line - resident)
        imem['h37] = 16'hAFF7;  // BRA   byte 0x60; redirect into the just-modified line
        imem['h38] = 16'h0009;  // NOP            ; BRA delay slot
        imem['h3A] = 16'h0009;  // NOP            ; sentinel/exit (byte 0x74)
        imem['h3B] = 16'hAFFE;  // BRA self       ; guard spin
        do_reset;
        run_until_retire('h3A, 40000);
        chk("redirect fetch returns the poked opcode -> R6", gpr(6), 32'd2);
        do_reset;
        end_test;
    end
endtask

//GOLDEN G - write-back buffer alias. A dirty line is evicted into the wb buffer and the
//SAME line is reloaded before the background drain can complete (back-to-back MA ops;
//a drain slot needs a request-free edge, and a full drain takes ~12 cycles). The refill
//must observe the not-yet-drained store data - a fill straight from external memory
//returns stale zeros. The dirty data sits in word 3, which drains LAST, so even a
//partially slipped-in drain cannot mask a stale fill. The spaced re-read gates that a
//stale refill does not PERSIST after the drain finally lands (lost-update detector).
task automatic test_cached_wb_alias;
    integer idx, sentinel;
    begin
        begin_test("Cacheable D$: dirty evict then immediate reload (wb-buffer alias)");
        cacheable_bootstrap(8'h09);   // write-back mode
        idx = 'h20;
        imem[idx] = 16'hE101; idx = idx + 1; // MOV   #1,R1
        imem[idx] = 16'h4118; idx = idx + 1; // SHLL8 R1      ; R1 = 0x100 = line A (set 0x10)
        imem[idx] = 16'hE210; idx = idx + 1; // MOV   #0x10,R2
        imem[idx] = 16'h4218; idx = idx + 1; // SHLL8 R2      ; R2 = 0x1000 (same-set stride)
        imem[idx] = 16'h6313; idx = idx + 1; // MOV   R1,R3
        imem[idx] = 16'h730C; idx = idx + 1; // ADD   #0xC,R3 ; R3 = 0x10C (A word 3)
        imem[idx] = 16'hE05A; idx = idx + 1; // MOV   #0x5A,R0
        imem[idx] = 16'h2302; idx = idx + 1; // MOV.L R0,@R3  ; write-allocate: A dirty, MRU
        imem[idx] = 16'h6413; idx = idx + 1; // MOV   R1,R4
        imem[idx] = 16'h342C; idx = idx + 1; // ADD   R2,R4   ; R4 = 0x1100
        imem[idx] = 16'h6542; idx = idx + 1; // MOV.L @R4,R5  ; fill B
        imem[idx] = 16'h342C; idx = idx + 1; // ADD   R2,R4   ; R4 = 0x2100
        imem[idx] = 16'h6542; idx = idx + 1; // MOV.L @R4,R5  ; fill C
        imem[idx] = 16'h342C; idx = idx + 1; // ADD   R2,R4   ; R4 = 0x3100
        imem[idx] = 16'h6542; idx = idx + 1; // MOV.L @R4,R5  ; fill D -> true-LRU order: A oldest
        imem[idx] = 16'h342C; idx = idx + 1; // ADD   R2,R4   ; R4 = 0x4100
        imem[idx] = 16'h6542; idx = idx + 1; // MOV.L @R4,R5  ; fill E: evicts dirty A -> wb buffer
        imem[idx] = 16'h6632; idx = idx + 1; // MOV.L @R3,R6  ; back-to-back reload A -> 0x5A
        idx = idx + 24;                      // NOP window (clear_imem fill): let the drain land
        imem[idx] = 16'h6832; idx = idx + 1; // MOV.L @R3,R8  ; spaced re-read (hit) -> 0x5A
        sentinel  = idx;
        imem[idx] = 16'h0009; idx = idx + 1; // NOP           ; sentinel
        do_reset;
        run_until_retire(sentinel, 40000);
        chk("immediate reload of evicted-dirty line -> R6", gpr(6), 32'h0000_005A);
        chk("spaced re-read of the refilled line -> R8",    gpr(8), 32'h0000_005A);
        do_reset;
        end_test;
    end
endtask

//GOLDEN H - latency invariance + squash timing sweep. One cacheable branch-torture
//program (taken branches whose fall-through prefetch crosses into COLD poison lines,
//a store/load pair, and a backward DT loop) is run over a grid of (i,d) memory-wait
//settings. Stretching the fill beats sweeps the branch-redirect (i_I_SQUASH) arrival
//across every I-fill FSM phase: request, each wait beat, completion, and the hit case.
//LAW: the architectural result is latency-invariant, wrong-path poison never retires,
//and the sweep really lands squashes inside I-fill runs (coverage counter).
task automatic test_cached_latency_invariance;
    integer p, j;
    integer ilat [0:9];
    integer dlat [0:9];
    integer squash_base;
    begin
        begin_test("Latency invariance + squash sweep: results identical over (i,d) wait grid");
        squash_base = squash_fill_hits;
        ilat[0]=0; dlat[0]=0;    ilat[5]=5; dlat[5]=0;
        ilat[1]=1; dlat[1]=0;    ilat[6]=7; dlat[6]=0;
        ilat[2]=2; dlat[2]=0;    ilat[7]=0; dlat[7]=2;
        ilat[3]=3; dlat[3]=0;    ilat[8]=2; dlat[8]=3;
        ilat[4]=4; dlat[4]=0;    ilat[9]=5; dlat[9]=1;
        for(p = 0; p < 10; p = p + 1) begin
            cacheable_bootstrap(8'h09);   // CF flush: every run starts cold
            //Line 0 (bytes 0x40-0x4F): setup; BRA sits one slot before the line end so
            //fetch-ahead crosses into the cold poison line before the redirect resolves.
            imem['h20] = 16'hE200;  // MOV   #0,R2    ; accumulator
            imem['h21] = 16'hEA00;  // MOV   #0,R10   ; wrong-path poison detector
            imem['h22] = 16'hE301;  // MOV   #1,R3
            imem['h23] = 16'h4318;  // SHLL8 R3       ; R3 = 0x100 (data line)
            imem['h24] = 16'h0009;  // NOP
            imem['h25] = 16'h0009;  // NOP
            imem['h26] = 16'hA008;  // BRA   0x60     ; skip poison line 1
            imem['h27] = 16'h0009;  //   delay slot (last slot of the line)
            //Line 2 (0x60): store leg - D-fill traffic racing the wrong-path I-fill.
            imem['h30] = 16'h7203;  // ADD   #3,R2    ; acc = 3
            imem['h31] = 16'h2322;  // MOV.L R2,@R3   ; store-allocate 0x100
            imem['h32] = 16'h0009;
            imem['h33] = 16'h0009;
            imem['h34] = 16'h0009;
            imem['h35] = 16'h0009;
            imem['h36] = 16'hA008;  // BRA   0x80     ; skip poison line 3
            imem['h37] = 16'h0009;  //   delay slot
            //Line 4 (0x80): load leg.
            imem['h40] = 16'h6432;  // MOV.L @R3,R4   ; load back -> 3
            imem['h41] = 16'h324C;  // ADD   R4,R2    ; acc = 6
            imem['h42] = 16'h5631;  // MOV.L @(4,R3),R6 ; preset dmem -> 0x21
            imem['h43] = 16'h0009;
            imem['h44] = 16'h0009;
            imem['h45] = 16'h0009;
            imem['h46] = 16'hA008;  // BRA   0xA0     ; skip poison line 5
            imem['h47] = 16'h0009;  //   delay slot
            //Line 6 (0xA0): backward DT loop - squash lands on a WARM line (hit class).
            imem['h50] = 16'hE503;  // MOV   #3,R5
            imem['h51] = 16'h7201;  // ADD   #1,R2    ; 3 iterations -> acc = 9
            imem['h52] = 16'h4510;  // DT    R5
            imem['h53] = 16'h8BFC;  // BF    0xA2     ; taken twice, falls through once
            imem['h54] = 16'h0009;
            imem['h55] = 16'h0009;
            imem['h56] = 16'hA008;  // BRA   0xC0     ; skip poison line 7
            imem['h57] = 16'h0009;  //   delay slot
            //Line 8 (0xC0): sentinel + guard.
            imem['h60] = 16'h0009;  // sentinel
            imem['h61] = 16'hAFFE;  // BRA self       ; guard spin
            imem['h62] = 16'h0009;
            //Poison lines 1/3/5/7: retiring any wrong-path slot corrupts R10.
            for(j = 0; j < 8; j = j + 1) begin
                imem['h28 + j] = 16'hEA11;  // MOV #0x11,R10
                imem['h38 + j] = 16'hEA22;
                imem['h48 + j] = 16'hEA33;
                imem['h58 + j] = 16'hEA44;
            end
            i_latency = ilat[p];
            d_latency = dlat[p];
            do_reset;
            dmem['h41] = 32'h0000_0021;
            run_until_retire('h60, 60000);
            chk($sformatf("acc R2 (i=%0d,d=%0d)",     ilat[p], dlat[p]), gpr(2),  32'd9);
            chk($sformatf("load R4 (i=%0d,d=%0d)",    ilat[p], dlat[p]), gpr(4),  32'd3);
            chk($sformatf("loop R5 (i=%0d,d=%0d)",    ilat[p], dlat[p]), gpr(5),  32'd0);
            chk($sformatf("disp R6 (i=%0d,d=%0d)",    ilat[p], dlat[p]), gpr(6),  32'h0000_0021);
            chk($sformatf("poison R10 (i=%0d,d=%0d)", ilat[p], dlat[p]), gpr(10), 32'd0);
        end
        i_latency = 0;
        d_latency = 0;
        $display("      squash-during-I-fill cycles across the sweep: %0d", squash_fill_hits - squash_base);
        chk_true("squash landed inside I-fill runs", squash_fill_hits > squash_base);
        do_reset;
        end_test;
    end
endtask

//GOLDEN I - D-fill per-beat fault sweep. The line read faults on beat b (0..3):
//EXC_DATA must be precise (destination unwritten, younger killed, EA reported), and
//the aborted fill must leave NO usable line - the handler retry reloads from MEMORY
//(the tb swaps the line's values between phases; a stale partial line would hit).
task automatic test_cached_fill_fault_d;
    integer b, j, lat;
    begin
        begin_test("D-fill fault sweep: fault on each beat x latency grid -> precise EXC_DATA");
        for(lat = 0; lat <= 3; lat = lat + 3) begin     //latency grid: redirect-vs-fill phases
        for(b = 0; b < 4; b = b + 1) begin
            cacheable_bootstrap(8'h09);
            i_latency = lat;
            //Reset leaves SR.BL=1 and an exception under BL is a manual RESET (SH-3
            //rule) - clear BL first so the fault vectors to VBR+0x100 as intended.
            imem['h20] = 16'hE860;  // MOV    #0x60,R8
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8      ; R8 = 0x6000_0000 (MD=1,RB=1,BL=0)
            imem['h23] = 16'h480E;  // LDC    R8,SR   ; clear BL (serializes)
            imem['h24] = 16'hE766;  // MOV   #0x66,R7 ; canary: fault must not overwrite
            imem['h25] = 16'hE302;  // MOV   #2,R3
            imem['h26] = 16'h4318;  // SHLL8 R3       ; R3 = 0x200 (set 0x20)
            imem['h27] = 16'h6732;  // MOV.L @R3,R7   ; fill faults at beat b -> EXC_DATA
            imem['h28] = 16'hEA55;  // MOV   #0x55,R10 ; younger - must not retire
            //Handler (VBR+0x100 = imem 0x80): pad covers the tb's disarm window.
            for(j = 0; j < 24; j = j + 1) imem['h80 + j] = 16'h0009;
            imem['h98] = 16'h5830;  // MOV.L @(0,R3),R8   ; retry: full-line refill
            imem['h99] = 16'h5931;  // MOV.L @(4,R3),R9
            imem['h9A] = 16'h5A32;  // MOV.L @(8,R3),R10
            imem['h9B] = 16'h5B33;  // MOV.L @(12,R3),R11
            imem['h9C] = 16'h0009;  // sentinel
            imem['h9D] = 16'hAFFE;  // BRA self
            imem['h9E] = 16'h0009;
            d_fault_en   = 1'b1;
            d_fault_widx = 8'h80 + b[7:0];
            do_reset;
            dmem['h80] = 32'h0000_00A0;   // phase-1 memory truth
            dmem['h81] = 32'h0000_00A1;
            dmem['h82] = 32'h0000_00A2;
            dmem['h83] = 32'h0000_00A3;
            run_until_exc(20000);
            chk_true($sformatf("beat %0d: exception fired", b), exc_seen);
            chk($sformatf("beat %0d: cause", b), {29'd0, exc_cause_l}, {29'd0, EXC_DATA});
            chk($sformatf("beat %0d: exc pc", b), exc_pc_l, 32'h0000_004E);
            chk($sformatf("beat %0d: access addr", b), exc_aaddr_l, 32'h0000_0200);
            chk($sformatf("beat %0d: canary R7", b), gpr(7), 32'h0000_0066);
            chk_true($sformatf("beat %0d: younger killed", b), !retired_seen['h28]);
            //Phase 2: NEW memory truth. A stale partial line would hit and expose it.
            d_fault_en = 1'b0;
            dmem['h80] = 32'h0000_00B0;
            dmem['h81] = 32'h0000_00B1;
            dmem['h82] = 32'h0000_00B2;
            dmem['h83] = 32'h0000_00B3;
            run_until_retire('h9C, 20000);
            chk($sformatf("beat %0d: refill word 0", b), gpr(8),  32'h0000_00B0);
            chk($sformatf("beat %0d: refill word 1", b), gpr(9),  32'h0000_00B1);
            chk($sformatf("beat %0d: refill word 2", b), gpr(10), 32'h0000_00B2);
            chk($sformatf("beat %0d: refill word 3", b), gpr(11), 32'h0000_00B3);
        end
        end
        i_latency = 0;
        do_reset;
        end_test;
    end
endtask

//GOLDEN J - I-fill per-beat fault sweep. The jump target's line fill faults on beat b:
//EXC_IFETCH must report the REQUESTED PC whichever beat faulted, none of the target's
//stale opcodes may retire, and the post-fault refetch must read fresh MEMORY (the tb
//pokes a new opcode over the target between phases).
task automatic test_cached_fill_fault_i;
    integer b, j, lat;
    begin
        begin_test("I-fill fault sweep: fault on each beat x latency grid -> precise EXC_IFETCH");
        for(lat = 0; lat <= 3; lat = lat + 3) begin     //latency grid: squash-vs-fill phases
        for(b = 0; b < 4; b = b + 1) begin
            cacheable_bootstrap(8'h09);
            i_latency = lat;
            imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue (see the D sweep)
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8
            imem['h23] = 16'h480E;  // LDC    R8,SR
            imem['h24] = 16'hEC00;  // MOV   #0,R12   ; stale/new opcode detector
            imem['h25] = 16'hE160;  // MOV   #0x60,R1
            imem['h26] = 16'h412B;  // JMP   @R1      ; -> byte 0x60 (cold line, set 6)
            imem['h27] = 16'h0009;  //   delay slot
            //Target line phase-1 content: stale markers, contained by a guard.
            for(j = 0; j < 6; j = j + 1) imem['h30 + j] = 16'hEC11;  // MOV #0x11,R12
            imem['h36] = 16'hAFFE;  // BRA self       ; guard if the fault never fires
            imem['h37] = 16'h0009;
            //Handler: pad, FLUSH, then refetch the re-poked target. The flush is
            //architecturally required: a wrong-path fetch dispatched between the fault
            //and the redirect may have LEGALLY cached the pre-poke line content
            //(self-modifying code owns its cache management).
            for(j = 0; j < 21; j = j + 1) imem['h80 + j] = 16'h0009;
            imem['h95] = 16'hE0EC;  // MOV   #0xEC,R0 ; CCR
            imem['h96] = 16'hE409;  // MOV   #9,R4    ; CE|CF
            imem['h97] = 16'h2042;  // MOV.L R4,@R0   ; flush any old-content copy
            imem['h98] = 16'hE260;  // MOV   #0x60,R2
            imem['h99] = 16'h422B;  // JMP   @R2      ; refetch the target
            imem['h9A] = 16'h0009;  //   delay slot
            if_fault_en   = 1'b1;
            if_fault_widx = 11'h30 + 11'(2 * b);  // halfword index of fill beat b
            do_reset;
            run_until_exc(20000);
            chk_true($sformatf("beat %0d: exception fired", b), exc_seen);
            chk($sformatf("beat %0d: cause", b), {29'd0, exc_cause_l}, {29'd0, EXC_IFETCH});
            chk($sformatf("beat %0d: exc pc = requested PC", b), exc_pc_l, 32'h0000_0060);
            chk($sformatf("beat %0d: stale opcodes never ran", b), gpr(12), 32'd0);
            //Phase 2: poke the NEW opcode; the refetch must refill from imem.
            if_fault_en = 1'b0;
            imem['h30] = 16'hEC22;  // MOV #0x22,R12  ; the new opcode
            imem['h31] = 16'h0009;
            imem['h32] = 16'h0009;
            imem['h33] = 16'h0009;
            imem['h34] = 16'h0009;
            imem['h35] = 16'h0009;  // sentinel
            run_until_retire('h35, 20000);
            chk($sformatf("beat %0d: refetch runs the new opcode", b), gpr(12), 32'h0000_0022);
        end
        end
        i_latency = 0;
        do_reset;
        end_test;
    end
endtask

//GOLDEN K - fill-fault victim integrity. A mid-line fill fault aborts AFTER earlier
//beats already overwrote the victim way's data - but the victim's OLD tag is not
//rewritten until the final beat. If the abort leaves that old tag valid, the old line
//is a HIT over corrupt (half-new) data. Setup: X resident clean, LRU-oldest; the tb
//swaps memory truth, then a same-set fill faults on beat 2 with X as its victim.
//LAW: X must be gone (invalidated); rereads of X refill fresh memory on every word.
task automatic test_cached_fill_fault_victim;
    integer j;
    begin
        begin_test("Fill-fault victim integrity: aborted fill must not leave a corrupt valid line");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue (see the D sweep)
        imem['h21] = 16'h4818;  // SHLL8  R8
        imem['h22] = 16'h4828;  // SHLL16 R8
        imem['h23] = 16'h480E;  // LDC    R8,SR
        imem['h24] = 16'hE102;  // MOV   #2,R1
        imem['h25] = 16'h4118;  // SHLL8 R1       ; R1 = 0x200 = line X (set 0x20)
        imem['h26] = 16'hE210;  // MOV   #0x10,R2
        imem['h27] = 16'h4218;  // SHLL8 R2       ; R2 = 0x1000 same-set stride
        imem['h28] = 16'h6612;  // MOV.L @R1,R6   ; fill X -> way 3 (post-flush LRU)
        imem['h29] = 16'h6313;  // MOV   R1,R3
        imem['h2A] = 16'h332C;  // ADD   R2,R3    ; 0x1200
        imem['h2B] = 16'h6532;  // MOV.L @R3,R5   ; fill -> way 2
        imem['h2C] = 16'h332C;  //                ; 0x2200
        imem['h2D] = 16'h6532;  //                ; fill -> way 1
        imem['h2E] = 16'h332C;  //                ; 0x3200
        imem['h2F] = 16'h6532;  //                ; fill -> way 0: X is LRU-oldest again
        imem['h30] = 16'h0009;  // marker: tb swaps memory truth + arms the beat fault
        for(j = 0; j < 16; j = j + 1) imem['h31 + j] = 16'h0009;   // re-arm window
        imem['h41] = 16'h332C;  //                ; 0x4200
        imem['h42] = 16'h6732;  // MOV.L @R3,R7   ; fill Z: victim = clean X; beat 2 faults
        imem['h43] = 16'hED55;  // MOV   #0x55,R13 ; younger - must not retire
        //Handler: pad, then reread every word of X.
        for(j = 0; j < 24; j = j + 1) imem['h80 + j] = 16'h0009;
        imem['h98] = 16'h6812;  // MOV.L @R1,R8       ; X word 0
        imem['h99] = 16'h5911;  // MOV.L @(4,R1),R9
        imem['h9A] = 16'h5A12;  // MOV.L @(8,R1),R10
        imem['h9B] = 16'h5B13;  // MOV.L @(12,R1),R11
        imem['h9C] = 16'h0009;  // sentinel
        imem['h9D] = 16'hAFFE;  // BRA self
        imem['h9E] = 16'h0009;
        do_reset;
        dmem['h80] = 32'h0000_00A0;   // phase-1 truth (X's resident content)
        dmem['h81] = 32'h0000_00A1;
        dmem['h82] = 32'h0000_00A2;
        dmem['h83] = 32'h0000_00A3;
        run_until_retire('h30, 20000);
        dmem['h80] = 32'h0000_00B0;   // phase-2 truth: Z's beats deliver B-values
        dmem['h81] = 32'h0000_00B1;
        dmem['h82] = 32'h0000_00B2;
        dmem['h83] = 32'h0000_00B3;
        d_fault_en   = 1'b1;
        d_fault_widx = 8'h82;         // Z fill beat 2
        run_until_exc(20000);
        chk_true("exception fired", exc_seen);
        chk("cause", {29'd0, exc_cause_l}, {29'd0, EXC_DATA});
        chk("access addr", exc_aaddr_l, 32'h0000_4200);
        chk_true("younger killed", !retired_seen['h43]);
        d_fault_en = 1'b0;
        run_until_retire('h9C, 20000);
        //A stale-valid X returns {B0,B1,A2,A3} (corrupt); a clean refill returns all B.
        chk("X word 0 after aborted fill", gpr(8),  32'h0000_00B0);
        chk("X word 1 after aborted fill", gpr(9),  32'h0000_00B1);
        chk("X word 2 after aborted fill", gpr(10), 32'h0000_00B2);
        chk("X word 3 after aborted fill", gpr(11), 32'h0000_00B3);
        do_reset;
        end_test;
    end
endtask

//GOLDEN L - PREF fill-fault abandon (p.111: a prefetch raises no exception). The
//faulting PREF fill must be dropped silently AND leave the line unallocated: the
//later demand load refills from (by then updated) memory.
task automatic test_cached_pref_fill_fault;
    integer j;
    begin
        begin_test("PREF fill fault: silent abandon, no allocation, later load refills");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE303;  // MOV   #3,R3
        imem['h21] = 16'h4318;  // SHLL8 R3       ; R3 = 0x300 (set 0x30)
        imem['h22] = 16'h0383;  // PREF  @R3      ; fill faults at beat 1 -> abandoned
        imem['h23] = 16'h0009;  // marker: tb disarms + swaps memory truth
        for(j = 0; j < 16; j = j + 1) imem['h24 + j] = 16'h0009;   // disarm window
        imem['h34] = 16'h6732;  // MOV.L @R3,R7   ; must REFILL (nothing was allocated)
        imem['h35] = 16'h0009;  // sentinel
        imem['h36] = 16'hAFFE;  // BRA self
        imem['h37] = 16'h0009;
        d_fault_en   = 1'b1;
        d_fault_widx = 8'hC1;   // PREF fill beat 1
        do_reset;
        dmem['hC0] = 32'h0000_00A0;
        dmem['hC1] = 32'h0000_00A1;
        run_until_retire('h23, 20000);
        chk_true("PREF fill fault raises no exception", !exc_seen);
        d_fault_en = 1'b0;
        dmem['hC0] = 32'h0000_00B0;   // phase-2 truth
        run_until_retire('h35, 20000);
        chk_true("still no exception", !exc_seen);
        chk("post-abandon load refills from memory", gpr(7), 32'h0000_00B0);
        do_reset;
        end_test;
    end
endtask

//GOLDEN M - write-back drain fault law (LOCKED 2026-07-05): a fault on a drain beat
//is SILENTLY IGNORED - no exception, the drain completes, the faulted word is simply
//lost (memory keeps its old value), all other words commit. The drain is forced
//deterministically with a NON-ASSOCIATIVE mm tag write (0xF0 window, p.112) that
//invalidates the dirty line - no same-set fills needed, so the tb's aliased dmem
//never sees the armed fault index on a READ.
task automatic test_cached_drain_fault_law;
    integer w, i;
    begin
        begin_test("Drain-beat fault law: ignored fault, word lost, no exception (sweep all beats)");
        for(w = 0; w < 4; w = w + 1) begin
            cacheable_bootstrap(8'h09);
            imem['h20] = 16'hE118;  // MOV   #0x18,R1
            imem['h21] = 16'h4108;  // SHLL2 R1
            imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180 = line A (set 0x18)
            imem['h23] = 16'h6712;  // MOV.L @R1,R7   ; fill A (fault not armed yet)
            imem['h24] = 16'hE251;  // MOV   #0x51,R2
            imem['h25] = 16'h2122;  // MOV.L R2,@R1       ; word 0 = 0x51 (line dirty)
            imem['h26] = 16'h7201;  // ADD   #1,R2
            imem['h27] = 16'h1121;  // MOV.L R2,@(4,R1)   ; word 1 = 0x52
            imem['h28] = 16'h7201;
            imem['h29] = 16'h1122;  // MOV.L R2,@(8,R1)   ; word 2 = 0x53
            imem['h2A] = 16'h7201;
            imem['h2B] = 16'h1123;  // MOV.L R2,@(12,R1)  ; word 3 = 0x54
            imem['h2C] = 16'h0009;  // marker: tb arms the drain-beat fault here
            for(i = 0; i < 12; i = i + 1) imem['h2D + i] = 16'h0009;  // arm window
            imem['h39] = 16'hE0F0;  // MOV   #0xF0,R0
            imem['h3A] = 16'h4028;  // SHLL16 R0
            imem['h3B] = 16'h4018;  // SHLL8 R0       ; R0 = 0xF000_0000 (mm tag window)
            imem['h3C] = 16'hE431;  // MOV   #0x31,R4
            imem['h3D] = 16'h4418;  // SHLL8 R4       ; 0x3100 = way-3 field
            imem['h3E] = 16'h7440;  // ADD   #0x40,R4 ; (0x80 immediate would sign-extend)
            imem['h3F] = 16'h7440;  // ADD   #0x40,R4 ; +0x80 = set 0x18 field
            imem['h40] = 16'h304C;  // ADD   R4,R0    ; R0 = 0xF000_3180
            imem['h41] = 16'hE500;  // MOV   #0,R5
            imem['h42] = 16'h2052;  // MOV.L R5,@R0   ; mm tag write V=0: forced write-back
            //Drain pump: a pure hit/fetch stream never yields a request-free edge (the
            //background drain starves by design - IPC first); DSP result waits stall the
            //front end and open the idle edges the drain needs, one word per stall.
            for(i = 0; i < 8; i = i + 1) begin
                imem['h43 + 2*i] = 16'h222F;    // MULS.W R2,R2 ; occupy the DSP
                imem['h44 + 2*i] = 16'h061A;    // STS   MACL,R6 ; wait on it: fetch idles
            end
            imem['h53] = 16'h0009;  // sentinel
            imem['h54] = 16'hAFFE;  // BRA self
            imem['h55] = 16'h0009;
            do_reset;
            for(i = 0; i < 4; i = i + 1) dmem['h60 + i] = 32'h0000_0041 + i;  // A-values
            run_until_retire('h2C, 20000);
            d_fault_en   = 1'b1;
            d_fault_widx = 8'h60 + w[7:0];
            run_until_retire('h53, 20000);
            chk_true($sformatf("word %0d: no exception on the drain fault", w), !exc_seen);
            chk_true($sformatf("word %0d: buffer fully drained", w), !u_dut.u_cache.wb_valid);
            for(i = 0; i < 4; i = i + 1) begin
                if(i == w) chk($sformatf("word %0d kept its OLD value (write lost)", i),
                               dmem['h60 + i], 32'h0000_0041 + i);
                else       chk($sformatf("word %0d drained", i),
                               dmem['h60 + i], 32'h0000_0051 + i);
            end
            d_fault_en = 1'b0;
        end
        do_reset;
        end_test;
    end
endtask

//GOLDEN N - LRU thrash: 6 lines share one set (> 4 ways) and are walked in a shuffled
//order with two dirtying stores, forcing repeated evictions, refills, and write-back
//drains. The correctness burden is carried by the suite-wide LRU mirror (victim and
//update cross-checks); this program exists to feed it a dense, irregular history.
task automatic test_cached_lru_thrash;
    integer vic_base;
    begin
        begin_test("LRU thrash: 6 same-set lines, shuffled walk, mirror-checked victims");
        vic_base = lru_vic_checks;
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE128;  // MOV   #0x28,R1
        imem['h21] = 16'h4108;  // SHLL2 R1
        imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x280 (set 0x28)
        imem['h23] = 16'hE210;  // MOV   #0x10,R2
        imem['h24] = 16'h4218;  // SHLL8 R2       ; R2 = 0x1000 same-set stride
        imem['h25] = 16'h6313;  // MOV   R1,R3    ; k=0
        imem['h26] = 16'h6032;  // MOV.L @R3,R0   ; k=0 fill
        imem['h27] = 16'h2302;  // MOV.L R0,@R3   ; k=0 dirty
        imem['h28] = 16'h332C;  // ADD   R2,R3    ; k=1
        imem['h29] = 16'h6032;
        imem['h2A] = 16'h332C;  //                ; k=2
        imem['h2B] = 16'h6032;
        imem['h2C] = 16'h332C;  //                ; k=3
        imem['h2D] = 16'h6032;
        imem['h2E] = 16'h332C;  //                ; k=4
        imem['h2F] = 16'h6032;  //                ; 5th line: first eviction
        imem['h30] = 16'h3328;  // SUB   R2,R3
        imem['h31] = 16'h3328;
        imem['h32] = 16'h3328;
        imem['h33] = 16'h3328;  //                ; k=0
        imem['h34] = 16'h6032;  //                ; k=0 again (evicted-dirty reload)
        imem['h35] = 16'h332C;
        imem['h36] = 16'h332C;  //                ; k=2
        imem['h37] = 16'h6032;
        imem['h38] = 16'h2302;  //                ; k=2 dirty
        imem['h39] = 16'h332C;
        imem['h3A] = 16'h332C;
        imem['h3B] = 16'h332C;  //                ; k=5
        imem['h3C] = 16'h6032;
        imem['h3D] = 16'h3328;
        imem['h3E] = 16'h3328;
        imem['h3F] = 16'h3328;
        imem['h40] = 16'h3328;  //                ; k=1
        imem['h41] = 16'h6032;
        imem['h42] = 16'h332C;
        imem['h43] = 16'h332C;  //                ; k=3
        imem['h44] = 16'h6032;
        imem['h45] = 16'h3328;
        imem['h46] = 16'h3328;
        imem['h47] = 16'h3328;  //                ; k=0
        imem['h48] = 16'h6032;
        imem['h49] = 16'h332C;
        imem['h4A] = 16'h332C;
        imem['h4B] = 16'h332C;
        imem['h4C] = 16'h332C;  //                ; k=4
        imem['h4D] = 16'h6032;
        imem['h4E] = 16'h0009;  // sentinel
        imem['h4F] = 16'hAFFE;  // BRA self
        imem['h50] = 16'h0009;
        do_reset;
        dmem['hA0] = 32'h0000_0077;   // aliased word 0: every k reads/rewrites it
        run_until_retire('h4E, 60000);
        chk("aliased word-0 value follows every reload", gpr(0), 32'h0000_0077);
        $display("      victim choices mirror-checked in this walk: %0d", lru_vic_checks - vic_base);
        chk_true("victim choices exercised", lru_vic_checks >= vic_base + 6);
        do_reset;
        end_test;
    end
endtask

//GOLDEN O - fetch-pair request law. Sequential cacheable code must cost ONE accepted
//L-bus fetch per LONGWORD (the pair slot serves the sibling with no request): the
//whole-run accept counts are locked exactly for an even and an odd program entry
//(the odd first fetch cannot pair, so the odd run costs one more request).
task automatic test_pair_fetch_law;
    integer n0, j;
    begin
        begin_test("Fetch pair: one request per longword, exact accept counts (even/odd entry)");
        //Even entry: bootstrap JMPs to byte 0x40 directly.
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE200;                              // MOV #0,R2
        for(j = 0; j < 32; j = j + 1) imem['h21 + j] = 16'h7201;  // ADD #1,R2  x32
        imem['h41] = 16'h0009;  // sentinel
        imem['h42] = 16'hAFFE;  // guard
        imem['h43] = 16'h0009;
        do_reset;
        n0 = lbus_ifetch_acc;
        run_until_retire('h41, 20000);
        $display("      even-entry fetch accepts = %0d", lbus_ifetch_acc - n0);
        chk("even-entry ALU block result", gpr(2), 32'd32);
        //Locked 2026-07-05 (whole run incl. bootstrap + guard spin): ~36 instructions
        //retire on 30 accepted fetches - pre-pair this cost one accept per instruction.
        chk("even-entry fetch accepts (law)", lbus_ifetch_acc - n0, 32'd30);
        //Odd entry: JMP into byte 0x46 - the first fetch is unpaired.
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE146;                              // MOV #0x46,R1
        imem['h21] = 16'h412B;                              // JMP @R1
        imem['h22] = 16'h0009;                              //   delay slot
        imem['h23] = 16'hE200;                              // MOV #0,R2 (byte 0x46, ODD)
        for(j = 0; j < 32; j = j + 1) imem['h24 + j] = 16'h7201;  // ADD #1,R2  x32
        imem['h44] = 16'h0009;  // sentinel
        imem['h45] = 16'hAFFE;  // guard
        imem['h46] = 16'h0009;
        do_reset;
        n0 = lbus_ifetch_acc;
        run_until_retire('h44, 20000);
        $display("      odd-entry fetch accepts = %0d", lbus_ifetch_acc - n0);
        chk("odd-entry ALU block result", gpr(2), 32'd32);
        chk("odd-entry fetch accepts (law)", lbus_ifetch_acc - n0, 32'd30);
        do_reset;
        end_test;
    end
endtask

//GOLDEN P - the sharpest pair-kill case: a taken branch whose OWN fetched longword
//carries its wrong-path sibling. The sibling is captured into the pair slot at the
//branch's insert edge and must be killed by the branch's redirect, never served.
task automatic test_pair_branch_kill;
    begin
        begin_test("Fetch pair: taken branch's own captured sibling is killed, never retires");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hEA00;  // MOV  #0,R10    ; poison detector
        imem['h21] = 16'h0008;  // CLRT           ; T=0 -> BF taken
        imem['h22] = 16'h8B02;  // BF   0x4C      ; even slot: sibling 'h23 rides its fetch
        imem['h23] = 16'hEA55;  // POISON - the branch's own pair sibling
        imem['h24] = 16'hEA66;  // POISON fall-through
        imem['h25] = 16'hEA77;  // POISON
        imem['h26] = 16'h0009;  // branch target = sentinel
        imem['h27] = 16'hAFFE;  // guard
        imem['h28] = 16'h0009;
        do_reset;
        run_until_retire('h26, 20000);
        chk_true("branch target retired", retired_seen['h26]);
        chk("branch-sibling poison never retired -> R10", gpr(10), 32'd0);
        chk_true("wrong-path slots never retired",
                 !retired_seen['h23] && !retired_seen['h24] && !retired_seen['h25]);
        do_reset;
        end_test;
    end
endtask

//GOLDEN R - pair-served cycles are request-free edges: the background wb drain now
//completes under a PLAIN sequential instruction stream (pre-pair it starved and
//needed DSP-stall pumping - see the drain-fault law test, which keeps that pump).
task automatic test_cached_drain_pairidle;
    integer i;
    begin
        begin_test("Fetch pair: NOP stream yields idle edges - background drain completes");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE118;  // MOV   #0x18,R1
        imem['h21] = 16'h4108;  // SHLL2 R1
        imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180 = line A (set 0x18)
        imem['h23] = 16'h6712;  // MOV.L @R1,R7   ; fill A
        imem['h24] = 16'hE251;  // MOV   #0x51,R2
        imem['h25] = 16'h2122;  // MOV.L R2,@R1   ; dirty word 0
        imem['h26] = 16'hE0F0;  // MOV   #0xF0,R0
        imem['h27] = 16'h4028;  // SHLL16 R0
        imem['h28] = 16'h4018;  // SHLL8 R0       ; 0xF000_0000
        imem['h29] = 16'hE431;  // MOV   #0x31,R4
        imem['h2A] = 16'h4418;  // SHLL8 R4
        imem['h2B] = 16'h7440;  // ADD   #0x40,R4
        imem['h2C] = 16'h7440;  // ADD   #0x40,R4 ; 0x3180 = way 3 / set 0x18
        imem['h2D] = 16'h304C;  // ADD   R4,R0
        imem['h2E] = 16'hE500;  // MOV   #0,R5
        imem['h2F] = 16'h2052;  // MOV.L R5,@R0   ; mm tag V=0: dirty A -> wb buffer
        for(i = 0; i < 32; i = i + 1) imem['h30 + i] = 16'h0009;  // plain NOP pump
        imem['h50] = 16'h0009;  // sentinel
        imem['h51] = 16'hAFFE;  // guard
        imem['h52] = 16'h0009;
        do_reset;
        for(i = 0; i < 4; i = i + 1) dmem['h60 + i] = 32'h0000_0041 + i;
        run_until_retire('h50, 20000);
        chk_true("no exception", !exc_seen);
        chk_true("buffer drained under a plain NOP stream", !u_dut.u_cache.wb_valid);
        chk("word 0 drained (stored value)",   dmem['h60], 32'h0000_0051);
        chk("word 1 drained (fill-back copy)", dmem['h61], 32'h0000_0042);
        do_reset;
        end_test;
    end
endtask

//GOLDEN S - cacheable LDC-SR tight spacing. The running registers (r_t/r_s/r_m/r_q)
//deposit from their PRODUCERS and resync on drain, but a full-SR writer (LDC Rm,SR)
//is not snooped - a consumer at back-to-back cacheable spacing reads stale state.
//Case 1/2: LDC flips T then BT/BF immediately (poison on the stale path).
//Case 3: LDC loads M=1,Q=0 then DIV1 - add-vs-subtract flips on stale M/Q.
task automatic test_cached_ldcsr_tight;
    integer l;
    begin
        begin_test("Cacheable LDC-SR tight spacing: BT/BF/DIV1 right behind a full-SR write");
        for(l = 0; l < 3; l = l + 1) begin
            cacheable_bootstrap(8'h09);
            imem['h20] = 16'hE860;  // MOV    #0x60,R8
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8      ; 0x6000_0000 (T=0)
            imem['h23] = 16'h6983;  // MOV    R8,R9   ; R9 = T=0 value
            imem['h24] = 16'h7801;  // ADD    #1,R8   ; R8 = T=1 value
            imem['h25] = 16'hEA00;  // MOV    #0,R10  ; stale-path poison detector
            //case 1: running T=0, LDC sets T=1, BT immediately after
            imem['h26] = 16'h0008;  // CLRT           ; r_t <- 0 (producer path)
            imem['h27] = 16'h480E;  // LDC   R8,SR    ; SR.T = 1
            imem['h28] = 16'h8902;  // BT    +2       ; MUST take (arch T=1)
            imem['h29] = 16'hEA01;  // POISON: only a stale T=0 falls through
            imem['h2A] = 16'h0009;
            imem['h2B] = 16'h0009;
            //case 2: running T=1, LDC clears T, BF immediately after
            imem['h2C] = 16'h0018;  // SETT           ; r_t <- 1
            imem['h2D] = 16'h490E;  // LDC   R9,SR    ; SR.T = 0
            imem['h2E] = 16'h8B02;  // BF    +2       ; MUST take (arch T=0)
            imem['h2F] = 16'hEA02;  // POISON: only a stale T=1 falls through
            imem['h30] = 16'h0009;
            imem['h31] = 16'h0009;
            //case 3: LDC loads M=1,Q=0; DIV1 with Q^M=1 ADDS (stale 0,0 subtracts)
            imem['h32] = 16'hEC02;  // MOV   #2,R12
            imem['h33] = 16'h4C18;  // SHLL8 R12      ; 0x200 = SR.M
            imem['h34] = 16'h6B93;  // MOV   R9,R11
            imem['h35] = 16'h3BCC;  // ADD   R12,R11  ; 0x6000_0200 (M=1,Q=0,T=0)
            imem['h36] = 16'hE100;  // MOV   #0,R1    ; DIV1 Rn
            imem['h37] = 16'hE201;  // MOV   #1,R2    ; DIV1 Rm
            imem['h38] = 16'h4B0E;  // LDC   R11,SR
            imem['h39] = 16'h3124;  // DIV1  R2,R1    ; M=1,Q=0: Rn = (Rn<<1|T) + Rm = 1
            imem['h3A] = 16'h0329;  // MOVT  R3       ; expected T = 1
            imem['h3B] = 16'h0009;  // sentinel
            imem['h3C] = 16'hAFFE;  // guard
            imem['h3D] = 16'h0009;
            i_latency = l;
            do_reset;
            run_until_retire('h3B, 20000);
            chk($sformatf("no stale-T path ran (i=%0d)", l), gpr(10), 32'd0);
            chk($sformatf("DIV1 added under fresh M/Q (i=%0d)", l), gpr(1), 32'd1);
            chk($sformatf("DIV1 T under fresh M/Q (i=%0d)", l), gpr(3), 32'd1);
        end
        i_latency = 0;
        do_reset;
        end_test;
    end
endtask

//GOLDEN T - core-level interrupt timing sweep. The SoC INTC's protocol (level+code
//held until o_INT_ACK) is driven directly, fired at every cycle offset across a
//cacheable store/load loop with DELAYED branches. Laws: exactly one entry per fire,
//the loop result is interrupt-transparent, and SPC NEVER points at a delay slot
//(SH defers acceptance between a delayed branch and its slot; SPC = the branch).
task automatic test_int_timing_sweep;
    integer off, w, fails;
    begin
        begin_test("Interrupt timing sweep: one entry, transparent result, SPC never a delay slot");
        for(off = 0; off < 25; off = off + 1) begin
            cacheable_bootstrap(8'h09);
            imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue (IMASK=0)
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8
            imem['h23] = 16'h480E;  // LDC    R8,SR
            imem['h24] = 16'hED00;  // MOV   #0,R13   ; handler entry counter
            imem['h25] = 16'hE304;  // MOV   #4,R3    ; loop count
            imem['h26] = 16'hE200;  // MOV   #0,R2
            imem['h27] = 16'hE101;  // MOV   #1,R1
            imem['h28] = 16'h4118;  // SHLL8 R1       ; R1 = 0x100
            //loop: ALU + store + DT + DELAYED BF/S with a load in the slot
            imem['h29] = 16'h7201;  // ADD   #1,R2
            imem['h2A] = 16'h2122;  // MOV.L R2,@R1
            imem['h2B] = 16'h4310;  // DT    R3
            imem['h2C] = 16'h8FFB;  // BF/S  loop     ; delayed
            imem['h2D] = 16'h6212;  // MOV.L @R1,R2   ; delay slot (identity reload)
            imem['h2E] = 16'h0009;  // sentinel
            imem['h2F] = 16'hAFFE;  // guard
            imem['h30] = 16'h0009;
            //handler at VBR+0x600: count the entry and return
            imem['h300] = 16'h7D01; // ADD   #1,R13
            imem['h301] = 16'h0009;
            imem['h302] = 16'h002B; // RTE
            imem['h303] = 16'h0009; //   delay slot
            do_reset;
            run_until_retire('h24, 20000);        //R13 initialized: loop is starting
            repeat(off) @(posedge clk);
            int_level_q = 4'd8;
            int_code_q  = 12'h600;
            int_valid_q = 1'b1;
            //Drop the level at the FIRST entry, keyed on the STICKY scoreboard counter:
            //sampling the combinational ack pulse from a tb procedural loop is racy, and
            //a level held across RTE correctly RE-ENTERS (real SH level semantics).
            w = 0;
            while(entry_count == 0 && w < 2000) begin @(posedge clk); w = w + 1; end
            @(posedge clk);
            int_valid_q = 1'b0;
            //A late offset can land in the guard spin AFTER the sentinel retired:
            //wait for the handler's RTE so R13/SR have settled before sampling.
            run_until_retire('h302, 20000);
            run_until_retire('h2E, 20000);
            chk($sformatf("off=%0d: exactly one entry", off), gpr(13), 32'd1);
            chk($sformatf("off=%0d: loop result transparent", off), gpr(2), 32'd4);
            chk($sformatf("off=%0d: loop count consumed", off), gpr(3), 32'd0);
            chk_true($sformatf("off=%0d: SPC is never the delay slot (spc=%08h)", off, entry_spc_l),
                     entry_spc_l[11:0] != 12'h05A);   //slot PC = byte 0x5A ('h2D)
            chk("INTEVT carries the driven code", intevt_o, 32'h0000_0600);
        end
        do_reset;
        end_test;
    end
endtask

//GOLDEN V - interrupt vs miss/fill/drain machinery. The old timing sweep runs at zero
//latency where cached hits respond combinationally, so an acceptance edge almost never
//overlaps an in-flight cache excursion. Here the loop walks EIGHT same-set (set 6)
//lines with dirtying stores - every store misses, iterations 5+ evict a DIRTY victim
//(drain), and the loop-tail code line shares set 6 so I-misses interleave too - over a
//(i,d) latency grid. LAWS at every offset: one entry, transparent result, no deadlock.
//The entry_cache_busy counter proves entries really landed mid-excursion.
task automatic test_int_miss_sweep;
    integer g, off, w, busy0, e0;
    integer il_g [0:3];
    integer dl_g [0:3];
    begin
        begin_test("Interrupt vs miss/fill/drain: acceptance mid-excursion is transparent (grid sweep)");
        il_g = '{0, 0, 3, 2};
        dl_g = '{0, 3, 0, 5};
        busy0 = entry_cache_busy;
        for(g = 0; g < 4; g = g + 1) begin
        for(off = 0; off < 120; off = off + 3) begin
            cacheable_bootstrap(8'h09);
            i_latency = il_g[g];
            d_latency = dl_g[g];
            imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue (IMASK=0)
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8
            imem['h23] = 16'h480E;  // LDC    R8,SR
            imem['h24] = 16'hED00;  // MOV   #0,R13   ; handler entry counter
            imem['h25] = 16'hE110;  // MOV   #0x10,R1
            imem['h26] = 16'h4118;  // SHLL8 R1
            imem['h27] = 16'h7160;  // ADD   #0x60,R1 ; R1 = 0x1060 (set 6)
            imem['h28] = 16'h6713;  // MOV   R1,R7    ; 6nm3: n=dst=7, m=src=1
            imem['h29] = 16'h7710;  // ADD   #0x10,R7 ; R7 = 0x1070 (set 7, never stored)
            imem['h2A] = 16'hE310;  // MOV   #0x10,R3
            imem['h2B] = 16'h4318;  // SHLL8 R3       ; R3 = 0x1000 same-set stride
            imem['h2C] = 16'hE508;  // MOV   #8,R5    ; 8 lines > 4 ways: iters 5+ drain
            imem['h2D] = 16'hE200;  // MOV   #0,R2
            imem['h2E] = 16'hE600;  // MOV   #0,R6
            //loop: miss store + hit load-back + COLD-MISS load (a pending-response
            //window every iteration - stores are notify-at-accept) + DELAYED BF/S;
            //the tail code lines share sets 6/7, so the walks evict them - I-misses too.
            imem['h2F] = 16'h7201;  // ADD   #1,R2
            imem['h30] = 16'h2122;  // MOV.L R2,@R1   ; write-allocate miss (dirty)
            imem['h31] = 16'h6412;  // MOV.L @R1,R4   ; load-back on the fresh line
            imem['h32] = 16'h313C;  // ADD   R3,R1    ; next same-set line
            imem['h33] = 16'h6972;  // MOV.L @R7,R9   ; COLD-MISS load (set 7)
            imem['h34] = 16'h373C;  // ADD   R3,R7
            imem['h35] = 16'h4510;  // DT    R5
            imem['h36] = 16'h8FF7;  // BF/S  loop     ; delayed
            imem['h37] = 16'h7601;  // ADD   #1,R6    ;   delay slot
            imem['h38] = 16'h0009;  // sentinel
            imem['h39] = 16'hAFFE;  // guard
            imem['h3A] = 16'h0009;
            //handler at VBR+0x600: count the entry and return.
            imem['h300] = 16'h7D01; // ADD   #1,R13
            imem['h301] = 16'h002B; // RTE
            imem['h302] = 16'h0009; //   delay slot
            do_reset;
            e0 = test_errors;
            dbg_trace = $test$plusargs("trace87") && (g == 0) && (off == 0);
            run_until_retire('h2E, 30000);        //loop is starting
            if(dbg_trace) $display("        [dbg] pre-loop bram: R1=%08h R3=%08h R7=%08h R13=%08h",
                                   gpr(1), gpr(3), gpr(7), gpr(13));
            repeat(off) @(posedge clk);
            int_level_q = 4'd8;
            int_code_q  = 12'h600;
            int_valid_q = 1'b1;
            //Drop at the first entry, keyed on the sticky counter (see the zero-lat sweep).
            w = 0;
            while(entry_count == 0 && w < 20000) begin @(posedge clk); w = w + 1; end
            @(posedge clk);
            int_valid_q = 1'b0;
            run_until_retire('h301, 60000);       //handler RTE retired
            run_until_retire('h38, 60000);        //main line completed
            chk($sformatf("g=%0d off=%0d: exactly one entry", g, off), entry_count, 32'd1);
            chk($sformatf("g=%0d off=%0d: handler ran once", g, off), gpr(13), 32'd1);
            chk($sformatf("g=%0d off=%0d: accumulator transparent", g, off), gpr(2), 32'd8);
            chk($sformatf("g=%0d off=%0d: load-back transparent", g, off), gpr(4), 32'd8);
            chk($sformatf("g=%0d off=%0d: cold-miss load transparent", g, off), gpr(9), 32'd0);
            chk($sformatf("g=%0d off=%0d: slot count transparent", g, off), gpr(6), 32'd8);
            chk($sformatf("g=%0d off=%0d: loop count consumed", g, off), gpr(5), 32'd0);
            chk_true($sformatf("g=%0d off=%0d: no spurious exception", g, off), !exc_seen);
            chk("INTEVT carries the driven code", intevt_o, 32'h0000_0600);
            chk_true($sformatf("g=%0d off=%0d: SPC is never the delay slot (spc=%08h)", g, off, entry_spc_l),
                     entry_spc_l[11:0] != 12'h06E);   //slot PC = byte 0x6E ('h37)
            if(test_errors != e0)
                $display("      [dbg] EXPEVT=%08h INTEVT=%08h SR=%08h SPC=%08h spc_l=%08h entries=%0d ack=%0d R13=%0d R2=%0d R5=%0d R6=%0d cstate=%0d",
                         expevt_o, intevt_o, sr, spc_o, entry_spc_l, entry_count, int_ack_count,
                         gpr(13), gpr(2), gpr(5), gpr(6), u_dut.u_cache.state);
            dbg_trace = 1'b0;
        end
        end
        //Coverage verdict: the sweep must have landed entries INSIDE cache excursions.
        $display("      entry-vs-machinery coverage: %0d entries with the cache FSM busy",
                 entry_cache_busy - busy0);
        chk_true("entries landed mid-excursion (coverage)", (entry_cache_busy - busy0) > 20);
        do_reset;
        end_test;
    end
endtask

//GOLDEN W - interrupt vs locked TAS.B RMW (atomicity law). The locked read-modify-write
//is indivisible: acceptance must never split it, and a killed-then-reexecuted TAS whose
//write already committed would read back its own 0x80 (T flips 1->0). Two back-to-back
//TAS windows are swept with the RMW stretched by d_latency; the lock pairing counter
//(every locked read paired with exactly one locked write) closes the bus-protocol law.
task automatic test_int_tas_atomic;
    integer dl, off, w, e0;
    begin
        begin_test("Interrupt vs TAS.B: locked RMW is indivisible and exactly-once (offset sweep)");
        for(dl = 0; dl <= 6; dl = dl + 6) begin
        for(off = 0; off < 35; off = off + 1) begin
            cacheable_bootstrap(8'h09);
            d_latency = dl;
            imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8
            imem['h23] = 16'h480E;  // LDC    R8,SR
            imem['h24] = 16'hED00;  // MOV   #0,R13   ; handler entry counter
            imem['h25] = 16'hE118;  // MOV   #0x18,R1
            imem['h26] = 16'h4108;  // SHLL2 R1
            imem['h27] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180 (memory byte, MSB lane)
            imem['h28] = 16'hE400;  // MOV   #0,R4
            imem['h29] = 16'hE600;  // MOV   #0,R6    ; anchor
            imem['h2A] = 16'h0009;  // NOP            ; retire pulses land in TAS's MA
            imem['h2B] = 16'h0009;  // NOP
            imem['h2C] = 16'h0009;  // NOP
            imem['h2D] = 16'h411B;  // TAS.B @R1      ; locked RMW #1: byte 0 -> T=1, set 0x80
            imem['h2E] = 16'h0429;  // MOVT  R4
            imem['h2F] = 16'h411B;  // TAS.B @R1      ; locked RMW #2: byte 0x80 -> T=0
            imem['h30] = 16'h0629;  // MOVT  R6
            imem['h31] = 16'h0009;  // sentinel
            imem['h32] = 16'hAFFE;  // guard
            imem['h33] = 16'h0009;
            imem['h300] = 16'h7D01; // ADD   #1,R13   ; VBR+0x600 handler
            imem['h301] = 16'h002B; // RTE
            imem['h302] = 16'h0009; //   delay slot
            do_reset;
            e0 = test_errors;
            dmem['h60] = 32'h0000_0041;   //byte 0x180 = 0x00 (big-endian MSB lane)
            run_until_retire('h29, 30000);
            repeat(off) @(posedge clk);
            int_level_q = 4'd8;
            int_code_q  = 12'h600;
            int_valid_q = 1'b1;
            w = 0;
            while(entry_count == 0 && w < 20000) begin @(posedge clk); w = w + 1; end
            @(posedge clk);
            int_valid_q = 1'b0;
            run_until_retire('h301, 20000);       //handler RTE retired
            run_until_retire('h31, 20000);        //sentinel
            chk($sformatf("dl=%0d off=%0d: TAS#1 saw the pre-RMW byte once (T=1)", dl, off), gpr(4), 32'd1);
            chk($sformatf("dl=%0d off=%0d: TAS#2 saw bit7 already set (T=0)", dl, off), gpr(6), 32'd0);
            chk($sformatf("dl=%0d off=%0d: memory byte mutated exactly once", dl, off), dmem['h60], 32'h8000_0041);
            chk($sformatf("dl=%0d off=%0d: locked reads paired", dl, off), locked_read_count, 32'd2);
            chk($sformatf("dl=%0d off=%0d: locked writes paired", dl, off), locked_write_count, 32'd2);
            chk($sformatf("dl=%0d off=%0d: exactly one entry", dl, off), entry_count, 32'd1);
            chk($sformatf("dl=%0d off=%0d: handler ran once", dl, off), gpr(13), 32'd1);
            chk_true($sformatf("dl=%0d off=%0d: no spurious exception", dl, off), !exc_seen);
            if(test_errors != e0)
                $display("      [dbg] EXPEVT=%08h INTEVT=%08h SR=%08h SPC=%08h spc_l=%08h entries=%0d ack=%0d lockR=%0d lockW=%0d cstate=%0d epc_l=%08h",
                         expevt_o, intevt_o, sr, spc_o, entry_spc_l, entry_count, int_ack_count,
                         locked_read_count, locked_write_count, u_dut.u_cache.state, exception_entry_pc_l);
        end
        end
        do_reset;
        end_test;
    end
endtask

//GOLDEN X2 - synchronous exception vs pending interrupt, collided at every offset.
//Four flavors fault/trap at F: ILLEGAL (ID), ADDRESS error (EX/AGU), TRAPA (a trap
//that IS a retirement - interrupt_boundary is genuinely open on its retire edge),
//and a BUS-FAULT load with a younger memory op in EX (in-flight kill arm). LAWS,
//whatever the offset: each event enters EXACTLY once (one ack, one exception),
//EXPEVT/INTEVT never mix, the mainline result is transparent, and the pre-decrement
//store executes exactly once (a wrong-path or re-executed tail would double it).
task automatic test_exc_int_collision;
    integer f, lat, off, w, e0;
    logic [15:0] f_op, g_op;
    logic [31:0] exp_ev, exp_r2;
    logic        skip;
    string       fname;
    begin
        begin_test("Exception x interrupt collision: both once, never mixed (4 flavors x offsets)");
        for(f = 0; f < 4; f = f + 1) begin
            case(f)
                0: begin fname="illegal"; f_op=16'hF000; g_op=16'h7201; exp_ev=32'h0000_0180; skip=1'b1; end
                1: begin fname="address"; f_op=16'h6402; g_op=16'h7201; exp_ev=32'h0000_00E0; skip=1'b1; end
                2: begin fname="trapa";   f_op=16'hC342; g_op=16'h7201; exp_ev=32'h0000_0160; skip=1'b0; end
                default: begin fname="busfault"; f_op=16'h6432; g_op=16'h6532; exp_ev=32'h0000_00E0; skip=1'b1; end
            endcase
            exp_r2 = (f == 3) ? 32'd2 : 32'd3;    //flavor 3's G is a load, not an ADD
        for(lat = 0; lat <= 2; lat = lat + 2) begin
        for(off = 0; off < 25; off = off + 1) begin
            cacheable_bootstrap(8'h09);
            i_latency = lat;
            d_latency = lat;
            imem['h20] = 16'hE860;  // MOV    #0x60,R8 ; BL-clear prologue
            imem['h21] = 16'h4818;  // SHLL8  R8
            imem['h22] = 16'h4828;  // SHLL16 R8
            imem['h23] = 16'h480E;  // LDC    R8,SR
            imem['h24] = 16'hED00;  // MOV   #0,R13   ; interrupt handler counter
            imem['h25] = 16'hEB00;  // MOV   #0,R11   ; exception handler counter
            imem['h26] = 16'hE200;  // MOV   #0,R2
            imem['h27] = 16'hE001;  // MOV   #1,R0    ; odd base (ADDRESS flavor)
            imem['h28] = 16'hE304;  // MOV   #4,R3
            imem['h29] = 16'h4318;  // SHLL8 R3       ; R3 = 0x400 (BUSFAULT target, widx 0)
            imem['h2A] = 16'h7201;  // ADD   #1,R2
            imem['h2B] = 16'h7201;  // ADD   #1,R2
            imem['h2C] = f_op;      // F: the flavor's faulting/trapping instruction
            imem['h2D] = g_op;      // G: ADD #1,R2 - or the in-flight younger load
            imem['h2E] = 16'h2326;  // MOV.L R2,@-R3  ; pre-dec: double-execution detector
            imem['h2F] = 16'h6432;  // MOV.L @R3,R4   ; load-back of the stored word
            imem['h30] = 16'h0009;  // sentinel
            imem['h31] = 16'hAFFE;  // guard
            imem['h32] = 16'h0009;
            //General-exception handler (VBR+0x100): count, skip F when it faults.
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
            imem['h300] = 16'h7D01; // ADD   #1,R13   ; VBR+0x600 interrupt handler
            imem['h301] = 16'h002B; // RTE
            imem['h302] = 16'h0009; //   delay slot
            if(f == 3) begin
                d_fault_en   = 1'b1;
                d_fault_widx = 8'd0;      //F's fill beat 0 (addr 0x400) faults
            end
            do_reset;
            e0 = test_errors;
            if(f == 3) dmem[0] = 32'h0000_00A5;   //G's clean reload value
            run_until_retire('h29, 30000);
            repeat(off) @(posedge clk);
            int_level_q = 4'd8;
            int_code_q  = 12'h600;
            int_valid_q = 1'b1;
            //Drop on the ACK - the SoC INTC protocol. An ack without a matching entry
            //(the collision hazard) LOSES the interrupt and fails the checks below.
            w = 0;
            while(int_ack_count == 0 && w < 20000) begin @(posedge clk); w = w + 1; end
            @(posedge clk);
            int_valid_q = 1'b0;
            //The synchronous event must fire exactly once; disarm the bus fault so the
            //skip-return path (and G's re-run) refills cleanly.
            w = 0;
            while(!exc_seen && !trapa_seen && w < 20000) begin @(posedge clk); w = w + 1; end
            if(f == 3) begin @(posedge clk); d_fault_en = 1'b0; end
            run_until_retire('h30, 30000);        //main line completed
            //A late interrupt lands in the guard spin: wait for its handler.
            w = 0;
            while(gpr(13) == 32'd0 && w < 20000) begin @(posedge clk); w = w + 1; end
            run_cycles(30);
            chk($sformatf("%s lat=%0d off=%0d: interrupt ack'd exactly once", fname, lat, off), int_ack_count, 32'd1);
            chk($sformatf("%s lat=%0d off=%0d: interrupt handler ran exactly once", fname, lat, off), gpr(13), 32'd1);
            chk($sformatf("%s lat=%0d off=%0d: exception handler ran exactly once", fname, lat, off), gpr(11), 32'd1);
            chk($sformatf("%s lat=%0d off=%0d: exactly two entries", fname, lat, off), entry_count, 32'd2);
            chk($sformatf("%s lat=%0d off=%0d: INTEVT", fname, lat, off), intevt_o, 32'h0000_0600);
            chk($sformatf("%s lat=%0d off=%0d: EXPEVT", fname, lat, off), expevt_o, exp_ev);
            chk($sformatf("%s lat=%0d off=%0d: mainline result transparent", fname, lat, off), gpr(2), exp_r2);
            chk($sformatf("%s lat=%0d off=%0d: pre-dec store executed once", fname, lat, off), gpr(3), 32'h0000_03FC);
            chk($sformatf("%s lat=%0d off=%0d: stored word load-back", fname, lat, off), gpr(4), exp_r2);
            case(f)
                1: begin
                    chk($sformatf("%s lat=%0d off=%0d: TEA = misaligned EA", fname, lat, off), tea_o, 32'd1);
                    chk($sformatf("%s lat=%0d off=%0d: cause", fname, lat, off), {29'd0, exc_cause_l}, {29'd0, EXC_ADDRESS});
                    chk($sformatf("%s lat=%0d off=%0d: exc pc = F", fname, lat, off), exc_pc_l, 32'h0000_0058);
                end
                2: begin
                    chk_true($sformatf("%s lat=%0d off=%0d: TRAPA pulsed", fname, lat, off), trapa_seen);
                    chk($sformatf("%s lat=%0d off=%0d: TRA = imm<<2", fname, lat, off), tra_o, 32'h0000_0108);
                end
                3: begin
                    chk($sformatf("%s lat=%0d off=%0d: TEA = faulting fill addr", fname, lat, off), tea_o, 32'h0000_0400);
                    chk($sformatf("%s lat=%0d off=%0d: cause", fname, lat, off), {29'd0, exc_cause_l}, {29'd0, EXC_DATA});
                    chk($sformatf("%s lat=%0d off=%0d: exc pc = F", fname, lat, off), exc_pc_l, 32'h0000_0058);
                    chk($sformatf("%s lat=%0d off=%0d: G reloaded cleanly", fname, lat, off), gpr(5), 32'h0000_00A5);
                end
                default: begin
                    chk($sformatf("%s lat=%0d off=%0d: cause", fname, lat, off), {29'd0, exc_cause_l}, {29'd0, EXC_ILLEGAL});
                    chk($sformatf("%s lat=%0d off=%0d: exc pc = F", fname, lat, off), exc_pc_l, 32'h0000_0058);
                end
            endcase
            if(test_errors != e0)
                $display("      [dbg] EXPEVT=%08h INTEVT=%08h TEA=%08h SR=%08h SPC=%08h spc_l=%08h entries=%0d ack=%0d R2=%0d R11=%0d R13=%0d cstate=%0d epc_l=%08h",
                         expevt_o, intevt_o, tea_o, sr, spc_o, entry_spc_l, entry_count, int_ack_count,
                         gpr(2), gpr(11), gpr(13), u_dut.u_cache.state, exception_entry_pc_l);
        end
        end
        end
        do_reset;
        end_test;
    end
endtask

//GOLDEN U - random-program latency-invariance oracle. Constrained-random cacheable
//programs (ALU / longword+byte memory ops on an R14 window / forward BT-BF over
//CMP/SETT/CLRT) are run once at zero wait states - that run IS the reference - and
//again at random (i,d) latencies: the architectural result must be identical.
task automatic test_random_latency_oracle;
    integer trial, i, k, cls, dst, srcr, disp, ilat, dlat;
    logic [31:0] ref_r [0:14];
    begin
        begin_test("Random-program oracle: results invariant under random (i,d) wait states");
        for(trial = 0; trial < 6; trial = trial + 1) begin
            cacheable_bootstrap((trial & 1) ? 8'h0B : 8'h09);   //alternate WT / WB
            imem['h20] = 16'hEE01;  // MOV   #1,R14
            imem['h21] = 16'h4E18;  // SHLL8 R14      ; data window base 0x100
            //Zero R1-R12: the GPR file PERSISTS across do_reset, so without this the
            //two runs start from different register states and the oracle is void.
            for(i = 1; i <= 12; i = i + 1) imem['h21 + i] = 16'hE000 | (i << 8);
            for(i = 0; i < 48; i = i + 1) begin
                cls  = $urandom_range(0, 9);
                dst  = $urandom_range(1, 12);
                srcr = $urandom_range(1, 12);
                disp = $urandom_range(0, 15);
                if(cls == 9 && i > 42) cls = 1;   //a late branch could skip the sentinel
                case(cls)
                    0: imem['h2E + i] = 16'hE000 | (dst << 8) | ($urandom_range(0, 255)); // MOV #imm
                    1: imem['h2E + i] = 16'h7001 | (dst << 8);                     // ADD #1,Rn
                    2: imem['h2E + i] = 16'h300C | (dst << 8) | (srcr << 4);       // ADD Rm,Rn
                    3: imem['h2E + i] = 16'h200A | (dst << 8) | (srcr << 4);       // XOR Rm,Rn
                    4: imem['h2E + i] = 16'h4000 | (dst << 8);                     // SHLL Rn
                    5: imem['h2E + i] = 16'h1E00 | (srcr << 4) | disp;             // MOV.L Rm,@(d,R14)
                    6: imem['h2E + i] = 16'h50E0 | (dst << 8) | disp;              // MOV.L @(d,R14),Rn
                    7: imem['h2E + i] = 16'h2E00 | (srcr << 4);                    // MOV.B Rm,@R14
                    8: imem['h2E + i] = ($urandom_range(0, 1)) ? 16'h0008 : 16'h0018; // CLRT/SETT
                    9: imem['h2E + i] = (($urandom_range(0, 1)) ? 16'h8900 : 16'h8B00)
                                        | $urandom_range(1, 3);                    // BT/BF fwd 1..3
                    default: imem['h2E + i] = 16'h0009;
                endcase
            end
            imem['h5E] = 16'h0009;  // sentinel
            imem['h5F] = 16'hAFFE;  // guard
            imem['h60] = 16'h0009;
            //reference: zero wait states
            i_latency = 0; d_latency = 0;
            do_reset;
            run_until_retire('h5E, 40000);
            for(k = 0; k <= 14; k = k + 1) ref_r[k] = gpr(k);
            //trial run: random wait states, same program, same initial memory
            ilat = $urandom_range(1, 6);
            dlat = $urandom_range(0, 4);
            i_latency = ilat; d_latency = dlat;
            do_reset;
            run_until_retire('h5E, 60000);
            for(k = 0; k <= 14; k = k + 1)
                chk($sformatf("trial %0d (i=%0d,d=%0d) R%0d", trial, ilat, dlat, k),
                    gpr(k), ref_r[k]);
        end
        i_latency = 0; d_latency = 0;
        do_reset;
        end_test;
    end
endtask

//GOLDEN V - CCR.CF discards dirty data WITHOUT write-back (p.106): a dirty line's
//store is LOST across a flush; the reload refills the old memory truth, and the
//flush walk itself stalls the following access cleanly (no deadlock).
task automatic test_ccr_flush_discard;
    begin
        begin_test("CCR.CF law: flush discards dirty data (no write-back), clean stall");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE118;  // MOV   #0x18,R1
        imem['h21] = 16'h4108;  // SHLL2 R1
        imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180 (line A)
        imem['h23] = 16'h6712;  // MOV.L @R1,R7   ; fill A (memory word 0 = 0x41)
        imem['h24] = 16'hE251;  // MOV   #0x51,R2
        imem['h25] = 16'h2122;  // MOV.L R2,@R1   ; dirty word 0 = 0x51 (cache only)
        imem['h26] = 16'hE0EC;  // MOV   #0xEC,R0 ; CCR address
        imem['h27] = 16'hE409;  // MOV   #9,R4    ; CE|CF
        imem['h28] = 16'h2042;  // MOV.L R4,@R0   ; FLUSH: 256-set walk (stalls followers)
        imem['h29] = 16'h6512;  // MOV.L @R1,R5   ; post-walk refill from MEMORY
        imem['h2A] = 16'h0009;  // sentinel
        imem['h2B] = 16'hAFFE;  // guard
        imem['h2C] = 16'h0009;
        do_reset;
        dmem['h60] = 32'h0000_0041;
        run_until_retire('h2A, 20000);
        chk("dirty store DISCARDED by CF (refill = old memory)", gpr(5), 32'h0000_0041);
        chk("memory never written (no write-back on CF)", dmem['h60], 32'h0000_0041);
        do_reset;
        end_test;
    end
endtask

//GOLDEN W - CCR.CE toggle revival: disabling the cache does NOT clear tags (p.104);
//a dirty line's value reappears when CE is re-enabled without a flush. LAW: load
//sequence reads cache 0x51 -> CE=0 bypass reads memory 0x41 -> CE=1 hits 0x51 again.
task automatic test_ccr_ce_revival;
    begin
        begin_test("CCR.CE toggle law: tags survive CE=0; the dirty line revives on CE=1");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE118;  // MOV   #0x18,R1
        imem['h21] = 16'h4108;  // SHLL2 R1
        imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180 (line A)
        imem['h23] = 16'h6712;  // MOV.L @R1,R7   ; fill A (memory word 0 = 0x41)
        imem['h24] = 16'hE251;  // MOV   #0x51,R2
        imem['h25] = 16'h2122;  // MOV.L R2,@R1   ; dirty word 0 = 0x51 (WB: cache only)
        imem['h26] = 16'h6412;  // MOV.L @R1,R4   ; hit -> 0x51
        imem['h27] = 16'hE0EC;  // MOV   #0xEC,R0 ; CCR address
        imem['h28] = 16'hE300;  // MOV   #0,R3
        imem['h29] = 16'h2032;  // MOV.L R3,@R0   ; CE=0 (no flush)
        imem['h2A] = 16'h6512;  // MOV.L @R1,R5   ; bypass -> MEMORY 0x41 (stale vs cache)
        imem['h2B] = 16'hE301;  // MOV   #1,R3
        imem['h2C] = 16'h2032;  // MOV.L R3,@R0   ; CE=1, NO CF
        imem['h2D] = 16'h6612;  // MOV.L @R1,R6   ; revived hit -> 0x51 again
        imem['h2E] = 16'h0009;  // sentinel
        imem['h2F] = 16'hAFFE;  // guard
        imem['h30] = 16'h0009;
        do_reset;
        dmem['h60] = 32'h0000_0041;
        run_until_retire('h2E, 20000);
        chk("pre-toggle hit reads the dirty value",  gpr(4), 32'h0000_0051);
        chk("CE=0 bypass reads stale memory",        gpr(5), 32'h0000_0041);
        chk("CE=1 revival hits the dirty line",      gpr(6), 32'h0000_0051);
        do_reset;
        end_test;
    end
endtask

//GOLDEN X - TAS.B vs a resident dirty line (software-coherency law): a locked access
//always bypasses the array, so TAS reads/writes MEMORY while the dirty cache copy
//stays live - both views are observable and neither corrupts the other.
task automatic test_tas_vs_dirty;
    begin
        begin_test("TAS.B vs dirty resident line: lock bypasses the array (both views live)");
        cacheable_bootstrap(8'h09);
        imem['h20] = 16'hE118;  // MOV   #0x18,R1
        imem['h21] = 16'h4108;  // SHLL2 R1
        imem['h22] = 16'h4108;  // SHLL2 R1       ; R1 = 0x180
        imem['h23] = 16'h6712;  // MOV.L @R1,R7   ; fill A (memory word 0 = 0x0000_0041)
        imem['h24] = 16'hE251;  // MOV   #0x51,R2
        imem['h25] = 16'h2122;  // MOV.L R2,@R1   ; dirty word 0 = 0x0000_0051 (cache only)
        imem['h26] = 16'h411B;  // TAS.B @R1      ; locked RMW on MEMORY byte 0x180 (MSB=0x00)
        imem['h27] = 16'h0429;  // MOVT  R4       ; byte was 0 -> T=1
        imem['h28] = 16'h6512;  // MOV.L @R1,R5   ; cache hit: still the dirty 0x51
        imem['h29] = 16'h0009;  // sentinel
        imem['h2A] = 16'hAFFE;  // guard
        imem['h2B] = 16'h0009;
        do_reset;
        dmem['h60] = 32'h0000_0041;
        run_until_retire('h29, 20000);
        chk("TAS saw the MEMORY byte (0) -> T",   gpr(4), 32'd1);
        chk("TAS set bit7 in MEMORY only",        dmem['h60], 32'h8000_0041);
        chk("dirty cache copy untouched by TAS",  gpr(5), 32'h0000_0051);
        do_reset;
        end_test;
    end
endtask

//GOLDEN Y - SR.BL=1 exception law (SH-3): an exception while BL is set is treated as
//a MANUAL RESET - EXPEVT = 0x020 and execution restarts at the reset vector. This
//burned three earlier tests as a silent reset-and-rerun; locked here on purpose.
task automatic test_bl_exception_reset;
    begin
        begin_test("SR.BL=1 exception = manual reset: EXPEVT 0x020, restart at the vector");
        //BL stays 1 (reset default - no prologue). The load faults -> reset behavior.
        imem[0] = 16'hE302;  // MOV   #2,R3
        imem[1] = 16'h4318;  // SHLL8 R3       ; R3 = 0x200
        imem[2] = 16'h6732;  // MOV.L @R3,R7   ; data abort under BL=1
        imem[3] = 16'h0009;
        d_fault_en   = 1'b1;
        d_fault_widx = 8'h80;
        do_reset;
        run_until_exc(2000);
        d_fault_en = 1'b0;      //let the rerun proceed past the load
        run_cycles(200);
        chk("EXPEVT = manual reset (0x020)", expevt_o, 32'h0000_0020);
        chk_true("execution restarted at the reset vector", retire_count[0] >= 2);
        end_test;
    end
endtask

//GOLDEN Z - wrong-path I-miss with a DIRTY victim, squashed mid-excursion. The
//skipped poison line's set holds a dirty D-line; the fetch-ahead miss evicts it
//(buffer + drain + fill run under a squash). LAWS: no deadlock, the wrong-path
//line never retires, and the evicted dirty data survives the round trip.
task automatic test_squash_victim_drain;
    integer j, l;
    begin
        begin_test("Squashed I-miss with dirty victim: drain integrity under the squash");
        for(l = 0; l < 3; l = l + 1) begin      //latency sweep varies the squash arrival
            cacheable_bootstrap(8'h09);
            imem['h20] = 16'hEA00;  // MOV   #0,R10   ; poison detector
            imem['h21] = 16'hE110;  // MOV   #0x10,R1
            imem['h22] = 16'h4118;  // SHLL8 R1
            imem['h23] = 16'h7160;  // ADD   #0x60,R1 ; R1 = 0x1060 (SET 6 = the poison line's set)
            imem['h24] = 16'hE310;  // MOV   #0x10,R3
            imem['h25] = 16'h4318;  // SHLL8 R3       ; same-set stride 0x1000
            imem['h26] = 16'hE077;  // MOV   #0x77,R0
            imem['h27] = 16'h2102;  // MOV.L R0,@R1   ; dirty -> way 3 of set 6
            imem['h28] = 16'h6213;  // MOV   R1,R2
            imem['h29] = 16'h323C;  // ADD   R3,R2    ; 0x2060
            imem['h2A] = 16'h6422;  // MOV.L @R2,R4   ; fill way 2
            imem['h2B] = 16'h323C;  //                ; 0x3060
            imem['h2C] = 16'h6422;  //                ; fill way 1
            imem['h2D] = 16'h323C;  //                ; 0x4060
            imem['h2E] = 16'h6422;  //                ; fill way 0: dirty way is next victim
            //BRA sits right before the poison line so fetch-ahead misses INTO it:
            //the wrong-path I-fill must evict the dirty way (buffer+drain under squash).
            imem['h2F] = 16'h0009;  //   (delay slot of nothing - spacing)
            imem['h30] = 16'hA008;  // BRA  0x74      ; skip poison line 0x60-0x6F... byte 0x60+
            imem['h31] = 16'h0009;  //   delay slot (line 0x62)
            for(j = 0; j < 7; j = j + 1) imem['h32 + j] = 16'hEA11;  //poison into set 7 too
            imem['h3A] = 16'h6912;  // MOV.L @R1,R9   ; BRA target: reload the dirty word
            imem['h3B] = 16'h0009;  // sentinel
            imem['h3C] = 16'hAFFE;  // guard
            imem['h3D] = 16'h0009;
            i_latency = l;
            do_reset;
            run_until_retire('h3B, 30000);
            chk($sformatf("dirty data survives the squashed eviction (i=%0d)", l),
                gpr(9), 32'h0000_0077);
            chk($sformatf("wrong-path poison never retired (i=%0d)", l), gpr(10), 32'd0);
        end
        i_latency = 0;
        do_reset;
        end_test;
    end
endtask

//Suite-wide property verdicts, evaluated LAST so every test fed the checkers.
task automatic test_property_summary;
    begin
        begin_test("Suite-wide properties: LRU divergence, L-bus/MEM-bus contracts");
        $display("      LRU: %0d update checks, %0d victim checks; L-bus: %0d req-pend, %0d rsp-pend cycles; MEM-bus: %0d pend cycles",
                 lru_upd_checks, lru_vic_checks, lbus_dreq_pends, lbus_drsp_pends, mbus_req_pends);
        chk_true("LRU checker exercised", (lru_upd_checks > 100) && (lru_vic_checks > 20));
        chk("LRU divergences",                    lru_mismatches[31:0], 32'd0);
        chk_true("D-request stability exercised", lbus_dreq_pends > 0);
        chk("L-bus D-request stability violations", lbus_dreq_viol[31:0], 32'd0);
        chk("L-bus D-response hold violations",     lbus_drsp_viol[31:0], 32'd0);
        chk("MEM-bus request stability violations", mbus_req_viol[31:0],  32'd0);
        end_test;
    end
endtask

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



///////////////////////////////////////////////////////////
//////  Test Cases
////

//WB is held at E0, BRAM writes/reads at E1, and ID/EX stores the value at E2.
task automatic test_gpr_bram_phases;
    begin
        begin_test("GPR BRAM phases: E1 write/read, E2 operand capture");
        imem[0] = 16'hE12A; // MOV #0x2A,R1     ; WB phase probe
        imem[1] = 16'h0009; // NOP              ; separate producer from BRAM reader
        imem[2] = 16'h0009; // NOP
        imem[3] = 16'h0009; // NOP
        imem[4] = 16'h6213; // MOV R1,R2        ; BRAM read and ID/EX capture probe
        imem[5] = 16'h0009; // NOP              ; sentinel
        do_reset;
        run_until(5, 400);
        chk_true("E1 GPR write observed", gpr_phase_write_seen);
        chk_true("E1 GPR read observed", gpr_phase_read_seen);
        chk_true("E2 ID/EX capture observed", gpr_phase_capture_seen);
        chk("R2 captured BRAM value", gpr(2), 32'h0000_002A);
        end_test;
    end
endtask

//Back-to-back ALU producers/consumers: current EX forwards into the E2 capture.
task automatic test_alu_forwarding;
    begin
        begin_test("ALU forwarding: EX result feeds the next ID/EX capture with no bubble");
        imem[0] = 16'hE101; // MOV  #1,R1        ; seed R1=1
        imem[1] = 16'h7101; // ADD  #1,R1        ; R1=2  capture current EX result
        imem[2] = 16'h7101; // ADD  #1,R1        ; R1=3  dependent chain
        imem[3] = 16'h7101; // ADD  #1,R1        ; R1=4  dependent chain
        imem[4] = 16'hE202; // MOV  #2,R2        ; R2=2
        imem[5] = 16'h312C; // ADD  R2,R1        ; R1=6  forward R2 and R1
        imem[6] = 16'h6313; // MOV  R1,R3        ; R3=6  forward R1
        imem[7] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(7, 400);
        chk("R1", gpr(1), 32'd6);
        chk("R3", gpr(3), 32'd6);
        end_test;
    end
endtask

//Load then immediate use: interlock until WB data can enter the E2 capture.
task automatic test_load_use_interlock;
    begin
        begin_test("Load-use interlock: consumer waits then captures completing WB load data");
        imem[0] = 16'hE000; // MOV  #0,R0        ; address 0
        imem[1] = 16'hE107; // MOV  #7,R1        ; value 7
        imem[2] = 16'h2012; // MOV.L R1,@R0      ; mem[0]=7  (store-data forward of R1)
        imem[3] = 16'h6202; // MOV.L @R0,R2      ; R2=mem[0]=7  (load)
        imem[4] = 16'h322C; // ADD  R2,R2        ; R2=14  wait, then capture WB load data
        imem[5] = 16'hE501; // MOV  #1,R5        ; independent
        imem[6] = 16'hE602; // MOV  #2,R6        ; independent
        imem[7] = 16'h356C; // ADD  R6,R5        ; R5=3  ALU producers forward, no stall
        imem[8] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(8, 400);
        chk("R2 (load-use)", gpr(2), 32'd14);
        chk("R5 (independent)", gpr(5), 32'd3);
        chk_true("load retired", retired_seen[3]);
        chk_true("consumer retired", retired_seen[4]);
        end_test;
    end
endtask

//An ALU result is consumed as store data: mem_data forwarding path.
task automatic test_store_data_forwarding;
    begin
        begin_test("Store-data forwarding: ALU result drives the store mem_data lane");
        imem[0] = 16'hE000; // MOV  #0,R0        ; address 0
        imem[1] = 16'hE105; // MOV  #5,R1
        imem[2] = 16'h7103; // ADD  #3,R1        ; R1=8
        imem[3] = 16'h2012; // MOV.L R1,@R0      ; mem[0]=8  forward R1 into store data
        imem[4] = 16'h6202; // MOV.L @R0,R2      ; R2=mem[0]=8
        imem[5] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(5, 400);
        chk("R2 (round-trip)", gpr(2), 32'd8);
        chk("dmem[0]", dmem[0], 32'd8);
        end_test;
    end
endtask

//Indexed @(R0,Rn): exercises ADDR_INDEX, the R0 third-read mirror, store/load.
task automatic test_indexed_r0;
    begin
        begin_test("Indexed @(R0,Rn): R0 third-read mirror supplies the index operand");
        imem[0] = 16'hE300; // MOV  #0,R3        ; base Rn=R3=0
        imem[1] = 16'hE010; // MOV  #16,R0       ; index R0=0x10
        imem[2] = 16'hE255; // MOV  #0x55,R2     ; store data
        imem[3] = 16'h0326; // MOV.L R2,@(R0,R3) ; mem[R3+R0=16]=0x55
        imem[4] = 16'h043E; // MOV.L @(R0,R3),R4 ; R4=mem[16]=0x55
        imem[5] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(5, 400);
        chk("R4 (indexed load)", gpr(4), 32'h0000_0055);
        chk("dmem[4]", dmem[4], 32'h0000_0055);
        end_test;
    end
endtask

//Pre-decrement and post-increment write the address-update (gpr1) lane.
task automatic test_addr_update;
    begin
        begin_test("Address update lanes: @-Rn / @Rm+ write the pointer; Rm==Rn keeps loaded value");
        imem[0] = 16'hE020; // MOV  #32,R0       ; R0=0x20
        imem[1] = 16'hE155; // MOV  #0x55,R1
        imem[2] = 16'h2016; // MOV.L R1,@-R0     ; R0=0x1C, mem[0x1C]=0x55  (pre-dec update lane)
        imem[3] = 16'h6206; // MOV.L @R0+,R2     ; R2=mem[0x1C]=0x55, R0=0x20 (forward updated R0, post-inc)
        imem[4] = 16'hE340; // MOV  #64,R3       ; R3=0x40
        imem[5] = 16'hE47A; // MOV  #0x7A,R4
        imem[6] = 16'h2342; // MOV.L R4,@R3      ; mem[0x40]=0x7A
        imem[7] = 16'h6336; // MOV.L @R3+,R3     ; Rm==Rn -> R3=loaded 0x7A (not the +4 update)
        imem[8] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(8, 400);
        chk("R2 (post-inc load)", gpr(2), 32'h0000_0055);
        chk("R0 (pre-dec then post-inc)", gpr(0), 32'h0000_0020);
        chk("R3 (Rm==Rn suppression)", gpr(3), 32'h0000_007A);
        end_test;
    end
endtask

//CMP sets T, BT consumes it next cycle: T forwarding + non-delayed taken redirect.
task automatic test_tbit_branch;
    begin
        begin_test("T-bit forward to BT: non-delayed taken branch redirects and kills wrong path");
        imem[0] = 16'hE105; // MOV  #5,R1
        imem[1] = 16'hE205; // MOV  #5,R2
        imem[2] = 16'h3120; // CMP/EQ R2,R1      ; T=1
        imem[3] = 16'h8901; // BT   +1 -> word 6 ; taken, forwards T from EX/MA
        imem[4] = 16'hE311; // MOV  #0x11,R3     ; wrong path (must not retire)
        imem[5] = 16'hE322; // MOV  #0x22,R3     ; wrong path
        imem[6] = 16'hE433; // MOV  #0x33,R4     ; branch target
        imem[7] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(7, 400);
        chk_true("T set by CMP/EQ", sr[0]);
        chk_true("target retired", retired_seen[6]);
        chk("R4 (target)", gpr(4), 32'h0000_0033);
        chk_true("wrong path w4 killed", !retired_seen[4]);
        chk_true("wrong path w5 killed", !retired_seen[5]);
        end_test;
    end
endtask

//DT/BF countdown: DT updates T, BF reads it, the loop redirects fetch three times.
task automatic test_dt_loop;
    begin
        begin_test("DT/BF loop: T forward to conditional branch over a fetch-redirect loop");
        imem[0] = 16'hE103; // MOV  #3,R1
        imem[1] = 16'h4110; // DT   R1           ; R1--, T=(R1==0)   loop body
        imem[2] = 16'h8BFD; // BF   -3 -> word 1 ; branch while T==0
        imem[3] = 16'hE544; // MOV  #0x44,R5     ; loop exit
        imem[4] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(4, 400);
        chk("R1 (counted to zero)", gpr(1), 32'd0);
        chk("R5 (after loop)", gpr(5), 32'h0000_0044);
        chk("DT iterations", retire_count[1], 3);
        end_test;
    end
endtask

//Delayed conditional branch: the delay slot retires, the fall-through is killed.
task automatic test_delayed_branch;
    begin
        begin_test("Delayed branch BT/S: delay slot commits, wrong path is flushed");
        imem[0] = 16'hE105; // MOV  #5,R1
        imem[1] = 16'h3110; // CMP/EQ R1,R1      ; T=1
        imem[2] = 16'h8D01; // BT/S +1 -> word 5 ; delayed, taken
        imem[3] = 16'hE455; // MOV  #0x55,R4     ; delay slot (must retire)
        imem[4] = 16'hE566; // MOV  #0x66,R5     ; wrong path (must not retire)
        imem[5] = 16'hE677; // MOV  #0x77,R6     ; branch target
        imem[6] = 16'h0009; // NOP               ; sentinel
        do_reset;
        run_until(6, 400);
        chk_true("delay slot retired", retired_seen[3]);
        chk_true("target retired", retired_seen[5]);
        chk_true("wrong path killed", !retired_seen[4]);
        chk("R4 (delay slot)", gpr(4), 32'h0000_0055);
        chk("R6 (target)", gpr(6), 32'h0000_0077);
        end_test;
    end
endtask

//BSR links PR, RTS returns through it: delayed call/return with delay slots.
task automatic test_bsr_rts;
    begin
        begin_test("BSR/RTS: PR link on call, PR-target return, both delay slots commit");
        imem[0] = 16'hE100; // MOV  #0,R1
        imem[1] = 16'hB002; // BSR  +2 -> word 5 ; delayed call, PR=word3
        imem[2] = 16'hE211; // MOV  #0x11,R2     ; call delay slot (must retire)
        imem[3] = 16'hE333; // MOV  #0x33,R3     ; return lands here
        imem[4] = 16'h0009; // NOP               ; sentinel
        imem[5] = 16'hE555; // MOV  #0x55,R5     ; subroutine body (BSR target)
        imem[6] = 16'h000B; // RTS               ; delayed return to PR(word3)
        imem[7] = 16'hE666; // MOV  #0x66,R6     ; RTS delay slot (must retire)
        imem[8] = 16'hE777; // MOV  #0x77,R7     ; past RTS (must not retire)
        do_reset;
        run_until(4, 500);
        chk_true("call delay slot retired", retired_seen[2]);
        chk_true("subroutine body retired", retired_seen[5]);
        chk_true("rts delay slot retired", retired_seen[7]);
        chk_true("return point retired", retired_seen[3]);
        chk_true("past-RTS not retired", !retired_seen[8]);
        chk("R2", gpr(2), 32'h0000_0011);
        chk("R5", gpr(5), 32'h0000_0055);
        chk("R6", gpr(6), 32'h0000_0066);
        chk("R3 (after return)", gpr(3), 32'h0000_0033);
        end_test;
    end
endtask

//Multiply unit and MACL/STS path. MACL has no forwarding, so STS is separated.
task automatic test_mul_mac;
    begin
        begin_test("Multiply/MAC: MULU.W, MUL.L, CLRMAC into MACL (STS reads need separation)");
        imem[0]  = 16'hE106; // MOV  #6,R1
        imem[1]  = 16'hE207; // MOV  #7,R2
        imem[2]  = 16'h221E; // MULU.W R1,R2     ; MACL = R2*R1 = 42
        imem[3]  = 16'h0009; // NOP              ; MACL has no forwarding: let it commit
        imem[4]  = 16'h0009; // NOP
        imem[5]  = 16'h0009; // NOP
        imem[6]  = 16'h031A; // STS  MACL,R3     ; R3 = 42
        imem[7]  = 16'hE403; // MOV  #3,R4
        imem[8]  = 16'hE505; // MOV  #5,R5
        imem[9]  = 16'h0547; // MUL.L R4,R5      ; MACL = R5*R4 = 15
        imem[10] = 16'h0009; // NOP
        imem[11] = 16'h0009; // NOP
        imem[12] = 16'h0009; // NOP
        imem[13] = 16'h061A; // STS  MACL,R6     ; R6 = 15
        imem[14] = 16'h0028; // CLRMAC           ; MACL=MACH=0
        imem[15] = 16'h0009; // NOP
        imem[16] = 16'h0009; // NOP
        imem[17] = 16'h0009; // NOP
        imem[18] = 16'h071A; // STS  MACL,R7     ; R7 = 0
        imem[19] = 16'h0009; // sentinel
        do_reset;
        run_until(19, 700);
        chk("R3 (MULU.W result)", gpr(3), 32'd42);
        chk("R6 (MUL.L result)", gpr(6), 32'd15);
        chk("R7 (after CLRMAC)", gpr(7), 32'd0);
        chk("MACL register", macl_o, 32'd0);
        end_test;
    end
endtask

//Signed word multiply checks sign handling on the MACL output.
task automatic test_muls_w;
    begin
        begin_test("MULS.W: signed 16x16 product to MACL");
        imem[0] = 16'hE1FF; // MOV  #-1,R1       ; R1=0xFFFFFFFF
        imem[1] = 16'hE202; // MOV  #2,R2
        imem[2] = 16'h221F; // MULS.W R1,R2      ; MACL = (signed)R2*R1 = -2
        imem[3] = 16'h0009; // NOP
        imem[4] = 16'h0009; // sentinel
        do_reset;
        run_until(4, 300);
        chk("MACL (signed -2)", macl_o, 32'hFFFF_FFFE);
        end_test;
    end
endtask

//DMULS.L and DMULU.L write the complete two-cycle product into MACH:MACL.
task automatic test_dmul_long;
    begin
        begin_test("DMUL.L: signed and unsigned 32x32 products write MACH:MACL");
        imem[0]  = 16'hE1FE; // MOV  #-2,R1
        imem[1]  = 16'hE203; // MOV  #3,R2
        imem[2]  = 16'h321D; // DMULS.L R1,R2    ; signed result = -6
        imem[3]  = 16'h031A; // STS  MACL,R3      ; waits for DMULS.L commit
        imem[4]  = 16'h040A; // STS  MACH,R4
        imem[5]  = 16'hE5FF; // MOV  #-1,R5       ; unsigned 0xFFFFFFFF
        imem[6]  = 16'hE602; // MOV  #2,R6
        imem[7]  = 16'h3655; // DMULU.L R5,R6    ; unsigned result = 0x1FFFFFFFE
        imem[8]  = 16'h071A; // STS  MACL,R7      ; waits for DMULU.L commit
        imem[9]  = 16'h080A; // STS  MACH,R8
        imem[10] = 16'h0009; // sentinel
        do_reset;
        run_until(10, 600);
        chk("R3 (DMULS.L low)", gpr(3), 32'hFFFF_FFFA);
        chk("R4 (DMULS.L high)", gpr(4), 32'hFFFF_FFFF);
        chk("R7 (DMULU.L low)", gpr(7), 32'hFFFF_FFFE);
        chk("R8 (DMULU.L high)", gpr(8), 32'h0000_0001);
        chk("DMULU.L latency", mac_last_latency, 32'd2);
        end_test;
    end
endtask

//MAC.L reads two longwords, updates both pointers, and accumulates full 64 bits.
task automatic test_mac_l_accumulate;
    begin
        begin_test("MAC.L: ordered longword reads, full accumulation, two-cycle DSP latency");
        imem[0] = 16'hE50A; // MOV  #10,R5       ; nonzero accumulator seed
        imem[1] = 16'h451A; // LDS  R5,MACL      ; MACH:MACL = 10
        imem[2] = 16'hE100; // MOV  #0,R1        ; Rm points to longword 3
        imem[3] = 16'hE208; // MOV  #8,R2        ; Rn points to longword 4
        imem[4] = 16'h021F; // MAC.L @R1+,@R2+  ; MACH:MACL = 10 + 4*3
        imem[5] = 16'h031A; // STS  MACL,R3      ; dependency logic waits for MAC
        imem[6] = 16'h040A; // STS  MACH,R4
        imem[7] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'd3;
        dmem[2] = 32'd4;
        run_until(7, 500);
        chk("R1 post-increment", gpr(1), 32'd4);
        chk("R2 post-increment", gpr(2), 32'd12);
        chk("R3 (MACL)", gpr(3), 32'd22);
        chk("R4 (MACH)", gpr(4), 32'd0);
        chk("MAC.L latency", mac_last_latency, 32'd2);
        end_test;
    end
endtask

//MAC.W sign-extends each memory word before full MACH:MACL accumulation.
task automatic test_mac_w_accumulate;
    begin
        begin_test("MAC.W: signed word accumulation and one-cycle DSP latency");
        imem[0] = 16'hE100; // MOV  #0,R1        ; Rm reads -2 from upper word
        imem[1] = 16'hE202; // MOV  #2,R2        ; Rn reads 3 from lower word
        imem[2] = 16'h0028; // CLRMAC
        imem[3] = 16'h421F; // MAC.W @R1+,@R2+  ; MACH:MACL += 3*(-2)
        imem[4] = 16'h031A; // STS  MACL,R3
        imem[5] = 16'h040A; // STS  MACH,R4
        imem[6] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'hFFFE_0003;
        run_until(6, 500);
        chk("R1 post-increment", gpr(1), 32'd2);
        chk("R2 post-increment", gpr(2), 32'd4);
        chk("R3 (MACL signed -6)", gpr(3), 32'hFFFF_FFFA);
        chk("R4 (MACH signed -6)", gpr(4), 32'hFFFF_FFFF);
        chk("MAC.W latency", mac_last_latency, 32'd1);
        end_test;
    end
endtask

//Equal pointer registers consume consecutive words and write one doubled update.
task automatic test_mac_w_same_pointer;
    begin
        begin_test("MAC.W same pointer: second read observes first post-increment");
        imem[0] = 16'hE100; // MOV  #0,R1
        imem[1] = 16'h0028; // CLRMAC
        imem[2] = 16'h411F; // MAC.W @R1+,@R1+  ; reads addresses zero then two
        imem[3] = 16'h021A; // STS  MACL,R2
        imem[4] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'h0002_0003;
        run_until(4, 500);
        chk("R1 double post-increment", gpr(1), 32'd4);
        chk("R2 (2*3)", gpr(2), 32'd6);
        chk("MAC.W latency", mac_last_latency, 32'd1);
        end_test;
    end
endtask

//SR.S selects the manual-defined 32-bit MAC.W and 48-bit MAC.L saturation modes.
task automatic test_mac_saturation;
    begin
        begin_test("MAC saturation: MAC.W clamps MACL and MAC.L clamps low 48 bits");
        imem[0]  = 16'hE310; // MOV  #16,R3       ; MAC.W initial MACL address
        imem[1]  = 16'h6432; // MOV.L @R3,R4      ; 0x7FFFFFFE
        imem[2]  = 16'h441A; // LDS  R4,MACL
        imem[3]  = 16'h0058; // SETS              ; enable saturation
        imem[4]  = 16'hE100; // MOV  #0,R1
        imem[5]  = 16'hE202; // MOV  #2,R2
        imem[6]  = 16'h421F; // MAC.W @R1+,@R2+  ; max-1 + 2*2 saturates
        imem[7]  = 16'h051A; // STS  MACL,R5
        imem[8]  = 16'h060A; // STS  MACH,R6      ; bit zero records overflow
        imem[9]  = 16'hE320; // MOV  #32,R3       ; initial MACH address
        imem[10] = 16'h6432; // MOV.L @R3,R4      ; MACH low half = 0x7FFF
        imem[11] = 16'h440A; // LDS  R4,MACH
        imem[12] = 16'hE324; // MOV  #36,R3       ; initial MACL address
        imem[13] = 16'h6432; // MOV.L @R3,R4      ; MACL = 0xFFFFFFFE
        imem[14] = 16'h441A; // LDS  R4,MACL
        imem[15] = 16'hE100; // MOV  #0,R1
        imem[16] = 16'hE204; // MOV  #4,R2
        imem[17] = 16'h021F; // MAC.L @R1+,@R2+  ; 48-bit max-1 + 2*2
        imem[18] = 16'h071A; // STS  MACL,R7
        imem[19] = 16'h080A; // STS  MACH,R8
        imem[20] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'h0002_0002;
        dmem[1] = 32'd2;
        dmem[4] = 32'h7FFF_FFFE;
        dmem[8] = 32'h0000_7FFF;
        dmem[9] = 32'hFFFF_FFFE;
        run_until(20, 1000);
        chk("R5 (MAC.W positive clamp)", gpr(5), 32'h7FFF_FFFF);
        chk("R6 (MAC.W overflow flag)", gpr(6), 32'h0000_0001);
        chk("R7 (MAC.L low clamp)", gpr(7), 32'hFFFF_FFFF);
        chk("R8 (MAC.L high clamp)", gpr(8), 32'h0000_7FFF);
        chk("final MAC.L latency", mac_last_latency, 32'd2);
        end_test;
    end
endtask

//ADDSUB unit: ADD/SUB plus the carry/overflow/borrow T flag of ADDC/ADDV/
//SUBC/SUBV/NEGC. CLRT/SETT seed the carry; MOVT captures each carry-out.
task automatic test_addsub_t;
    begin
        begin_test("ADDSUB+T: ADD, ADDC carry chain, ADDV/SUBV overflow, SUBC borrow, NEG/NEGC");
        imem[0]  = 16'hE105; // MOV  #5,R1
        imem[1]  = 16'hE203; // MOV  #3,R2
        imem[2]  = 16'h312C; // ADD  R2,R1        ; R1=8
        imem[3]  = 16'hE2FF; // MOV  #-1,R2       ; R2=0xFFFFFFFF
        imem[4]  = 16'hE301; // MOV  #1,R3        ; temp addend
        imem[5]  = 16'h0008; // CLRT              ; T=0 (carry-in)
        imem[6]  = 16'h323E; // ADDC R3,R2        ; R2=0, T=1 (carry out)
        imem[7]  = 16'h0329; // MOVT R3           ; R3=1
        imem[8]  = 16'hE405; // MOV  #5,R4
        imem[9]  = 16'hE50A; // MOV  #10,R5
        imem[10] = 16'h345E; // ADDC R5,R4        ; R4=5+10+T(1)=16, T=0
        imem[11] = 16'h0629; // MOVT R6           ; R6=0
        imem[12] = 16'hE7FF; // MOV  #-1,R7
        imem[13] = 16'h4701; // SHLR R7           ; R7=0x7FFFFFFF, T=1
        imem[14] = 16'hEC01; // MOV  #1,R12       ; constant 1
        imem[15] = 16'h37CF; // ADDV R12,R7       ; R7=0x80000000, T=1 (signed overflow)
        imem[16] = 16'h0829; // MOVT R8           ; R8=1
        imem[17] = 16'h37CB; // SUBV R12,R7       ; R7=0x7FFFFFFF, T=1 (signed overflow)
        imem[18] = 16'h0929; // MOVT R9           ; R9=1
        imem[19] = 16'hEA00; // MOV  #0,R10
        imem[20] = 16'h0008; // CLRT              ; T=0 (borrow-in)
        imem[21] = 16'h3ACA; // SUBC R12,R10      ; R10=0-1-0=0xFFFFFFFF, T=1 (borrow)
        imem[22] = 16'h0B29; // MOVT R11          ; R11=1
        imem[23] = 16'hED0C; // MOV  #12,R13
        imem[24] = 16'h6EDB; // NEG  R13,R14      ; R14=-12=0xFFFFFFF4 (no T)
        imem[25] = 16'h0008; // CLRT              ; T=0 (borrow-in)
        imem[26] = 16'h60DA; // NEGC R13,R0       ; R0=-12=0xFFFFFFF4, T=1 (borrow)
        imem[27] = 16'h0009; // sentinel
        do_reset;
        run_until(27, 700);
        chk("R1 (ADD)", gpr(1), 32'd8);
        chk("R2 (ADDC wrap)", gpr(2), 32'd0);
        chk("R3 (ADDC carry out)", gpr(3), 32'd1);
        chk("R4 (ADDC carry in)", gpr(4), 32'd16);
        chk("R6 (no carry out)", gpr(6), 32'd0);
        chk("R7 (after SUBV)", gpr(7), 32'h7FFF_FFFF);
        chk("R8 (ADDV overflow)", gpr(8), 32'd1);
        chk("R9 (SUBV overflow)", gpr(9), 32'd1);
        chk("R10 (SUBC borrow result)", gpr(10), 32'hFFFF_FFFF);
        chk("R11 (SUBC borrow out)", gpr(11), 32'd1);
        chk("R14 (NEG)", gpr(14), 32'hFFFF_FFF4);
        chk("R0 (NEGC)", gpr(0), 32'hFFFF_FFF4);
        chk_true("T set by NEGC borrow", sr[0]);
        end_test;
    end
endtask

//LOGIC unit: AND/OR/XOR/NOT in register and #imm,R0 forms, plus TST setting
//T from a bitwise-AND zero test (both the zero and non-zero cases).
task automatic test_logic_t;
    begin
        begin_test("LOGIC+T: AND/OR/XOR/NOT, immediate logic on R0, TST zero/non-zero into T");
        imem[0]  = 16'hE10F; // MOV  #15,R1
        imem[1]  = 16'hE23C; // MOV  #60,R2
        imem[2]  = 16'h2129; // AND  R2,R1        ; R1=0x0F & 0x3C = 0x0C
        imem[3]  = 16'hE30F; // MOV  #15,R3
        imem[4]  = 16'hE43C; // MOV  #60,R4
        imem[5]  = 16'h234B; // OR   R4,R3        ; R3=0x3F
        imem[6]  = 16'hE50F; // MOV  #15,R5
        imem[7]  = 16'hE63C; // MOV  #60,R6
        imem[8]  = 16'h256A; // XOR  R6,R5        ; R5=0x33
        imem[9]  = 16'hE70F; // MOV  #15,R7
        imem[10] = 16'h6877; // NOT  R7,R8        ; R8=0xFFFFFFF0
        imem[11] = 16'hE90F; // MOV  #15,R9
        imem[12] = 16'hEAF0; // MOV  #-16,R10     ; R10=0xFFFFFFF0
        imem[13] = 16'h29A8; // TST  R10,R9       ; 0x0F & 0xFFFFFFF0 = 0 -> T=1
        imem[14] = 16'h0B29; // MOVT R11          ; R11=1
        imem[15] = 16'hEC03; // MOV  #3,R12
        imem[16] = 16'h29C8; // TST  R12,R9       ; 0x0F & 0x03 = 3 -> T=0
        imem[17] = 16'h0D29; // MOVT R13          ; R13=0
        imem[18] = 16'hE0FF; // MOV  #-1,R0       ; R0=0xFFFFFFFF
        imem[19] = 16'hC90F; // AND  #0x0F,R0     ; R0=0x0F (imm is zero-extended)
        imem[20] = 16'hCB30; // OR   #0x30,R0     ; R0=0x3F
        imem[21] = 16'hCA0F; // XOR  #0x0F,R0     ; R0=0x30
        imem[22] = 16'hC840; // TST  #0x40,R0     ; 0x30 & 0x40 = 0 -> T=1 (result stored in T)
        //Chain A: T=1 must survive a run of operations that never write T, then
        //ADDC consumes it. The non-T ops also form a data chain (each feeds the
        //next). R14=16 only happens if T was still 1 at the ADDC.
        imem[23] = 16'h6203; // MOV  R0,R2        ; non-T (R2=0x30)
        imem[24] = 16'h7201; // ADD  #1,R2        ; non-T (R2=0x31, uses previous R2)
        imem[25] = 16'h4208; // SHLL2 R2          ; non-T (R2=0xC4, uses previous R2)
        imem[26] = 16'h622C; // EXTU.B R2,R2      ; non-T (R2=0xC4)
        imem[27] = 16'hEE05; // MOV  #5,R14
        imem[28] = 16'hEF0A; // MOV  #10,R15
        imem[29] = 16'h3EFE; // ADDC R15,R14      ; R14=5+10+T(1)=16, T=0 (carry consumed)
        imem[30] = 16'h0429; // MOVT R4           ; R4=0 (carry out of the ADDC)
        //Chain B: CLRT stores T=0; the same kinds of non-T ops follow; ADDC then
        //adds carry-in 0. R12=21 only happens if T was still 0 at the ADDC.
        imem[31] = 16'h0008; // CLRT              ; T=0 (result stored in T)
        imem[32] = 16'hE907; // MOV  #7,R9        ; scratch
        imem[33] = 16'h2299; // AND  R9,R2        ; non-T (R2=0xC4 & 7 = 4)
        imem[34] = 16'h229B; // OR   R9,R2        ; non-T (R2=4 | 7 = 7)
        imem[35] = 16'hEC01; // MOV  #1,R12
        imem[36] = 16'hEF14; // MOV  #20,R15
        imem[37] = 16'h3CFE; // ADDC R15,R12      ; R12=1+20+T(0)=21, T=0
        imem[38] = 16'h0629; // MOVT R6           ; R6=0 (carry out of the ADDC)
        imem[39] = 16'h0009; // sentinel
        do_reset;
        run_until(39, 700);
        chk("R1 (AND)", gpr(1), 32'h0000_000C);
        chk("R3 (OR)", gpr(3), 32'h0000_003F);
        chk("R5 (XOR)", gpr(5), 32'h0000_0033);
        chk("R8 (NOT)", gpr(8), 32'hFFFF_FFF0);
        chk("R11 (TST zero -> T)", gpr(11), 32'd1);
        chk("R13 (TST non-zero -> T)", gpr(13), 32'd0);
        chk("R0 (imm AND/OR/XOR)", gpr(0), 32'h0000_0030);
        chk("R2 (non-T data chain result)", gpr(2), 32'd7);
        chk("R14 (TST T=1 survived non-T ops into ADDC)", gpr(14), 32'd16);
        chk("R4 (chain A carry out)", gpr(4), 32'd0);
        chk("R12 (CLRT T=0 survived non-T ops into ADDC)", gpr(12), 32'd21);
        chk("R6 (chain B carry out)", gpr(6), 32'd0);
        end_test;
    end
endtask

//SHIFT unit: constant SHLLn/SHLRn re-wires, single-bit SHLL/SHLR/SHAR with the
//shifted-out bit in T, and the dynamic SHAD/SHLD barrel with +/- counts.
task automatic test_shift_ops;
    begin
        begin_test("SHIFT+T: constant shifts, SHLL/SHLR/SHAR shifted-out bit to T, SHAD/SHLD barrel");
        imem[0]  = 16'hE101; // MOV  #1,R1
        imem[1]  = 16'h4108; // SHLL2 R1          ; R1=4
        imem[2]  = 16'h4118; // SHLL8 R1          ; R1=0x400
        imem[3]  = 16'h4128; // SHLL16 R1         ; R1=0x04000000
        imem[4]  = 16'hE2FF; // MOV  #-1,R2       ; 0xFFFFFFFF
        imem[5]  = 16'h4229; // SHLR16 R2         ; 0x0000FFFF
        imem[6]  = 16'h4219; // SHLR8 R2          ; 0x000000FF
        imem[7]  = 16'h4209; // SHLR2 R2          ; 0x0000003F
        imem[8]  = 16'hE301; // MOV  #1,R3
        imem[9]  = 16'h4301; // SHLR R3           ; R3=0, T=1 (bit0)
        imem[10] = 16'h0429; // MOVT R4           ; R4=1
        imem[11] = 16'hE5FF; // MOV  #-1,R5
        imem[12] = 16'h4500; // SHLL R5           ; R5=0xFFFFFFFE, T=1 (bit31)
        imem[13] = 16'h0629; // MOVT R6           ; R6=1
        imem[14] = 16'hE7FF; // MOV  #-1,R7
        imem[15] = 16'h4721; // SHAR R7           ; R7=0xFFFFFFFF (sign fill), T=1 (bit0)
        imem[16] = 16'h0829; // MOVT R8           ; R8=1
        imem[17] = 16'hE901; // MOV  #1,R9
        imem[18] = 16'hEA04; // MOV  #4,R10       ; left count +4
        imem[19] = 16'h49AD; // SHLD R10,R9       ; R9=1<<4=0x10
        imem[20] = 16'hEBFC; // MOV  #-4,R11      ; right count -4
        imem[21] = 16'h49BD; // SHLD R11,R9       ; R9=0x10>>4=1 (zero fill)
        imem[22] = 16'hED80; // MOV  #-128,R13    ; R13=0xFFFFFF80
        imem[23] = 16'hECFE; // MOV  #-2,R12      ; right count -2
        imem[24] = 16'h4DCC; // SHAD R12,R13      ; R13=0xFFFFFF80>>>2=0xFFFFFFE0 (sign fill)
        imem[25] = 16'h0009; // sentinel
        do_reset;
        run_until(25, 600);
        chk("R1 (SHLL2/8/16)", gpr(1), 32'h0400_0000);
        chk("R2 (SHLR16/8/2)", gpr(2), 32'h0000_003F);
        chk("R3 (SHLR)", gpr(3), 32'd0);
        chk("R4 (SHLR bit0 -> T)", gpr(4), 32'd1);
        chk("R5 (SHLL)", gpr(5), 32'hFFFF_FFFE);
        chk("R6 (SHLL bit31 -> T)", gpr(6), 32'd1);
        chk("R7 (SHAR sign fill)", gpr(7), 32'hFFFF_FFFF);
        chk("R8 (SHAR bit0 -> T)", gpr(8), 32'd1);
        chk("R9 (SHLD left then right)", gpr(9), 32'd1);
        chk("R13 (SHAD arithmetic right)", gpr(13), 32'hFFFF_FFE0);
        end_test;
    end
endtask

//ROTATE unit: ROTL/ROTR put the rotated-out bit in T; ROTCL/ROTCR rotate the
//register through T as a 33-bit shift, both consuming and producing T.
task automatic test_rotate_t;
    begin
        begin_test("ROTATE+T: ROTL/ROTR bit to T, ROTCL/ROTCR rotate through the T carry");
        imem[0]  = 16'hE101; // MOV  #1,R1
        imem[1]  = 16'h4105; // ROTR R1           ; R1=0x80000000, T=1 (bit0 out)
        imem[2]  = 16'h0229; // MOVT R2           ; R2=1
        imem[3]  = 16'h4104; // ROTL R1           ; R1=0x00000001, T=1 (bit31 out)
        imem[4]  = 16'h0329; // MOVT R3           ; R3=1
        imem[5]  = 16'h0018; // SETT              ; T=1 (carry-in)
        imem[6]  = 16'hE401; // MOV  #1,R4
        imem[7]  = 16'h4424; // ROTCL R4          ; R4={R4[30:0],T}=0x03, T=bit31=0
        imem[8]  = 16'h0529; // MOVT R5           ; R5=0
        imem[9]  = 16'h0018; // SETT              ; T=1 (carry-in)
        imem[10] = 16'hE601; // MOV  #1,R6
        imem[11] = 16'h4625; // ROTCR R6          ; R6={T,R6[31:1]}=0x80000000, T=bit0=1
        imem[12] = 16'h0729; // MOVT R7           ; R7=1
        imem[13] = 16'h0009; // sentinel
        do_reset;
        run_until(13, 400);
        chk("R1 (ROTR then ROTL)", gpr(1), 32'd1);
        chk("R2 (ROTR bit0 -> T)", gpr(2), 32'd1);
        chk("R3 (ROTL bit31 -> T)", gpr(3), 32'd1);
        chk("R4 (ROTCL through T)", gpr(4), 32'd3);
        chk("R5 (ROTCL bit31 -> T)", gpr(5), 32'd0);
        chk("R6 (ROTCR through T)", gpr(6), 32'h8000_0000);
        chk("R7 (ROTCR bit0 -> T)", gpr(7), 32'd1);
        end_test;
    end
endtask

//Compare unit (CEU): every CMP form and CMP/STR drive only T. Each result is
//captured with MOVT, exercising both true and false outcomes.
task automatic test_compare_t;
    begin
        begin_test("COMPARE+T: CMP/EQ,HS,HI,GE,GT,PZ,PL,STR and CMP/EQ #imm all into T");
        imem[0]  = 16'hE105; // MOV  #5,R1
        imem[1]  = 16'hE205; // MOV  #5,R2
        imem[2]  = 16'h3120; // CMP/EQ R2,R1      ; 5==5 -> T=1
        imem[3]  = 16'h0329; // MOVT R3           ; R3=1
        imem[4]  = 16'hE206; // MOV  #6,R2
        imem[5]  = 16'h3120; // CMP/EQ R2,R1      ; 5==6 -> T=0
        imem[6]  = 16'h0429; // MOVT R4           ; R4=0
        imem[7]  = 16'hE203; // MOV  #3,R2
        imem[8]  = 16'h3122; // CMP/HS R2,R1      ; 5>=3 unsigned -> T=1
        imem[9]  = 16'h0529; // MOVT R5           ; R5=1
        imem[10] = 16'hE205; // MOV  #5,R2
        imem[11] = 16'h3126; // CMP/HI R2,R1      ; 5>5 unsigned -> T=0
        imem[12] = 16'h0629; // MOVT R6           ; R6=0
        imem[13] = 16'hE1FF; // MOV  #-1,R1
        imem[14] = 16'hE201; // MOV  #1,R2
        imem[15] = 16'h3123; // CMP/GE R2,R1      ; -1>=1 signed -> T=0
        imem[16] = 16'h0729; // MOVT R7           ; R7=0
        imem[17] = 16'hE101; // MOV  #1,R1
        imem[18] = 16'hE2FF; // MOV  #-1,R2
        imem[19] = 16'h3127; // CMP/GT R2,R1      ; 1>-1 signed -> T=1
        imem[20] = 16'h0829; // MOVT R8           ; R8=1
        imem[21] = 16'hE100; // MOV  #0,R1
        imem[22] = 16'h4111; // CMP/PZ R1         ; 0>=0 -> T=1
        imem[23] = 16'h0929; // MOVT R9           ; R9=1
        imem[24] = 16'h4115; // CMP/PL R1         ; 0>0 -> T=0
        imem[25] = 16'h0A29; // MOVT R10          ; R10=0
        imem[26] = 16'hE1AB; // MOV  #-85,R1      ; 0xFFFFFFAB
        imem[27] = 16'h611C; // EXTU.B R1,R1      ; R1=0x000000AB
        imem[28] = 16'hE2AB; // MOV  #-85,R2      ; 0xFFFFFFAB
        imem[29] = 16'h212C; // CMP/STR R2,R1     ; low byte AB==AB -> T=1
        imem[30] = 16'h0B29; // MOVT R11          ; R11=1
        imem[31] = 16'hE07F; // MOV  #127,R0      ; R0=0x7F
        imem[32] = 16'h887F; // CMP/EQ #127,R0    ; T=1
        imem[33] = 16'h0C29; // MOVT R12          ; R12=1
        imem[34] = 16'h0009; // sentinel
        do_reset;
        run_until(34, 800);
        chk("R3 (CMP/EQ true)", gpr(3), 32'd1);
        chk("R4 (CMP/EQ false)", gpr(4), 32'd0);
        chk("R5 (CMP/HS true)", gpr(5), 32'd1);
        chk("R6 (CMP/HI false)", gpr(6), 32'd0);
        chk("R7 (CMP/GE false)", gpr(7), 32'd0);
        chk("R8 (CMP/GT true)", gpr(8), 32'd1);
        chk("R9 (CMP/PZ true)", gpr(9), 32'd1);
        chk("R10 (CMP/PL false)", gpr(10), 32'd0);
        chk("R11 (CMP/STR true)", gpr(11), 32'd1);
        chk("R12 (CMP/EQ #imm true)", gpr(12), 32'd1);
        chk_true("T set by CMP/EQ #imm", sr[0]);
        end_test;
    end
endtask

//MISC unit: sign/zero extension, byte/word swap, XTRCT, register move, and MOVT.
//These are pure steering ops with no T side effect except MOVT reading T.
task automatic test_misc_ops;
    begin
        begin_test("MISC: EXTU/EXTS, SWAP.B/W, XTRCT, MOV Rm,Rn, MOVT reads SETT");
        imem[0]  = 16'hE1AB; // MOV  #-85,R1      ; R1=0xFFFFFFAB
        imem[1]  = 16'h621C; // EXTU.B R1,R2      ; R2=0x000000AB
        imem[2]  = 16'h631D; // EXTU.W R1,R3      ; R3=0x0000FFAB
        imem[3]  = 16'h641E; // EXTS.B R1,R4      ; R4=0xFFFFFFAB (byte sign)
        imem[4]  = 16'h651F; // EXTS.W R1,R5      ; R5=0xFFFFFFAB (word sign)
        imem[5]  = 16'h6628; // SWAP.B R2,R6      ; R6=0x0000AB00
        imem[6]  = 16'h6739; // SWAP.W R3,R7      ; R7=0xFFAB0000
        imem[7]  = 16'h6973; // MOV  R7,R9        ; R9=0xFFAB0000
        imem[8]  = 16'h296D; // XTRCT R6,R9       ; R9={R6[15:0],R9[31:16]}=0xAB00FFAB
        imem[9]  = 16'h0018; // SETT              ; T=1
        imem[10] = 16'h0A29; // MOVT R10          ; R10=1
        imem[11] = 16'h0009; // sentinel
        do_reset;
        run_until(11, 400);
        chk("R2 (EXTU.B)", gpr(2), 32'h0000_00AB);
        chk("R3 (EXTU.W)", gpr(3), 32'h0000_FFAB);
        chk("R4 (EXTS.B)", gpr(4), 32'hFFFF_FFAB);
        chk("R5 (EXTS.W)", gpr(5), 32'hFFFF_FFAB);
        chk("R6 (SWAP.B)", gpr(6), 32'h0000_AB00);
        chk("R7 (SWAP.W)", gpr(7), 32'hFFAB_0000);
        chk("R9 (XTRCT)", gpr(9), 32'hAB00_FFAB);
        chk("R10 (MOVT of SETT)", gpr(10), 32'd1);
        end_test;
    end
endtask

//One randomized ALU iteration: load a->R1 and b->R2 from data memory, optionally
//seed SR.T, run the op (its register result lands in R1), then capture the
//resulting T with MOVT R3. The caller decides which of result/T to check.
task automatic alu_iter(input logic [15:0] op, input logic [31:0] a, input logic [31:0] b,
                        input logic seed_t, input logic t_in,
                        input logic do_res, input logic [31:0] exp_res,
                        input logic do_t, input logic exp_t, input string nm);
    integer p;
    begin
        imem[0] = 16'hE000; // MOV  #0,R0
        imem[1] = 16'h6106; // MOV.L @R0+,R1   ; R1 = a
        imem[2] = 16'h6206; // MOV.L @R0+,R2   ; R2 = b
        p = 3;
        if(seed_t) begin imem[p] = t_in ? 16'h0018 : 16'h0008; p = p + 1; end // SETT / CLRT
        imem[p] = op;       p = p + 1;
        imem[p] = 16'h0329; p = p + 1; // MOVT R3 ; capture T produced by the op
        imem[p] = 16'h0009;            // sentinel
        do_reset;
        dmem[0] = a;
        dmem[1] = b;
        run_until(p, 400);
        if(do_res) chk({nm, " result"}, gpr(1), exp_res);
        if(do_t)   chk({nm, " T"}, gpr(3), {31'd0, exp_t});
    end
endtask

//ARITH family: ADD/SUB plus the carry/overflow/borrow producers, each against a
//SystemVerilog reference of the exact DUT carry chain.
task automatic test_addsub_rand;
    integer i;
    logic [31:0] a, b, rval, rnd;
    logic [32:0] wide;
    logic t, tt;
    begin
        begin_test("ADDSUB random: ADD/SUB/ADDC/SUBC/ADDV/SUBV/NEG/NEGC/DT vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a + b;
            alu_iter(16'h312C, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("ADD[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a - b;
            alu_iter(16'h3128, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SUB[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rnd = $urandom; t = rnd[0];
            wide = {1'b0, a} + {1'b0, b} + {32'd0, t}; rval = wide[31:0];
            alu_iter(16'h312E, a, b, 1'b1,t, 1'b1,rval, 1'b1,wide[32], $sformatf("ADDC[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rnd = $urandom; t = rnd[0];
            rval = a - b - {31'd0, t}; tt = (a < b) || ((a - b) < {31'd0, t});
            alu_iter(16'h312A, a, b, 1'b1,t, 1'b1,rval, 1'b1,tt, $sformatf("SUBC[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a + b;
            tt = (~(a[31] ^ b[31])) & (a[31] ^ rval[31]);
            alu_iter(16'h312F, a, b, 1'b0,1'b0, 1'b1,rval, 1'b1,tt, $sformatf("ADDV[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a - b;
            tt = (a[31] ^ b[31]) & (a[31] ^ rval[31]);
            alu_iter(16'h312B, a, b, 1'b0,1'b0, 1'b1,rval, 1'b1,tt, $sformatf("SUBV[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = 32'd0 - b; //NEG reads Rm
            alu_iter(16'h612B, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("NEG[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rnd = $urandom; t = rnd[0];
            rval = 32'd0 - b - {31'd0, t}; tt = (b != 32'd0) || t;
            alu_iter(16'h612A, a, b, 1'b1,t, 1'b1,rval, 1'b1,tt, $sformatf("NEGC[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a - 32'd1; tt = (rval == 32'd0);
            alu_iter(16'h4110, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,tt, $sformatf("DT[%0d]", i)); end
        end_test;
    end
endtask

//LOGIC family: bitwise ops plus TST setting T from a zero AND test.
task automatic test_logic_rand;
    integer i;
    logic [31:0] a, b, rval;
    logic tt;
    begin
        begin_test("LOGIC random: AND/OR/XOR/NOT result and TST T vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a & b;
            alu_iter(16'h2129, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("AND[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a | b;
            alu_iter(16'h212B, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("OR[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = a ^ b;
            alu_iter(16'h212A, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("XOR[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = ~b; //NOT reads Rm
            alu_iter(16'h6127, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("NOT[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? (a & 32'h0F0F_0F0F) : $urandom;
            tt = ((a & b) == 32'd0);
            alu_iter(16'h2128, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("TST[%0d]", i)); end
        end_test;
    end
endtask

//SHIFT family: constant shifts, single-bit shifts with the shifted-out T, and
//the dynamic SHAD/SHLD barrel with random signed counts.
task automatic test_shift_rand;
    integer i;
    logic [31:0] a, b, rval;
    logic [4:0]  amt, ramt;
    begin
        begin_test("SHIFT random: SHLL/SHLR/SHAR(+T), SHLLn/SHLRn, SHAD/SHLD vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = {a[30:0], 1'b0};
            alu_iter(16'h4100, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,a[31], $sformatf("SHLL[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = {1'b0, a[31:1]};
            alu_iter(16'h4101, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,a[0], $sformatf("SHLR[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = {a[31], a[31:1]};
            alu_iter(16'h4121, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,a[0], $sformatf("SHAR[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a << 2;
            alu_iter(16'h4108, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLL2[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a << 8;
            alu_iter(16'h4118, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLL8[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a << 16;
            alu_iter(16'h4128, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLL16[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a >> 2;
            alu_iter(16'h4109, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLR2[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a >> 8;
            alu_iter(16'h4119, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLR8[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = a >> 16;
            alu_iter(16'h4129, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLR16[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; amt = b[4:0]; ramt = (~amt) + 5'd1;
            if(!b[31])         rval = a << amt;
            else if(amt == 0)  rval = {32{a[31]}};
            else               rval = $signed(a) >>> ramt;
            alu_iter(16'h412C, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHAD[%0d] cnt=%08h", i, b)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; amt = b[4:0]; ramt = (~amt) + 5'd1;
            if(!b[31])         rval = a << amt;
            else if(amt == 0)  rval = 32'd0;
            else               rval = a >> ramt;
            alu_iter(16'h412D, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SHLD[%0d] cnt=%08h", i, b)); end
        end_test;
    end
endtask

//ROTATE family: ROTL/ROTR plus ROTCL/ROTCR rotating through the T carry.
task automatic test_rotate_rand;
    integer i;
    logic [31:0] a, rval, rnd;
    logic t;
    begin
        begin_test("ROTATE random: ROTL/ROTR(+T), ROTCL/ROTCR through T vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = {a[30:0], a[31]};
            alu_iter(16'h4104, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,a[31], $sformatf("ROTL[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rval = {a[0], a[31:1]};
            alu_iter(16'h4105, a, 32'd0, 1'b0,1'b0, 1'b1,rval, 1'b1,a[0], $sformatf("ROTR[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rnd = $urandom; t = rnd[0]; rval = {a[30:0], t};
            alu_iter(16'h4124, a, 32'd0, 1'b1,t, 1'b1,rval, 1'b1,a[31], $sformatf("ROTCL[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; rnd = $urandom; t = rnd[0]; rval = {t, a[31:1]};
            alu_iter(16'h4125, a, 32'd0, 1'b1,t, 1'b1,rval, 1'b1,a[0], $sformatf("ROTCR[%0d]", i)); end
        end_test;
    end
endtask

//COMPARE family: every CMP form and CMP/STR drive only T; b equals a on half the
//iterations so the equality and string comparisons hit both outcomes.
task automatic test_compare_rand;
    integer i;
    logic [31:0] a, b;
    logic tt;
    begin
        begin_test("COMPARE random: CMP/EQ,HS,GE,HI,GT,PZ,PL,STR T vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom; tt = (a == b);
            alu_iter(16'h3120, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/EQ[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom; tt = (a >= b);
            alu_iter(16'h3122, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/HS[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom; tt = ($signed(a) >= $signed(b));
            alu_iter(16'h3123, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/GE[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom; tt = (a > b);
            alu_iter(16'h3126, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/HI[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom; tt = ($signed(a) > $signed(b));
            alu_iter(16'h3127, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/GT[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; tt = (a[31] == 1'b0);
            alu_iter(16'h4111, a, 32'd0, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/PZ[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; tt = (a[31] == 1'b0) && (a != 32'd0);
            alu_iter(16'h4115, a, 32'd0, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/PL[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = (i[0]) ? a : $urandom;
            tt = (a[31:24] == b[31:24]) || (a[23:16] == b[23:16]) || (a[15:8] == b[15:8]) || (a[7:0] == b[7:0]);
            alu_iter(16'h212C, a, b, 1'b0,1'b0, 1'b0,32'd0, 1'b1,tt, $sformatf("CMP/STR[%0d]", i)); end
        end_test;
    end
endtask

//MISC family: sign/zero extension, byte/word swap, XTRCT, and register move.
task automatic test_misc_rand;
    integer i;
    logic [31:0] a, b, rval;
    begin
        begin_test("MISC random: EXTU/EXTS, SWAP.B/W, XTRCT, MOV vs reference, 100x each");
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {24'd0, b[7:0]}; //ops read Rm=R2
            alu_iter(16'h612C, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("EXTU.B[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {16'd0, b[15:0]};
            alu_iter(16'h612D, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("EXTU.W[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {{24{b[7]}}, b[7:0]};
            alu_iter(16'h612E, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("EXTS.B[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {{16{b[15]}}, b[15:0]};
            alu_iter(16'h612F, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("EXTS.W[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {b[31:16], b[7:0], b[15:8]};
            alu_iter(16'h6128, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SWAP.B[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = {b[15:0], b[31:16]};
            alu_iter(16'h6129, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("SWAP.W[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin a = $urandom; b = $urandom; rval = {b[15:0], a[31:16]};
            alu_iter(16'h212D, a, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("XTRCT[%0d]", i)); end
        for(i = 0; i < 100; i = i + 1) begin b = $urandom; rval = b;
            alu_iter(16'h6123, 32'd0, b, 1'b0,1'b0, 1'b1,rval, 1'b0,1'b0, $sformatf("MOV[%0d]", i)); end
        end_test;
    end
endtask

//One randomized multiply loads both operands and compares MACH:MACL with a reference.
task automatic mac_mul_iter(input logic [15:0] op, input logic [31:0] a, input logic [31:0] b,
                            input logic [31:0] exp_l, input logic [31:0] exp_h, input string nm);
    begin
        imem[0] = 16'hE000; // MOV  #0,R0
        imem[1] = 16'h6106; // MOV.L @R0+,R1   ; R1 = a
        imem[2] = 16'h6206; // MOV.L @R0+,R2   ; R2 = b
        imem[3] = op;       // MUL R2,R1 -> MACL
        imem[4] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = a;
        dmem[1] = b;
        run_until(4, 300);
        chk({nm, " MACL"}, macl_o, exp_l);
        chk({nm, " MACH"}, mach_o, exp_h);
    end
endtask

//One randomized multiply-accumulate: seed MACH:MACL through LDS, place the two
//memory operands, run MAC, and compare the 64-bit accumulator result.
task automatic mac_acc_iter(input logic [15:0] op,
                            input logic [31:0] init_l, input logic [31:0] init_h,
                            input logic [31:0] dx, input logic [31:0] dy,
                            input logic [31:0] exp_l, input logic [31:0] exp_h, input string nm);
    begin
        imem[0] = 16'hE000; // MOV  #0,R0
        imem[1] = 16'h6306; // MOV.L @R0+,R3   ; R3 = init MACL
        imem[2] = 16'h6406; // MOV.L @R0+,R4   ; R4 = init MACH
        imem[3] = 16'h431A; // LDS  R3,MACL
        imem[4] = 16'h440A; // LDS  R4,MACH
        imem[5] = 16'hE108; // MOV  #8,R1      ; first operand pointer
        imem[6] = 16'hE20C; // MOV  #12,R2     ; second operand pointer
        imem[7] = op;       // MAC.L/MAC.W @R1+,@R2+
        imem[8] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = init_l;
        dmem[1] = init_h;
        dmem[2] = dx;
        dmem[3] = dy;
        run_until(8, 400);
        chk({nm, " MACL"}, macl_o, exp_l);
        chk({nm, " MACH"}, mach_o, exp_h);
    end
endtask

//Randomized multiply/MAC coverage. Each multiplier and accumulator runs 100
//iterations against an independent SystemVerilog reference, with fresh random
//operands per iteration. SR.S is zero out of reset, so MAC accumulates the full
//64-bit value (saturation modes are covered separately by test_mac_saturation).
//Seed the run for reproducibility or system-time entropy with the runtime
//plusarg, e.g. +verilator+seed+$(date +%s).
task automatic test_mac;
    integer i;
    logic        [31:0] a, b, init_l, init_h, exp_l, exp_h;
    logic        [63:0] ux, uy, product_u;
    logic signed [31:0] sa, sb;
    logic signed [63:0] sx, sy, acc;
    begin
        begin_test("MUL/MAC random: word, long, double, and accumulate operations, 100x each");

        for(i = 0; i < 100; i = i + 1) begin //MUL.L: low 32 bits of the product
            a = $urandom; b = $urandom;
            exp_l = a * b;
            mac_mul_iter(16'h0127, a, b, exp_l, 32'd0,
                         $sformatf("MUL.L[%0d] a=%08h b=%08h", i, a, b));
        end

        for(i = 0; i < 100; i = i + 1) begin //MULS.W: signed 16x16
            a = $urandom; b = $urandom;
            sa = {{16{a[15]}}, a[15:0]}; sb = {{16{b[15]}}, b[15:0]};
            exp_l = sa * sb;
            mac_mul_iter(16'h212F, a, b, exp_l, 32'd0,
                         $sformatf("MULS.W[%0d] a=%04h b=%04h", i, a[15:0], b[15:0]));
        end

        for(i = 0; i < 100; i = i + 1) begin //MULU.W: unsigned 16x16
            a = $urandom; b = $urandom;
            exp_l = a[15:0] * b[15:0];
            mac_mul_iter(16'h212E, a, b, exp_l, 32'd0,
                         $sformatf("MULU.W[%0d] a=%04h b=%04h", i, a[15:0], b[15:0]));
        end

        for(i = 0; i < 100; i = i + 1) begin //DMULS.L: signed 32x32 to 64 bits
            a = $urandom; b = $urandom;
            sx = {{32{a[31]}}, a}; sy = {{32{b[31]}}, b};
            acc = sx * sy;
            exp_l = acc[31:0]; exp_h = acc[63:32];
            mac_mul_iter(16'h312D, a, b, exp_l, exp_h,
                         $sformatf("DMULS.L[%0d] a=%08h b=%08h", i, a, b));
        end

        for(i = 0; i < 100; i = i + 1) begin //DMULU.L: unsigned 32x32 to 64 bits
            a = $urandom; b = $urandom;
            ux = {32'd0, a}; uy = {32'd0, b};
            product_u = ux * uy;
            exp_l = product_u[31:0]; exp_h = product_u[63:32];
            mac_mul_iter(16'h3125, a, b, exp_l, exp_h,
                         $sformatf("DMULU.L[%0d] a=%08h b=%08h", i, a, b));
        end

        for(i = 0; i < 100; i = i + 1) begin //MAC.L: 64-bit accumulate of signed 32x32
            a = $urandom; b = $urandom; init_l = $urandom; init_h = $urandom;
            sx = {{32{a[31]}}, a}; sy = {{32{b[31]}}, b};
            acc = $signed({init_h, init_l}) + sx * sy;
            exp_l = acc[31:0]; exp_h = acc[63:32];
            mac_acc_iter(16'h021F, init_l, init_h, a, b, exp_l, exp_h,
                         $sformatf("MAC.L[%0d] x=%08h y=%08h", i, a, b));
        end

        for(i = 0; i < 100; i = i + 1) begin //MAC.W: 64-bit accumulate of signed 16x16
            a = $urandom; b = $urandom; init_l = $urandom; init_h = $urandom;
            sx = {{48{a[15]}}, a[15:0]}; sy = {{48{b[15]}}, b[15:0]};
            acc = $signed({init_h, init_l}) + sx * sy;
            exp_l = acc[31:0]; exp_h = acc[63:32];
            //Memory words land in the upper halfword at each aligned longword address.
            mac_acc_iter(16'h421F, init_l, init_h, {a[15:0], 16'h0000}, {b[15:0], 16'h0000},
                         exp_l, exp_h, $sformatf("MAC.W[%0d] x=%04h y=%04h", i, a[15:0], b[15:0]));
        end

        end_test;
    end
endtask

//Fixed and dynamic shifts, including the T side-effect forwarded to MOVT.
task automatic test_shift;
    begin
        begin_test("Shifts: SHLL8, dynamic SHLD +/- count, SHAD, SHLR->T->MOVT forward");
        imem[0]  = 16'hE101; // MOV  #1,R1
        imem[1]  = 16'h4118; // SHLL8 R1          ; R1=0x100  (fixed shift)
        imem[2]  = 16'hE204; // MOV  #4,R2
        imem[3]  = 16'h412D; // SHLD R2,R1        ; R1=0x1000 (dynamic left)
        imem[4]  = 16'hE3FC; // MOV  #-4,R3
        imem[5]  = 16'h413D; // SHLD R3,R1        ; R1=0x100  (dynamic right)
        imem[6]  = 16'hE401; // MOV  #1,R4
        imem[7]  = 16'h4401; // SHLR R4           ; R4=0, T=1
        imem[8]  = 16'h0529; // MOVT R5           ; R5=T=1   (T forwarded from SHLR)
        imem[9]  = 16'hE6FF; // MOV  #-1,R6
        imem[10] = 16'hE702; // MOV  #2,R7
        imem[11] = 16'h467C; // SHAD R7,R6        ; R6 = -1<<2 = 0xFFFFFFFC
        imem[12] = 16'h0009; // sentinel
        do_reset;
        run_until(12, 500);
        chk("R1 (SHLL8/SHLD)", gpr(1), 32'h0000_0100);
        chk("R4 (SHLR)", gpr(4), 32'd0);
        chk("R5 (MOVT of T)", gpr(5), 32'd1);
        chk("R6 (SHAD left)", gpr(6), 32'hFFFF_FFFC);
        end_test;
    end
endtask

//LDC to GBR must serialize before a GBR-relative access reads the new GBR.
task automatic test_state_serialization;
    begin
        begin_test("State serialization: GBR-relative load sees the just-loaded GBR");
        imem[0] = 16'hE140; // MOV  #64,R1       ; 0x40
        imem[1] = 16'hE255; // MOV  #0x55,R2
        imem[2] = 16'h2122; // MOV.L R2,@R1      ; mem[0x40]=0x55  (direct addressing)
        imem[3] = 16'h411E; // LDC  R1,GBR       ; GBR=0x40  (serializes younger decode)
        imem[4] = 16'hC600; // MOV.L @(0,GBR),R0 ; R0=mem[GBR=0x40]=0x55
        imem[5] = 16'h0009; // sentinel
        do_reset;
        run_until(5, 400);
        chk("GBR", gbr_o, 32'h0000_0040);
        chk("R0 (GBR-relative load)", gpr(0), 32'h0000_0055);
        end_test;
    end
endtask

//R0-R7 are banked by SR (MD && RB): privileged+RB=1 selects BANK1, user mode
//selects BANK0. The same logical register must map to different physical words
//in the two modes, so a write in each mode cannot overlap the other.
task automatic test_bank_switch;
    begin
        begin_test("Register banking: privileged R3 (BANK1) and user-mode R3 (BANK0) stay separate");
        imem[0] = 16'hE33C; // MOV  #0x3C,R3   ; privileged: BANK1 R3 = physical 11 = 0x3C
        imem[1] = 16'hE800; // MOV  #0,R8      ; SR source value (R8 is shared, not banked)
        imem[2] = 16'h480E; // LDC  R8,SR      ; SR=0 -> user mode (MD=0); serializes the bank change
        imem[3] = 16'hE35A; // MOV  #0x5A,R3   ; user: BANK0 R3 = physical 3 = 0x5A
        imem[4] = 16'h6433; // MOV  R3,R4      ; user read of R3 -> R4 (BANK0 physical 4); must see 0x5A
        imem[5] = 16'h0009; // NOP             ; sentinel
        do_reset;
        run_until(5, 400);
        chk_true("switched to user mode (MD=0)",     sr[30] === 1'b0);
        chk("BANK1 R3 (privileged) preserved",       u_dut.u_int_pipe.u_gpr_bram.ram[11], 32'h0000_003C);
        chk("BANK0 R3 (user) written",               u_dut.u_int_pipe.u_gpr_bram.ram[3],  32'h0000_005A);
        chk("user read of R3 saw BANK0, not BANK1",  u_dut.u_int_pipe.u_gpr_bram.ram[4],  32'h0000_005A);
        chk_true("banks hold distinct values",       u_dut.u_int_pipe.u_gpr_bram.ram[11] !== u_dut.u_int_pipe.u_gpr_bram.ram[3]);
        end_test;
    end
endtask

//DIV0S/DIV0U initialize M/Q/T; randomized DIV1 steps follow the manual model.
task automatic test_division;
    integer         i;
    logic   [31:0]  rn, rm, sr_seed, exp_rn, exp_sr;
    begin
        begin_test("Division: DIV0S/DIV0U state and randomized DIV1 quotient steps");

        imem[0] = 16'hE1FE; // MOV  #-2,R1
        imem[1] = 16'hE203; // MOV  #3,R2
        imem[2] = 16'h2127; // DIV0S R2,R1       ; M=0, Q=1, T=1
        imem[3] = 16'h0302; // STC   SR,R3
        imem[4] = 16'h0019; // DIV0U             ; clear M/Q/T
        imem[5] = 16'h0402; // STC   SR,R4
        imem[6] = 16'h0009; // sentinel
        do_reset;
        run_until(6, 500);
        chk("DIV0S M/Q/T", gpr(3) & 32'h0000_0301, 32'h0000_0101);
        chk("DIV0U M/Q/T", gpr(4) & 32'h0000_0301, 32'd0);

        for(i = 0; i < 64; i = i + 1) begin
            clear_imem;
            rn      = $urandom;
            rm      = $urandom;
            sr_seed = 32'h6000_0000 | (($urandom & 3) << 8) | ($urandom & 1);
            div1_reference(rn, rm, sr_seed, exp_rn, exp_sr);

            imem[0] = 16'hE800; // MOV   #0,R8
            imem[1] = 16'h6182; // MOV.L @R8,R1
            imem[2] = 16'hE804; // MOV   #4,R8
            imem[3] = 16'h6282; // MOV.L @R8,R2
            imem[4] = 16'hE808; // MOV   #8,R8
            imem[5] = 16'h6382; // MOV.L @R8,R3
            imem[6] = 16'h430E; // LDC   R3,SR
            imem[7] = 16'h3124; // DIV1  R2,R1
            imem[8] = 16'h0402; // STC   SR,R4
            imem[9] = 16'h0009; // sentinel
            do_reset;
            dmem[0] = rn;
            dmem[1] = rm;
            dmem[2] = sr_seed;
            run_until(9, 700);
            if((gpr(4) & 32'h0000_0301) !== (exp_sr & 32'h0000_0301)) begin
                $display("      DIV1 state rn=%08h rm=%08h sr=%08h", rn, rm, sr_seed);
            end
            chk($sformatf("DIV1[%0d] Rn", i), gpr(1), exp_rn);
            chk($sformatf("DIV1[%0d] M/Q/T", i),
                gpr(4) & 32'h0000_0301, exp_sr & 32'h0000_0301);
        end
        end_test;
    end
endtask

//Each selector is exercised through direct, post-increment, and pre-decrement forms.
task automatic test_control_transfers;
    integer         selector;
    logic   [31:0]  value, expected;
    begin
        begin_test("Control transfers: STC/STC.L and LDC/LDC.L cover SR through SPC");
        for(selector = 0; selector < 5; selector = selector + 1) begin
            value    = selector == 0 ? 32'h6000_0301 : 32'h1234_5000 + selector;
            expected = control_mask(selector, value);

            clear_imem;
            imem[0] = 16'hE800;                         // MOV   #0,R8
            imem[1] = 16'h6A82;                         // MOV.L @R8,R10
            imem[2] = {4'h4, 4'hA, selector[3:0], 4'hE};// LDC   R10,<control>
            imem[3] = {4'h0, 4'h9, selector[3:0], 4'h2};// STC   <control>,R9
            imem[4] = 16'h0009;                         // sentinel
            do_reset;
            dmem[0] = value;
            run_until(4, 700);
            chk($sformatf("direct control selector %0d", selector), gpr(9), expected);
            case(selector)
                0: chk("SR output",  sr,    expected);
                1: chk("GBR output", gbr_o, expected);
                2: chk("VBR output", vbr_o, expected);
                3: chk("SSR output", ssr_o, expected);
                4: chk("SPC output", spc_o, expected);
                default: begin end
            endcase

            clear_imem;
            imem[0] = 16'hE800;                         // MOV   #0,R8
            imem[1] = {4'h4, 4'h8, selector[3:0], 4'h7};// LDC.L @R8+,<control>
            imem[2] = {4'h0, 4'h9, selector[3:0], 4'h2};// STC   <control>,R9
            imem[3] = 16'h0009;                         // sentinel
            do_reset;
            dmem[0] = value;
            run_until(3, 700);
            chk($sformatf("LDC.L control selector %0d", selector), gpr(9), expected);
            chk($sformatf("LDC.L selector %0d postincrement", selector), gpr(8), 32'd4);

            clear_imem;
            imem[0] = 16'hE800;                         // MOV   #0,R8
            imem[1] = 16'h6A82;                         // MOV.L @R8,R10
            imem[2] = {4'h4, 4'hA, selector[3:0], 4'hE};// LDC   R10,<control>
            imem[3] = 16'hE908;                         // MOV   #8,R9
            imem[4] = {4'h4, 4'h9, selector[3:0], 4'h3};// STC.L <control>,@-R9
            imem[5] = 16'h0009;                         // sentinel
            do_reset;
            dmem[0] = value;
            run_until(5, 700);
            chk($sformatf("STC.L control selector %0d", selector), dmem[1], expected);
            chk($sformatf("STC.L selector %0d predecrement", selector), gpr(9), 32'd4);
        end

        end_test;
    end
endtask

//Bank transfers address the bank opposite SR.RB while shared R8-R15 remain unchanged.
task automatic test_bank_control_transfers;
    begin
        begin_test("Bank transfers: direct and memory forms access the inactive register bank");
        imem[0] = 16'hE85A; // MOV   #0x5A,R8
        imem[1] = 16'h48BE; // LDC   R8,R3_BANK
        imem[2] = 16'h09B2; // STC   R3_BANK,R9
        imem[3] = 16'hEA00; // MOV   #0,R10
        imem[4] = 16'h4AC7; // LDC.L @R10+,R4_BANK
        imem[5] = 16'h0BC2; // STC   R4_BANK,R11
        imem[6] = 16'hEC08; // MOV   #8,R12
        imem[7] = 16'h4CC3; // STC.L R4_BANK,@-R12
        imem[8] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'h89AB_CDEF;
        run_until(8, 900);
        chk("inactive BANK0 R3", u_dut.u_int_pipe.u_gpr_bram.ram[3], 32'h0000_005A);
        chk("STC R3_BANK", gpr(9), 32'h0000_005A);
        chk("inactive BANK0 R4", u_dut.u_int_pipe.u_gpr_bram.ram[4], 32'h89AB_CDEF);
        chk("STC R4_BANK", gpr(11), 32'h89AB_CDEF);
        chk("LDC.L bank postincrement", gpr(10), 32'd4);
        chk("STC.L bank predecrement", gpr(12), 32'd4);
        chk("STC.L bank memory", dmem[1], 32'h89AB_CDEF);
        end_test;
    end
endtask

//GBR-indexed byte logic uses one read and optional locked write; TAS always sets bit seven.
task automatic test_byte_rmw;
    begin
        begin_test("Byte RMW: TST/AND/XOR/OR and TAS data, T forwarding, and lock phases");
        imem[0]  = 16'hE100; // MOV   #0,R1
        imem[1]  = 16'h411E; // LDC   R1,GBR
        imem[2]  = 16'hE000; // MOV   #0,R0
        imem[3]  = 16'hCCA5; // TST.B #0xA5,@(R0,GBR) ; T=0
        imem[4]  = 16'h0229; // MOVT  R2
        imem[5]  = 16'hE001; // MOV   #1,R0
        imem[6]  = 16'hCD0F; // AND.B #0x0F,@(R0,GBR)
        imem[7]  = 16'hE002; // MOV   #2,R0
        imem[8]  = 16'hCE0F; // XOR.B #0x0F,@(R0,GBR)
        imem[9]  = 16'hE003; // MOV   #3,R0
        imem[10] = 16'hCF80; // OR.B  #0x80,@(R0,GBR)
        imem[11] = 16'hE304; // MOV   #4,R3
        imem[12] = 16'h431B; // TAS.B @R3             ; zero byte becomes 0x80, T=1
        imem[13] = 16'h0429; // MOVT  R4
        imem[14] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'hA5F3_5501;
        dmem[1] = 32'h0000_0000;
        run_until(14, 1200);
        chk("TST.B forwarded T", gpr(2), 32'd0);
        chk("AND/XOR/OR bytes", dmem[0], 32'hA503_5A81);
        chk("TAS.B byte", dmem[1], 32'h8000_0000);
        chk("TAS.B forwarded T", gpr(4), 32'd1);
        chk("locked read phases", locked_read_count[31:0], 32'd4);
        chk("locked write phases", locked_write_count[31:0], 32'd4);
        chk("byte operation requests", data_request_count[31:0], 32'd9);
        end_test;
    end
endtask

//With the cache disabled (CCR.CE=0, the reset default) PREF is a no-op: it issues
//no external access and, addressing a line, never raises an address error.
task automatic test_pref_no_cache;
    begin
        begin_test("PREF: cache-disabled prefetch retires without a data access");
        imem[0] = 16'hE101; // MOV  #1,R1        ; deliberately unaligned address
        imem[1] = 16'h0183; // PREF @R1          ; line allocate; no-op while cache off
        imem[2] = 16'h0009; // sentinel
        do_reset;
        run_until(2, 400);
        chk_true("PREF retired", retired_seen[1]);
        chk("PREF data requests", data_request_count[31:0], 32'd0);
        chk_true("PREF did not fault", !exc_seen);
        end_test;
    end
endtask

//Illegal opcode: precise EXC_ILLEGAL, no commit of the faulting or younger word.
task automatic test_exc_illegal;
    begin
        begin_test("Exception: illegal opcode -> EXC_ILLEGAL, younger work killed");
        imem[0] = 16'hE101; // MOV  #1,R1        ; retires
        imem[1] = 16'hF000; // .word 0xF000      ; illegal encoding
        imem[2] = 16'hE202; // MOV  #2,R2        ; younger (must not retire)
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_ILLEGAL});
        chk("exc pc", exc_pc_l, 32'hA000_0002);
        chk_true("illegal word not retired", !retired_seen[1]);
        chk_true("younger word killed", !retired_seen[2]);
        chk("R1 (older committed)", gpr(1), 32'd1);
        end_test;
    end
endtask

//Misaligned longword load: EXC_ADDRESS raised in EX before any data request.
task automatic test_exc_address;
    begin
        begin_test("Exception: misaligned longword load -> EXC_ADDRESS, no data request");
        imem[0] = 16'hE001; // MOV  #1,R0        ; misaligned base for a longword
        imem[1] = 16'h6702; // MOV.L @R0,R7      ; address error
        imem[2] = 16'hE302; // MOV  #2,R3        ; younger (must not retire)
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_ADDRESS});
        chk("exc pc", exc_pc_l, 32'hA000_0002);
        chk("no data request issued", data_request_count[31:0], 32'd0);
        chk_true("faulting load not retired", !retired_seen[1]);
        end_test;
    end
endtask

//Instruction-port fault travels with its PC to a precise EXC_IFETCH. The faulting
//instruction sits on a LONGWORD boundary: a bus fault is transaction-granular, and
//the bypass reuse buffer serves an odd halfword from its (fault-free) sibling read,
//so a fault parked on an odd index would never be requested on the bus.
task automatic test_exc_ifetch;
    begin
        begin_test("Exception: instruction fetch fault -> EXC_IFETCH at the faulting PC");
        imem[0] = 16'hE101; // MOV  #1,R1        ; retires
        imem[1] = 16'hE202; // MOV  #2,R2        ; retires (buffer-served sibling)
        imem[2] = 16'hE303; // (fetch returns fault for this word)
        imem[3] = 16'hE404; // MOV  #4,R4        ; younger (must not retire)
        if_fault_en   = 1'b1;
        if_fault_widx = 11'd2;
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_IFETCH});
        chk("exc pc", exc_pc_l, 32'hA000_0004);
        chk_true("faulting fetch not retired", !retired_seen[2]);
        chk_true("younger killed", !retired_seen[3]);
        chk("R1 (older committed)", gpr(1), 32'd1);
        chk("R2 (older committed)", gpr(2), 32'd2);
        end_test;
    end
endtask

//Data abort on a load: EXC_DATA, and the destination register is not written.
task automatic test_exc_data;
    begin
        begin_test("Exception: data abort on load -> EXC_DATA, destination unchanged");
        imem[0] = 16'hE000; // MOV  #0,R0        ; load address 0
        imem[1] = 16'hE712; // MOV  #0x12,R7     ; R7=0x12 (commits before the fault)
        imem[2] = 16'h6702; // MOV.L @R0,R7      ; data abort -> must not overwrite R7
        imem[3] = 16'hE303; // MOV  #3,R3        ; younger (must not retire)
        d_fault_en   = 1'b1;
        d_fault_widx = 8'd0;
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_DATA});
        chk("exc pc", exc_pc_l, 32'hA000_0004);
        chk_true("faulting load not retired", !retired_seen[2]);
        chk("R7 (write suppressed)", gpr(7), 32'h0000_0012);
        end_test;
    end
endtask

//Privileged instruction in user mode: EXC_PRIVILEGE after SR drops MD.
task automatic test_exc_privilege;
    begin
        begin_test("Exception: privileged op in user mode -> EXC_PRIVILEGE");
        imem[0] = 16'hE100; // MOV  #0,R1        ; SR value with MD=0
        imem[1] = 16'h410E; // LDC  R1,SR        ; enter user mode (serializes)
        imem[2] = 16'h0038; // LDTLB             ; privileged -> faults in user mode
        imem[3] = 16'hE505; // MOV  #5,R5        ; younger (must not retire)
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_PRIVILEGE});
        chk("exc pc", exc_pc_l, 32'hA000_0004);
        chk_true("privileged op not retired", !retired_seen[2]);

        clear_imem;
        imem[0] = 16'hE100; // MOV  #0,R1        ; SR value with MD=0
        imem[1] = 16'h410E; // LDC  R1,SR        ; enter user mode
        imem[2] = 16'h0212; // STC  GBR,R2       ; GBR transfer remains user-accessible
        imem[3] = 16'h0322; // STC  VBR,R3       ; privileged -> faults
        imem[4] = 16'h0009;
        do_reset;
        run_until_exc(500);
        chk_true("user STC GBR retired", retired_seen[2]);
        chk("STC VBR privilege cause", {29'd0, exc_cause_l}, {29'd0, EXC_PRIVILEGE});
        chk("STC VBR privilege pc", exc_pc_l, 32'hA000_0006);
        chk_true("user STC VBR not retired", !retired_seen[3]);
        end_test;
    end
endtask

//Serialized CPU-state events surface as one-cycle output pulses.
task automatic test_events;
    begin
        begin_test("Events: TRAPA / SLEEP / LDTLB raise their commit-point pulses");
        imem[0] = 16'hE800; // MOV  #0,R8
        imem[1] = 16'h4807; // LDC.L @R8+,SR     ; privileged, BL=0
        imem[2] = 16'hC342; // TRAPA #0x42       ; enters exception handler
        do_reset;
        dmem[0] = 32'h6000_0000;
        run_until_exc(400);
        chk_true("TRAPA pulsed", trapa_seen);
        chk("TRAPA imm", {24'd0, trapa_imm_l}, 32'h0000_0042);
        chk("TRA register", tra_o, 32'h0000_0108);
        chk("EXPEVT TRAPA", expevt_o, 32'h0000_0160);

        clear_imem;
        imem[0] = 16'h001B; // SLEEP             ; o_SLEEP_VALID (privileged)
        imem[1] = 16'h0038; // LDTLB             ; o_LDTLB_VALID (privileged)
        imem[2] = 16'h0009; // sentinel
        do_reset;
        run_until(2, 400);
        chk_true("SLEEP pulsed", sleep_seen);
        chk_true("LDTLB pulsed", ldtlb_seen);
        end_test;
    end
endtask

//RTE is a delayed branch with a serialized restore; the delay slot still runs.
//Its event_rte serialization must not block its own delay slot from issuing.
task automatic test_rte;
    begin
        begin_test("RTE: delayed restore pulses dbg_o_RTE_VALID, delay slot commits, past-slot killed");
        imem[0] = 16'hE111; // MOV  #0x11,R1
        imem[1] = 16'h002B; // RTE               ; delayed branch to SPC, dbg_o_RTE_VALID
        imem[2] = 16'hE222; // MOV  #0x22,R2     ; delay slot (must retire)
        imem[3] = 16'hE333; // MOV  #0x33,R3     ; past slot (must not retire)
        do_reset;
        run_cycles(120);
        chk_true("RTE pulsed", rte_seen);
        chk_true("delay slot retired", retired_seen[2]);
        chk("R2 (delay slot)", gpr(2), 32'h0000_0022);
        chk_true("past-slot word never retired", !retired_seen[3]);
        end_test;
    end
endtask

//A load whose base register was committed (not forwarded) must keep that operand
//even when the load stalls in EX behind a prior store occupying MA.
task automatic test_stalled_ex_operand;
    begin
        begin_test("Stalled-EX operand: load base survives a stall behind a store in MA");
        imem[0] = 16'hE340; // MOV  #64,R3       ; R3=0x40 (commits to BRAM, later read unforwarded)
        imem[1] = 16'hE47A; // MOV  #0x7A,R4
        imem[2] = 16'h2342; // MOV.L R4,@R3      ; store occupies MA
        imem[3] = 16'h6432; // MOV.L @R3,R4      ; load stalls behind the store; base R3 must hold 0x40
        imem[4] = 16'h0009; // sentinel
        do_reset;
        run_until(4, 400);
        chk("R4 (reload of mem[0x40])", gpr(4), 32'h0000_007A);
        chk("dmem[0x10]", dmem[16], 32'h0000_007A);
        end_test;
    end
endtask

//Multi-cycle data latency must freeze younger stages until the response lands;
//architectural results must stay identical across a sweep of random latencies.
task automatic test_ma_wait;
    integer phase;
    begin
        begin_test("MA wait: random data latency holds the pipeline; results stay latency-invariant");
        imem[0] = 16'hE000; // MOV  #0,R0        ; address 0
        imem[1] = 16'hE15A; // MOV  #0x5A,R1     ; value
        imem[2] = 16'h2012; // MOV.L R1,@R0      ; mem[0]=0x5A  (store waits 1+d_latency cycles)
        imem[3] = 16'h6202; // MOV.L @R0,R2      ; R2=mem[0]=0x5A  (load ordered after the store)
        imem[4] = 16'h322C; // ADD  R2,R2        ; R2=0xB4  load-use held across the whole wait
        imem[5] = 16'hE321; // MOV  #0x21,R3     ; younger independent op
        imem[6] = 16'h0009; // NOP               ; sentinel
        for(phase = 0; phase < 8; phase = phase + 1) begin
            d_latency = $urandom_range(5, 50); // random external-memory wait, 5..50 cycles
            do_reset;
            run_until(6, 1500);
            chk($sformatf("R2 load-use, latency %0d", d_latency), gpr(2), 32'h0000_00B4);
            chk("R3 (younger op)", gpr(3), 32'h0000_0021);
            chk("dmem[0] (store landed)", dmem[0], 32'h0000_005A);
            chk_true("load retired", retired_seen[3]);
            chk_true("younger op retired", retired_seen[5]);
        end
        d_latency = 0;
        end_test;
    end
endtask

//After a precise fault, cpu_core redirects through the bare-metal handler path.
task automatic test_redirect_recovery;
    begin
        begin_test("Redirect recovery: cpu_core enters VBR+0x100 after an exception");
        imem[0]   = 16'hE800; // MOV  #0,R8       ; SR load pointer
        imem[1]   = 16'h4807; // LDC.L @R8+,SR    ; privileged, BL=0
        imem[2]   = 16'hE001; // MOV  #1,R0       ; misaligned base
        imem[3]   = 16'h6702; // MOV.L @R0,R7     ; address error
        imem[4]   = 16'hE3AA; // MOV  #-86,R3     ; killed by the fault
        imem[128] = 16'hE85A; // MOV  #0x5A,R8    ; VBR+0x100 handler body
        imem[129] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'h6000_0000;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause", {29'd0, exc_cause_l}, {29'd0, EXC_ADDRESS});
        chk_true("exception entry pulsed", exception_entry_seen);
        chk("exception entry PC", exception_entry_pc_l, 32'h0000_0100);
        //Architectural entry state: handler translates the cause to EXPEVT, latches
        //the faulting address into TEA, and ctrl_reg saves SR->SSR, restart PC->SPC.
        chk("EXPEVT = address-read code", expevt_o, 32'h0000_00E0);
        chk("TEA = faulting access address", tea_o, 32'h0000_0001);
        chk("SSR = SR captured at entry (BL=0)", ssr_o, 32'h6000_0000);
        chk("SPC = restart PC (faulting longword)", spc_o, 32'hA000_0006);

        run_until_retire(129, 400);
        chk_true("handler word retired", retired_seen[128]);
        chk("R8 (after redirect)", gpr(8), 32'h0000_005A);
        end_test;
    end
endtask


//Full exception round trip through ctrl_reg's arbiter: entry saves SR->SSR and
//restart PC->SPC, then RTE restores SR<-SSR and branches PC<-SPC.  TRAPA is used
//so the restart PC is the instruction after TRAPA, avoiding a re-fault on return.
task automatic test_exc_rte_roundtrip;
    begin
        begin_test("RTE round trip: entry saves SSR/SPC, RTE restores SR and returns to SPC");
        imem[0]   = 16'hE800; // MOV   #0,R8        ; SR load pointer
        imem[1]   = 16'h4807; // LDC.L @R8+,SR      ; SR<-0x60000000 (MD=1,RB=1,BL=0)
        imem[2]   = 16'hC342; // TRAPA #0x42        ; general exception, SPC=PC+2
        imem[3]   = 16'hEB4B; // MOV   #0x4B,R11    ; RTE return target (SPC)
        imem[4]   = 16'h0009; // sentinel
        imem[128] = 16'hE85A; // MOV   #0x5A,R8     ; VBR+0x100 handler body
        imem[129] = 16'h002B; // RTE                ; delayed restore + branch to SPC
        imem[130] = 16'hEA6B; // MOV   #0x6B,R10    ; RTE delay slot (must retire)
        imem[131] = 16'hE777; // MOV   #0x77,R7     ; past-slot (must not retire)
        do_reset;
        dmem[0] = 32'h6000_0000;

        //Stop once the handler body has committed: entry is complete, RTE not yet.
        run_until(128, 400);
        chk_true("handler entered", retired_seen[128]);
        chk("SSR = SR captured at entry", ssr_o, 32'h6000_0000);
        chk("SPC = instruction after TRAPA", spc_o, 32'hA000_0006);
        chk("EXPEVT = TRAPA", expevt_o, 32'h0000_0160);
        chk("TRA = imm<<2", tra_o, 32'h0000_0108);

        //Run to the post-return sentinel: RTE must restore SR and resume at SPC.
        run_until(4, 400);
        chk_true("RTE pulsed", rte_seen);
        chk_true("delay slot retired", retired_seen[130]);
        chk("R10 (delay slot)", gpr(10), 32'h0000_006B);
        chk_true("past-slot word killed", !retired_seen[131]);
        chk_true("returned to SPC target", retired_seen[3]);
        chk("R11 (return target)", gpr(11), 32'h0000_004B);
        chk("SR restored from SSR", sr, 32'h6000_0000);
        chk("SPC unchanged by RTE", spc_o, 32'hA000_0006);
        end_test;
    end
endtask


//Byte and word MOV sizes (with load sign-extension) and longword displacement.
task automatic test_mov_sizes;
    begin
        begin_test("MOV sizes: byte/word load sign-extension, store lanes, @(disp,Rn) longword");
        imem[0]  = 16'hE000; // MOV  #0,R0
        imem[1]  = 16'hE17F; // MOV  #0x7F,R1
        imem[2]  = 16'h2010; // MOV.B R1,@R0      ; dmem[0][31:24]=0x7F
        imem[3]  = 16'h6200; // MOV.B @R0,R2      ; R2=sign_ext(0x7F)=0x0000007F
        imem[4]  = 16'hE001; // MOV  #1,R0
        imem[5]  = 16'hE3F0; // MOV  #0xF0,R3     ; R3=0xFFFFFFF0
        imem[6]  = 16'h2030; // MOV.B R3,@R0      ; dmem[0][23:16]=0xF0
        imem[7]  = 16'h6400; // MOV.B @R0,R4      ; R4=sign_ext(0xF0)=0xFFFFFFF0
        imem[8]  = 16'hE004; // MOV  #4,R0
        imem[9]  = 16'hE555; // MOV  #0x55,R5
        imem[10] = 16'h2051; // MOV.W R5,@R0      ; dmem[1][31:16]=0x0055
        imem[11] = 16'h6601; // MOV.W @R0,R6      ; R6=sign_ext(0x0055)=0x55
        imem[12] = 16'hE7AB; // MOV  #0xAB,R7     ; R7=0xFFFFFFAB
        imem[13] = 16'hE008; // MOV  #8,R0
        imem[14] = 16'h1072; // MOV.L R7,@(2,R0)  ; addr=R0+2*4=16 -> dmem[4]
        imem[15] = 16'h5802; // MOV.L @(2,R0),R8  ; R8=dmem[4]=0xFFFFFFAB
        imem[16] = 16'h0009; // sentinel
        do_reset;
        run_until(16, 700);
        chk("MOV.B store/load positive", gpr(2), 32'h0000_007F);
        chk("MOV.B load sign-extension", gpr(4), 32'hFFFF_FFF0);
        chk("MOV.W store/load", gpr(6), 32'h0000_0055);
        chk("MOV.L @(disp,Rn) roundtrip", gpr(8), 32'hFFFF_FFAB);
        chk("byte store lanes", dmem[0], 32'h7FF0_0000);
        chk("word store lane", dmem[1], 32'h0055_0000);
        chk("disp longword store", dmem[4], 32'hFFFF_FFAB);
        end_test;
    end
endtask

//PC-relative loads and MOVA compute addresses from the aligned architectural PC.
task automatic test_pc_relative;
    begin
        begin_test("PC-relative: MOVA, MOV.L @(disp,PC), MOV.W @(disp,PC) sign-extension");
        imem[0] = 16'hC703; // MOVA @(3,PC),R0   ; R0=(PC&~3)+4+3*4 = 0xA0000010
        imem[1] = 16'h0009; // NOP
        imem[2] = 16'hD101; // MOV.L @(1,PC),R1  ; addr=(PC&~3)+4+1*4 = 0xA000000C -> dmem[3]
        imem[3] = 16'h0009; // NOP
        imem[4] = 16'h9302; // MOV.W @(2,PC),R3  ; addr=PC+4+2*2 = 0xA0000010 -> dmem[4] upper word
        imem[5] = 16'h0009; // sentinel
        do_reset;
        dmem[3] = 32'hDEAD_BEEF;
        dmem[4] = 32'h1234_0000;
        run_until(5, 400);
        chk("MOVA address", gpr(0), 32'hA000_0010);
        chk("MOV.L @(disp,PC)", gpr(1), 32'hDEAD_BEEF);
        chk("MOV.W @(disp,PC) sign-ext", gpr(3), 32'h0000_1234);
        end_test;
    end
endtask

//Remaining branch forms: unconditional/far/subroutine and the delayed conditional.
task automatic test_branch_extra;
    begin
        begin_test("Branch forms: BRA, BF/S, BRAF, BSRF, JMP, JSR with delay slots and links");

        //BRA: unconditional delayed branch over a skipped instruction.
        clear_imem;
        imem[0] = 16'hA002; // BRA  -> word 4
        imem[1] = 16'hE111; // MOV  #0x11,R1     ; delay slot
        imem[2] = 16'hE522; // MOV  #0x22,R5     ; skipped
        imem[3] = 16'h0009; // NOP
        imem[4] = 16'hE233; // MOV  #0x33,R2     ; target
        imem[5] = 16'h0009; // sentinel
        do_reset; run_until(5, 400);
        chk("BRA delay slot", gpr(1), 32'h0000_0011);
        chk("BRA target", gpr(2), 32'h0000_0033);
        chk_true("BRA skipped not retired", !retired_seen[2]);

        //BF/S: delayed conditional, taken because T is clear.
        clear_imem;
        imem[0] = 16'h0008; // CLRT             ; T=0
        imem[1] = 16'h8F02; // BF/S -> word 5
        imem[2] = 16'hE144; // MOV  #0x44,R1    ; delay slot
        imem[3] = 16'hE555; // MOV  #0x55,R5    ; skipped
        imem[4] = 16'h0009; // NOP
        imem[5] = 16'hE266; // MOV  #0x66,R2    ; target
        imem[6] = 16'h0009; // sentinel
        do_reset; run_until(6, 400);
        chk("BF/S delay slot", gpr(1), 32'h0000_0044);
        chk("BF/S target", gpr(2), 32'h0000_0066);
        chk_true("BF/S skipped not retired", !retired_seen[3]);

        //BRAF: branch to PC + Rm.
        clear_imem;
        imem[0] = 16'hE304; // MOV  #4,R3       ; displacement
        imem[1] = 16'h0323; // BRAF R3          ; target = PC+4+R3 = word 5
        imem[2] = 16'hE177; // MOV  #0x77,R1    ; delay slot
        imem[3] = 16'hE555; // MOV  #0x55,R5    ; skipped
        imem[4] = 16'h0009; // NOP
        imem[5] = 16'hE27A; // MOV  #0x7A,R2    ; target
        imem[6] = 16'h0009; // sentinel
        do_reset; run_until(6, 400);
        chk("BRAF delay slot", gpr(1), 32'h0000_0077);
        chk("BRAF target", gpr(2), 32'h0000_007A);

        //JMP: absolute jump through a register loaded from memory.
        clear_imem;
        imem[0] = 16'hE800; // MOV  #0,R8
        imem[1] = 16'h6382; // MOV.L @R8,R3     ; R3 = target address
        imem[2] = 16'h432B; // JMP  @R3
        imem[3] = 16'hE15B; // MOV  #0x5B,R1    ; delay slot
        imem[4] = 16'hE555; // MOV  #0x55,R5    ; skipped
        imem[5] = 16'h0009; // NOP
        imem[6] = 16'hE26C; // MOV  #0x6C,R2    ; target (word 6)
        imem[7] = 16'h0009; // sentinel
        do_reset; dmem[0] = 32'hA000_000C; run_until(7, 400);
        chk("JMP delay slot", gpr(1), 32'h0000_005B);
        chk("JMP target", gpr(2), 32'h0000_006C);

        //JSR + RTS: subroutine call links PR, returns past the delay slot.
        clear_imem;
        imem[0] = 16'hE800; // MOV  #0,R8
        imem[1] = 16'h6382; // MOV.L @R8,R3     ; R3 = subroutine address
        imem[2] = 16'h430B; // JSR  @R3         ; PR = word 4
        imem[3] = 16'hE17D; // MOV  #0x7D,R1    ; call delay slot
        imem[4] = 16'hE63E; // MOV  #0x3E,R6    ; return point
        imem[5] = 16'h0009; // sentinel
        imem[6] = 16'hE24F; // MOV  #0x4F,R2    ; subroutine body (word 6)
        imem[7] = 16'h000B; // RTS
        imem[8] = 16'hE751; // MOV  #0x51,R7    ; RTS delay slot
        do_reset; dmem[0] = 32'hA000_000C; run_until(4, 500);
        chk("JSR delay slot", gpr(1), 32'h0000_007D);
        chk("JSR subroutine", gpr(2), 32'h0000_004F);
        chk("JSR RTS delay slot", gpr(7), 32'h0000_0051);
        chk("JSR return point", gpr(6), 32'h0000_003E);

        //BSRF: far subroutine call links PR and targets PC + Rm.
        clear_imem;
        imem[0] = 16'hE304; // MOV  #4,R3       ; displacement
        imem[1] = 16'h0303; // BSRF R3          ; PR = word 3, target = word 5
        imem[2] = 16'hE15D; // MOV  #0x5D,R1    ; call delay slot
        imem[3] = 16'hE66E; // MOV  #0x6E,R6    ; return point
        imem[4] = 16'h0009; // sentinel
        imem[5] = 16'hE270; // MOV  #0x70,R2    ; subroutine body (word 5)
        imem[6] = 16'h000B; // RTS
        imem[7] = 16'hE771; // MOV  #0x71,R7    ; RTS delay slot
        do_reset; run_until(3, 500);
        chk("BSRF delay slot", gpr(1), 32'h0000_005D);
        chk("BSRF subroutine", gpr(2), 32'h0000_0070);
        chk("BSRF RTS delay slot", gpr(7), 32'h0000_0071);
        chk("BSRF return point", gpr(6), 32'h0000_006E);

        end_test;
    end
endtask

//Remaining system-register paths: PR load/store and the S-bit set/clear.
task automatic test_sys_misc;
    begin
        begin_test("System misc: LDS/STS PR round-trip, CLRS/SETS observed through STC SR");
        imem[0] = 16'hE15A; // MOV  #0x5A,R1
        imem[1] = 16'h412A; // LDS  R1,PR        ; PR=0x5A (serializes younger decode)
        imem[2] = 16'h022A; // STS  PR,R2        ; R2=PR=0x5A
        imem[3] = 16'h0058; // SETS              ; S=1
        imem[4] = 16'h0302; // STC  SR,R3
        imem[5] = 16'h0048; // CLRS              ; S=0
        imem[6] = 16'h0402; // STC  SR,R4
        imem[7] = 16'h0009; // sentinel
        do_reset;
        run_until(7, 500);
        chk("LDS/STS PR round-trip", gpr(2), 32'h0000_005A);
        chk("PR output register", pr_o, 32'h0000_005A);
        chk("SETS sets SR.S", gpr(3) & 32'h0000_0002, 32'h0000_0002);
        chk("CLRS clears SR.S", gpr(4) & 32'h0000_0002, 32'd0);
        end_test;
    end
endtask


//Sign-extend a loaded value for the size index (0=byte, 1=word, 2=longword).
function automatic logic [31:0] sext(input logic [31:0] v, input integer sz);
    case(sz)
        0:       sext = {{24{v[7]}},  v[7:0]};
        1:       sext = {{16{v[15]}}, v[15:0]};
        default: sext = v;
    endcase
endfunction

//Roundtrip helper for modes whose data register is R5 (loaded from dmem[8]) and
//whose load destination is R6. Stores then loads the same effective address.
task automatic mem_rt(input logic [15:0] s_op, input logic [15:0] l_op,
                      input logic [7:0] base, input logic [7:0] idx,
                      input integer sz, input logic [31:0] v, input string nm);
    begin
        clear_imem;
        imem[0] = 16'hE720;                      // MOV  #0x20,R7
        imem[1] = 16'h6572;                      // MOV.L @R7,R5     ; R5 = value
        imem[2] = 16'(16'hE400 + {8'd0, base});  // MOV  #base,R4
        imem[3] = 16'(16'hE000 + {8'd0, idx});   // MOV  #idx,R0
        imem[4] = s_op;                          // store R5
        imem[5] = l_op;                          // load into R6
        imem[6] = 16'h0009;                      // sentinel
        do_reset;
        dmem[8] = v;
        run_until(6, 400);
        chk(nm, gpr(6), sext(v, sz));
    end
endtask

//Roundtrip helper for the R0-data modes (GBR displacement and byte/word @(disp,Rn)).
//R0 holds the store data; the load destination is also R0 after a deliberate clobber.
task automatic mem_rt_r0(input logic [15:0] s_op, input logic [15:0] l_op,
                         input logic with_gbr, input logic [7:0] base,
                         input integer sz, input logic [31:0] v, input string nm);
    begin
        clear_imem;
        imem[0] = 16'hE720;                          // MOV  #0x20,R7
        imem[1] = 16'h6572;                          // MOV.L @R7,R5
        imem[2] = 16'h6053;                          // MOV  R5,R0      ; R0 = value
        if(with_gbr) begin
            imem[3] = 16'(16'hE100 + {8'd0, base});  // MOV  #base,R1
            imem[4] = 16'h411E;                      // LDC  R1,GBR
        end
        else begin
            imem[3] = 16'(16'hE400 + {8'd0, base});  // MOV  #base,R4
            imem[4] = 16'h0009;                      // NOP
        end
        imem[5] = s_op;                              // store R0
        imem[6] = 16'hE000;                          // MOV  #0,R0      ; clobber before load
        imem[7] = l_op;                              // load into R0
        imem[8] = 16'h0009;                          // sentinel
        do_reset;
        dmem[8] = v;
        run_until(8, 400);
        chk(nm, gpr(0), sext(v, sz));
    end
endtask

//Full size x addressing-mode matrix: byte/word/longword roundtrips across every
//MOV addressing mode, with random data so load sign-extension is exercised.
task automatic test_mem_size_matrix;
    integer sz, rep;
    begin
        begin_test("Memory size matrix: B/W/L across @Rn, @-Rn/@Rm+, @(R0,Rn), @(disp), @(disp,GBR)");
        for(rep = 0; rep < 6; rep = rep + 1) begin
            //Register indirect, sizes B/W/L.
            for(sz = 0; sz < 3; sz = sz + 1)
                mem_rt(16'(16'h2450 + sz), 16'(16'h6640 + sz), 8'd0, 8'd0, sz,
                       $urandom, $sformatf("@Rn size%0d", sz));
            //Pre-decrement store then post-increment load.
            for(sz = 0; sz < 3; sz = sz + 1)
                mem_rt(16'(16'h2454 + sz), 16'(16'h6644 + sz), 8'd4, 8'd0, sz,
                       $urandom, $sformatf("@-Rn/@Rm+ size%0d", sz));
            //Indexed register indirect @(R0,Rn).
            for(sz = 0; sz < 3; sz = sz + 1)
                mem_rt(16'(16'h0454 + sz), 16'(16'h064C + sz), 8'd2, 8'd2, sz,
                       $urandom, $sformatf("@(R0,Rn) size%0d", sz));
            //Longword register displacement @(disp,Rn).
            mem_rt(16'h1451, 16'h5641, 8'd0, 8'd0, 2,
                   $urandom, "@(disp,Rn) longword");
            //Byte/word R0 displacement @(disp,Rn).
            for(sz = 0; sz < 2; sz = sz + 1)
                mem_rt_r0(16'(16'h8040 + (sz << 8)), 16'(16'h8440 + (sz << 8)), 1'b0, 8'd0, sz,
                          $urandom, $sformatf("@(disp,Rn) R0 size%0d", sz));
            //GBR displacement, sizes B/W/L.
            for(sz = 0; sz < 3; sz = sz + 1)
                mem_rt_r0(16'(16'hC000 + (sz << 8)), 16'(16'hC400 + (sz << 8)), 1'b1, 8'd0, sz,
                          $urandom, $sformatf("@(disp,GBR) size%0d", sz));
        end
        end_test;
    end
endtask


//LDS.L/STS.L memory forms for MACH/MACL/PR. The six forms (0x4m06/16/26 and
//0x4n02/12/22) are part of the ISA; this fails until the core decodes them.
task automatic test_lds_sts_l;
    begin
        begin_test("System: LDS.L/STS.L MACH/MACL/PR memory forms");

        //Load MACH/MACL/PR from memory and read them back through STS.
        imem[0]  = 16'hE000; // MOV  #0,R0
        imem[1]  = 16'h4006; // LDS.L @R0+,MACH ; MACH=dmem[0], R0=4
        imem[2]  = 16'h4016; // LDS.L @R0+,MACL ; MACL=dmem[1], R0=8
        imem[3]  = 16'h4026; // LDS.L @R0+,PR   ; PR=dmem[2], R0=12
        imem[4]  = 16'h0009; // NOP
        imem[5]  = 16'h0009;
        imem[6]  = 16'h0009;
        imem[7]  = 16'h010A; // STS  MACH,R1
        imem[8]  = 16'h021A; // STS  MACL,R2
        imem[9]  = 16'h032A; // STS  PR,R3
        imem[10] = 16'h0009; // sentinel
        do_reset;
        dmem[0] = 32'h1111_1111;
        dmem[1] = 32'h2222_2222;
        dmem[2] = 32'h3333_3333;
        run_until(10, 600);
        chk("LDS.L MACH", gpr(1), 32'h1111_1111);
        chk("LDS.L MACL", gpr(2), 32'h2222_2222);
        chk("LDS.L PR",   gpr(3), 32'h3333_3333);
        chk("LDS.L post-increment", gpr(0), 32'd12);

        //Set MACH/MACL/PR and store them back to memory with STS.L.
        clear_imem;
        imem[0]  = 16'hE411; // MOV  #0x11,R4
        imem[1]  = 16'h440A; // LDS  R4,MACH
        imem[2]  = 16'hE522; // MOV  #0x22,R5
        imem[3]  = 16'h451A; // LDS  R5,MACL
        imem[4]  = 16'hE633; // MOV  #0x33,R6
        imem[5]  = 16'h462A; // LDS  R6,PR
        imem[6]  = 16'h0009; // NOP
        imem[7]  = 16'h0009;
        imem[8]  = 16'hE710; // MOV  #16,R7
        imem[9]  = 16'h4702; // STS.L MACH,@-R7 ; R7=12, dmem[3]=0x11
        imem[10] = 16'h4712; // STS.L MACL,@-R7 ; R7=8,  dmem[2]=0x22
        imem[11] = 16'h4722; // STS.L PR,@-R7   ; R7=4,  dmem[1]=0x33
        imem[12] = 16'h0009; // sentinel
        do_reset;
        run_until(12, 600);
        chk("STS.L MACH memory", dmem[3], 32'h0000_0011);
        chk("STS.L MACL memory", dmem[2], 32'h0000_0022);
        chk("STS.L PR memory",   dmem[1], 32'h0000_0033);
        chk("STS.L pre-decrement", gpr(7), 32'd4);
        end_test;
    end
endtask

//SHAL (0100nnnn00100000) has a distinct opcode from SHLL (0100nnnn00000000);
//a decoder that aliases them silently skips one path. T captures bit 31.
task automatic test_shal;
    begin
        begin_test("SHAL: distinct opcode from SHLL, shifts left 1, T=old bit31");
        imem[0] = 16'hE101; // MOV  #1,R1
        imem[1] = 16'h4120; // SHAL R1           ; R1=2, T=0 (bit31 of 1 = 0)
        imem[2] = 16'hE2FF; // MOV  #-1,R2       ; R2=0xFFFFFFFF
        imem[3] = 16'h4220; // SHAL R2           ; R2=0xFFFFFFFE, T=1 (bit31 = 1)
        imem[4] = 16'h0529; // MOVT R5           ; R5=T=1
        imem[5] = 16'h0009; // sentinel
        do_reset;
        run_until(5, 400);
        chk("R1 (SHAL 1<<1)", gpr(1), 32'h0000_0002);
        chk("R2 (SHAL 0xFFFFFFFF<<1)", gpr(2), 32'hFFFF_FFFE);
        chk("R5 (T from SHAL, old bit31=1)", gpr(5), 32'd1);
        end_test;
    end
endtask

//CMP/EQ #imm,R0 sign-extends an 8-bit immediate.  The existing test only
//exercises imm=127; these cases cover 0, -1, and -128 (see pp.62 SH7709S.PDF).
task automatic test_cmp_eq_imm_range;
    begin
        begin_test("CMP/EQ #imm,R0: boundary values 0, -1, -128 exercise signed 8-bit decode");
        imem[0]  = 16'hE000; // MOV  #0,R0
        imem[1]  = 16'h8800; // CMP/EQ #0,R0      ; T=1
        imem[2]  = 16'h0129; // MOVT R1            ; R1=1
        imem[3]  = 16'hE0FF; // MOV  #-1,R0       ; R0=0xFFFFFFFF
        imem[4]  = 16'h88FF; // CMP/EQ #-1,R0     ; T=1 (0xFF sign-extended = -1)
        imem[5]  = 16'h0229; // MOVT R2            ; R2=1
        imem[6]  = 16'hE080; // MOV  #-128,R0     ; R0=0xFFFFFF80
        imem[7]  = 16'h8880; // CMP/EQ #-128,R0   ; T=1 (0x80 sign-extended = -128)
        imem[8]  = 16'h0329; // MOVT R3            ; R3=1
        imem[9]  = 16'hE001; // MOV  #1,R0
        imem[10] = 16'h8880; // CMP/EQ #-128,R0   ; T=0 (1 != -128)
        imem[11] = 16'h0429; // MOVT R4            ; R4=0
        imem[12] = 16'h0009; // sentinel
        do_reset;
        run_until(12, 400);
        chk("R1 (CMP/EQ #0 true)", gpr(1), 32'd1);
        chk("R2 (CMP/EQ #-1 true)", gpr(2), 32'd1);
        chk("R3 (CMP/EQ #-128 true)", gpr(3), 32'd1);
        chk("R4 (CMP/EQ #-128 false)", gpr(4), 32'd0);
        end_test;
    end
endtask

//Illegal slot instruction: exc_in_delay_slot=1 and exc_pc=branch PC (not the
//delay-slot PC), per §4.5.2 and §4.6 (SH7709S.PDF pp.95-101).
//Illegal slot instruction (condition a from §4.5.2): undefined opcode in a
//delay slot raises EXC_ILLEGAL with exc_in_delay_slot=1.  exc_pc is the slot
//instruction's address; branch PC = exc_pc-2, used by the handler to set SPC.
task automatic test_exc_slot;
    begin
        begin_test("Exception: illegal opcode in delay slot -> exc_in_delay_slot, exc_pc=slot addr");
        imem[0] = 16'hA001; // BRA  #1           ; target=word 3; delay slot=word 1
        imem[1] = 16'hF000; // .word 0xF000      ; undefined opcode in delay slot
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause (slot illegal)", {29'd0, exc_cause_l}, {29'd0, EXC_ILLEGAL});
        chk("exc pc = slot instruction address", exc_pc_l, 32'hA000_0002);
        chk_true("exc_in_delay_slot asserted", exc_delay_l);
        chk_true("slot word not retired", !retired_seen[1]);
        chk_true("branch PC derivable as exc_pc-2", (exc_pc_l - 32'd2) == 32'hA000_0000);
        end_test;
    end
endtask

//Multiple in-flight exceptions: the oldest (lowest-PC) exception must be the
//only one accepted, per Figure 4.2 (SH7709S.PDF p.89).  Younger exceptions
//that are simultaneously visible in the pipeline must be squashed.
task automatic test_exc_simultaneous;
    begin
        // Scenario A: EXC_ADDRESS (older, detected at EX) and EXC_ILLEGAL (younger, ID)
        // become visible in the same pipeline cycle; only EXC_ADDRESS must fire.
        begin_test("Exception priority: oldest exception wins when multiple are in-flight together");
        imem[0] = 16'hE001; // MOV  #1,R0        ; misaligned base
        imem[1] = 16'h6102; // MOV.L @R0,R1      ; EXC_ADDRESS: word 1 in EX
        imem[2] = 16'hF000; // .word 0xF000      ; EXC_ILLEGAL: word 2 in ID (same cycle)
        imem[3] = 16'hE303; // MOV  #3,R3        ; younger (must not retire)
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause: older EXC_ADDRESS beats EXC_ILLEGAL", {29'd0, exc_cause_l}, {29'd0, EXC_ADDRESS});
        chk("exc pc: word 1 (the older instruction)", exc_pc_l, 32'hA000_0002);
        chk_true("EXC_ILLEGAL word not retired", !retired_seen[2]);
        chk_true("younger word not retired", !retired_seen[3]);

        // Scenario B: EXC_DATA (oldest, MA) in-flight with EXC_ADDRESS (EX) and
        // EXC_ILLEGAL (ID); only EXC_DATA from the oldest instruction may fire.
        // Models §4.2.3 Figure 4.2: TLB-miss(MA), TLB-miss(IF), RIE(ID) simultaneous.
        clear_imem;
        d_fault_en   = 1'b1;
        d_fault_widx = 8'd0;       // fault any data access to byte address 0
        imem[0] = 16'hE000; // MOV  #0,R0        ; load address 0 -> will fault
        imem[1] = 16'hE101; // MOV  #1,R1        ; misaligned base for EXC_ADDRESS
        imem[2] = 16'h6402; // MOV.L @R0,R4      ; EXC_DATA (word 2, oldest; in MA)
        imem[3] = 16'h6512; // MOV.L @R1,R5      ; EXC_ADDRESS (word 3; in EX same cycle)
        imem[4] = 16'hF000; // .word 0xF000      ; EXC_ILLEGAL (word 4; in ID same cycle)
        imem[5] = 16'hE707; // MOV  #7,R7        ; younger (must not retire)
        do_reset;
        run_until_exc(400);
        chk_true("exception fired", exc_seen);
        chk("exc cause: oldest EXC_DATA beats later faults", {29'd0, exc_cause_l}, {29'd0, EXC_DATA});
        chk("exc pc: word 2 (the oldest faulting)", exc_pc_l, 32'hA000_0004);
        chk_true("EXC_ADDRESS word not retired", !retired_seen[3]);
        chk_true("EXC_ILLEGAL word not retired", !retired_seen[4]);
        chk_true("youngest word not retired", !retired_seen[5]);
        end_test;
    end
endtask

//MAC.W negative saturation clamps MACL to 0x80000000 (INT32_MIN); MAC.L
//negative saturation clamps MACH:MACL to the 48-bit minimum (0xFFFF8000:0).
//These complement the positive-side clamps in test_mac_saturation.
task automatic test_mac_neg_saturation;
    begin
        // MAC.W: seed MACL = INT32_MIN+2; product = -2*2 = -4 -> underflow
        begin_test("MAC negative saturation: MAC.W->0x80000000, MAC.L->48-bit minimum");
        imem[0]  = 16'hE310; // MOV  #16,R3      ; dmem[4] byte address
        imem[1]  = 16'h6432; // MOV.L @R3,R4     ; R4=0x8000_0002 (INT32_MIN+2)
        imem[2]  = 16'h441A; // LDS  R4,MACL
        imem[3]  = 16'h0058; // SETS              ; saturation enable
        imem[4]  = 16'hE100; // MOV  #0,R1       ; R1=0: high halfword addr
        imem[5]  = 16'hE202; // MOV  #2,R2       ; R2=2: low halfword addr
        imem[6]  = 16'h421F; // MAC.W @R1+,@R2+  ; MACL + (-2*2) -> negative overflow -> clamp
        imem[7]  = 16'h051A; // STS  MACL,R5
        imem[8]  = 16'h060A; // STS  MACH,R6
        imem[9]  = 16'h0009; // sentinel
        do_reset;
        dmem[0]  = 32'hFFFE_0002; // big-endian halfwords: [0]=-2 [2]=+2 (see pp.143)
        dmem[4]  = 32'h8000_0002; // initial MACL = INT32_MIN + 2
        run_until(9, 600);
        chk("R5 (MAC.W negative clamp)", gpr(5), 32'h8000_0000);
        chk("R6 (MAC.W overflow flag)", gpr(6), 32'h0000_0001);

        // MAC.L: seed MACH:MACL = 48-bit MIN+2; product = -2*2 = -4 -> underflow
        clear_imem;
        imem[0]  = 16'hE328; // MOV  #40,R3      ; dmem[10] byte address
        imem[1]  = 16'h6432; // MOV.L @R3,R4     ; R4=0xFFFF_8000 (48-bit min, upper 32)
        imem[2]  = 16'h440A; // LDS  R4,MACH
        imem[3]  = 16'hE32C; // MOV  #44,R3      ; dmem[11] byte address
        imem[4]  = 16'h6432; // MOV.L @R3,R4     ; R4=0x0000_0002 (2 above 48-bit min)
        imem[5]  = 16'h441A; // LDS  R4,MACL
        imem[6]  = 16'h0058; // SETS              ; saturation enable
        imem[7]  = 16'hE130; // MOV  #48,R1      ; R1=48: addr of Rn=-2
        imem[8]  = 16'hE234; // MOV  #52,R2      ; R2=52: addr of Rm=+2
        imem[9]  = 16'h021F; // MAC.L @R1+,@R2+  ; 48-bit(MIN+2) + (-2*2) -> clamp
        imem[10] = 16'h0A0A; // STS  MACH,R10
        imem[11] = 16'h0B1A; // STS  MACL,R11
        imem[12] = 16'h0009; // sentinel
        do_reset;
        dmem[10] = 32'hFFFF_8000; // MACH: upper word of 48-bit minimum
        dmem[11] = 32'h0000_0002; // MACL: 2 above 48-bit minimum
        dmem[12] = 32'hFFFF_FFFE; // MAC.L Rn = -2
        dmem[13] = 32'h0000_0002; // MAC.L Rm = +2
        run_until(12, 600);
        chk("R10 (MAC.L MACH negative clamp)", gpr(10), 32'hFFFF_8000);
        chk("R11 (MAC.L MACL negative clamp)", gpr(11), 32'h0000_0000);
        end_test;
    end
endtask


///////////////////////////////////////////////////////////
//////  Test Sequencer
////

initial begin
    clk            = 1'b0;
    por_n          = 1'b1;
    rst_n          = 1'b0;
    if_fault_en    = 1'b0;
    if_fault_widx  = 11'd0;
    d_fault_en     = 1'b0;
    d_fault_widx   = 8'd0;
    d_latency      = 1;
    i_latency      = 0;    //IPC baselines were locked with instant instruction reads

    $display("######## cpu_core_tb ########");

    //+focus: run only the interrupt/exception collision sweeps (debug subset).
    if($test$plusargs("focus")) begin
        group("focus: interrupt/exception machinery subset");
        test_int_timing_sweep;
        test_int_miss_sweep;
        test_int_tas_atomic;
        test_exc_int_collision;
        $display("");
        if(errors == 0) $display("cpu_core_tb: PASS (focus subset, %0d tests)", test_count);
        else            $display("cpu_core_tb: FAIL (focus subset, %0d errors over %0d tests)", errors, test_count);
        $finish;
    end

    bench_ipc_straightline(200);
    //Cached straight-line steady state runs 1 hit/cycle (look_ov accepts a fetch every
    //cycle); the measured loop IPC is that minus the 2 taken-branch bubbles per iteration
    //(Fig 10.40). body=100 is near the max single-loop reach for an 8-bit BF displacement.
    bench_ipc_cached(100, 12);
    bench_ipc_store(80, 6);

    group("0. Reset causes");
    test_reset_event_codes;

    group("1. Datapath: register file, operand routing, result forwarding");
    test_gpr_bram_phases;
    test_alu_forwarding;
    test_store_data_forwarding;

    group("2. Hazard detection: interlocks, serialization, pipeline stalls");
    test_load_use_interlock;
    test_state_serialization;
    test_stalled_ex_operand;
    test_ma_wait;

    group("3. Privilege mode and special functions: banks, control regs, events");
    test_bank_switch;
    test_control_transfers;
    test_bank_control_transfers;
    test_sys_misc;
    test_lds_sts_l;
    test_pref_no_cache;
    test_events;
    test_rte;
    test_exc_privilege;

    group("4. Branch: conditional, delayed, far, and subroutine control flow");
    test_tbit_branch;
    test_dt_loop;
    test_delayed_branch;
    test_bsr_rts;
    test_branch_extra;

    group("5. ALU: arithmetic, logic, shift, rotate, compare, multiply, divide");
    test_addsub_t;
    test_logic_t;
    test_shift_ops;
    test_rotate_t;
    test_compare_t;
    test_cmp_eq_imm_range;
    test_misc_ops;
    test_addsub_rand;
    test_logic_rand;
    test_shift_rand;
    test_rotate_rand;
    test_compare_rand;
    test_misc_rand;
    test_shift;
    test_shal;
    test_mul_mac;
    test_muls_w;
    test_dmul_long;
    test_mac_l_accumulate;
    test_mac_w_accumulate;
    test_mac_w_same_pointer;
    test_mac_saturation;
    test_mac_neg_saturation;
    test_mac;
    test_division;

    group("6. Memory read/write: sizes, addressing modes, PC-relative, byte RMW");
    test_mov_sizes;
    test_mem_size_matrix;
    test_pc_relative;
    test_indexed_r0;
    test_addr_update;
    test_byte_rmw;

    group("7. Exceptions and faults: illegal/address/fetch/data, redirect recovery");
    test_exc_illegal;
    test_exc_address;
    test_exc_ifetch;
    test_exc_data;
    test_redirect_recovery;
    test_exc_rte_roundtrip;
    test_exc_slot;
    test_exc_simultaneous;

    //Cacheable D$ rebuild gates: ISA-correct store->load round-trips. EXPECTED TO FAIL on
    //the present FSM cache; the cen_p 1-cycle rebuild (with the store-bypass register)
    //must turn both green. See cache-rebuild-plan-and-dpath-finding memory.
    group("8. Cacheable D$ goldens (rebuild gates)");
    test_cached_store_load_spaced;
    test_cached_store_load_fwd;
    test_cached_store_hit_load;

    //Cache coherency + I-side RDW goldens: unified-array self-modify visibility, the
    //byte-lane bypass compose, and wb-buffer alias ordering. See cache-microbench-gaps.
    group("9. Cache coherency goldens: self-modify, byte-lane RDW, wb-buffer alias");
    test_cached_byte_store_fwd;
    test_cached_selfmod_sweep;
    test_cached_selfmod_branch(8'h09, "Self-modify via redirect: poked line refetched after BRA (write-back)");
    test_cached_selfmod_branch(8'h0B, "Self-modify via redirect: poked line refetched after BRA (write-through)");
    test_cached_wb_alias;

    //Miss-flow stress: fill-stretch latency sweeps (squash timing), per-beat fill
    //faults, the drain-fault law, and dense LRU traffic. See cache-microbench-gaps.
    group("10. Cache stress: latency/squash sweep, fill faults, drain law, LRU thrash");
    test_cached_latency_invariance;
    test_cached_fill_fault_d;
    test_cached_fill_fault_i;
    test_cached_fill_fault_victim;
    test_cached_pref_fill_fault;
    test_cached_drain_fault_law;
    test_cached_lru_thrash;
    test_ccr_flush_discard;
    test_ccr_ce_revival;
    test_tas_vs_dirty;
    test_bl_exception_reset;
    test_squash_victim_drain;

    //Fetch-pair laws: the sibling of every even fetch issues without a bus request.
    group("11. Fetch-pair laws: request halving, branch kill, idle-edge drain");
    test_pair_fetch_law;
    test_pair_branch_kill;
    test_cached_drain_pairidle;

    //Pipeline-state and interrupt corners + the random invariance oracle.
    group("12. Pipeline state: LDC-SR spacing, interrupt timing/machinery, exc collision, random oracle");
    test_cached_ldcsr_tight;
    test_int_timing_sweep;
    test_int_miss_sweep;
    test_int_tas_atomic;
    test_exc_int_collision;
    test_random_latency_oracle;

    //Verdicts of the always-on passive checkers, judged over the WHOLE suite.
    group("13. Suite-wide property checkers");
    test_property_summary;

    $display("");
    $display("################################");
    if(errors == 0) $display("cpu_core_tb: PASS (%0d tests)", test_count);
    else            $display("cpu_core_tb: FAIL (%0d errors over %0d tests)", errors, test_count);
    if(errors != 0) $fatal(1, "test failures: %0d", errors);
    $finish;
end

endmodule

`default_nettype wire
