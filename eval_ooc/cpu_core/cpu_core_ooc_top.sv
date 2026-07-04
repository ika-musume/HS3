`default_nettype wire

/*
    Flat-port out-of-context top for cpu_core.

    The OOC wrapper generator only parses plain ANSI ports, so this shim exposes
    the single IBus_1.master I_BUS interface as flat scalar/vector ports and
    re-bundles them into the IBus_1 the core expects. Every other cpu_core port is
    already a flat scalar/vector and is passed straight through. Logic content is
    unchanged; this only exists so TimeQuest sees a registered-boundary cpu_core.
*/

module cpu_core_ooc_top (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* I BUS 1 (master) — flattened IBus_1 */
    output  wire            m_req_valid,
    output  wire            m_req_write,
    output  wire    [1:0]   m_req_size,
    output  wire    [31:0]  m_req_addr,
    output  wire    [31:0]  m_req_wdata,
    output  wire    [3:0]   m_req_wstrb,
    output  wire            m_req_lock,
    output  wire            m_rsp_ready,
    input   wire            m_req_ready,
    input   wire            m_rsp_valid,
    input   wire    [31:0]  m_rsp_rdata,
    input   wire            m_rsp_fault,

    /* ALREADY-PRIORITIZED EXTERNAL INTERRUPTS */
    input   wire            i_NMI_VALID,
    input   wire            i_NMI_BLMSK,
    input   wire            i_INT_VALID,
    input   wire    [3:0]   i_INT_LEVEL,
    input   wire    [11:0]  i_INT_CODE,
    output  wire            o_INT_ACK,
    output  wire            o_NMI_ACK,

    /* DEBUG OBSERVATION */
    output  wire            dbg_o_RETIRE_VALID,
    output  wire    [31:0]  dbg_o_RETIRE_PC,
    output  wire    [15:0]  dbg_o_RETIRE_INST,
    output  wire            dbg_o_RETIRE_GPR_WE,
    output  wire    [4:0]   dbg_o_RETIRE_GPR,
    output  wire    [31:0]  dbg_o_RETIRE_GPR_DATA,
    output  wire    [31:0]  dbg_o_FETCH_PC,
    output  wire    [31:0]  dbg_o_SR,
    output  wire    [31:0]  dbg_o_GBR,
    output  wire    [31:0]  dbg_o_SSR,
    output  wire    [31:0]  dbg_o_SPC,
    output  wire    [31:0]  dbg_o_VBR,
    output  wire    [31:0]  dbg_o_MACH,
    output  wire    [31:0]  dbg_o_MACL,
    output  wire    [31:0]  dbg_o_PR,
    output  wire    [31:0]  dbg_o_TRA,
    output  wire    [31:0]  dbg_o_EXPEVT,
    output  wire    [31:0]  dbg_o_INTEVT,
    output  wire    [31:0]  dbg_o_TEA,
    output  wire            dbg_o_EXC_VALID,
    output  wire    [2:0]   dbg_o_EXC_CAUSE,
    output  wire    [31:0]  dbg_o_EXC_PC,
    output  wire            dbg_o_EXC_IN_DELAY_SLOT,
    output  wire            dbg_o_EXC_ACCESS_WRITE,
    output  wire    [31:0]  dbg_o_EXC_ACCESS_ADDR,
    output  wire            dbg_o_TRAPA_VALID,
    output  wire    [7:0]   dbg_o_TRAPA_IMM,
    output  wire            dbg_o_RTE_VALID,

    /* STATE-CONTROLLER EVENTS */
    output  wire            o_EXCEPTION_ENTRY_VALID,
    output  wire    [31:0]  o_EXCEPTION_ENTRY_PC,
    output  wire            o_SLEEP_VALID,
    output  wire            o_LDTLB_VALID
);

    IBus_1 MEM_BUS();

    //MEM_BUS (master): core drives request side, wrapper drives response side.
    assign m_req_valid = MEM_BUS.req_valid;
    assign m_req_write = MEM_BUS.req_write;
    assign m_req_size  = MEM_BUS.req_size;
    assign m_req_addr  = MEM_BUS.req_addr;
    assign m_req_wdata = MEM_BUS.req_wdata;
    assign m_req_wstrb = MEM_BUS.req_wstrb;
    assign m_req_lock  = MEM_BUS.req_lock;
    assign m_rsp_ready = MEM_BUS.rsp_ready;
    assign MEM_BUS.req_ready = m_req_ready;
    assign MEM_BUS.rsp_valid = m_rsp_valid;
    assign MEM_BUS.rsp_rdata = m_rsp_rdata;
    assign MEM_BUS.rsp_fault = m_rsp_fault;

    cpu_core #(
        .RESET_PC               (32'hA000_0000              ),
        .BIG_ENDIAN             (1'b1                       )
    ) u_dut (
        .i_POR_n                (i_POR_n                    ),
        .i_RST_n                (i_RST_n                    ),
        .i_CLK                  (i_CLK                      ),
        .i_CEN                  (i_CEN                      ),

        .I_BUS                  (MEM_BUS                    ),

        .i_NMI_VALID            (i_NMI_VALID                ),
        .i_NMI_BLMSK            (i_NMI_BLMSK                ),
        .i_INT_VALID            (i_INT_VALID                ),
        .i_INT_LEVEL            (i_INT_LEVEL                ),
        .i_INT_CODE             (i_INT_CODE                 ),
        .o_INT_ACK              (o_INT_ACK                  ),
        .o_NMI_ACK              (o_NMI_ACK                  ),

        .dbg_o_RETIRE_VALID     (dbg_o_RETIRE_VALID         ),
        .dbg_o_RETIRE_PC        (dbg_o_RETIRE_PC            ),
        .dbg_o_RETIRE_INST      (dbg_o_RETIRE_INST          ),
        .dbg_o_RETIRE_GPR_WE    (dbg_o_RETIRE_GPR_WE        ),
        .dbg_o_RETIRE_GPR       (dbg_o_RETIRE_GPR           ),
        .dbg_o_RETIRE_GPR_DATA  (dbg_o_RETIRE_GPR_DATA      ),
        .dbg_o_FETCH_PC         (dbg_o_FETCH_PC             ),
        .dbg_o_SR               (dbg_o_SR                   ),
        .dbg_o_GBR              (dbg_o_GBR                  ),
        .dbg_o_SSR              (dbg_o_SSR                  ),
        .dbg_o_SPC              (dbg_o_SPC                  ),
        .dbg_o_VBR              (dbg_o_VBR                  ),
        .dbg_o_MACH             (dbg_o_MACH                 ),
        .dbg_o_MACL             (dbg_o_MACL                 ),
        .dbg_o_PR               (dbg_o_PR                   ),
        .dbg_o_TRA              (dbg_o_TRA                  ),
        .dbg_o_EXPEVT           (dbg_o_EXPEVT               ),
        .dbg_o_INTEVT           (dbg_o_INTEVT               ),
        .dbg_o_TEA              (dbg_o_TEA                  ),
        .dbg_o_EXC_VALID        (dbg_o_EXC_VALID            ),
        .dbg_o_EXC_CAUSE        (dbg_o_EXC_CAUSE            ),
        .dbg_o_EXC_PC           (dbg_o_EXC_PC               ),
        .dbg_o_EXC_IN_DELAY_SLOT(dbg_o_EXC_IN_DELAY_SLOT    ),
        .dbg_o_EXC_ACCESS_WRITE (dbg_o_EXC_ACCESS_WRITE     ),
        .dbg_o_EXC_ACCESS_ADDR  (dbg_o_EXC_ACCESS_ADDR      ),
        .dbg_o_TRAPA_VALID      (dbg_o_TRAPA_VALID          ),
        .dbg_o_TRAPA_IMM        (dbg_o_TRAPA_IMM            ),
        .dbg_o_RTE_VALID        (dbg_o_RTE_VALID            ),

        .o_EXCEPTION_ENTRY_VALID(o_EXCEPTION_ENTRY_VALID    ),
        .o_EXCEPTION_ENTRY_PC   (o_EXCEPTION_ENTRY_PC       ),
        .o_SLEEP_VALID          (o_SLEEP_VALID              ),
        .o_LDTLB_VALID          (o_LDTLB_VALID              )
    );

endmodule

`default_nettype none
