`timescale 1 ns / 1 ps

// Top-level IP wrapper.
// AXI-Lite register map:
//   0x00 CONFIG_REG (write): [7:0]=ws  [15:8]=we  [27:16]=baseline
//   0x04 reserved
//   0x08 STATUS_REG (read):  [0]=done
//   0x0C CHARGE_REG (read):  [21:0]=charge result
// AXI-Stream: one 32-bit word per ADC sample, TLAST on last sample.

module charge_ip_v1_0 #(
    parameter integer C_S00_AXI_DATA_WIDTH   = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH   = 4,
    parameter integer C_S00_AXIS_TDATA_WIDTH = 32
)(
    // AXI-Lite
    input  wire                                s00_axi_aclk,
    input  wire                                s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]    s00_axi_awaddr,
    input  wire [2:0]                          s00_axi_awprot,
    input  wire                                s00_axi_awvalid,
    output wire                                s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1:0]    s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire                                s00_axi_wvalid,
    output wire                                s00_axi_wready,
    output wire [1:0]                          s00_axi_bresp,
    output wire                                s00_axi_bvalid,
    input  wire                                s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]    s00_axi_araddr,
    input  wire [2:0]                          s00_axi_arprot,
    input  wire                                s00_axi_arvalid,
    output wire                                s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1:0]    s00_axi_rdata,
    output wire [1:0]                          s00_axi_rresp,
    output wire                                s00_axi_rvalid,
    input  wire                                s00_axi_rready,

    // AXI-Stream (ADC samples from DMA)
    input  wire                                s00_axis_aclk,
    input  wire                                s00_axis_aresetn,
    output wire                                s00_axis_tready,
    input  wire [C_S00_AXIS_TDATA_WIDTH-1:0]  s00_axis_tdata,
    input  wire [(C_S00_AXIS_TDATA_WIDTH/8)-1:0] s00_axis_tstrb,
    input  wire                                s00_axis_tlast,
    input  wire                                s00_axis_tvalid
);

    wire [31:0] config_reg;
    wire [31:0] status_reg;
    wire [31:0] charge_reg;

    wire        charge_clk;
    wire        charge_rst;
    wire        charge_start;
    wire [47:0] charge_sample_word;  // flat 48-bit: [3:0][11:0]
    wire [7:0]  charge_ws;
    wire [7:0]  charge_we;
    wire [11:0] charge_baseline;
    wire [21:0] charge_out;
    wire        charge_done;

    localparam N_BUF = 64;
    reg [11:0] sample_mem [0:N_BUF-1];

    // Sequencer FSM
    localparam ST_IDLE  = 2'b00;
    localparam ST_START = 2'b01;
    localparam ST_FEED  = 2'b10;

    reg [1:0]  seq_state;
    reg [6:0]  seq_cnt;
    reg        stream_done;
    reg        done_flag;
    reg [21:0] charge_latch;
    reg [7:0]  wr_ptr;
    reg [11:0] sw0, sw1, sw2, sw3;
    reg [7:0]  rd_base;

    assign charge_clk      = s00_axi_aclk;
    assign charge_rst      = ~s00_axi_aresetn;
    assign charge_ws       = config_reg[7:0];
    assign charge_we       = config_reg[15:8];
    assign charge_baseline = config_reg[27:16];

    charge_ip_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) charge_ip_v1_0_S00_AXI_inst (
        .CONFIG_REG   (config_reg),
        .STATUS_REG   (status_reg),
        .CHARGE_REG   (charge_reg),
        .S_AXI_ACLK   (s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),
        .S_AXI_AWADDR (s00_axi_awaddr),
        .S_AXI_AWPROT (s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),
        .S_AXI_WDATA  (s00_axi_wdata),
        .S_AXI_WSTRB  (s00_axi_wstrb),
        .S_AXI_WVALID (s00_axi_wvalid),
        .S_AXI_WREADY (s00_axi_wready),
        .S_AXI_BRESP  (s00_axi_bresp),
        .S_AXI_BVALID (s00_axi_bvalid),
        .S_AXI_BREADY (s00_axi_bready),
        .S_AXI_ARADDR (s00_axi_araddr),
        .S_AXI_ARPROT (s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),
        .S_AXI_RDATA  (s00_axi_rdata),
        .S_AXI_RRESP  (s00_axi_rresp),
        .S_AXI_RVALID (s00_axi_rvalid),
        .S_AXI_RREADY (s00_axi_rready)
    );

    assign status_reg = {31'b0, done_flag};
    assign charge_reg = {10'b0, charge_latch};

    wire seq_active = (seq_state != ST_IDLE);
    assign s00_axis_tready = s00_axi_aresetn && !stream_done && !seq_active;

    // Fill sample_mem from AXI-Stream; mark stream_done on TLAST.
    always @(posedge charge_clk) begin
        if (!s00_axi_aresetn) begin
            wr_ptr      <= 8'h00;
            stream_done <= 1'b0;
        end else begin
            if (seq_state == ST_IDLE && stream_done)
                stream_done <= 1'b0;

            if (!stream_done && !seq_active && s00_axis_tvalid) begin
                if (wr_ptr < N_BUF)
                    sample_mem[wr_ptr] <= s00_axis_tdata[11:0];
                wr_ptr <= wr_ptr + 1;
                if (s00_axis_tlast) begin
                    stream_done <= 1'b1;
                    wr_ptr      <= 8'h00;
                end
            end
        end
    end

    // Drives charge.sv once a full frame has been received.
    always @(posedge charge_clk) begin
        if (!s00_axi_aresetn) begin
            seq_state    <= ST_IDLE;
            seq_cnt      <= 7'h00;
            done_flag    <= 1'b0;
            charge_latch <= 22'h0;
        end else begin
            case (seq_state)
                ST_IDLE: begin
                    if (stream_done) begin
                        done_flag <= 1'b0;
                        seq_cnt   <= 7'h00;
                        seq_state <= ST_START;
                    end
                end
                ST_START: begin
                    seq_cnt   <= 7'h00;
                    seq_state <= ST_FEED;
                end
                ST_FEED: begin
                    if (charge_done) begin
                        charge_latch <= charge_out;
                        done_flag    <= 1'b1;
                        seq_state    <= ST_IDLE;
                    end else begin
                        seq_cnt <= seq_cnt + 1;
                    end
                end
                default: seq_state <= ST_IDLE;
            endcase
        end
    end

    assign charge_start = (seq_state == ST_START);

    always @(*) begin
        rd_base = {seq_cnt[5:0], 2'b00};
        if (seq_state == ST_FEED) begin
            sw0 = (rd_base     < N_BUF) ? sample_mem[rd_base    ] : 12'h0;
            sw1 = (rd_base + 1 < N_BUF) ? sample_mem[rd_base + 1] : 12'h0;
            sw2 = (rd_base + 2 < N_BUF) ? sample_mem[rd_base + 2] : 12'h0;
            sw3 = (rd_base + 3 < N_BUF) ? sample_mem[rd_base + 3] : 12'h0;
        end else begin
            sw0 = 12'h0; sw1 = 12'h0; sw2 = 12'h0; sw3 = 12'h0;
        end
    end

    // Pack 4x12-bit into flat 48-bit word (charge.sv: sample_word[0]=bits[11:0] etc.)
    assign charge_sample_word = {sw3, sw2, sw1, sw0};

    charge #(
        .ADC_BITS(12),
        .ACC_BITS(22),
        .IDX_BITS(8)
    ) charge_inst (
        .clk         (charge_clk),
        .rst         (charge_rst),
        .start       (charge_start),
        .sample_word (charge_sample_word),
        .window_start(charge_ws),
        .window_end  (charge_we),
        .baseline    (charge_baseline),
        .charge      (charge_out),
        .done        (charge_done)
    );

endmodule
