// =============================================================
//  Ports:
//    clk          — 250 MHz clock
//    rst          — synchronous reset, active high
//    start        — 1-cycle pulse: "start processing a new frame"
//    sample_word  — [3:0][11:0]: 4 ADC samples, [0] = oldest
//    window_start — first ADC sample index of the integration window
//    window_end   — last  ADC sample index of the integration window (inclusive)
//    baseline     — DC offset to subtract from each sample (12-bit)
//    charge       — final result (valid when done = 1)
//    done         — 1-cycle pulse marking that "charge" is valid
// =============================================================

module charge #(
    parameter int ADC_BITS = 12,
    parameter int ACC_BITS = 22,   // big enough for the largest window (256 x 12-bit ADC samples)
    parameter int IDX_BITS = 8
)(
    input  logic                        clk,
    input  logic                        rst,
    input  logic                        start,

    input  logic [3:0][ADC_BITS-1:0]   sample_word,

    input  logic [IDX_BITS-1:0]        window_start,
    input  logic [IDX_BITS-1:0]        window_end,
    input  logic [ADC_BITS-1:0]        baseline,

    output logic [ACC_BITS-1:0]        charge,
    output logic                       done
);

    localparam int RATIO = 4;

    typedef enum logic [1:0] {
        IDLE      = 2'b00,
        INTEGRATE = 2'b01,
        DONE_ST   = 2'b10
    } state_t;

    state_t state;

    logic [IDX_BITS-1:0]  sample_idx;   // ADC index of sample [0] in the current group
    logic [ACC_BITS-1:0]  acc;          // running total (already multiplied by 2)
    logic                 armed;        // set by "start", cleared once "done" fires

    logic [IDX_BITS-1:0]  ws_r;         // window_start, exact value
    logic [IDX_BITS-1:0]  we_r;         // window_end, exact value
    logic [IDX_BITS-1:0]  ws_base_r;    // ws rounded down to a group of 4 (ws & ~3)
    logic [IDX_BITS-1:0]  we_base_r;    // we rounded down to a group of 4 (we & ~3)

    // Weight LUT: 0 = outside window, 1 = boundary (ws/we), 2 = interior.
    logic [1:0] w_lut [0:63];

    // Subtract baseline, clip negative results to 0 (sc = max(sample - baseline, 0))
    logic [12:0] sigs0, sigs1, sigs2, sigs3;   // 13-bit: bit 12 = sign
    assign sigs0 = {1'b0, sample_word[0]} - {1'b0, baseline};
    assign sigs1 = {1'b0, sample_word[1]} - {1'b0, baseline};
    assign sigs2 = {1'b0, sample_word[2]} - {1'b0, baseline};
    assign sigs3 = {1'b0, sample_word[3]} - {1'b0, baseline};

    logic [11:0] sc0, sc1, sc2, sc3;
    assign sc0 = sigs0[12] ? 12'd0 : sigs0[11:0];
    assign sc1 = sigs1[12] ? 12'd0 : sigs1[11:0];
    assign sc2 = sigs2[12] ? 12'd0 : sigs2[11:0];
    assign sc3 = sigs3[12] ? 12'd0 : sigs3[11:0];

    logic [ACC_BITS-1:0] c0, c1, c2, c3;

    assign c0 = (w_lut[sample_idx    ] == 2'd0) ? '0 :
                (w_lut[sample_idx    ] == 2'd1) ? ACC_BITS'(sc0) : ACC_BITS'(sc0) << 1;

    assign c1 = (w_lut[sample_idx + 1] == 2'd0) ? '0 :
                (w_lut[sample_idx + 1] == 2'd1) ? ACC_BITS'(sc1) : ACC_BITS'(sc1) << 1;

    assign c2 = (w_lut[sample_idx + 2] == 2'd0) ? '0 :
                (w_lut[sample_idx + 2] == 2'd1) ? ACC_BITS'(sc2) : ACC_BITS'(sc2) << 1;

    assign c3 = (w_lut[sample_idx + 3] == 2'd0) ? '0 :
                (w_lut[sample_idx + 3] == 2'd1) ? ACC_BITS'(sc3) : ACC_BITS'(sc3) << 1;

    logic [ACC_BITS-1:0] grp_delta;
    assign grp_delta = c0 + c1 + c2 + c3;

    always_ff @(posedge clk) begin
        if (rst) begin
            state      <= IDLE;
            sample_idx <= '0;
            acc        <= '0;
            armed      <= 1'b0;
            charge     <= '0;
            done       <= 1'b0;
            ws_r       <= '0;
            we_r       <= '0;
            ws_base_r  <= '0;
            we_base_r  <= '0;
            for (int i = 0; i < 64; i++) w_lut[i] <= 2'd0;

        end else begin
            done       <= 1'b0;
            sample_idx <= sample_idx + IDX_BITS'(RATIO);

            case (state)

                // Wait for "start", then wait until sample_idx reaches ws_base_r.
                IDLE: begin
                    if (start) begin
                        armed      <= 1'b1;
                        sample_idx <= '0;
                        acc        <= '0;
                        ws_r       <= window_start;
                        we_r       <= window_end;
                        ws_base_r  <= {window_start[IDX_BITS-1:2], 2'b00};
                        we_base_r  <= {window_end  [IDX_BITS-1:2], 2'b00};
                        for (int i = 0; i < 64; i++) begin
                            if (i < int'(window_start) || i > int'(window_end))
                                w_lut[i] <= 2'd0;
                            else if (i == int'(window_start) || i == int'(window_end))
                                w_lut[i] <= 2'd1;
                            else
                                w_lut[i] <= 2'd2;
                        end
                    end

                    if (armed && sample_idx == ws_base_r) begin
                        acc <= grp_delta;
                        // Single-group window -> already done; else keep integrating.
                        state <= (ws_base_r == we_base_r) ? DONE_ST : INTEGRATE;
                    end
                end

                INTEGRATE: begin
                    acc <= acc + grp_delta;
                    if (sample_idx == we_base_r)
                        state <= DONE_ST;
                end

                DONE_ST: begin
                    charge <= acc >> 1;
                    done   <= 1'b1;
                    armed  <= 1'b0;
                    state  <= IDLE;
                end

            endcase
        end
    end

endmodule
