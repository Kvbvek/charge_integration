// =============================================================
//  charge_auto_bl.sv  —  charge integrator z automatycznym baseline
//
//  Różnica vs charge.sv:
//    - nowy stan BASELINE: zbiera próbki pre-window (0..ws_base-1)
//      i oblicza z nich baseline jako średnią ważoną
//    - baseline_out: wynik baseline widoczny przez AXI-Lite (nowy port)
//    - nie wymaga żadnych zmian w charge_ip_v1_0.v (wrapper)
//    - identyczna liczba cykli jak charge.sv (BASELINE zastępuje
//      martwy czas oczekiwania w IDLE na ws_base)
//
//  Obliczenie baseline bez dzielnika sprzętowego:
//    n_groups = ws_base / 4   (liczba pre-window grup)
//    baseline = (bl_sum / 4) / n_groups
//             = (bl_sum >> 2) * inv_lut[n_groups] >> INV_SHIFT
//    inv_lut[n] = round(2^INV_SHIFT / n)  — 16-wpisowa ROM
//
//  Ports:
//    clk, rst, start      — jak w charge.sv
//    sample_word          — 4 probki ADC / takt (jak w charge.sv)
//    window_start/end     — granice okna integracji (jak w charge.sv)
//    charge               — wynik ładunku (jak w charge.sv)
//    done                 — impuls gotowości (jak w charge.sv)
//    baseline_out         — NOWY: obliczony baseline (do odczytu PS)
// =============================================================

module charge_auto_bl #(
    parameter int ADC_BITS = 12,
    parameter int ACC_BITS = 22,
    parameter int IDX_BITS = 8
)(
    input  logic                       clk,
    input  logic                       rst,
    input  logic                       start,

    input  logic [3:0][ADC_BITS-1:0]  sample_word,

    input  logic [IDX_BITS-1:0]       window_start,
    input  logic [IDX_BITS-1:0]       window_end,

    output logic [ACC_BITS-1:0]       charge,
    output logic                      done,
    output logic [ADC_BITS-1:0]       baseline_out    // obliczony baseline
);
    localparam int RATIO     = 4;
    localparam int INV_SHIFT = 10;  // precyzja: 2^10 = 1024

    // =========================================================
    //  FSM — 4 stany
    // =========================================================
    typedef enum logic [1:0] {
        IDLE      = 2'b00,
        BASELINE  = 2'b01,   // zbiera próbki pre-window, oblicza baseline
        INTEGRATE = 2'b10,   // akumulacja ładunku (jak w charge.sv)
        DONE_ST   = 2'b11    // latch wyniku, done=1
    } state_t;
    state_t state;

    // =========================================================
    //  Reciprocal LUT — unika dzielnika sprzętowego
    //  inv_lut[n] = round(1024 / n)  dla n = 0..15
    //  n = liczba pre-window grup (ws_base / 4)
    // =========================================================
    logic [10:0] inv_lut [0:15];
    initial begin
        inv_lut[0]  = 11'd0;     // n=0: ws=0, brak pre-window (baseline=0)
        inv_lut[1]  = 11'd1024;  // 1024/1  = 1024
        inv_lut[2]  = 11'd512;   // 1024/2  = 512
        inv_lut[3]  = 11'd341;   // 1024/3  ≈ 341  (błąd 0.03%)
        inv_lut[4]  = 11'd256;   // 1024/4  = 256
        inv_lut[5]  = 11'd205;   // 1024/5  ≈ 205  (błąd 0.10%)
        inv_lut[6]  = 11'd171;   // 1024/6  ≈ 171  (błąd 0.06%)
        inv_lut[7]  = 11'd146;   // 1024/7  ≈ 146  (błąd 0.05%)
        inv_lut[8]  = 11'd128;   // 1024/8  = 128
        inv_lut[9]  = 11'd114;   // 1024/9  ≈ 114  (błąd 0.09%)
        inv_lut[10] = 11'd102;   // 1024/10 ≈ 102  (błąd 0.20%)
        inv_lut[11] = 11'd93;    // 1024/11 ≈ 93   (błąd 0.08%)
        inv_lut[12] = 11'd85;    // 1024/12 ≈ 85   (błąd 0.08%)
        inv_lut[13] = 11'd79;    // 1024/13 ≈ 79   (błąd 0.08%)
        inv_lut[14] = 11'd73;    // 1024/14 ≈ 73   (błąd 0.02%)
        inv_lut[15] = 11'd68;    // 1024/15 ≈ 68   (błąd 0.49%)
    end

    // =========================================================
    //  Rejestry
    // =========================================================
    logic [IDX_BITS-1:0]  sample_idx;
    logic [ACC_BITS-1:0]  acc;
    logic [IDX_BITS-1:0]  ws_base_r;
    logic [IDX_BITS-1:0]  we_base_r;

    // Akumulator pre-window (max: 15 grup × 4 × 4095 = 245 700 → 18 bit)
    logic [20:0]          bl_sum;
    // Zarejestrowany wynik baseline
    logic [ADC_BITS-1:0]  baseline_est;

    // Weight LUT — precomputed przy starcie (jak w charge.sv z optymalizacją LUT)
    logic [1:0] w_lut [0:63];

    // =========================================================
    //  Kombinacyjne obliczenie baseline z bieżącego bl_sum
    //  n_bl_groups = ws_base_r / 4 = ws_base_r[IDX_BITS-1:2]
    //  baseline = (bl_sum / 4) / n_groups
    //           = (bl_sum >> 2) * inv_lut[n_groups] >> INV_SHIFT
    //
    //  WAŻNE: (* use_dsp = "no" *) — wymusza implementację w LUT zamiast DSP48.
    //  DSP48 ma zarejestrowane wyjście (pipeline stage), co dawałoby bl_comb
    //  opóźniony o 1 takt — błąd dla pierwszej grupy okna.
    //  Ścieżka jest aktywna tylko przez 1 takt (przejście BASELINE→INTEGRATE),
    //  więc LUT jest tutaj właściwym wyborem.
    // =========================================================
    logic [3:0]            n_bl_groups;
    (* use_dsp = "no" *)
    logic [30:0]           bl_prod;       // 19-bit × 11-bit = 30-bit
    logic [ADC_BITS-1:0]   baseline_comb;

    assign n_bl_groups   = ws_base_r[IDX_BITS-1:2];
    assign bl_prod       = (bl_sum >> 2) * {1'b0, inv_lut[n_bl_groups]};
    assign baseline_comb = ADC_BITS'(bl_prod >> INV_SHIFT);

    // =========================================================
    //  Aktywny baseline:
    //    - podczas przejścia BASELINE→INTEGRATE: używaj baseline_comb
    //      (wire, gotowy od razu, zanim baseline_est się zarejestruje)
    //    - w pozostałych stanach: używaj zarejestrowanego baseline_est
    //  Dzięki temu pierwsza grupa okna (przy ws_base) jest poprawnie
    //  obliczona bez dodatkowego taktu opóźnienia.
    // =========================================================
    logic [ADC_BITS-1:0]  baseline_active;
    assign baseline_active = (state == BASELINE && sample_idx == ws_base_r)
                             ? baseline_comb
                             : baseline_est;

    // =========================================================
    //  Odejmowanie baseline i clip do 0
    // =========================================================
    logic [12:0] sigs [0:3];
    logic [11:0] sc   [0:3];
    always_comb begin
        for (int k = 0; k < 4; k++) begin
            sigs[k] = {1'b0, sample_word[k]} - {1'b0, baseline_active};
            sc[k]   = sigs[k][12] ? 12'd0 : sigs[k][11:0];
        end
    end

    // =========================================================
    //  Wagi c0..c3 z w_lut (bez komparatorów — brak timing violation)
    // =========================================================
    logic [ACC_BITS-1:0] c [0:3];

    assign c[0] = (w_lut[sample_idx    ] == 2'd0) ? '0 :
                  (w_lut[sample_idx    ] == 2'd1) ? ACC_BITS'(sc[0]) : ACC_BITS'(sc[0]) << 1;
    assign c[1] = (w_lut[sample_idx + 1] == 2'd0) ? '0 :
                  (w_lut[sample_idx + 1] == 2'd1) ? ACC_BITS'(sc[1]) : ACC_BITS'(sc[1]) << 1;
    assign c[2] = (w_lut[sample_idx + 2] == 2'd0) ? '0 :
                  (w_lut[sample_idx + 2] == 2'd1) ? ACC_BITS'(sc[2]) : ACC_BITS'(sc[2]) << 1;
    assign c[3] = (w_lut[sample_idx + 3] == 2'd0) ? '0 :
                  (w_lut[sample_idx + 3] == 2'd1) ? ACC_BITS'(sc[3]) : ACC_BITS'(sc[3]) << 1;

    logic [ACC_BITS-1:0] grp_delta;
    assign grp_delta = c[0] + c[1] + c[2] + c[3];

    // =========================================================
    //  FSM — główna maszyna stanów
    // =========================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= IDLE;
            sample_idx   <= '0;
            acc          <= '0;
            ws_base_r    <= '0;
            we_base_r    <= '0;
            bl_sum       <= '0;
            baseline_est <= '0;
            baseline_out <= '0;
            charge       <= '0;
            done         <= 1'b0;
            for (int i = 0; i < 64; i++) w_lut[i] <= 2'd0;

        end else begin
            done       <= 1'b0;
            sample_idx <= sample_idx + IDX_BITS'(RATIO);

            case (state)

                // ── IDLE ──────────────────────────────────────────────────
                IDLE: begin
                    if (start) begin
                        sample_idx   <= '0;
                        acc          <= '0;
                        bl_sum       <= '0;
                        baseline_est <= '0;
                        ws_base_r    <= {window_start[IDX_BITS-1:2], 2'b00};
                        we_base_r    <= {window_end  [IDX_BITS-1:2], 2'b00};

                        for (int i = 0; i < 64; i++) begin
                            if (i < int'(window_start) || i > int'(window_end))
                                w_lut[i] <= 2'd0;
                            else if (i == int'(window_start) || i == int'(window_end))
                                w_lut[i] <= 2'd1;
                            else
                                w_lut[i] <= 2'd2;
                        end

                        state <= BASELINE;
                    end
                end

                // ── BASELINE ──────────────────────────────────────────────
                BASELINE: begin
                    if (sample_idx < ws_base_r) begin
                        bl_sum <= bl_sum
                                  + 21'(sample_word[0])
                                  + 21'(sample_word[1])
                                  + 21'(sample_word[2])
                                  + 21'(sample_word[3]);
                    end else begin
                        baseline_est <= baseline_comb;
                        baseline_out <= baseline_comb;
                        acc   <= grp_delta;
                        state <= (ws_base_r == we_base_r) ? DONE_ST : INTEGRATE;
                    end
                end

                // ── INTEGRATE ─────────────────────────────────────────────
                INTEGRATE: begin
                    acc <= acc + grp_delta;
                    if (sample_idx == we_base_r)
                        state <= DONE_ST;
                end

                // ── DONE_ST ───────────────────────────────────────────────
                DONE_ST: begin
                    charge <= acc >> 1;
                    done   <= 1'b1;
                    state  <= IDLE;
                end

                default: state <= IDLE;

            endcase
        end
    end

endmodule
