module ibex_rfcache import ibex_pkg::*; #(
  parameter logic [31:0]          BootRegFile       = '0,
  parameter int                   NumRegFiles       = 1,
  parameter regfile_e             RegFile           = RegFileFF,
  parameter bit                   RV32E             = 0,
  parameter int unsigned          DataWidth         = 32,
  parameter bit                   DummyInstructions = 0,
  parameter logic [DataWidth-1:0] WordZeroVal       = '0
) (
  // Clock and Reset
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  logic                 test_en_i,
  input  logic                 dummy_instr_id_i,

  // Internal data memory interface
  input logic                  data_req_i,
  output logic                 data_gnt_o,
  output logic                 data_rvalid_o,
  input logic                  data_we_i,
  input logic [3:0]            data_be_i,
  input logic [31:0]           data_addr_i,
  input logic [31:0]           data_wdata_i,
  output logic [31:0]          data_rdata_o,
  output logic                 data_err_o,

  // External data memory interface
  output logic                 data_req_o,
  input  logic                 data_gnt_i,
  input  logic                 data_rvalid_i,
  output logic                 data_we_o,
  output logic [3:0]           data_be_o,
  output logic [31:0]          data_addr_o,
  output logic [31:0]          data_wdata_o,
  input  logic [31:0]          data_rdata_i,
  input  logic                 data_err_i,

  input logic [31:0]           rf_sel_i,
  output logic                 rf_busy_o,

  // Register file interface
  // Read port R1
  input  logic [          4:0] rf_raddr_a_i,
  output logic [DataWidth-1:0] rf_rdata_a_o,
  // Read port R2
  input  logic [          4:0] rf_raddr_b_i,
  output logic [DataWidth-1:0] rf_rdata_b_o,
  // Write port W1
  input  logic [          4:0] rf_waddr_a_i,
  input  logic [DataWidth-1:0] rf_wdata_a_i,
  input  logic                 rf_we_a_i
);

  logic [4:0]           rf_raddr_c;
  logic [DataWidth-1:0] rf_rdata_c;

  logic [31:0] active_rf_q, active_rf_d;

  logic has_req_q, has_req_d;

  assign has_req_d = data_req_i && data_addr_i[31:4] == active_rf_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      active_rf_q <= BootRegFile;
      has_req_q <= '0;
    end else begin
      active_rf_q <= active_rf_d;
      has_req_q <= has_req_d;
    end
  end

  typedef enum {normal, spill_req, spill_wait, load_req, load_wait} state_t;
  state_t state_q, state_d;

  logic [4:0] reg_count_q, reg_count_d;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= normal;
      reg_count_q <= '0;
    end else begin
      state_q <= state_d;
      reg_count_q <= reg_count_d;
    end
  end

  assign rf_busy_o = state_q != normal;

  logic [4:0]  rf_waddr;
  logic [31:0] rf_wdata;
  logic rf_we;

  always_comb begin
    state_d = state_q;
    active_rf_d = active_rf_q;
    reg_count_d = reg_count_q;

    rf_waddr = rf_waddr_a_i;
    rf_wdata = rf_wdata_a_i;
    rf_we    = rf_we_a_i;

    data_req_o = '0;
    data_gnt_o = '0;
    data_rvalid_o = '0;
    data_we_o = '0;
    data_be_o = '0;
    data_addr_o = '0;
    data_wdata_o = '0;
    data_rdata_o = '0;
    data_err_o = '0;
    rf_raddr_c = data_addr_i[6:2];

    unique case (state_q)
      normal: begin
        if (rf_sel_i != active_rf_q) begin
          state_d = spill_req;
          reg_count_d = '0;
        end else begin
          if (has_req_d) begin
            data_gnt_o = 1'b1;
          end

          if (has_req_q) begin
            if (data_we_i) begin
              data_err_o = 1'b1;
            end else begin
              data_rdata_o = rf_rdata_c;
              data_err_o = 1'b0;
              data_rvalid_o = 1'b1;
            end
          end else begin
            data_req_o = data_req_i;
            data_gnt_o = data_gnt_i;
            data_rvalid_o = data_rvalid_i;
            data_we_o = data_we_i;
            data_be_o = data_be_i;
            data_addr_o = data_addr_i;
            data_wdata_o = data_wdata_i;
            data_rdata_o = data_rdata_i;
            data_err_o = data_err_i;
          end
        end
      end
      spill_req: begin
        data_req_o = 1'b1;
        data_gnt_o = 1'b1;
        data_we_o = 1'b1;
        data_addr_o = active_rf_q + (reg_count_q << 2);
        data_be_o = 4'hF;
        data_wdata_o = rf_rdata_c;
        rf_raddr_c = reg_count_q;
        state_d = spill_wait;
      end
      spill_wait: begin
        if (data_rvalid_i) begin
          if (reg_count_q == 31) begin
            reg_count_d = '0;
            state_d = load_req;
          end else begin
            reg_count_d = reg_count_q + 1;
            state_d = spill_req;
          end
        end
      end
      load_req: begin
        data_req_o = 1'b1;
        data_gnt_o = 1'b1;
        data_addr_o = rf_sel_i + (reg_count_q << 2);
        data_be_o = 4'hF;
        state_d = load_wait;
      end
      load_wait: begin
        if (data_rvalid_i) begin
          if (reg_count_q == 31) begin
            state_d = normal;
            active_rf_d = rf_sel_i;
          end else begin
            state_d = load_req;
          end
          reg_count_d = reg_count_q + 1;
          rf_waddr = reg_count_q;
          rf_wdata = data_rdata_i;
          rf_we = 1'b1;
        end
      end
    endcase
  end

  ibex_register_file #(
    .RegFile           (RegFile),
    .RV32E             (RV32E),
    .DataWidth         (DataWidth),
    .DummyInstructions (DummyInstructions),
    .WordZeroVal       (WordZeroVal)
  ) register_file (
    .clk_i (clk_i),
    .rst_ni(rst_ni),

    .test_en_i       (test_en_i),
    .dummy_instr_id_i(dummy_instr_id_i),

    .raddr_a_i(rf_raddr_a_i),
    .rdata_a_o(rf_rdata_a_o),
    .raddr_b_i(rf_raddr_b_i),
    .rdata_b_o(rf_rdata_b_o),
    .raddr_c_i(rf_raddr_c),
    .rdata_c_o(rf_rdata_c),
    .waddr_a_i(rf_waddr),
    .wdata_a_i(rf_wdata),
    .we_a_i   (rf_we)
  );
endmodule
