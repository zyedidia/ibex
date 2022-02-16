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

  always_comb begin
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
    .waddr_a_i(rf_waddr_a_i),
    .wdata_a_i(rf_wdata_a_i),
    .we_a_i   (rf_we_a_i)
  );
endmodule
