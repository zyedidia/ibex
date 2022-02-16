module ibex_rfcache import ibex_pkg::*; #(
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

  input logic [31:0]           rf_sel_i,

  //Read port R1
  input  logic [          4:0] raddr_a_i,
  output logic [DataWidth-1:0] rdata_a_o,
  //Read port R2
  input  logic [          4:0] raddr_b_i,
  output logic [DataWidth-1:0] rdata_b_o,
  // Write port W1
  input  logic [          4:0] waddr_a_i,
  input  logic [DataWidth-1:0] wdata_a_i,
  input  logic                 we_a_i
);
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

    .raddr_a_i(raddr_a_i),
    .rdata_a_o(rdata_a_o),
    .raddr_b_i(raddr_b_i),
    .rdata_b_o(rdata_b_o),
    .waddr_a_i(waddr_a_i),
    .wdata_a_i(wdata_a_i),
    .we_a_i   (we_a_i)
  );
endmodule
