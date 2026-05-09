interface tl_c_interface #(
  parameter int ADDR_WIDTH   = 32,
  parameter int DATA_WIDTH   = 512, // 512-bit data bus
  parameter int SOURCE_WIDTH = 4, // The ID of the Client's transaction. Like ARID/AWID in AXI. (e.g., ID of the 16-entry L1 MSHR)
  parameter int SINK_WIDTH   = 4, // The ID of the Manager's transaction tracker.
  parameter int SIZE_WIDTH   = 3  // Size of the transaction (e.g., 2^6 = 64 bytes cache block)
);

  import tl_c_pkg::*;

  // ==========================================
  // Channel A: Acquire (Client -> Manager)
  // ==========================================
  logic                    a_valid;
  logic                    a_ready;
  logic [DATA_WIDTH-1:0]   a_data;
  logic [ADDR_WIDTH-1:0]   a_address;
  tl_a_op_e                a_opcode;
  tl_param_e               a_param;
  logic [SIZE_WIDTH-1:0]   a_size;
  logic [SOURCE_WIDTH-1:0] a_source;
  logic [(DATA_WIDTH/8)-1:0] a_mask;
  logic a_corrupt;

  // ==========================================
  // Channel B: Probe (Manager -> Client)
  // ==========================================
  logic                    b_valid;
  logic                    b_ready;
  logic [DATA_WIDTH-1:0]   b_data;
  logic [ADDR_WIDTH-1:0]   b_address;
  tl_b_op_e                b_opcode;
  tl_param_e               b_param;
  logic [SIZE_WIDTH-1:0]   b_size;
  logic [SOURCE_WIDTH-1:0] b_source;
  logic [(DATA_WIDTH/8)-1:0] b_mask;
  logic b_corrupt;


  // ==========================================
  // Channel C: Release/ProbeAck (Client -> Manager)
  // ==========================================
  logic                    c_valid;
  logic                    c_ready;
  logic [ADDR_WIDTH-1:0]   c_address;
  logic [DATA_WIDTH-1:0]   c_data;
  tl_c_op_e                c_opcode;
  tl_param_e               c_param;
  logic [SIZE_WIDTH-1:0]   c_size;
  logic [SOURCE_WIDTH-1:0] c_source;
  logic c_corrupt;

  // ==========================================
  // Channel D: Grant/ReleaseAck (Manager -> Client)
  // ==========================================
  logic                    d_valid;
  logic                    d_ready;
  logic [DATA_WIDTH-1:0]   d_data;
  tl_d_op_e                d_opcode;
  tl_param_e               d_param;
  logic [SIZE_WIDTH-1:0]   d_size;
  logic [SOURCE_WIDTH-1:0] d_source;
  logic [SINK_WIDTH-1:0]   d_sink;
  logic d_denied;
  logic d_corrupt;

  // ==========================================
  // Channel E: GrantAck (Client -> Manager)
  // ==========================================
  logic                    e_valid;
  logic                    e_ready;
  logic [SINK_WIDTH-1:0]   e_sink;

  // ==========================================
  // Modports
  // ==========================================
  
  // The L1 Cache uses the 'client' modport
  modport client (
    output a_valid, a_opcode, a_param, a_size, a_source, a_address, a_data, a_mask, a_corrupt,
    input  a_ready,
    input  b_valid, b_opcode, b_param, b_size, b_source, b_address, b_data, b_mask, b_corrupt,
    output b_ready,
    output c_valid, c_opcode, c_param, c_size, c_source, c_address, c_data, c_corrupt,
    input  c_ready,
    input  d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_data, d_denied, d_corrupt,
    output d_ready,
    output e_valid, e_sink,
    input  e_ready
  );

  // The L2 Cache / Interconnect uses the 'manager' modport
  modport manager ( /*exact inverse of client */ 
    input a_valid, a_opcode, a_param, a_size, a_source, a_address, a_data, a_mask, a_corrupt,
    output  a_ready,
    output  b_valid, b_opcode, b_param, b_size, b_source, b_address, b_data, b_mask, b_corrupt,
    input   b_ready,
    input   c_valid, c_opcode, c_param, c_size, c_source, c_address, c_data, c_corrupt,
    output  c_ready,
    output  d_valid, d_opcode, d_param, d_size, d_source, d_sink, d_data, d_denied, d_corrupt,
    input   d_ready,
    input   e_valid, e_sink,
    output  e_ready
  );

endinterface
