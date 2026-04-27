package tl_c_pkg;

  // Permission Transitions (Param fields)
  // For N-to-T subset
  typedef enum logic [2:0] {
    TO_T = 3'h0, // Grow to Trunk
    TO_N = 3'h2, // Grow to None (Not used in A, but good to know)
    N_TO_T = 3'h0, // ProbeAck/Release transition
    T_TO_N = 3'h1, // ProbeAck/Release transition
    N_TO_N = 3'h5,
    T_TO_T = 3'h3
  } tl_param_e;


  // Channel A Opcodes
  typedef enum logic [2:0] {
    AcquireBlock = 3'h6,
    AcquirePerm = 3'h7
  } tl_a_op_e;

  // Channel B Opcodes
  typedef enum logic [2:0] {
    ProbeBlock = 3'h6,
    ProbePerm = 3'h7
  } tl_b_op_e;

  // Channel C Opcodes
  typedef enum logic [2:0] {
    ProbeAck = 3'h4,
    ProbeAckData= 3'h5,
    Release = 3'h6,
    ReleaseData = 3'h7
  } tl_c_op_e;

  // Channel D Opcodes
  typedef enum logic [2:0] {
    Grant  = 3'h4,
    GrantData = 3'h5,
    ReleaseAck = 3'h6
  } tl_d_op_e;

  // Channel E does not actually have an opcode field in the TileLink spec!
  // It only sends a "sink ID" to acknowledge the Channel D grant.

endpackage : tl_c_pkg