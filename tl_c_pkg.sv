package tl_c_pkg;

  // Permission Transitions (Param fields)
  // TileLink Permissions:
  // N (None)   = Invalid (No read/write access)
  // B (Branch) = Shared (Read-only access)
  // T (Trunk)  = Modified/Exclusive (Read/Write access)
  typedef enum logic [2:0] {
    // Channel A (Acquire) / Channel C (Release) Grow/Shrink requests
    TO_T   = 3'h0, // Grow to Trunk (Read/Write)
    TO_B   = 3'h1, // Grow to Branch (Read-Only) - Not used in our N-to-T subset
    TO_N   = 3'h2, // Shrink to None (Invalid)
    
    // Specific Permission Transitions for Channel C/D
    N_TO_T = 3'h0, // Client had None, wants/granted Trunk
    T_TO_N = 3'h1, // Client had Trunk, downgrading to None
    N_TO_N = 3'h5, // ProbeAck (Had None, staying None)
    T_TO_T = 3'h3  // ProbeAck (Had Trunk, staying Trunk - usually for informational probes)
  } tl_param_e;


  // Channel A Opcodes
  typedef enum logic [2:0] {
    AcquireBlock = 3'h6, // Request Manager for a data block
    AcquirePerm  = 3'h7  // Request Manager for a permission change (e.g., Read to Write) without needing the data
  } tl_a_op_e;
  /* Channel A Example
    Client (Master) ask the Manager (Client) for data or permissions.
    e.g., core/L1 wants to write to a cache line it doesn't have, so it send an AcquireBlock request.
  */


  // Channel B Opcodes
  typedef enum logic [2:0] {
    ProbeBlock = 3'h6, // Ask Client to return modified data (T permission)
    ProbePerm = 3'h7   // Ask Client to downgrade its permissions (T -> N)
  } tl_b_op_e;
  /* Channel B Example
    Manager asks the Client to downgrade its permissions or return modified data.
    e.g., another core/L1 wants a cache line that your L1 currently holds, so the shared L2 sends a ProbeBlock to your L1)
  */


  // Channel C Opcodes
  typedef enum logic [2:0] {
    ProbeAck     = 3'h4, // INVOLUNTARY: Ack Channel B Probe (Cache line was Clean or Invalid)
    ProbeAckData = 3'h5, // INVOLUNTARY: Return Dirty Data to Manager due to Probe 
    Release      = 3'h6, // VOLUNTARY: Release clean cache line to Manager (Eviction)
    ReleaseData  = 3'h7  // VOLUNTARY: Release dirty cache line to Manager (Eviction)
  } tl_c_op_e;
  /* Channel C Example
    Client VOLUNTARILY writes back dirty data (ReleaseData), OR INVOLUNTARILY responds to a Channel B Probe (ProbeAckData).
  */


  // Channel D Opcodes
  typedef enum logic [2:0] {
    Grant  = 3'h4,      // Grant Client's data block or permissions request
    GrantData = 3'h5,   // Grant data to Client
    ReleaseAck = 3'h6   // Ack to channel C's voluntary release
  } tl_d_op_e;
  /* Channel D Example
    Manager responds to a Channel A request by giving the Client the data (GrantData) and permissions. 
    It also acknowledges Channel C releases (ReleaseAck).
  */


  // Channel E does not actually have an opcode field in the TileLink spec!
  // It only sends a "sink ID" to acknowledge the Channel D grant.
  /* Channel E Example
    (GrantAck): Client tells the Manager "I have successfully received the Channel D Grant data." 
    This finalizes the 3-hop transaction (A $\rightarrow$ D $\rightarrow$ E) so the Manager can free up its trackers.
  */


endpackage : tl_c_pkg