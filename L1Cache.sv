`timescale 1ps/1ps

// This is a module implementing VIPT L1 cache featuring MSHR (Miss Status Handling Register) and Coalescing Store Buffer. 
// Eviction Policy: This design adopts a mathematichal tree-based PLRU policy with generic number of ways support
// Inteface with L2 is now simplified

// 32-bit memory addressing is assumed
/* 
    Note that:
    In standard Sv32, the PPN is 22 bits, supporting 34-bit physical addresses.
    Sv32 satp register format: [31] MODE, [30:22] ASID, [21:0] PPN
    However, this implementation only supports 32-bit physical addresses (PABITS=32).
    Bits [21:20] of the PPN are ignored/hardwired to 0 per implementation.
    The whole memory subsystem addressing is designed accordingly.
*/


`define CacheLineSize 64   // 64 bytes per line
`define L1CacheSize 2**12 // 4KB
`define NUM_OF_LINES `L1CacheSize/`CacheLineSize //2**6 = 64 cache lines
`define NUM_OF_WAYS 4     // 4-way associative cache
`define NUM_OF_SETS  `L1CacheSize/ (`NUM_OF_WAYS*`CacheLineSize) // 16 sets

`define INDEX_BITS $clog2(`NUM_OF_SETS)             // = 4
`define OFFSET_BITS $clog2(`CacheLineSize)          // = 6
`define TAG_BITS `XLEN - `INDEX_BITS - `OFFSET_BITS // = 32 - 4 - 6 = 22
`define BLOCK_ID_SIZE `TAG_BITS+`INDEX_BITS         // = 22 + 4 = 26
`define PAGE_ID_SIZE 20                      // = 20 (32-12=20; a page is 2^12 = 4KB)
// When TAG_BITS < PAGE_ID_SIZE, there will be aliasing (INDEX_BITS or NUM_OF_WAYS are too large)

`define MSHR_SIZE 16                                // 16-entry MSHR (4 bits)
`define TARGET_LIST_SIZE 4                          // 4-entry Target List (2 bits)
`define STORE_BUFFER_SIZE 4                         // 4-entry Store Buffer (2 bits)    
`define PLRU_BITS_SIZE `NUM_OF_WAYS-1               // = How many bits to construct the PLRU Tree

`define XLEN 32                                     // RV32
`define LSQ_SIZE 16                                 // 16-entry LSQ in CPU

import tl_c_pkg::*;                                 // Import TileLink Package (opcodes and permissions)

module L1Cache (
// global signals
input clk,
input rst_n,
input flush,

// Core input signals (from CPU, MMU/TLB)
input [`XLEN-1:0] virtual_address_in, // Provides Virtual Index & Offset
input [`TAG_BITS-1:0] physical_page_id_in, // From MMU/TLB (arrives at least 1 cycle later)
input physical_tag_valid_in,          
input [$clog2(`LSQ_SIZE)-1:0] lsq_tag_in, // To be stored in MSHR target list and returned with data for retirement

// CPU output signals (to CPU)
output TLB_physical_tag_valid_out, // when the tag translation is ready from TLB, can be used as stall signal for LSQ pipeline

// L1 is ready in IDLE state (to CPU)
output L1_ready_out,

// Load interface
input load_req_in,
output logic [`XLEN-1:0] load_data_out,
output logic load_data_valid_out,
output logic [`LSQ_SIZE-1:0] lsq_wakeup_vector_out, // wake up the corresponding lsq entry in the same order as target list

// Store interface
input store_req_in,
input [`XLEN-1:0] store_data_in,
input [3:0] store_byte_en_in, 

// TileLink-C Interface (Client Modport)
tl_c_interface.client tl_bus
);

// Cacheline struct
typedef struct packed {
logic valid;
logic dirty;
logic [`TAG_BITS-1:0] tag;
logic [`CacheLineSize-1:0][7:0] data; // 64 bytes per line
logic [1:0] TL_Perm; // Tilelink's permissions (states); only N, T permissions are supported; B is ignored

} CacheLine_t;

CacheLine_t [$clog2(`NUM_OF_WAYS)-1:0] L1Cache_inst [`NUM_OF_SETS-1:0];

// MSHR struct
typedef struct packed {
    logic busy;
    logic [`BLOCK_ID_SIZE-1:0] block_id; // We need the tag (and index) to match a cache line
    logic [`TARGET_LIST_SIZE-1:0][$clog2(`LSQ_SIZE)-1:0] target_list;
    logic [$clog2(`TARGET_LIST_SIZE)-1:0] target_list_ptr; // pointer to the front of the target list
    //logic issued; // Has this entried been issued?
} MSHR_t;

MSHR_t MSHR_inst [`MSHR_SIZE-1:0];

// MSHR Issue Queue struct
typedef struct packed {
    logic [$clog2(`MSHR_SIZE)-1:0] a_source;  // MSHR ID
    tl_a_op_e                      a_opcode;  // AcquireBlock or AcquirePerm
    tl_param_e                     a_param;   // Request for permission change from N to T
    logic [`XLEN-1:0]              a_address; // Physical address for L2, since it is PIPT
    logic [3-1:0]                  a_size;    // (optional) fixed 2^6 = 64 bytes cache block size           
} MSHR_Issue_Queue_t;

MSHR_Issue_Queue_t MSHR_Issue_Queue_inst[`MSHR_SIZE-1:0]; // = the number of MSHR entries

// Store Buffer struct
typedef struct packed {
    logic busy;
    logic [`BLOCK_ID_SIZE-1:0] block_id;
    logic [`CacheLineSize-1:0][7:0] store_data; // Coalesced write data
    logic [`CacheLineSize-1:0] store_byte_en; // Coalesced byte enable for the whole cache line
} Store_Buffer_t;

Store_Buffer_t Store_Buffer_inst [`MSHR_SIZE-1:0];


logic ALIASING;
assign ALIASING = (`TAG_BITS < `PAGE_ID_SIZE);

// req type passthrough
logic req_type_passthrough; // 0:load, 1:store
logic [`XLEN-1:0] store_data_passthrough;
logic [3:0] store_byte_en_passthrough;
logic [$clog2(`LSQ_SIZE)-1:0] lsq_tag_passthrough;


// Decode request address
logic [`OFFSET_BITS-1:0] extracted_offset;
logic [`INDEX_BITS-1:0] extracted_index;
logic [`TAG_BITS-1:0] extracted_tag;
logic [`BLOCK_ID_SIZE-1:0] extracted_block_id;
logic [`XLEN-1:0] extracted_physical_address; // For PIPT L2
assign extracted_index = virtual_address_in[`OFFSET_BITS +: `INDEX_BITS]; // Virtual Index  (= Physical Index if no aliasing)
assign extracted_offset = virtual_address_in[0 +: `OFFSET_BITS];      
assign extracted_tag = `ALIASING ? {physical_page_id_in[(`PAGE_ID_SIZE-1) -: `TAG_BITS]} : {physical_page_id_in, virtual_address_passthrough[(`OFFSET_BITS +`INDEX_BITS)+:`TAG_BITS - `PAGE_ID_SIZE]}; // Physical Tag
assign extracted_block_id = {extracted_tag, extracted_index_passthrough};             // Unique Cache Block ID for L1 Cache 
assign extracted_physical_address = {physical_page_id_in, virtual_address_passthrough[PAGE_ID_SIZE-1:0]};

// extracted address passthrough
logic [`XLEN-1:0] virtual_address_passthrough;
logic [`TAG_BITS-1:0] extracted_tag_passthrough;
logic [`INDEX_BITS-1:0] extracted_index_passthrough;
logic [`OFFSET_BITS-1:0] extracted_offset_passthrough;
logic [`BLOCK_ID_SIZE-1:0] extracted_block_id_passthrough;
logic [`XLEN-1:0] extracted_physical_address_passthrough;

// MSHR Issue Queue Signals
logic [$clog2(`MSHR_SIZE)-1:0] MSHR_Issue_Queue_head;
logic [$clog2(`MSHR_SIZE)-1:0] MSHR_Issue_Queue_tail;
logic MSHR_Issue_Queue_full, MSHR_Issue_Queue_empty;
assign MSHR_Issue_Queue_full  = (MSHR_Issue_Queue_head == MSHR_Issue_Queue_tail + 1);
assign MSHR_Issue_Queue_empty = (MSHR_Issue_Queue_head == MSHR_Issue_Queue_tail);

// MSHR Signals
logic MSHR_hit; // find a MSHR waiting for the same cache line
logic [$clog2(`MSHR_SIZE)-1:0] MSHR_hit_ID;
logic [$clog2(`MSHR_SIZE)-1:0] MSHR_alloc_ID; // the allocated MSHR ID
logic MSHR_AVAILABLE; // no MSHR available
logic TARGET_LIST_AVAIABLE;
assign TARGET_LIST_AVAILABLE = MSHR_inst[MSHR_hit_ID].target_list_ptr < `TARGET_LIST_SIZE;


// L2 Cache Block Return logic
logic [`TAG_BITS-1:0] L2_return_tag;
logic [`INDEX_BITS-1:0] L2_return_index;
assign L2_return_tag   = MSHR_inst[tl_bus.d_source].block_id[`BLOCK_ID_SIZE-1:`INDEX_BITS];
assign L2_return_index = MSHR_inst[tl_bus.d_source].block_id[`INDEX_BITS-1:0];

// Eviction logic
logic [$clog2(`NUM_OF_WAYS)-1:0] Eviction_Target_Way;       // PLRU core output - Target way to be evicted
logic [`PLRU_BITS_SIZE-1:0] PLRU_bits[`NUM_OF_SETS-1:0];    // PLRU logic
logic Cache_hit;                                            // PLRU logic
logic [$clog2(`NUM_OF_WAYS)-1:0] Cache_hit_Way;             // PLRU logic
logic [`INDEX_BITS-1:0] hit_index;                          // Latches the index to match the 1-cycle delay of Cache_hit

logic WRITEBACK_REQUIRED;                           // The target evicted block is dirty and required to be writen back
logic [`XLEN-1:0] evicted_block_physical_address;   // Physical address to L2 (L2 is PIPT)
logic [`CacheLineSize-1:0][7:0] evicted_block_data; // The cache block data to be written back to L2

assign WRITEBACK_REQUIRED = (L1Cache_inst[L2_return_index][Eviction_Target_Way].valid && L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty)
assign evicted_block_physical_address = ALIASING? {26'h0 , 6'h0}:{L1Cache_inst[L2_return_index][Eviction_Target_Way].tag, L2_return_index, 6'h0}; // ignore Aliasing case now
assign evicted_block_data = L1Cache_inst[L2_return_index][Eviction_Target_Way].data;

logic [`XLEN-1:0] evicted_block_physical_address_passthrough;
logic [`CacheLineSize-1:0][7:0] evicted_block_data_passthrough; 

logic Return_Block_Hit;

// Store Buffer Signals
logic Store_Buffer_ID_Alloc;
assign Store_Buffer_ID_Alloc = MSHR_alloc_ID;

// Cache Logic FSM
typedef enum logic[2:0] {
    IDLE,
    WAIT_TLB,
    WAIT_BLOCK,
    SETTLE_REQ
    // LOAD_DATA_FORMATTING (byte selection; skipped in this design)
} FSM_state_t;

FSM_state_t [2:0] state;
FSM_state_t [2:0] next_state;
logic d_handshake; // Note that handshake here doesn't mean d_valid == d_ready
logic c_handshake; // Note that handshake here doesn't mean c_valid == c_ready
assign d_handshake = Block_Return_state == WAIT_BLOCK_RETURN_D && tl_bus.d_valid;  
assign c_handshake = Block_Return_state == WAIT_WRITEBACK_C && tl_bus.c_ready;

always_comb begin : FSM_next_state
    L1_ready_out   = 1'b0;
    tl_bus.d_ready = 1'b0;
    next_state     = state;

    case(state)
        IDLE: begin
            if(load_req_in || store_req_in) begin // Assume req_in depends on L1_ready_out
                next_state   = WAIT_TLB;
            end
            else begin
                L1_ready_out = 1'b1;
            end
        end
        
        WAIT_TLB: begin
            if(physical_tag_valid_in) begin
                if(d_handshake && MSHR_inst[tl_bus.d_source].block_id == extracted_block_id) begin
                    // if the returned block happens to be the requested block when physical tag comes in
                    // go to Settle_REQ (try it again after filling in the cache block)
                    next_state = SETTLE_REQ; 
                end
                else begin
                    if(req_type_passthrough == 0) begin
                        // MSHR Found or MSHR Allocated -> return to IDLE state to assert ready for new request
                        if(Cache_Hit || MSHR_HIT && TARGET_LIST_AVAILABLE || (!MSHR_HIT && MSHR_AVAILABLE)) begin 
                            next_state = IDLE;
                        end
                        else begin
                            next_state = WAIT_BLOCK;
                        end
                    end
                    else begin
                        if(Cache_Hit || MSHR_HIT || (!MSHR_HIT && MSHR_AVAILABLE)) begin
                            next_state = IDLE;
                        end
                        else begin
                            next_state = WAIT_BLOCK;
                        end
                    end
                end
            end
        end

        WAIT_BLOCK: begin
            tl_bus.d_ready = 1'b1;
            if(d_handshake) begin
                tl_bus.d_ready = 1'b0;
                if(req_type_passthrough == 0 && MSHR_inst[tl_bus.d_source].block_id == extracted_block_id_passthrough) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = SETTLE_REQ;                               
                end
            end
        end


        SETTLE_REQ: begin
            // write to the just-released MSHR
            next_state = IDLE;
        end

        default: begin
            next_state = state;
        end

    endcase
end

always_ff @(posedge clk or negedge rst_n) begin : FSM_state_update
    if(!rst_n || flush) begin
        state <= IDLE;
        extracted_index_passthrough            <= '0;
        extracted_offset_passthrough           <= '0;
        virtual_address_passthrough            <= '0;
        extracted_tag_passthrough              <= '0;
        extracted_block_id_passthrough         <= '0;
        extracted_physical_address_passthrough <= '0;
        req_type_passthrough                   <= '0;
        store_data_passthrough                 <= '0;
        store_byte_en_passthrough              <= '0;
        lsq_tag_passthrough                    <= '0;
    end
    else begin
        state <= next_state;
        if(load_req_in || store_req_in) begin
            extracted_index_passthrough  <= extracted_index;
            extracted_offset_passthrough <= extracted_offset;
            virtual_address_passthrough  <= virtual_address_in; // req_in and virtual_address_in are transient for one cycle
            if(load_req_in) begin 
                req_type_passthrough <= 1'b0; // keep req type throughout the entire request FSM lifecycle 
                lsq_tag_passthrough  <= lsq_tag_in;
            end
            else begin
                req_type_passthrough <= 1'b1; // keep req type throughout the entire request FSM lifecycle
                store_data_passthrough <= store_data_in;
                store_byte_en_passthrough <= store_byte_en_in;
            end
        end
        if(physical_tag_valid_in) begin // latch the translated address for potential stalls in the pipeline
            extracted_tag_passthrough              <= extracted_tag;
            extracted_block_id_passthrough         <= extracted_block_id;
            extracted_physical_address_passthrough <= extracted_physical_address;
        end
    end
end

// Cache Block Return Logic FSM
typedef enum logic {
    WAIT_BLOCK_RETURN_D,
    WAIT_WRITEBACK_C
} FSM_Block_Return_state_t;

FSM_Block_Return_state_t [1:0] Block_Return_state;
FSM_Block_Return_state_t [1:0] Block_Return_next_state;
logic WAIT_READY_E; // flag to indicate waiting for Master e_ready signal after receiving returned block
logic [SINK_WIDTH-1:0] d_sink_passthrough;

always_comb begin: FSM_Block_Return_next_state
    Block_Return_next_state = Block_Return_state;
    tl_bus.d_ready = 1'b1;
    case(Block_Return_state)
        WAIT_BLOCK_RETURN_D: begin // Assume d_valid depends on d_ready
            if(tl_bus.d_valid && (WRITEBACK_REQUIRED && !tl_bus.c_ready)) begin
                tl_bus.d_ready = 1'b0;
                Block_Return_next_state = WAIT_WRITEBACK_C; // Wait for channel C ready signal to write back 
            end
        end
        WAIT_WRITEBACK_C: begin
            tl_bus.d_ready = 1'b0;
            if(tl_bus.c_ready) begin
                Block_Return_next_state = WAIT_BLOCK_RETURN_D; // Go back to wait block return
            end
        end
        default: begin
            Block_Return_next_state = Block_Return_state;
        end
    endcase
end
always_ff @(posedge clk or negedge rst_n) begin : FSM_Block_Return_state_update
    if(!rst_n || flush) begin
        Block_Return_state <= WAIT_BLOCK_RETURN_D;
        WAIT_READY_E       <= 1'b0;
    end
    else begin
        Block_Return_state <= Block_Return_next_state;
        tl_bus.e_valid <= 1'b0;
        if(d_handshake) begin // Change to WAIT_WRITEBACK_C state on posedge
            if(tl_bus.e_ready) begin
                tl_bus.e_valid <= 1'b1;
                tl_bus.e_sink  <= tl_bus.d_sink;
            end
            else begin
                d_sink_passthrough <= tl_bus.d_sink;
                WAIT_READY_E   <= 1'b1;
            end
        end

        if(WAIT_READY_E) begin
            if(tl_bus.e_ready) begin
                tl_bus.e_valid <= 1'b1;
                tl_bus.e_sink  <= d_sink_passthrough;
                WAIT_READY_E   <= 1'b0;
            end
        end
    end
end


// Cache logic (load, store, miss)
always_ff @(posedge clk or negedge rst_n) begin : CACHE_LOGIC
    if(!rst_n || flush) begin
        load_data_out            <= '0;
        load_data_valid_out      <= 1'b0;
        tl_bus.a_valid           <= 1'b0;
        lsq_wakeup_vector_out    <= '0;
        MSHR_Issue_Queue_head <= 'd0;

        for(int i=0;i<`NUM_OF_SETS;i=i+1) begin
            for(int j=0;j<`NUM_OF_WAYS;j=j+1) begin
                L1Cache_inst[i][j] <= '{default: '0};
            end
        end
        for(int i=0;i<`MSHR_SIZE;i=i+1) begin
            MSHR_inst[i] <= '{default: '0};
        end
        for(int i=0;i<`STORE_BUFFER_SIZE;i=i+1) begin
            Store_Buffer_inst[i] <= '{default: '0};
        end
        Cache_hit           <= 1'b0;
        Cache_hit_Way       <= '0;
        hit_index           <= '0;
        Return_Block_Hit    <= 1'b0;

    end
    else begin
        Cache_hit                <= 1'b0;
        lsq_wakeup_vector_out    <= '0; // Default to 0 to create 1-cycle pulses
        tl_bus.a_valid           <= 1'b0; // Default to 0 to create 1-cycle pulses
        load_data_valid_out      <= 1'b0;
        Return_Block_Hit         <= 1'b0;

        case(state) // On req_in, use virtual_index to get the set and wait for (at least) one cycle for the physical tag from the TLB.
            WAIT_TLB: begin
                if(physical_tag_valid_in) begin
                    if(d_handshake && MSHR_inst[tl_bus.d_source].block_id == extracted_block_id) begin
                        // next_state is SETTLE_REQ
                        Return_Block_Hit <= 1'b1;
                    end
                    else begin // Check if Channel D is returning the brequested lock
                        if(req_type_passthrough == 0) begin
                            if(!MSHR_hit) begin // No MSHR is waiting for the tag, check if any way in the set is holding the cache line
                                for(int i=0;i<`NUM_OF_WAYS;i=i+1) begin 
                                    // Search if the requested address is in the Cache 
                                    if(L1Cache_inst[extracted_index_passthrough][i].valid && L1Cache_inst[extracted_index_passthrough][i].tag == extracted_tag) begin
                                        load_data_out       <= L1Cache_inst[extracted_index_passthrough][i].data;
                                        load_data_valid_out <= 1'b1;
                                        Cache_hit           <= 1'b1; // Cache hit in the set
                                        Cache_hit_Way       <= i;
                                        hit_index           <= extracted_index_passthrough; // Latch index for delayed PLRU update
                                        break;
                                    end
                                end
                                if(i == (`NUM_OF_WAYS-1)) begin // Cache line was not found in the set, we need to allocate a MSHR
                                    if (MSHR_AVAILABLE) begin
                                        MSHR_inst[MSHR_alloc_ID].block_id        <= extracted_block_id; // Record the unique block ID for L1  
                                        MSHR_inst[MSHR_alloc_ID].target_list_ptr <= 'd1;
                                        MSHR_inst[MSHR_alloc_ID].target_list[0]  <= lsq_tag_passthrough;

                                        // TileLink Interface - Push to MSHR Issue Queue for AcquireBlock Request through A channel
                                        if(tl_bus.a_ready && MSHR_Issue_Queue_empty) begin 
                                            // If no other MSHR is already waiting in the queue, issue the request to L2 directly
                                            tl_bus.a_valid   <= 1'b1;
                                            tl_bus.a_param   <= tl_param_e.N_TO_T;      // Request for permission change from N to T
                                            tl_bus.a_opcode  <= tl_a_op_e.AcquireBlock;
                                            tl_bus.a_address <= extracted_physical_address; // Send the physical address since L2 is PIPT
                                            tl_bus.a_size    <= 3'h6;                   // 2^6 = 64 bytes cache block size
                                            tl_bus.a_source  <= MSHR_alloc_ID;          // MSHR ID for returning the cache block
                                        end
                                        else if(!MSHR_Issue_Queue_full) begin // MSHR_Issue_Queue_full won't actually happen when MSHR_AVAILABLE is true
                                            // Push to MSHR Issue Queue
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_source  <= MSHR_alloc_ID;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_param   <= tl_param_e.N_TO_T;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_opcode  <= tl_a_op_e.AcquireBlock;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_address <= extracted_physical_address;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_size    <= 3'h6;
                                            MSHR_Issue_Queue_tail <= (MSHR_Issue_Queue_tail + 1);                                   
                                        end                              
                                    end
                                end            
                            end
                            else begin // Found the MSHR waiting for the same tag, add the coming lsq_tag to the target list
                                if(TARGET_LIST_AVAILABLE) begin
                                    MSHR_inst[MSHR_hit_ID].target_list[(MSHR_inst[MSHR_hit_ID].target_list_ptr)] <= lsq_tag_passthrough;
                                    MSHR_inst[MSHR_hit_ID].target_list_ptr <= MSHR_inst[MSHR_hit_ID].target_list_ptr + 1;
                                end
                            end
                        
                        end
                        else begin // req_type_passthrough == 1 (store request)
                            if(!MSHR_hit) begin // No MSHR is waiting for the tag, check if any way in the set is holding the line
                                for(int i=0;i<`NUM_OF_WAYS;i=i+1) begin // Cache hit in the set
                                    if(L1Cache_inst[extracted_index_passthrough][i].valid && L1Cache_inst[extracted_index_passthrough][i].tag == extracted_tag) begin
                                        if(store_byte_en_passthrough[0]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough]   <= store_data_passthrough[7:0];
                                        if(store_byte_en_passthrough[1]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+1] <= store_data_passthrough[15:8];
                                        if(store_byte_en_passthrough[2]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+2] <= store_data_passthrough[23:16];
                                        if(store_byte_en_passthrough[3]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+3] <= store_data_passthrough[31:24];
                                        L1Cache_inst[extracted_index_passthrough][i].dirty <= 1'b1;
                                        Cache_hit     <= 1'b1;
                                        Cache_hit_Way <= i;
                                        hit_index     <= extracted_index_passthrough; // Latch index for delayed PLRU update
                                        break;
                                    end
                                end
                                if(i == (`NUM_OF_WAYS-1)) begin
                                    // Cache line not found in the set, need to allocate a MSHR
                                    if (MSHR_AVAILABLE) begin
                                        MSHR_inst[MSHR_alloc_ID].block_id        <= extracted_block_id;
                                        MSHR_inst[MSHR_alloc_ID].busy            <= 1'b1;
                                        MSHR_inst[MSHR_alloc_ID].target_list_ptr <= 'd0;
                                        if(tl_bus.a_ready && MSHR_Issue_Queue_empty) begin 
                                            // If no other MSHR is already waiting in the queue, issue the request to L2 directly
                                            tl_bus.a_valid   <= 1'b1;
                                            tl_bus.a_param   <= tl_param_e.N_TO_T;          // Request for permission change from N to T
                                            tl_bus.a_opcode  <= tl_a_op_e.AcquireBlock;
                                            tl_bus.a_address <= extracted_physical_address; // Send the physical address since L2 is PIPT
                                            tl_bus.a_size    <= 3'h6;                       // 2^6 = 64 bytes cache block size
                                            tl_bus.a_source  <= MSHR_alloc_ID;              // MSHR ID for returning the cache block
                                        end
                                        else if(!MSHR_Issue_Queue_full) begin // MSHR_Issue_Queue_full won't actually happen when MSHR_AVAILABLE is true
                                            // Push to MSHR Issue Queue
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_source  <= MSHR_alloc_ID;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_param   <= tl_param_e.N_TO_T;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_opcode  <= tl_a_op_e.AcquireBlock;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_address <= extracted_physical_address;
                                            MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_size    <= 3'h6;
                                            MSHR_Issue_Queue_tail <= (MSHR_Issue_Queue_tail + 1);                                   
                                        end 
                                        
                                        Store_Buffer_inst[Store_Buffer_ID_Alloc].busy      <= 1'b1;
                                        Store_Buffer_inst[Store_Buffer_ID_Alloc].block_id  <= extracted_block_id;   
                                        Store_Buffer_inst[Store_Buffer_ID_Alloc].store_byte_en[extracted_offset_passthrough +:4] <= Store_Buffer_inst[Store_Buffer_ID_Alloc].store_byte_en[extracted_offset_passthrough +:4] | store_byte_en_passthrough;         
                                        if(store_byte_en_passthrough[0]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset_passthrough]   <= store_data_passthrough[7:0];
                                        if(store_byte_en_passthrough[1]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset_passthrough+1] <= store_data_passthrough[15:8];
                                        if(store_byte_en_passthrough[2]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset_passthrough+2] <= store_data_passthrough[23:16];
                                        if(store_byte_en_passthrough[3]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset_passthrough+3] <= store_data_passthrough[31:24];
                                    end
                                end          
                            end
                            else begin // Found the MSHR waiting for the same tag, pass the store req to store buffer (no need to be added to target list)
                                Store_Buffer_inst[MSHR_hit_ID].busy     <= 1'b1; 
                                Store_Buffer_inst[MSHR_hit_ID].block_id <= extracted_block_id;   
                                Store_Buffer_inst[MSHR_hit_ID].store_byte_en[extracted_offset_passthrough +:4] <= Store_Buffer_inst[MSHR_hit_ID].store_byte_en[extracted_offset_passthrough +:4] | store_byte_en_passthrough;         
                                if(store_byte_en_passthrough[0]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset_passthrough]   <= store_data_passthrough[7:0];
                                if(store_byte_en_passthrough[1]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset_passthrough+1] <= store_data_passthrough[15:8];
                                if(store_byte_en_passthrough[2]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset_passthrough+2] <= store_data_passthrough[23:16];
                                if(store_byte_en_passthrough[3]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset_passthrough+3] <= store_data_passthrough[31:24];
                            end                        
                        end
                    end
                end
            end

            // The req waits here until Block Return
            // On block return we check if the returned cache block is what's requested, otherwise allocate the just-released MSHR to it in next cycle
            WAIT_BLOCK: begin 
                if(d_handshake && MSHR_inst[tl_bus.d_source].block_id == extracted_block_id_passthrough) begin
                    if(req_type_passthrough == 0) begin
                        // load the data here instead of in SETTLE_REQ state
                        load_data_out       <= tl_bus.d_data[8*(extracted_offset_passthrough + 4) - 1 : 8*extracted_offset_passthrough]; // Assume Word-Alignment: extracted_offset_passthrough > 60 is prohibited
                        load_data_valid_out <= 1'b1;
                        // will return to IDLE state in next cycle
                    end
                    else begin
                        // store must wait until filled
                        Return_Block_Hit    <= 1'b1; // This is used in SETTLE_REQ state
                    end
                end
            end

            SETTLE_REQ: begin
                if(Return_Block_Hit) begin // Requested block found in the returned block         
                    for(int i=0;i<`NUM_OF_WAYS;i=i+1) begin 
                        // Search for the requested address in the Cache 
                        if(L1Cache_inst[extracted_index_passthrough][i].valid && L1Cache_inst[extracted_index_passthrough][i].tag == extracted_tag_passthrough) begin
                            if(req_type_passthrough == 0) begin  // load request
                                load_data_out       <= L1Cache_inst[extracted_index_passthrough][i].data;
                                load_data_valid_out <= 1'b1;
                                Cache_hit           <= 1'b1; // Cache hit in the set
                                Cache_hit_Way       <= i;
                                hit_index           <= extracted_index_passthrough; // Latch index for delayed PLRU update
                                break;
                            end
                            else begin // store request
                                if(store_byte_en_passthrough[0]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough]   <= store_data_passthrough[7:0];
                                if(store_byte_en_passthrough[1]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+1] <= store_data_passthrough[15:8];
                                if(store_byte_en_passthrough[2]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+2] <= store_data_passthrough[23:16];
                                if(store_byte_en_passthrough[3]) L1Cache_inst[extracted_index_passthrough][i].data[extracted_offset_passthrough+3] <= store_data_passthrough[31:24];
                                L1Cache_inst[extracted_index_passthrough][i].dirty <= 1'b1;
                                Cache_hit     <= 1'b1;
                                Cache_hit_Way <= i;
                                hit_index     <= extracted_index_passthrough; // Latch index for delayed PLRU update
                                break;
                            end
                        end
                    end
                end
                else if(MSHR_AVAILABLE) begin // The returned block was not a hit, allocate a MSHR
                    MSHR_inst[MSHR_alloc_ID].block_id        <= extracted_block_id_passthrough; // Record the unique block ID for L1  
                    MSHR_inst[MSHR_alloc_ID].target_list_ptr <= 'd1;
                    MSHR_inst[MSHR_alloc_ID].target_list[0]  <= lsq_tag_passthrough;

                    // TileLink Interface - Push to MSHR Issue Queue for AcquireBlock Request through A channel
                    if(tl_bus.a_ready && MSHR_Issue_Queue_empty) begin 
                        // If no other MSHR is already waiting in the queue, issue the request to L2 directly
                        tl_bus.a_valid   <= 1'b1;
                        tl_bus.a_param   <= tl_param_e.N_TO_T;      // Request for permission change from N to T
                        tl_bus.a_opcode  <= tl_a_op_e.AcquireBlock;
                        tl_bus.a_address <= extracted_physical_address; // Send the physical address since L2 is PIPT
                        tl_bus.a_size    <= 3'h6;                   // 2^6 = 64 bytes cache block size
                        tl_bus.a_source  <= MSHR_alloc_ID;          // MSHR ID for returning the cache block
                    end
                    else if(!MSHR_Issue_Queue_full) begin // MSHR_Issue_Queue_full won't actually happen when MSHR_AVAILABLE is true
                        // Push to MSHR Issue Queue
                        MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_source  <= MSHR_alloc_ID;
                        MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_param   <= tl_param_e.N_TO_T;
                        MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_opcode  <= tl_a_op_e.AcquireBlock;
                        MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_address <= extracted_physical_address;
                        MSHR_Issue_Queue_inst[MSHR_Issue_Queue_tail].a_size    <= 3'h6;
                        MSHR_Issue_Queue_tail <= (MSHR_Issue_Queue_tail + 1);                                   
                    end                              
                end
            end
        endcase
        
        // Block Return Handling: Write-Back, Block Filling, MSHR & Store Buffer Retirement 
        if(d_handshake) begin
                // Dirty Block Eviction 
                if(WRITEBACK_REQUIRED) begin
                    if(tl_bus.c_ready) begin
                        // Channel C is ready: Write back dirty block when c_ready are asserted
                        tl_bus.c_valid   <= 1'b1;
                        tl_bus.c_address <= evicted_block_physical_address; // phyiscal address (L2 is PIPT)
                        tl_bus.c_data    <= evicted_block_data;    
                        tl_bus.c_size    <= 3'h6;
                        tl_bus.c_opcode  <= ReleaseData;
                        tl_bus.c_param   <= tl_param_e.T_TO_N;
                        // tl_bus.c_source?
                    end
                    else begin
                        // Channel C is not ready: buffer the dirty block so the returned block can be filled in this posedege
                        // The buffered evicted block waits in the WAIT_WRITEBACK_C state until c_ready is asserted
                        evicted_block_physical_address_passthrough <= evicted_block_physical_address;
                        evicted_block_data_passthrough             <= evicted_block_data;
                    end
                end
                // Block Filling
                L1Cache_inst[L2_return_index][Eviction_Target_Way].tag     <= L2_return_tag;
                L1Cache_inst[L2_return_index][Eviction_Target_Way].data    <= tl_bus.d_data;
                L1Cache_inst[L2_return_index][Eviction_Target_Way].valid   <= 1'b1;
                L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty   <= 1'b0;
                L1Cache_inst[L2_return_index][Eviction_Target_Way].TL_Perm <= 2'b01; // T Permission (Modified or Exclusive)

                // MSHR Retirement
                for(int i=0;i<`TARGET_LIST_SIZE;i=i+1) begin
                    // write to wakeup_vector_out to inform lsq of the returned block
                    lsq_wakeup_vector_out[MSHR_inst[tl_bus.d_source].target_list[i]] <= 1'b1;
                    if(i == MSHR_inst[tl_bus.d_source].target_list_ptr - 1) break;
                end
                MSHR_inst[tl_bus.d_source] <= '{default: '0};

                // Store Buffer Retirement
                if(Store_Buffer_inst[tl_bus.d_source].busy) begin
                    L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty <= 1'b1; // Mark dirty since we're storing data to it
                    for(int i=0; i<`CacheLineSize; i=i+1) begin
                        // store data to the newly returned block
                        if(Store_Buffer_inst[tl_bus.d_source].store_byte_en[i]) begin
                            L1Cache_inst[L2_return_index][Eviction_Target_Way].data[i] <= Store_Buffer_inst[tl_bus.d_source].store_data[i];
                        end
                    end
                    Store_Buffer_inst[tl_bus.d_source] <= '{default: '0};
                end
        end
        else if(c_handshake) begin // Write-Back Handling if not handled on block return because c_ready was not asserted
            tl_bus.c_valid   <= 1'b1;
            tl_bus.c_address <= evicted_block_physical_address_passthrough;
            tl_bus.c_data    <= evicted_block_data_passthrough;    
            tl_bus.c_size    <= 3'h6;
            tl_bus.c_opcode  <= ReleaseData;
            tl_bus.c_param   <= tl_param_e.T_TO_N;
        end


        // MSHR Issue Queue Logic
        if(!MSHR_Issue_Queue_empty && tl_bus.a_ready) begin
            // Issue head entry
            tl_bus.a_valid    <= 1'b1;
            tl_bus.a_param    <= MSHR_Issue_Queue_inst[MSHR_Issue_Queue_head].a_param;
            tl_bus.a_opcode   <= MSHR_Issue_Queue_inst[MSHR_Issue_Queue_head].a_opcode;
            tl_bus.a_address  <= MSHR_Issue_Queue_inst[MSHR_Issue_Queue_head].a_address;
            tl_bus.a_size     <= MSHR_Issue_Queue_inst[MSHR_Issue_Queue_head].a_size;
            tl_bus.a_source   <= MSHR_Issue_Queue_inst[MSHR_Issue_Queue_head].a_source;
            // Advance Head pointer (Assert valid for only one cycle)
            MSHR_Issue_Queue_head <= (MSHR_Issue_Queue_head + 1);
        end
        else begin
            tl_bus.a_valid <= 1'b0;
        end

    end
end



// MSHR Allocation logic
always_comb begin : MSHR_ALLOC
    MSHR_AVAILABLE = 1'b0;
    MSHR_alloc_ID = 'd0;
    for (int i=0;i<`MSHR_SIZE;i=i+1) begin
        if(!MSHR_inst[i].busy) begin
            MSHR_alloc_ID = i;
            MSHR_AVAILABLE = 1'b1;
            break;
        end
    end
end

// MSHR Hit Check logic
always_comb begin : MSHR_HIT
    MSHR_hit = 1'b0;
    MSHR_hit_ID = 'd0;
    for (int i=0;i<`MSHR_SIZE;i=i+1) begin
        if(MSHR_inst[i].busy && MSHR_inst[i].block_id == extracted_block_id) begin
            MSHR_hit = 1'b1;
            MSHR_hit_ID = i;
            break;
        end
    end
end


/// Cache Eviction Logic ///

// Path: {cycle 1: TLB Tag Arrival -> Cache Tag Compare (Cache_hit)} -> {cycle 2: PLRU Tree Traversal -> update PLRU SRAM bits} 
// The path coulb be further pipelined

/*
              bit[0]
            /       \      
        bit[1]      bit[2]    
        /   \        /    \
    bit[3]   bit[4] bit[5] bit[6]    
       
    0: turn left  (2i+1)
    1: turn right (2i+2)
*/


// Record the tree index to be updated; totally depth=$clog2(NUM_OF_WAYS) nodes to be updated
logic [$clog2(`PLRU_BITS_SIZE)-1:0] plru_update_idx[$clog2(`NUM_OF_WAYS)-1:0];
logic [$clog2(`PLRU_BITS_SIZE)-1:0] evict_plru_update_idx[$clog2(`NUM_OF_WAYS)-1:0];
// plru_update_idx[i+1] = 2*plru_update_idx[i]+2**Cache_hit_Way[$clog2(`NUM_OF_WAYS)-1-i];
always_comb begin
    for (int i=0;i<$clog2(`NUM_OF_WAYS);i=i+1) begin
        plru_update_idx[i] = '0;
        evict_plru_update_idx[i] = '0;
    end
    for(int i=0;i<$clog2(`NUM_OF_WAYS)-1;i++) begin
        // Generic index tracking for Hit Update
        if(Cache_hit_Way[$clog2(`NUM_OF_WAYS)-1-i]) begin
            plru_update_idx[i+1] = 2*plru_update_idx[i]+2;
        end
        else begin
            plru_update_idx[i+1] = 2*plru_update_idx[i]+1;
        end

        // Generic index tracking for Eviction Update
        if(Eviction_Target_Way[$clog2(`NUM_OF_WAYS)-1-i]) begin
            evict_plru_update_idx[i+1] = 2*evict_plru_update_idx[i]+2;
        end
        else begin
            evict_plru_update_idx[i+1] = 2*evict_plru_update_idx[i]+1;
        end
    end
end

// Tree-based PLRU
always_ff @(posedge clk or negedge rst_n) begin : UPDATE_PLRU_TREE
    if(!rst_n || flush) begin
        for(int i = 0; i < `NUM_OF_SETS; i = i + 1) PLRU_bits[i] <= '0;
    end else begin
        // Implementation for PLRU eviction logic on Hit
        if(Cache_hit) begin
            // Point the tree AWAY from the accessed way
            for(int i=0;i<$clog2(`NUM_OF_WAYS);i=i+1) begin
                PLRU_bits[hit_index][plru_update_idx[i]] <= ~Cache_hit_Way[$clog2(`NUM_OF_WAYS)-1-i];
            end
        end

        // Implementation for PLRU eviction logic on Cache Fill (Miss Return)
        if(d_handshake) begin
            // Point the tree AWAY from the newly filled way
            for(int i=0;i<$clog2(`NUM_OF_WAYS);i=i+1) begin
                PLRU_bits[L2_return_index][evict_plru_update_idx[i]] <= ~Eviction_Target_Way[$clog2(`NUM_OF_WAYS)-1-i];
            end
        end
    end
end

// Evicted Target Selection Logic
always_comb begin : Eviction_Target
    integer node_idx;
    
    node_idx = '0; // Start at root node
    
    // Traverse down the tree levels (log2 of ways)
    for (int i = 0; i < $clog2(`NUM_OF_WAYS); i = i + 1) begin
        // If current bit is 0, point to left child (2*idx + 1)
        // If current bit is 1, point to right child (2*idx + 2)
        if (PLRU_bits[L2_return_index][node_idx] == 1'b0)
            node_idx = 2 * node_idx + 1;
        else
            node_idx = 2 * node_idx + 2;
    end
    
    // Map the leaf node index back to a Way index (0 to NUM_OF_WAYS-1)
    // The first leaf in a binary tree starts at index (NUM_OF_WAYS - 1)
    Eviction_Target_Way = node_idx - (`NUM_OF_WAYS - 1);
end

assign TLB_physical_tag_valid_out = physical_tag_valid_in; // Directly use the TLB's physical tag valid signal as the stall signal for LSQ pipeline. 

endmodule
