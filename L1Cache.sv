`timescale 1ps/1ps

// This is a module implementing VIPT L1 cache featuring MSHR (Miss Status Handling Register) and Coalescing Store Buffer. 
// Eviction Policy: This design adopts a mathematichal tree-based PLRU policy with generic number of ways support


`define CacheLineSize 64   // 64 bytes per line
`define L1CacheSize 2**12 // 4KB
`define NUM_OF_LINES L1CacheSize/CacheLineSize //2**6 = 64 cache lines
`define NUM_OF_WAYS 4     // 4-way associative cache
`define NUM_OF_SETS  L1CacheSize/ (NUM_OF_WAYS*CacheLineSize)

`define INDEX_BITS $clog2(NUM_OF_SETS)
`define OFFSET_BITS $clog2(CacheLineSize)
`define TAG_BITS XLEN - INDEX_BITS - OFFSET_BITS

`define BLOCK_ID_SIZE TAG_BITS+INDEX_BITS

`define MSHR_SIZE 16
`define TARGET_LIST_SIZE 4
`define STORE_BUFFER_SIZE 4
`define PLRU_BITS_SIZE NUM_OF_WAYS-1

`define XLEN 32 
`define LSQ_SIZE 16

module L1Cache (
// global signals
input clk,
input rst_n,
input flush,

// Core input signals (from CPU, MMU/TLB)
input [XLEN-1:0] virtual_address_in, // Provides Virtual Index & Offset
input [TAG_BITS-1:0] physical_tag_in, // From MMU/TLB (arrives at least 1 cycle later)
input physical_tag_valid_in,          
input [$clog2(LSQ_SIZE)-1:0] lsq_tag_in, // To be stored in MSHR target list and returned with data for retirement

// CPU output signals (to CPU)
output TLB_physical_tag_valid_out, // when the tag translation is ready from TLB, can be used as stall signal for LSQ pipeline

// Load interface
input load_req_in,
input load_byte_en_in,
output load_port_stall_out, // stall
output [XLEN-1:0] load_data_out,
output load_data_valid_out,
output logic [LSQ_SIZE-1:0] lsq_wakeup_vector_out, // wake up the corresponding lsq entry in the same order as target list

// Store interface
input store_req_in,
input [XLEN-1:0] store_data_in,
input [3:0] store_byte_en_in, 
output store_port_stall_out, // stall

// L2 interface 
// cache block request
output [BLOCK_ID_SIZE-1:0] L2_req_block_id_out, 
output L2_block_req_out,
input [BLOCK_ID_SIZE-1:0] L2_return_block_id_in,
input [CacheLineSize-1:0] L2_return_block_data_in,
// cache block return
input L2_return_block_valid_in,

// cache block eviction
output L2_evict_block_valid_out, // Write-back dirty lines
output [BLOCK_ID_SIZE-1:0] L2_evict_block_id_out,
output [CacheLineSize-1:0][7:0] L2_evict_block_data_out
);

// Cacheline struct
typedef struct packed {
// logic [`NUM_OF_STATE:0] state; // Assuming L1 is private; this will be present in L2
logic valid;
logic dirty;
logic [TAG_BITS-1:0] tag;
logic [CacheLineSize-1:0][7:0] data; // 64 bytes per line

} CacheLine_t;

CacheLine_t [$clog2(NUM_OF_WAYS)-1:0] L1Cache_inst [NUM_OF_SETS-1:0];

// Decode request address
logic [TAG_BITS-1:0] extracted_tag;
logic [INDEX_BITS-1:0] extracted_index;
logic [OFFSET_BITS-1:0] extracted_offset;
assign extracted_tag = physical_tag_in; // VIPT: Tag comes from Physical Address (TLB)
assign extracted_index = virtual_address_in[OFFSET_BITS +: INDEX_BITS]; // VIPT: Index from Virtual Addr
assign extracted_offset = virtual_address_in[0 +: OFFSET_BITS];

logic [BLOCK_ID_SIZE-1:0] extracted_block_id;
assign extracted_block_id = {extracted_tag, extracted_index};

// MSHR struct
typedef struct packed {
    logic valid = 0;
    logic [BLOCK_ID_SIZE-1:0] block_id; // We need tag and index to match a cache line
    logic [$clog2(LSQ_SIZE)-1:0] target_list [TARGET_LIST_SIZE-1:0];
    logic [$clog2(TARGET_LIST_SIZE)-1:0] target_list_ptr = 0; // pointer to the front of the target list
} MSHR_t;

MSHR_t MSHR_inst [MSHR_SIZE-1:0];

// Store Buffer struct
typedef struct packed {
    logic valid;
    logic [BLOCK_ID_SIZE-1:0] block_id;
    logic [CacheLineSize-1:0][7:0] store_data; // Coalesced write data
    logic [CacheLineSize-1:0] store_byte_en; // Coalesced byte enable for the whole cache line
} Store_Buffer_t;

Store_Buffer_t Store_Buffer_inst [MSHR_SIZE-1:0];


// MSHR Signals for Checking on Cacheline Return
logic MSHR_hit; // find a MSHR waiting for the same cache line
logic [$clog2(MSHR_SIZE)-1:0] MSHR_hit_ID;
// MSHR Signals for Allocation 
logic [$clog2(MSHR_SIZE)-1:0] MSHR_alloc_ID; // the allocated MSHR ID
logic MSHR_AVAILABLE; // no MSHR available
// L2 Cache Block Return logic
logic [TAG_BITS-1:0] L2_return_tag;
logic [INDEX_BITS-1:0] L2_return_index;
assign L2_return_tag = L2_return_block_id_in[(BLOCK_ID_SIZE-1)-:TAG_BITS];
assign L2_return_index = L2_return_block_id_in[INDEX_BITS-1:0];

// Eviction logic
logic [$clog2(NUM_OF_WAYS)-1:0] Eviction_Target_Way; // Target way to be evicted, chosen by Tree-based PLRU
logic [PLRU_BITS_SIZE-1:0] PLRU_bits[NUM_OF_SETS-1:0];
logic Cache_hit;
logic [$clog2(NUM_OF_WAYS)-1:0] Cache_hit_Way;
logic [INDEX_BITS-1:0] hit_index; // Latches the index to match the 1-cycle delay of Cache_hit

//Store Buffer Signals
logic store_req_passthrough; 
logic Store_Buffer_ID_Alloc;
assign Store_Buffer_ID_Alloc = MSHR_alloc_ID;

// Collision Detection: True if a request matches the block being written by L2 this exact cycle
logic Collision_Stall;
assign Collision_Stall = (L2_return_block_valid_in && L2_return_block_id_in == extracted_block_id);

// Cache logic (load, store, miss)
always_ff @(posedge clk or negedge rst_n) begin : CACHE_LOGIC
    if(!rst_n) begin
        load_data_out <= 'd0;
        load_port_stall_out <= 1'b0;
        store_port_stall_out <= 1'b0;
        load_data_valid_out <= 1'b0;
        L2_block_req_out <= 1'b0;
        L2_evict_block_valid_out <= 1'b0;
        L2_evict_block_id_out <= '0;
        L2_evict_block_data_out <= '0;
        lsq_wakeup_vector_out <= '0;
        for(int i=0;i<NUM_OF_SETS;i=i+1) begin
            for(int j=0;j<NUM_OF_WAYS;j=j+1) begin
                L1Cache_inst[i][j] <= '{default: '0};
            end
        end
        for(int i=0;i<MSHR_SIZE;i=i+1) begin
            MSHR_inst[i] <= '{default: '0};
        end
        for(int i=0;i<STORE_BUFFER_SIZE;i=i+1) begin
            Store_Buffer_inst[i] <= '{default: '0};
        end
        Cache_hit <= 1'b0;
        Cache_hit_Way <= '0;
        hit_index <= '0;
    end
    else begin
        Cache_hit <= 1'b0;
        lsq_wakeup_vector_out <= '0; // Default to 0 to create 1-cycle pulses
        L2_evict_block_valid_out <= 1'b0; // Default to 0
        if(physical_tag_valid_in) begin
            if(load_req_in) begin // On req_in, use virtual_index to get the set and wait for (at least) one cycle for the physical tag from the TLB.
                L2_block_req_out <= 1'b0;
                load_port_stall_out <= 1'b0;
                load_data_valid_out <= 1'b0;
                if(Collision_Stall) begin
                    load_port_stall_out <= 1'b1; // Stall for 1 cycle to let L2 fill finish, next cycle will be a clean hit
                end
                else if(!MSHR_hit) begin // No MSHR is waiting for the tag, check if any way in the set is holding the line
                    for(int i=0;i<NUM_OF_WAYS;i=i+1) begin // Cache hit in the set
                        if(L1Cache_inst[extracted_index][i].valid && L1Cache_inst[extracted_index][i].tag == extracted_tag) begin
                            load_data_out <= L1Cache_inst[extracted_index][i].data;
                            load_data_valid_out <= 1'b1; // Hit in the L1Cache!
                            load_port_stall_out <= 1'b0;
                            Cache_hit <= 1'b1;
                            Cache_hit_Way <= i;
                            hit_index <= extracted_index; // Latch index for delayed PLRU update
                            break;
                        end
                        
                        if(i == (NUM_OF_WAYS-1)) begin
                            // Cache line not found in the set, need to allocate a MSHR
                            if (MSHR_AVAILABLE) begin
                                MSHR_inst[MSHR_alloc_ID].block_id <= extracted_block_id;
                                MSHR_inst[MSHR_alloc_ID].valid <= 1'b1;
                                MSHR_inst[MSHR_alloc_ID].target_list_ptr <= 'd1;
                                MSHR_inst[MSHR_alloc_ID].target_list[0] <= lsq_tag_in;
                                L2_block_req_out <= 1'b1;
                                L2_req_block_id_out <= extracted_block_id;
                            end
                            else begin
                                // No MSHR is free, need to stall
                                load_port_stall_out <= 1'b1; // assuming lsq handles this approprioately, e.g. holding the same request until stall is de-asserted
                            end
                        end
                    end
                end
                else begin // Found the MSHR waiting for the same tag, add the coming lsq_tag to the target list
                    if(MSHR_inst[MSHR_hit_ID].target_list_ptr < TARGET_LIST_SIZE) begin
                        MSHR_inst[MSHR_hit_ID].target_list[(MSHR_inst[MSHR_hit_ID].target_list_ptr)] <= lsq_tag_in;
                        MSHR_inst[MSHR_hit_ID].target_list_ptr <= MSHR_inst[MSHR_hit_ID].target_list_ptr + 1;
                    end
                    else begin
                        //The MSHR's target list is full, so we need to stall the read port
                        load_port_stall_out <= 1'b1;                
                    end
                end
            end
            else if(store_req_in) begin
                L2_block_req_out <= 1'b0;
                store_port_stall_out <= 1'b0; 
                store_req_passthrough <= 1'b0; 
                if(Collision_Stall) begin
                    store_port_stall_out <= 1'b1; // Stall for 1 cycle to let L2 fill finish, next cycle will be a clean hit
                end
                else if(!MSHR_hit) begin // No MSHR is waiting for the tag, check if any way in the set is holding the line
                    for(int i=0;i<NUM_OF_WAYS;i=i+1) begin // Cache hit in the set
                        if(L1Cache_inst[extracted_index][i].valid && L1Cache_inst[extracted_index][i].tag == extracted_tag) begin
                            if(store_byte_en_in[0]) L1Cache_inst[extracted_index][i].data[extracted_offset] <= store_data_in[7:0];
                            if(store_byte_en_in[1]) L1Cache_inst[extracted_index][i].data[extracted_offset+1] <= store_data_in[15:8];
                            if(store_byte_en_in[2]) L1Cache_inst[extracted_index][i].data[extracted_offset+2] <= store_data_in[23:16];
                            if(store_byte_en_in[3]) L1Cache_inst[extracted_index][i].data[extracted_offset+3] <= store_data_in[31:24];
                            L1Cache_inst[extracted_index][i].dirty <= 1'b1;
                            Cache_hit <= 1'b1;
                            Cache_hit_Way <= i;
                            hit_index <= extracted_index; // Latch index for delayed PLRU update
                            break;
                        end

                        if(i == (NUM_OF_WAYS-1)) begin
                            // Cache line not found in the set, need to allocate a MSHR
                            if (MSHR_AVAILABLE) begin
                                MSHR_inst[MSHR_alloc_ID].block_id <= {extracted_tag, extracted_index};
                                MSHR_inst[MSHR_alloc_ID].valid <= 1'b1;
                                MSHR_inst[MSHR_alloc_ID].target_list_ptr <= 'd1;
                                MSHR_inst[MSHR_alloc_ID].target_list[0] <= lsq_tag_in;
                                L2_block_req_out <= 1'b1;
                                L2_req_block_id_out <= extracted_block_id;
                                Store_Buffer_inst[Store_Buffer_ID_Alloc].valid <= 1'b1;
                                Store_Buffer_inst[Store_Buffer_ID_Alloc].block_id <= extracted_block_id;   
                                Store_Buffer_inst[Store_Buffer_ID_Alloc].store_byte_en[extracted_offset +:4] <= Store_Buffer_inst[Store_Buffer_ID_Alloc].store_byte_en[extracted_offset +:4] | store_byte_en_in;         
                                if(store_byte_en_in[0]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset] <= store_data_in[7:0];
                                if(store_byte_en_in[1]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset+1] <= store_data_in[15:8];
                                if(store_byte_en_in[2]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset+2] <= store_data_in[23:16];
                                if(store_byte_en_in[3]) Store_Buffer_inst[Store_Buffer_ID_Alloc].store_data[extracted_offset+3] <= store_data_in[31:24];
                            end
                            else begin
                                // No MSHR is free, need to stall
                                store_port_stall_out <= 1'b1;  
                            end    
                        end 
                    end        
                end
                else begin // Found the MSHR waiting for the same tag, pass the store req to store buffer (no need to be added to target list)
                    Store_Buffer_inst[MSHR_hit_ID].valid <= 1'b1; // MSHR shares the same ID with Store Buffer
                    Store_Buffer_inst[MSHR_hit_ID].block_id <= extracted_block_id;   
                    Store_Buffer_inst[MSHR_hit_ID].store_byte_en[extracted_offset +:4] <= Store_Buffer_inst[MSHR_hit_ID].store_byte_en[extracted_offset +:4] | store_byte_en_in;         
                    if(store_byte_en_in[0]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset] <= store_data_in[7:0];
                    if(store_byte_en_in[1]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset+1] <= store_data_in[15:8];
                    if(store_byte_en_in[2]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset+2] <= store_data_in[23:16];
                    if(store_byte_en_in[3]) Store_Buffer_inst[MSHR_hit_ID].store_data[extracted_offset+3] <= store_data_in[31:24];
                end
            end
        end

        // Return Block handling - replace a block in the set and retire corresponding MSHR and Store buffer
        if(L2_return_block_valid_in) begin
            
            // NOTE: No handshaking to de-assert the L2_return_block_valid_in; assuming it's high for only a cycle because L1 is always ready for return block 
            // NOTE: When the evicted block happens to be the newly requested block by CPU, the PLRU won't update it to reflect this to prevent it from being evicted 

            // Block Eviction - 1. write back if dirty 2. move in the return block
            if(L1Cache_inst[L2_return_index][Eviction_Target_Way].valid && L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty) begin
                L2_evict_block_valid_out <= 1'b1;  // L2 must mark it dirty
                L2_evict_block_id_out <= {L1Cache_inst[L2_return_index][Eviction_Target_Way].tag, L2_return_index};
                L2_evict_block_data_out <= L1Cache_inst[L2_return_index][Eviction_Target_Way].data;
            end
            L1Cache_inst[L2_return_index][Eviction_Target_Way].tag <= L2_return_tag;
            L1Cache_inst[L2_return_index][Eviction_Target_Way].data <= L2_return_block_data_in;
            L1Cache_inst[L2_return_index][Eviction_Target_Way].valid <= 1'b1;
            L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty <= 1'b0;
            
            // MSHR & Store Buffer Retirement
            for(int i=0;i<MSHR_SIZE;i=i+1) begin
                if(MSHR_inst[i].valid && MSHR_inst[i].block_id == L2_return_block_id_in) begin // found an MSHR waiting for this returned block (CAM)
                   
                    // 1. Retire MSHR: Ask LSQ's lsq_tag entries to send request again, since the block has just arrived and reset the MSHR entry
                    MSHR_inst[i].valid <= 1'b0;     
                    for(int j=0;j<TARGET_LIST_SIZE;j=j+1) begin
                        if(MSHR_inst[i].target_list[j] != 'd0) begin
                            lsq_wakeup_vector_out[MSHR_inst[i].target_list[j]] <= 1'b1; // wake up the corresponding lsq entry
                            MSHR_inst[i].target_list[j] <= 'd0; // clear the target list entry after wake up
                        end
                        if(j == MSHR_inst[i].target_list_ptr - 1) break;
                    end
                    MSHR_inst[i].target_list_ptr <= 'd0;
                    
                    // 2. Retire Store Buffer: Merge the coalesced data from Store buffer with the L2 return block
                    // NOTE: Assuming LSQ has dealt with disambiguation, so we can store the data in without concerning the MSHR's loads
                    if(Store_Buffer_inst[i].valid) begin
                        for(int k=0; k<CacheLineSize; k=k+1) begin
                            if(Store_Buffer_inst[i].store_byte_en[k]) begin
                                L1Cache_inst[L2_return_index][Eviction_Target_Way].data[k] <= Store_Buffer_inst[i].store_data[k];
                            end
                        end
                        L1Cache_inst[L2_return_index][Eviction_Target_Way].dirty <= 1'b1; // Mark dirty since we modified it
                        Store_Buffer_inst[i].valid <= 1'b0;
                        Store_Buffer_inst[i].store_data <= 'd0;
                        Store_Buffer_inst[i].store_byte_en <= 'd0;
                    end
                    break;
                end
            end
        end

    end

end


// MSHR Allocation logic
always_comb begin : MSHR_ALLOC
    MSHR_AVAILABLE = 1'b0;
    MSHR_alloc_ID = 'd0;
    for (int i=0;i<MSHR_SIZE;i=i+1) begin
        if(!MSHR_inst[i].valid) begin
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
    for (int i=0;i<MSHR_SIZE;i=i+1) begin
        if(MSHR_inst[i].valid && MSHR_inst[i].block_id == extracted_block_id) begin
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
logic [$clog2(PLRU_BITS_SIZE)-1:0] plru_update_idx[$clog2(NUM_OF_WAYS)-1:0];
logic [$clog2(PLRU_BITS_SIZE)-1:0] evict_plru_update_idx[$clog2(NUM_OF_WAYS)-1:0];
// plru_update_idx[i+1] = 2*plru_update_idx[i]+2**Cache_hit_Way[$clog2(NUM_OF_WAYS)-1-i];
always_comb begin
    for (int i=0;i<$clog2(NUM_OF_WAYS);i=i+1) begin
        plru_update_idx[i] = '0;
        evict_plru_update_idx[i] = '0;
    end
    for(int i=0;i<$clog2(NUM_OF_WAYS)-1;i++) begin
        // Generic index tracking for Hit Update
        if(Cache_hit_Way[$clog2(NUM_OF_WAYS)-1-i]) begin
            plru_update_idx[i+1] = 2*plru_update_idx[i]+2;
        end
        else begin
            plru_update_idx[i+1] = 2*plru_update_idx[i]+1;
        end

        // Generic index tracking for Eviction Update
        if(Eviction_Target_Way[$clog2(NUM_OF_WAYS)-1-i]) begin
            evict_plru_update_idx[i+1] = 2*evict_plru_update_idx[i]+2;
        end
        else begin
            evict_plru_update_idx[i+1] = 2*evict_plru_update_idx[i]+1;
        end
    end
end

// Tree-based PLRU
always_ff @(posedge clk or negedge rst_n) begin : UPDATE_PLRU_TREE
    if(!rst_n) begin
        for(int i = 0; i < NUM_OF_SETS; i = i + 1) PLRU_bits[i] <= '0;
    end else begin
        // Implementation for PLRU eviction logic on Hit
        if(Cache_hit) begin
            // Point the tree AWAY from the accessed way
            for(int i=0;i<$clog2(NUM_OF_WAYS);i=i+1) begin
                PLRU_bits[hit_index][plru_update_idx[i]] <= ~Cache_hit_Way[$clog2(NUM_OF_WAYS)-1-i];
            end
        end

        // Implementation for PLRU eviction logic on Cache Fill (Miss Return)
        if(L2_return_block_valid_in) begin
            // Point the tree AWAY from the newly filled way
            for(int i=0;i<$clog2(NUM_OF_WAYS);i=i+1) begin
                PLRU_bits[L2_return_index][evict_plru_update_idx[i]] <= ~Eviction_Target_Way[$clog2(NUM_OF_WAYS)-1-i];
            end
        end
    end
end

// Evicted Target Selection Logic
always_comb begin : Eviction_Target
    integer node_idx;
    
    node_idx = 0; // Start at root node
    
    // Traverse down the tree levels (log2 of ways)
    for (int i = 0; i < $clog2(NUM_OF_WAYS); i = i + 1) begin
        // If current bit is 0, point to left child (2*idx + 1)
        // If current bit is 1, point to right child (2*idx + 2)
        if (PLRU_bits[L2_return_index][node_idx] == 1'b0)
            node_idx = 2 * node_idx + 1;
        else
            node_idx = 2 * node_idx + 2;
    end
    
    // Map the leaf node index back to a Way index (0 to NUM_OF_WAYS-1)
    // The first leaf in a binary tree starts at index (NUM_OF_WAYS - 1)
    Eviction_Target_Way = node_idx - (NUM_OF_WAYS - 1);
end

assign TLB_physical_tag_valid_out = physical_tag_valid_in; // Directly use the TLB's physical tag valid signal as the stall signal for LSQ pipeline. 

endmodule
