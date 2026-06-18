`ifndef CACHE_MACROS_SVH
`define CACHE_MACROS_SVH

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

`endif // CACHE_MACROS_SVH
