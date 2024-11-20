#ifndef _WMAP_H_
#define _WMAP_H_

#include "param.h"  // Ensure MAX_MAPPINGS is defined here or include it appropriately

// Flags for wmap
#define MAP_SHARED     0x0002
#define MAP_ANONYMOUS  0x0004
#define MAP_FIXED      0x0008

// When any system call fails, returns -1
#define FAILED         -1
#define SUCCESS         0

// for `getwmapinfo`
#define MAX_WMMAP_INFO 16
#define MAX_MAPPINGS   MAX_WMMAP_INFO // Ensure consistency

struct wmapinfo {
    int total_mmaps;                    // Total number of wmap regions
    int addr[MAX_WMMAP_INFO];           // Starting address of mapping
    int length[MAX_WMMAP_INFO];         // Size of mapping
    int n_loaded_pages[MAX_WMMAP_INFO]; // Number of pages physically loaded into memory
};

// Define the mapping structure
struct mapping {
    uint addr;              // Starting virtual address
    uint length;            // Length of the mapping
    int flags;              // Mapping flags (e.g., MAP_SHARED, MAP_ANONYMOUS)
    int fd;                 // File descriptor (for file-backed mappings)
    struct file *file;      // Pointer to the file (for file-backed mappings)
    int n_loaded_pages;     // Number of pages loaded
};

#endif // _WMAP_H_
