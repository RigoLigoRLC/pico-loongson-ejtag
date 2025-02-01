
#pragma once

#include <stdint.h>

//==============================================================================
// Common - for extracting opcode
typedef struct {
    uint16_t reserved : 10;
    uint16_t op : 6;
} lsejtag_cmd_header_common;

typedef struct {
    lsejtag_cmd_header_common header;
} lsejtag_cmd_common;

//==============================================================================
// 0x01 - Probe memory space R/W
typedef struct {
    uint16_t is_write : 1;
    uint16_t reserved : 9;
    uint16_t op : 6;
} lsejtag_cmd_header_probe_mem_rw;

typedef struct __attribute((packed)) {
    lsejtag_cmd_header_probe_mem_rw header;
    uint32_t addr;
    uint32_t data_to_write; // OPTIONAL
} lsejtag_cmd_probe_mem_rw;

//==============================================================================
// 0x03 - Probe I/O port manipulation
typedef struct {
    uint16_t level : 1;
    uint16_t port_id : 7;
    uint16_t reserved : 2;
    uint16_t op : 6;
} lsejtag_cmd_header_io_manip;

typedef struct {
    lsejtag_cmd_header_io_manip header;
} lsejtag_cmd_io_manip;

//==============================================================================
// 0x04 - IR R/W
typedef struct {
    uint16_t reserved : 8;
    uint16_t immediate_send_back : 1;
    uint16_t read_back : 1;
    uint16_t op : 6;
} lsejtag_cmd_header_ir_rw;

typedef struct {
    lsejtag_cmd_header_ir_rw header;
    uint16_t irseq_len_bits;
    uint32_t irseq[0];
} lsejtag_cmd_ir_rw;

//==============================================================================
// 0x05 - DR R/W
typedef struct {
    uint16_t reserved : 8;
    uint16_t immediate_send_back : 1;
    uint16_t read_back : 1;
    uint16_t op : 6;
} lsejtag_cmd_header_dr_rw;

typedef struct {
    lsejtag_cmd_header_dr_rw header;
    uint16_t drseq_len_bits;
    uint32_t drseq[0];
} lsejtag_cmd_dr_rw;

//==============================================================================
// 0x08 - Loopback test
typedef struct {
    uint16_t reserved : 10;
    uint16_t op : 6;
} lsejtag_cmd_header_loopback_test;

typedef struct {
    lsejtag_cmd_header_loopback_test header;
    uint32_t number;
} lsejtag_cmd_loopback_test;

//==============================================================================
// 0x0c/0x0d - Fast target memory write
typedef struct {
    uint16_t chained_core_count : 7;
    uint16_t cpu_is_64bit : 1;
    uint16_t reserved : 2;
    uint16_t op : 6;
} lsejtag_cmd_header_fast_target_mem_write;

typedef struct __attribute((packed)) {
    lsejtag_cmd_header_fast_target_mem_write header;
    uint16_t jtag_clk_div;
    uint32_t data_len_dword_count;
    uint16_t at_cpu_core;
    uint32_t data[0];
} lsejtag_cmd_fast_target_mem_write_at_core;

typedef struct {
    lsejtag_cmd_header_fast_target_mem_write header;
    uint16_t jtag_clk_div;
    uint32_t data_len_dword_count;
    uint32_t data[0];
} lsejtag_cmd_fast_target_mem_write;

//==============================================================================
// 0x0e/0x0f - Fast target memory read
typedef struct {
    uint16_t chained_core_count : 7;
    uint16_t cpu_is_64bit : 1;
    uint16_t reserved : 2;
    uint16_t op : 6;
} lsejtag_cmd_header_fast_target_mem_read;

typedef struct __attribute((packed)) {
    lsejtag_cmd_header_fast_target_mem_read header;
    uint16_t jtag_clk_div;
    uint32_t data_len_dword_count;
    uint16_t at_cpu_core;
    uint32_t data[0];
} lsejtag_cmd_fast_target_mem_read_at_core;
