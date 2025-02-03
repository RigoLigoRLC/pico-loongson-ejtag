
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "cmd_headers.h"

// INTERNAL USE.
typedef enum {
    op_probe_mem_rw                     = 0x01,
    op_io_manip                         = 0x03,
    op_ir_rw                            = 0x04,
    op_dr_rw                            = 0x05,
    op_loopback_test                    = 0x08,
    op_fast_target_mem_write_fastdata   = 0x0c,
    op_fast_target_mem_write            = 0x0d,
    op_fast_target_mem_read_fastdata    = 0x0e,
    op_fast_target_mem_read             = 0x0f,
    op_get_firmware_date                = 0x1f,
} lsejtag_cmd_op;

// INTERNAL USE.
typedef enum {
    io_led,                 // Green LED. Default to low.
    io_fpga_reset,          // Non-used.
    io_oe,                  // JTAG output enable. When OE is low output is enabled. Default to low.
    io_trst,                // TRST pin. Default to high.
    io_brst,                // BRST pin. Default to high.
    io_dint,                // DINT pin. Default to high.
    io_tap_reset,           // Non-used.
    io_maximum,
} lsejtag_io;

// INTERNAL USE.
typedef struct {
    union {
        uint16_t u16;
        lsejtag_cmd_header_common common;
        lsejtag_cmd_header_probe_mem_rw probe_mem_rw;
        lsejtag_cmd_header_io_manip io_manip;
        lsejtag_cmd_header_ir_rw ir_rw;
        lsejtag_cmd_header_dr_rw dr_rw;
        lsejtag_cmd_header_loopback_test loopback_test;
        lsejtag_cmd_header_fast_target_mem_write fast_target_mem_write;
        lsejtag_cmd_header_fast_target_mem_read fast_target_mem_read;
    };
} lsejtag_cmd_header;

// INTERNAL USE.
typedef struct {
    union {
        lsejtag_cmd_common common;
        lsejtag_cmd_probe_mem_rw probe_mem_rw;
        lsejtag_cmd_io_manip io_manip;
        lsejtag_cmd_ir_rw ir_rw;
        lsejtag_cmd_dr_rw dr_rw;
        lsejtag_cmd_loopback_test loopback_test;
        lsejtag_cmd_fast_target_mem_write_at_core fast_mem_write_at_core;
        lsejtag_cmd_fast_target_mem_write fast_mem_write;
        lsejtag_cmd_fast_target_mem_read_at_core fast_mem_read_at_core;
    };
} lsejtag_cmd;

// INTERNAL USE. Command buffering state machine state.
typedef enum {
    cbs_clean,          // The buffer is clean, no pending commands
    cbs_wait_header,    // Pending command has only 1 byte available, command header is incomplete
    cbs_wait_cfg,       // Pending command's header is available, but config section is incomplete
    cbs_wait_payload,   // Pending command's header and config section is available, but payload
                        // is incomplete. In this case, `cmd_buf_wait_payload_size` represents how
                        // many bytes of payload data is yet to be read.
    cbs_execute,        // The buffer contains a valid command and awaits execution.
} lsejtag_cmd_buf_state;

typedef enum {
    dpr_execute,        // A command was successfully fetched, call `lsejtag_exec_cmd` immediately.
    dpr_clean,          // Command buffer and user provided USB RX FIFO are both empty. This result
                        // indicates a clean end state.
    dpr_incomplete,     // An incomplete command was fetched and buffered. User USB RX FIFO is
                        // exhausted.
    dpr_flush_tdo,      // User should call `lsejtag_flush_tdo` immediately to flush TDO. This code
                        // does not imply any status of command buffer; `lsejtag_dispatch` must be
                        // called again immediately after flushing TDO buffer to find out current
                        // status of command buffer.
    dpr_run_jtag,       // A prepared JTAG buffer is available and user must call `lsejtag_run_jtag`
                        // to start an JTAG transaction. This is usually returned after a successful
                        // JTAG command was executed.
    dpr_too_long,       // An excessively long command was found and was too large to fit in scratch
                        // buffer. This is a fatal error. Notice that this error is not emitted for
                        // `op_fast_target_mem_write*` ops, since this op really do deal with large
                        // amount of data. Special continuation logic is implemented for this op
                        // to make it work across multiple `lsejtag_execute` calls.
    dpr_unknown_cmd,    // An unknown command was found. This may be emitted after command header
                        // was read. Ignore this error.
    dpr_corrupt_state,  // An invalid cmd_buf_state was found. This is a fatal error.
} lsejtag_dispatch_result;

typedef enum {
    cer_success,        // Command executed successfully. This does not imply peripheral state (like
                        // JTAG controller). When dispatch routine is called again, user might get a
                        // useful instruction about what to do with other peripherals, like USB and
                        // JTAG controller.
    cer_continue,       // Command was partially executed, because more data has to be sent to/
                        // received from the host. This is a special return code for
                        // fast_target_mem_read/write ops. When this code was returned, buffer state
                        // is not cleared to cbs_clean, and command continuation context info will
                        // be filled in preparation to next stage of execution.
    cer_await_jtag_buf, // Command was not executed because JTAG I/O buffer is full. There are two
                        // JTAG buffers, and a JTAG related command may execute when any of them is
                        // free. But when both buffer blocks are full, this error is emitted.
    cer_await_usb_tx,   // Command was not executed because USB TX FIFO is full. This is only
                        // emitted when a command that requires immediate access to USB TX FIFO
                        // (like probe memory RW, or firmware version read) is executed. When USB TX
                        // FIFO is full but a JTAG buffer is free, JTAG related commands still may
                        // execute, their USB TX FIFO check happens 
    cer_not_ready,      // Command was not executed because command buffer state is not ready.
    cer_bad_continue,   // Continuation context was valid for a command that does not support
                        // continuation. This is a fatal error.
} lsejtag_cmd_exec_result;

typedef struct {
    // This command buffer is internally used as both a scratch area and a command buffer. Any
    // command sequence read by LS-EJTAG will be first written to this buffer before it's contents
    // were processed. That was the scratch area case. When a long command was split by USB
    // transaction boundary, the full content of that command might not be available immediately.
    // LS-EJTAG will buffer what is immediately available into this buffer first, and when later
    // USB packets arrived it will assemble the full content of the command in this buffer and send
    // it to execution.
    // Depending on how many bytes of a command is available at first, the cmd_buf_state will be
    // changed so that proper read strategy can be decided when next packet was received.
    uint32_t buffered_length;
    uint8_t buffered_cmd[256];  // Keep this field 32-bit aligned pls :(
    uint32_t cmd_buf_wait_payload_size;
    lsejtag_cmd_buf_state cmd_buf_state;

    // Configurable parameters. These are mostly accessed by `usblooptest` commands by "writing
    // directly into probe memory space", used by fast_mem_read/write ops.
    // Nobody would ever use an IR wider than 8 bits, right? Right????
    uint8_t param_ir_len_bits;
    uint8_t param_ir_skip;
    uint8_t param_ir_control;
    uint8_t param_ir_data;
    uint8_t param_ir_address;
    uint8_t param_ir_fastdata;
    uint16_t param_jtag_clk_div;
    uint8_t param_jtag_tdo_sample_timing;

    // When this flag is set, hardware must be reconfigured
    bool pending_reconfiguration;

    // Command continuation context. fast_target_mem_read/write ops may require receiving/sending
    // more data than it can receive/send in one go, therefore they have to be able to save the
    // partially executed state, and continue later on when more data can be received/sent.
    bool continuation;  // Valid flag
    uint8_t cont_op;    // Which op was this continuation context created for
    bool is_64bit_cpu;  // Is target CPU 64 bit (affects Address/Data register length)
    bool use_fastdata;  // Is FASTDATA instruction used
    uint16_t core_count;        // Target core count (decides IR sequence length)
    uint16_t target_core;       // Target core (starting from 0)
    uint32_t bits_per_xfer;     // How many bits of TDI/TMS data is needed for one USize R/W xfer
    uint32_t xfers_per_bufblk;  // How many USize R/Ws should fit into one JTAG buffer block
    uint32_t drseq_dwordlen;    // How many DWORDs will be buffered from USB in each transaction
    uint32_t drseq_bitlen;      // How many valid bits are in FastWrite data in each transaction
    uint32_t tdi_bitlen;// TDI/TMS bit length of the whole premade TDI/TMS sequence
    uint32_t xfers_left;// How many xfers to read/write before command finishes.
                        // For fast_writes: how many xfers of payload need to be received from host
                        // For fast_reads: how many xfers of RAM need to be read from target (and
                        // encapsulated in the format that host understands and sent)
    
    struct lsejtag_jtagbuf_blk {
        // JTAG IO busy indicators. This is used by `lsejtag_execute` to determine whether a command
        // can execute (if JTAG peripheral is still working and the current command has something to
        // do with JTAG port, then execution should be delayed). All flags are set when DMA was
        // started. TDI and TMS are cleared by DMA finish ISRs.
        union {
            struct {
                bool tdi_busy;
                bool tms_busy;
                // This prepared flag indicates that the buffer is filled with data and it's ready
                // to be executed by `lsejtag_run_jtag`.
                bool prepared;
            };
            uint32_t busy;
        };
        bool immediate_flush_tdo;
        // TDI/TMS buffer.
        uint32_t tdi_data[64];
        uint32_t tms_data[64];
        // Misc data used when commanding JTAG peripherals.
        uint32_t tdi_bits;      // How many bits should be sent to TDI and TMS.
        uint32_t tdo_bits;      // How many bits should be read from TDO, 0 for not reading back.
        uint32_t tdo_skip_bits; // How many bits should be skipped before start reading TDO. This
                                // value meant first n-th TCK cycles are skipped before reading TDO.
    } jtagbuf_a, jtagbuf_b, *active_bufblk;

    // TDO in buffer. tdo_data is the DMA-reader destination buffer. When DMA is finished,
    // tdo_dwords should be increased by DMA ISR.
    // Notice how TDO buffer is shared: this is because sometimes a command would demand the TDO
    // read data not be sent back immediately, probably done to increase throughput. Therefore when
    // a command doesn't demand TDO be sent back immediately, TDO data persists even when TDI/TMS
    // buffer block changes. This has caused TDO buffer to stay outside of TDI/TMS buffer blocks.
    // Of course, TDO busy bit is also checked when TDO flush was called/a new command was executed.
    uint32_t tdo_in_dwords;     // How many DWORDs of collected TDO data exists in buffer. This is
                                // increased by TDO readout DMA completion routine, and decreased/
                                // cleared only when TDO buffer is flushed back to host.
    uint32_t tdo_in_data[64];
    uint32_t *tdo_dma_ptr;      // TDO buffer pointer, for DMA readout to write
    uint32_t *tdo_flush_ptr;    // TDO buffer flush pointer, for TDO buffer flush routine.
                                // Incremented when TDO is only partially flushed, and set back to
                                // TDO buffer address when TDO is completely flushed.
    bool tdo_busy;
    // Set when running JTAG actions. Checked when on dispatch, to decide whether the buffer
    // should be flushed when buffer has new data. Cleared only when TDO buffer is fully flushed.
    bool tdo_immediate_flush;
    // Set by TDO readout DMA completion routine, cleared by dispatch routine when it made a
    // decision about whether the new data should be flushed.
    bool tdo_have_new_data;
    // Set by dispatch routine when it finds out the TDO buffer needs a flush, and this flag forces
    // dispatch routine to tell user to flush TDO buffer indefinitely, until TDO buffer is cleared
    // and the flush routine cleared this flag.
    // Note: when command execution fails due to TDO buffer being too full
    bool tdo_should_flush;
} lsejtag_ctx;

//==============================================================================
// API

void lsejtag_init_ctx(lsejtag_ctx *ctx);

lsejtag_dispatch_result lsejtag_dispatch(lsejtag_ctx *ctx);

lsejtag_cmd_exec_result lsejtag_execute(lsejtag_ctx *ctx);

uint32_t lsejtag_flush_tdo(lsejtag_ctx *ctx);

void lsejtag_run_jtag(lsejtag_ctx *ctx);

static inline void lsejtag_jtag_complete_tdi(lsejtag_ctx *ctx) {
    ctx->active_bufblk->tdi_busy = false;
}

static inline void lsejtag_jtag_complete_tms(lsejtag_ctx *ctx) {
    ctx->active_bufblk->tms_busy = false;
}

static inline void lsejtag_jtag_complete_tdo(lsejtag_ctx *ctx, uint32_t dwords_recvd) {
    ctx->tdo_busy = false;
    ctx->tdo_have_new_data = true;
    ctx->tdo_dma_ptr += dwords_recvd;
    ctx->tdo_in_dwords += dwords_recvd;
}
