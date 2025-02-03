
#include <stdio.h>
#include <string.h>
#include "lsejtag/lsejtag.h"
#include "lsejtag/lsejtag_impl.h"
#include "lsejtag/cmdimpl.h"

// Function prototypes
static inline lsejtag_cmd_exec_result lsejtag_execute_continuation(lsejtag_ctx *ctx);

// Code
#define CHECK_CMD_LENGTH(ctx) \
    do { \
        if ((ctx)->buffered_length + (ctx)->cmd_buf_wait_payload_size > \
            sizeof((ctx)->buffered_cmd)) { \
            return dpr_too_long; \
        } \
    } while (0); \

static inline bool is_continuation_capable_op(lsejtag_cmd_op op) {
    return op == op_fast_target_mem_read || op == op_fast_target_mem_write ||
           op == op_fast_target_mem_read_fastdata || op == op_fast_target_mem_write_fastdata;
}

void lsejtag_init_ctx(lsejtag_ctx *ctx) {
    memset(ctx, 0, sizeof(lsejtag_ctx));
    ctx->tdo_dma_ptr = ctx->tdo_in_data;
    ctx->tdo_flush_ptr = ctx->tdo_in_data;

    // Default IR params
    ctx->param_ir_len_bits = 4;
    ctx->param_ir_skip = 0;
    ctx->param_ir_control = 5;
    ctx->param_ir_data = 4;
    ctx->param_ir_address = 3;
    ctx->param_ir_fastdata = 8;
}

lsejtag_dispatch_result lsejtag_dispatch(lsejtag_ctx *ctx) {
    uint32_t len_rx = lsejtag_impl_usbrx_len();
    uint32_t len_avail = len_rx + ctx->buffered_length;
    const lsejtag_cmd_header *header = (const lsejtag_cmd_header *)ctx->buffered_cmd;
    const lsejtag_cmd *cmd = (const lsejtag_cmd *)ctx->buffered_cmd;

    // If new data appeared in TDO buffer, check if it needs an immediate flush. If not, ignore the
    // new data for now. If it needs, force the user flush TDO buffer until TDO buffer is clear.
    if (ctx->tdo_should_flush) {
        return dpr_flush_tdo;
    } else if (ctx->tdo_immediate_flush) {
        if (ctx->tdo_have_new_data) {
            ctx->tdo_have_new_data = false;
            ctx->tdo_should_flush = true;
            return dpr_flush_tdo;
        }
    }

    if (ctx->jtagbuf_a.prepared || ctx->jtagbuf_b.prepared) {
        return dpr_run_jtag;
    }

    if (len_rx == 0 && ctx->buffered_length == 0) {
        return dpr_clean;
    }

    if (ctx->cmd_buf_state == cbs_clean) {
        // Starting from a clean buffer
        if (len_rx == 0) {
            return dpr_clean;
        } else if (len_rx == 1) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd, 1);
            ctx->buffered_length = 1;
            ctx->cmd_buf_state = cbs_wait_header;
            return dpr_incomplete;
        }

        // Get a command header
        lsejtag_impl_usbrx_consume(ctx->buffered_cmd, 2);
        len_rx -= 2;
        len_avail -= 2;
        ctx->buffered_length = 2;
        ctx->cmd_buf_state = cbs_wait_cfg;
        goto wait_cfg; // FIXME: Gotos are so convenient I love them
    } else if (ctx->cmd_buf_state == cbs_wait_header) {
        // Reading header
        if (len_rx != 0) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + 1, 1);
            --len_rx;
            --len_avail;
            ctx->buffered_length = 2;
            ctx->cmd_buf_state = cbs_wait_cfg;
            goto wait_cfg;
        } else {
            return dpr_incomplete;
        }
    } else if (ctx->cmd_buf_state == cbs_wait_cfg) {
        // Reading configuration section
wait_cfg:
        switch (header->common.op) {
        case op_probe_mem_rw:
            ctx->cmd_buf_wait_payload_size = (header->probe_mem_rw.is_write ? 8 : 4);
            ctx->cmd_buf_state = cbs_wait_payload;
            goto wait_payload;
        case op_io_manip:
            ctx->cmd_buf_state = cbs_execute;
            return dpr_execute;
        case op_ir_rw:
        case op_dr_rw:
            if (len_avail > 4) { // header(u16) + ir_seq_bitlen(u16)
                const uint32_t consume_len = 4 - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                ctx->cmd_buf_wait_payload_size = BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4;
                ctx->cmd_buf_state = cbs_wait_payload;
                CHECK_CMD_LENGTH(ctx);
                goto wait_payload;
            }
            break;
        case op_loopback_test:
            if (len_avail > 6) { // header(u16) + number(u32)
                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           6 - ctx->buffered_length);
                ctx->cmd_buf_state = cbs_execute;
                return dpr_execute;
            }
            break;
        case op_fast_target_mem_write_fastdata:
        case op_fast_target_mem_write: {
            // header(u16) + clkdiv(u16) + len(u32) + cpucore(u16)
            const uint32_t expect_len = header->fast_target_mem_write.chained_core_count == 0 ?
                                        8 : 10; // When chained core count is 0, cpucore is gone

            if (len_avail > expect_len) {
                const uint32_t consume_len = expect_len - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                // ctx->cmd_buf_wait_payload_size = cmd->fast_mem_read_at_core.data_len_dword_count *
                //                                  4;
                // ctx->cmd_buf_state = cbs_wait_payload;
                // goto wait_payload;
                ctx->cmd_buf_state = cbs_execute;
                return dpr_execute;
            }
            break;
        }
        case op_fast_target_mem_read_fastdata:
        case op_fast_target_mem_read:
            if (len_avail > 10) { // header(u16) + clkdiv(u16) + len(u32) + cpucore(u16)
                const uint32_t consume_len = 10 - ctx->buffered_length;

                lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length,
                                           consume_len);
                len_rx -= consume_len;
                ctx->buffered_length += consume_len;
                ctx->cmd_buf_state = cbs_execute;
                return dpr_execute;
            }
            break;
        case op_get_firmware_date:
            ctx->cmd_buf_state = cbs_execute;
            return dpr_execute;
        default:
            ctx->buffered_length = 0;
            ctx->cmd_buf_state = cbs_clean;
            return dpr_unknown_cmd;
        }

        // This is where we landed after breaks
        // Since available length is not enough for config section anyways, we consume them all
        lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, len_rx);
        ctx->buffered_length += len_rx;
        return dpr_incomplete;
    } else if (ctx->cmd_buf_state == cbs_wait_payload) {
wait_payload:
        // Payload isn't long enough anyways, consume them all
        if (len_rx < ctx->cmd_buf_wait_payload_size) {
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, len_rx);
            ctx->buffered_length += len_rx;
            ctx->cmd_buf_wait_payload_size -= len_rx;
            return dpr_incomplete;
        } else {
            // We can assemble a complete command now
            const uint32_t consume_len = ctx->cmd_buf_wait_payload_size;
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, consume_len);
            ctx->buffered_length += consume_len;
            ctx->cmd_buf_wait_payload_size = 0;
            ctx->cmd_buf_state = cbs_execute;
            return dpr_execute;
        }
    } else if (ctx->cmd_buf_state == cbs_execute) {
        return dpr_execute;
    } else {
        return dpr_corrupt_state;
    }
}

lsejtag_cmd_exec_result lsejtag_execute(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;

    if (ctx->cmd_buf_state != cbs_execute) {
        return cer_not_ready;
    }

    if (ctx->continuation) {
        return lsejtag_execute_continuation(ctx);
    }

    switch (cmd->common.header.op) {
        case op_probe_mem_rw:
            return cmdimpl_probe_mem_rw(ctx);
        case op_io_manip:
            return cmdimpl_io_manip(ctx);
        case op_ir_rw:
            return cmdimpl_irdr_write(ctx, true);
        case op_dr_rw:
            return cmdimpl_irdr_write(ctx, false);
        case op_loopback_test:
            break;
        case op_fast_target_mem_write_fastdata:
            return cmdimpl_fast_mem_write(ctx, true);
        case op_fast_target_mem_write:
            return cmdimpl_fast_mem_write(ctx, false);
        case op_fast_target_mem_read_fastdata:
            break;
        case op_fast_target_mem_read:
            break;
        case op_get_firmware_date:
            return cmdimpl_get_firmware_date(ctx);
    }

    return cer_success;
}

static inline lsejtag_cmd_exec_result lsejtag_execute_continuation(lsejtag_ctx *ctx) {
    switch ((lsejtag_cmd_op)ctx->cont_op) {

    case op_fast_target_mem_write_fastdata:
    case op_fast_target_mem_write:
        return cmdimpl_continue_fast_mem_write(ctx);
    case op_fast_target_mem_read_fastdata:
    case op_fast_target_mem_read:
        // break; // TODO:

    default:
        return cer_bad_continue;
    }
}

uint32_t lsejtag_flush_tdo(lsejtag_ctx *ctx) {
    const uint32_t max_send_bytes = lsejtag_impl_usbtx_free_len();

    if (max_send_bytes >= ctx->tdo_in_dwords * 4) {
        // We can completely flush TDO data
        lsejtag_impl_usbtx((uint8_t *)ctx->tdo_flush_ptr, ctx->tdo_in_dwords * 4);
        ctx->tdo_in_dwords = 0;
        ctx->tdo_flush_ptr = ctx->tdo_in_data;
        ctx->tdo_dma_ptr = ctx->tdo_in_data;
        ctx->tdo_immediate_flush = false;
        ctx->tdo_should_flush = false;
        return ctx->tdo_in_dwords * 4;
    } else {
        // We can only flush a portion of TDO data
        const uint32_t send_dwords = max_send_bytes / 4;
        lsejtag_impl_usbtx((uint8_t *)ctx->tdo_flush_ptr, send_dwords * 4);
        ctx->tdo_flush_ptr += send_dwords;
        ctx->tdo_in_dwords -= send_dwords;
        return send_dwords * 4;
    }
}

void lsejtag_run_jtag(lsejtag_ctx *ctx) {
    struct lsejtag_jtagbuf_blk *bufblk;

    // Deny the RunJtag request if JTAG peripheral is still working
    if (ctx->tdo_busy || ctx->jtagbuf_a.tdi_busy || ctx->jtagbuf_a.tms_busy ||
        ctx->jtagbuf_b.tdi_busy || ctx->jtagbuf_b.tms_busy) {
        return;
    }

    if (ctx->jtagbuf_a.prepared) {
        bufblk = &ctx->jtagbuf_a;
    } else if (ctx->jtagbuf_b.prepared) {
        bufblk = &ctx->jtagbuf_b;
    } else {
        // ??
        return;
    }

    ctx->active_bufblk = bufblk;
    ctx->tdo_immediate_flush = bufblk->immediate_flush_tdo;
    bufblk->tdi_busy = true;
    bufblk->tms_busy = true;
    lsejtag_impl_run_jtag(bufblk->tdi_data, bufblk->tms_data, ctx->tdo_dma_ptr, bufblk->tdi_bits,
                          bufblk->tdo_bits, bufblk->tdo_skip_bits);
    bufblk->prepared = false;
    if (bufblk->tdo_bits) {
        ctx->tdo_busy = true;
    }
}
