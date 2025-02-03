
#include <pico/platform/compiler.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "lsejtag/config.h"
#include "lsejtag/cmdimpl.h"
#include "lsejtag/lsejtag_impl.h"

static uint32_t return_data_len_required(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    
    switch (cmd->common.header.op) {
    case op_probe_mem_rw:
        return cmd->probe_mem_rw.header.is_write ? 0 : 4;
    case op_io_manip:
        return 0;
    // For IR/DR RW this is TDO buffer size required
    case op_ir_rw:
        return cmd->ir_rw.header.read_back ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
    case op_dr_rw:
        return cmd->dr_rw.header.read_back ? 0 : (BIT2DWORD(cmd->ir_rw.irseq_len_bits) * 4);
    case op_loopback_test:
    case op_get_firmware_date:
        return 4;
    case op_fast_target_mem_write_fastdata:
    case op_fast_target_mem_write:
        return 0;
    case op_fast_target_mem_read_fastdata:
    case op_fast_target_mem_read: {
        lsejtag_cmd_fast_target_mem_read_at_core *read_cmd = &cmd->fast_mem_read_at_core;
        uint32_t bytes_per_machine_word = 
            BIT2DWORD((read_cmd->header.cpu_is_64bit ? 64 : 32) +
                      read_cmd->header.chained_core_count) * 4;
        uint32_t bytes_to_send = bytes_per_machine_word * read_cmd->data_len_dword_count;
        return bytes_to_send;
    }
    default:
        return 0;
    }
}

static inline void wait_until_jtag_periph_free(lsejtag_ctx *ctx) {
    while (ctx->jtagbuf_a.busy || ctx->jtagbuf_b.busy || ctx->tdo_busy);
}

static inline void cmd_buf_clean(lsejtag_ctx *ctx) {
    ctx->buffered_length = 0;
    ctx->cmd_buf_state = cbs_clean;
    ctx->cmd_buf_wait_payload_size = 0;
}

lsejtag_cmd_exec_result cmdimpl_get_firmware_date(lsejtag_ctx *ctx) {
    static const union {
        uint32_t bcd;
        uint8_t bytes[4];
    } version_date = { .bcd = LSEJTAG_FIRMWARE_DATE };

    if (return_data_len_required(ctx) > lsejtag_impl_usbtx_free_len()) {
        return cer_await_usb_tx;
    }

    lsejtag_impl_usbtx(version_date.bytes, 4);
    cmd_buf_clean(ctx);
    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_probe_mem_rw(lsejtag_ctx *ctx) {
    lsejtag_cmd_probe_mem_rw *cmd = (lsejtag_cmd_probe_mem_rw *)ctx->buffered_cmd;
    union {
        uint32_t u32;
        uint8_t bytes[4];
    } data_read;

    if (cmd->header.is_write) {
        // Writes
        switch (cmd->addr) {
        case 0x81000070: // JTAG timing configuration
            switch ((cmd->data_to_write & 0xffff0000) >> 16) {
            case 1: {
                // Clock divider. Find out highest bit set
                bool found = false;
                for (int i = 0x8000; i != 0; i >>= 1) {
                    if (i & cmd->data_to_write) {
                        ctx->param_jtag_clk_div = i & cmd->data_to_write;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    // In reality this should prevent JTAG port from doing any IO
                    ctx->param_jtag_clk_div = 0;
                }

                // Reconfigure JTAG when JTAG port is free
                wait_until_jtag_periph_free(ctx);
                lsejtag_impl_reconfigure(impl_recfg_clkfreq, ctx->param_jtag_clk_div);
                break;
            }
            case 2:
                // TDO sampling timing
                ctx->param_jtag_tdo_sample_timing = cmd->data_to_write & 0x3;
                break;
            }
            break;
        case LSEJTAG_PARAM_ADDR_IR_LEN     : ctx->param_ir_len_bits = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : ctx->param_ir_skip     = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : ctx->param_ir_control  = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : ctx->param_ir_data     = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : ctx->param_ir_address  = cmd->data_to_write; break;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: ctx->param_ir_fastdata = cmd->data_to_write; break;
        }
    } else {
        // Reads
        if (return_data_len_required(ctx) > lsejtag_impl_usbtx_free_len()) {
            return cer_await_usb_tx;
        }

        switch (cmd->addr) {
        case 0x40: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_LEN     ; break;
        case 0x44: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_SKIP    ; break;
        case 0x48: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_CONTROL ; break;
        case 0x4c: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_DATA    ; break;
        case 0x50: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_ADDRESS ; break;
        case 0x54: data_read.u32 = LSEJTAG_PARAM_ADDR_IR_FASTDATA; break;
        case LSEJTAG_PARAM_ADDR_IR_LEN     : data_read.u32 = ctx->param_ir_len_bits; break;
        case LSEJTAG_PARAM_ADDR_IR_SKIP    : data_read.u32 = ctx->param_ir_skip;     break;
        case LSEJTAG_PARAM_ADDR_IR_CONTROL : data_read.u32 = ctx->param_ir_control;  break;
        case LSEJTAG_PARAM_ADDR_IR_DATA    : data_read.u32 = ctx->param_ir_data;     break;
        case LSEJTAG_PARAM_ADDR_IR_ADDRESS : data_read.u32 = ctx->param_ir_address;  break;
        case LSEJTAG_PARAM_ADDR_IR_FASTDATA: data_read.u32 = ctx->param_ir_fastdata; break;
        default: data_read.u32 = 0; break;
        }
        lsejtag_impl_usbtx(data_read.bytes, 4);
    }
    cmd_buf_clean(ctx);
    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_io_manip(lsejtag_ctx *ctx) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;

    if (cmd->io_manip.header.port_id > io_maximum) {
        return cer_success;
    }

    switch ((lsejtag_io)cmd->io_manip.header.port_id) {
    case io_led:
        lsejtag_impl_io_manip(impl_io_led, cmd->io_manip.header.level == 1);
        break;
    case io_trst:
        lsejtag_impl_io_manip(impl_io_trst, cmd->io_manip.header.level == 1);
        break;
    case io_brst:
        lsejtag_impl_io_manip(impl_io_brst, cmd->io_manip.header.level == 1);
        break;
    case io_dint:
        lsejtag_impl_io_manip(impl_io_dint, cmd->io_manip.header.level == 1);
        break;

    case io_fpga_reset:
        // This one probably clears off internal FIFOs or such. Unclear
        break;

    case io_oe:
        // When #OE is pulled high, all input/output are cut off. TODO: UNIMPLEMENTED
        break;

    case io_tap_reset: {
        // An absolutely overkill TMS reset sequence
        static const uint32_t tms_seq[] = { 0xffffffff, 0x7fffffff };
        static const uint32_t tdi_seq[] = { 0x0, 0x0 };

        // Since any JTAG sequence will finish pretty fast, we simply wait right here until JTAG
        // peripheral is free, then we send the TAP reset sequence
        wait_until_jtag_periph_free(ctx);

        // Let it go
        lsejtag_impl_run_jtag(tdi_seq, tms_seq, NULL, 64, 0, 0);

        // Also wait until the reset sequence finishes
        wait_until_jtag_periph_free(ctx);
        break;
    }
    default:
    case io_maximum:
        break;
    }

    cmd_buf_clean(ctx);

    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_irdr_write(lsejtag_ctx *ctx, bool isir) {
    struct lsejtag_jtagbuf_blk *bufblk;
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    uint32_t *tmsbuf, *tdibuf, *dataseq;
    uint32_t bitlen, dwordlen, bitshift = 0;
    uint32_t data_dwordlen = BIT2DWORD(cmd->ir_rw.irseq_len_bits);

    // Choose a free JTAG buffer
    if (!ctx->jtagbuf_a.busy) {
        bufblk = &ctx->jtagbuf_a;
    } else if (!ctx->jtagbuf_b.busy) {
        bufblk = &ctx->jtagbuf_b;
    } else {
        return cer_await_jtag_buf;
    }

    tmsbuf = (uint32_t *)&bufblk->tms_data;
    tdibuf = (uint32_t *)&bufblk->tdi_data;
    dataseq = (uint32_t *)&cmd->ir_rw.irseq;

    // See if TDO buffer is enough, if not, force a TDO flush
    if (sizeof(ctx->tdo_in_data) - 4 * ctx->tdo_in_dwords < return_data_len_required(ctx)) {
        ctx->tdo_should_flush = true;
        return cer_await_usb_tx;
    }

    // Calculate bit length, and clear the scratch area
    // TMS: [IR:01100  DR:0100] + [(SeqBitLen-1)x 0] + [110]
    // TDI: [IR:00000  DR:0000] + [DataSeq]           + [00]
    bitlen = 4 + (isir ? 1 : 0) + cmd->ir_rw.irseq_len_bits + 2;
    dwordlen = BIT2DWORD(bitlen);
    memset(tmsbuf, 0, dwordlen * 4);
    memset(tdibuf, 0, dwordlen * 4);
    // We fill this metadata right here for convenience
    bufblk->tdi_bits = bitlen;

    // Fill data
    // First stage
    *tmsbuf = isir ? 0x6 : 0x2;

    // Data sequence
    // The whole data sequence is operated on per DWORD basis
    if (isir) {
        for (uint32_t i = 0; i < data_dwordlen; ++i) {
            *tdibuf |= (dataseq[i]) << 5;
            ++tdibuf;
            *tdibuf = (dataseq[i]) >> 27;
        }
    } else {
        for (uint32_t i = 0; i < data_dwordlen; ++i) {
            *tdibuf |= (dataseq[i]) << 4;
            ++tdibuf;
            *tdibuf = (dataseq[i]) >> 28;
        }
    }

    // Ending TMS sequence
    tmsbuf += cmd->ir_rw.irseq_len_bits / 32;
    bitshift = isir ? 5 : 4;
    bitshift = (bitshift + cmd->ir_rw.irseq_len_bits - 1);
    bitshift %= 32;

    *tmsbuf |= (0x3 << bitshift);
    if (bitshift >= 30) {
        ++tmsbuf;
        *tmsbuf = (0x3 >> (32 - bitshift));
    }

    // Fill in metadata
    if (cmd->ir_rw.header.read_back) {
        bufblk->tdo_skip_bits = isir ? 5 : 4;
        bufblk->tdo_bits = cmd->ir_rw.irseq_len_bits; 
    } else {
        bufblk->tdo_bits = 0;
    }
    
    bufblk->immediate_flush_tdo = (cmd->ir_rw.header.immediate_send_back ? true : false);

    bufblk->prepared = true;

    cmd_buf_clean(ctx);
    return cer_success;
}

static inline uint8_t append_bits(uint32_t *dest_buf, uint32_t bitshift, uint32_t field,
                                  uint8_t bits) {
    const uint32_t dwordshift = bitshift / 32, local_bitshift = bitshift % 32;
    const uint32_t split_field_thresh = 32 - bits;
    const uint32_t field_masked = field & ~(0xFFFFFFFFu << bits);

    if (local_bitshift < split_field_thresh) {
        dest_buf[dwordshift] |= (field_masked << local_bitshift);
    } else {
        dest_buf[dwordshift] |= (field_masked << local_bitshift);
        dest_buf[dwordshift + 1] |= (field_masked >> (32 - local_bitshift));
    }
    return bits;
}

static inline uint8_t append_dword(uint32_t *dest_buf, uint32_t bitshift, uint32_t field) {
    const uint32_t dwordshift = bitshift / 32, local_bitshift = bitshift % 32;

    if (local_bitshift == 0) {
        dest_buf[dwordshift] = field;
    } else {
        dest_buf[dwordshift] |= (field << local_bitshift);
        dest_buf[dwordshift + 1] |= (field >> (32 - local_bitshift));
    }
    return 32;
}
static inline uint8_t replace_bits(uint32_t *dest_buf, uint32_t bitshift, uint32_t field,
                                   uint8_t bits) {
    const uint32_t dwordshift = bitshift / 32, local_bitshift = bitshift % 32;
    const uint32_t split_field_thresh = 32 - bits;
    const uint32_t field_masked = field & ~(0xFFFFFFFFu << bits);
    uint32_t tmp;

    if (local_bitshift < split_field_thresh) {
        tmp = dest_buf[dwordshift] & ~(0xFFFFFFFFu >> (32 - bits) << bits);
        tmp |= (field_masked << local_bitshift);
        dest_buf[dwordshift] = tmp;
    } else {
        tmp = dest_buf[dwordshift] & (0xFFFFFFFFu >> (32 - bits));
        tmp |= (field_masked << local_bitshift);
        dest_buf[dwordshift] = tmp;

        tmp = dest_buf[dwordshift + 1] & (0xFFFFFFFFu << (local_bitshift + bits - 32));
        tmp |= (field_masked >> (32 - local_bitshift));
        dest_buf[dwordshift + 1] = tmp;
    }

    return bits;
}

static inline uint8_t replace_dword(uint32_t *dest_buf, uint32_t bitshift, uint32_t field) {
    const uint32_t dwordshift = bitshift / 32, local_bitshift = bitshift % 32;

    if (local_bitshift == 0) {
        dest_buf[dwordshift] = field;
    } else {
        uint32_t tmp;
        tmp = dest_buf[dwordshift] & ~(0xFFFFFFFFu << local_bitshift);
        tmp |= (field << local_bitshift);
        dest_buf[dwordshift] = tmp;

        tmp = dest_buf[dwordshift + 1] & ~(0xFFFFFFFFu >> (32 - local_bitshift));
        tmp |= (field >> (32 - local_bitshift));
        dest_buf[dwordshift + 1] = tmp;
    }
    return 32;
}

lsejtag_cmd_exec_result cmdimpl_fast_mem_write(lsejtag_ctx *ctx, bool use_fastdata) {
    lsejtag_cmd *cmd = (lsejtag_cmd *)ctx->buffered_cmd;
    uint32_t bits_per_xfer;
    uint32_t dwords_per_xfer;
    uint32_t bits_in_tditms_buf = sizeof(ctx->jtagbuf_a.tdi_data) * 8;
    struct lsejtag_jtagbuf_blk *bufblk = &ctx->jtagbuf_a;
    uint32_t *tms_data = bufblk->tms_data, *tdi_data = bufblk->tdi_data;
    uint32_t bitshift = 0;
    uint32_t core_count; // Real core count; when ctx->core_count == 0 it means 1
    uint32_t skipbits;
    uint32_t dr_bitlen, dr_dwordlen;

    ctx->cont_op = cmd->common.header.op;
    ctx->is_64bit_cpu = cmd->fast_mem_write.header.cpu_is_64bit;
    ctx->use_fastdata = use_fastdata;
    ctx->core_count = cmd->fast_mem_write.header.chained_core_count;
    // Target core is used to determine how many SKIP bits comes before data DWORD
    // If core_count == 0, enters a special "GS232" mode that basically assumes:
    //      is_64bit_cpu = 0, use_fastdata = 1, core_count = 1, target_core = 1
    //      and never manipulates IR at all
    if (ctx->core_count) {
        ctx->target_core = cmd->fast_mem_write_at_core.at_cpu_core;
        core_count = ctx->core_count;
    } else {
        ctx->target_core = 0;
        ctx->use_fastdata = true;
        core_count = 1;
    }

    // Calculate how many SKIP register bits needs to be considered when calculating bit length
    skipbits = ctx->core_count ? (ctx->core_count - 1) : 0;

    // Debugger software sends us prepared DR sequences, they can be arbitrary bits long but are
    // always padded to 32-bit boundary, for each transaction.
    // The length is used to calculate how many transaction worth of DR sequence we can buffer in
    // command buffer.
    // FASTDATA adds 1 bit of SPrAcc
    ctx->drseq_bitlen = skipbits + (ctx->is_64bit_cpu ? 64 : 32) + (use_fastdata ? 1 : 0);
    ctx->drseq_dwordlen = BIT2DWORD(ctx->drseq_bitlen);

    // Calculate how many DWORDs we can write in one JTAG buffer worth of operations.
    // With FASTDATA: we switch IR to FASTDATA, and then we no longer need to tinker with IR again.
    //      Just write everything into DR. So, we first switch to IR right here, and only calculate
    //      how many DR writes we can fit into one JTAG buffer block:
    //      TMS: [0100] + [0]         + [(SkipBits+BitWidth-1)x 0] + [110]
    //      TDI: [0000] + [0] (PrAcc) + [SkipBits + Data]          +  [00]
    //
    //          32 Bit CPU: 39 Bits + SkipBits
    //          64 Bit CPU: 71 Bits + SkipBits
    //
    // Without FASTDATA: we have to write DATA once and write CONTROL once to clear PrAcc to finish
    //      transaction.
    //      TMS: [01100] + [(IRSeqLen-1)x 0]   + [11100] + [(SkipBits+BitWidth-1)x 0]
    //      TDI: [00000] + [IRSeq select DATA] +  [0000] + [SkipBits + Data]
    //
    //      TMS: + [111100] + [(IRSeqLen-1)x 0]      + [11100]
    //      TDI: +  [00000] + [IRSeq select Control] + [00000]
    //
    //      TMS: + [(SkipBits+31)x 0]          + [110]
    //      TDI: + [SkipBits + 32 bit Control] +  [00]
    //
    //          32 Bit CPU: 86 + 2 * (CoreCount * IrLen - 1) + 2 * SkipBits Bits
    //          64 Bit CPU: 118 + 2 * (CoreCount * IrLen - 1) + 2 * SkipBits Bits
    if (use_fastdata) {
        bits_per_xfer = (ctx->is_64bit_cpu ? 71 : 39) + skipbits;
    } else {
        if (ctx->is_64bit_cpu) {
            bits_per_xfer = 118 + 2 * (core_count * ctx->param_ir_len_bits - 1 + skipbits);
        } else {
            bits_per_xfer = 86 + 2 * (core_count * ctx->param_ir_len_bits - 1 + skipbits);
        }
    }

    // Calculate some metadata, will be useful later
    ctx->bits_per_xfer = bits_per_xfer;
    // xfer_per_bufblk should be determined by the minimal value of:
    // How many transactions this command demands
    // How many transactions worth of DR sequence we can buffer in command buffer
    // How many transactions worth of JTAG sequence we can create in JTAG data buffer
    ctx->xfers_per_bufblk = MIN(cmd->fast_mem_write.data_len_dword_count,
                                MIN(
                                    sizeof(ctx->buffered_cmd) / (ctx->drseq_dwordlen * 4),
                                    bits_in_tditms_buf / bits_per_xfer
                                )
                            );
    ctx->tdi_bitlen = ctx->xfers_per_bufblk * bits_per_xfer;
    dwords_per_xfer = BIT2DWORD(bits_per_xfer);

    // We're about to destroy some JTAG buffers, be safe
    wait_until_jtag_periph_free(ctx);

    // Reconfigure JTAG clock division as specified
    lsejtag_impl_reconfigure(impl_recfg_clkfreq, cmd->fast_mem_write.jtag_clk_div);

    // Select FASTDATA right now if needed. GS232 mode doesn't need IR manipulation, skip it
    if (use_fastdata && ctx->core_count) {
        // TMS: [01100] + [(IRSeqLen-1)x 0]       + [110]
        // TDI: [00000] + [IRSeq select FASTDATA] + [000]
        // Bit length: 5 + (CoreCount * IrLen - 1) + 3
        const uint32_t bitlen_select_fastdata = 7 + core_count * ctx->param_ir_len_bits;
        const uint32_t dwordlen_select_fastdata = BIT2DWORD(bitlen_select_fastdata);

        memset(tms_data, 0, dwordlen_select_fastdata * 4);
        memset(tdi_data, 0, dwordlen_select_fastdata * 4);

        // 01100 (Shift-IR)
        bitshift += append_bits(tms_data, bitshift, 0x6, 5);

        // TDI Sequence selecting FASTDATA
        if (ctx->core_count) {
            // Fill non-target cores' TAP IRs with SKIP, and fill target core's TAP IR with FASTDATA
            for (int32_t core = ctx->core_count - 1; core >= 0; --core) {
                append_bits(tdi_data,
                            bitshift,
                            core == ctx->target_core ? ctx->param_ir_fastdata : ctx->param_ir_skip,
                            ctx->param_ir_len_bits);
                bitshift += ctx->param_ir_len_bits;
            }
        } else {
            // Fill the TAP IR with FASTDATA
            bitshift += 
                append_bits(tdi_data, bitshift, ctx->param_ir_fastdata, ctx->param_ir_len_bits);
        }

        // DRSeqBitLen-1
        --bitshift;

        // 110 (Idle state)
        bitshift += append_bits(tms_data, bitshift, 0x3, 3);

        // Execute JTAG sequence
        lsejtag_run_jtag(ctx);
        wait_until_jtag_periph_free(ctx);
    }

    // Prepare the TMS and TDI sequence
    // We fill one instance, then we copy them to other instances
    memset(tms_data, 0, dwords_per_xfer * ctx->xfers_per_bufblk * 4);
    memset(tdi_data, 0, dwords_per_xfer * ctx->xfers_per_bufblk * 4);
    bitshift = 0;
    if (use_fastdata) {
        // Normal mode with FASTDATA & GS232 Mode: Just write DR, no IR writes
        // 0100 (Shift-DR)
        bitshift += append_bits(tms_data, bitshift, 0x2, 4);

        // Shift in (BitWidth+1) bits of data (including PrAcc) and SKIP bits
        bitshift += (ctx->drseq_bitlen - 1);

        // 110 (Idle state)
        bitshift += append_bits(tms_data, bitshift, 0x3, 3);
    } else {
        // 01100 (Shift-IR)
        bitshift += append_bits(tms_data, bitshift, 0x6, 5);

        // IR sequence selecting DATA
        if (ctx->core_count) {
            for (int32_t core = ctx->core_count - 1; core >= 0; --core) {
                append_bits(tdi_data,
                            bitshift,
                            (core == ctx->target_core) ? ctx->param_ir_data : ctx->param_ir_skip,
                            ctx->param_ir_len_bits);
                bitshift += ctx->param_ir_len_bits;
            }
        } else {
            bitshift += append_bits(tdi_data, bitshift, ctx->param_ir_data, ctx->param_ir_len_bits);
        }
        --bitshift;

        // 11100 (Shift-DR)
        bitshift += append_bits(tms_data, bitshift, 0x7, 5);

        // Data to be written
        bitshift += (ctx->drseq_bitlen - 1);

        // 111100 (Shift-IR)
        bitshift += append_bits(tms_data, bitshift, 0xf, 6);

        // IR sequence selecting CONTROL
        if (ctx->core_count) {
            for (uint32_t core = 0; core < ctx->core_count; ++core) {
                append_bits(tdi_data,
                            bitshift,
                            (core == ctx->target_core) ? ctx->param_ir_control : ctx->param_ir_skip,
                            ctx->param_ir_len_bits);
                bitshift += ctx->param_ir_len_bits;
            }
        } else {
            bitshift += append_bits(tdi_data, bitshift, ctx->param_ir_control,
                                    ctx->param_ir_len_bits);
        }
        --bitshift;

        // 11100 (Shift-DR)
        bitshift += append_bits(tms_data, bitshift, 0x7, 5);

        // Control to be written
        bitshift += skipbits;
        bitshift += append_dword(tdi_data, bitshift, 0xc000);
        --bitshift;

        // 110 (Idle)
        bitshift += append_bits(tms_data, bitshift, 0x3, 3);
    }
    assert(bitshift == bits_per_xfer);

    // Copy to other instances
    void dump_binary_to_console(const uint8_t * const data, uint32_t length);
    for (uint32_t i = 1; i < ctx->xfers_per_bufblk; ++i) {
        uint32_t j;
        for (j = 0; j < (bits_per_xfer / 32); ++j) {
            append_dword(tdi_data, bitshift, tdi_data[j]);
            bitshift += append_dword(tms_data, bitshift, tms_data[j]);
            
        }
        if (bits_per_xfer % 32) {
            append_bits(tdi_data, bitshift, tdi_data[j], bits_per_xfer % 32);
            bitshift += append_bits(tms_data, bitshift, tms_data[j], bits_per_xfer % 32);
        }
    }

    // Copy to another JTAG buffer
    memcpy(ctx->jtagbuf_b.tms_data, tms_data, BIT2DWORD(ctx->tdi_bitlen) * 4);
    memcpy(ctx->jtagbuf_b.tdi_data, tdi_data, BIT2DWORD(ctx->tdi_bitlen) * 4);

    // Clear command buffer. It'll become TX data buffer later
    ctx->buffered_length = 0;

    // Set total data amount
    ctx->xfers_left = cmd->fast_mem_write.data_len_dword_count;

    // Set continuation context valid flag
    ctx->continuation = true;

    // printf("FASTWRITE first stage parsing complete\n");
    // printf("    xfers_left: %d\n", ctx->xfers_left);
    // printf("    bits_per_xfer: %d\n", ctx->bits_per_xfer);
    // printf("    xfers_per_bufblk: %d\n", ctx->xfers_per_bufblk);
    // printf("    tdi_bitlen: %d\n", ctx->tdi_bitlen);
    // printf("    drseq_dwordlen: %d\n", ctx->drseq_dwordlen);
    // printf("    drseq_bitlen: %d\n", ctx->drseq_bitlen);

    // Let the continuation worker do the job
    return cer_continue;
}

lsejtag_cmd_exec_result cmdimpl_fast_mem_read(lsejtag_ctx *ctx, bool use_fastdata) {
    // FIXME: NOT IMPLEMENTED
    return cer_success;
}

lsejtag_cmd_exec_result cmdimpl_continue_fast_mem_write(lsejtag_ctx *ctx) {
    struct lsejtag_jtagbuf_blk *bufblk;
    uint32_t *tmsbuf, *tdibuf, *databuf = (uint32_t *)ctx->buffered_cmd;
    const uint32_t skipbits = ctx->core_count ? (ctx->core_count - 1) : 0;
    const uint32_t core_count = (ctx->core_count == 0) ? 1 : ctx->core_count;
    const uint32_t drseq_size = ctx->drseq_dwordlen * 4;
    // With FASTDATA: [0000] + [0 (PrAcc)] + [SKIPs] + [DATA]
    // No FASTDATA: [00000] + [IRSeq] + [0000] + [SKIPs] + [DATA]
    // Bit offset at where DATA should be placed, relative to beginning of an xfer's JTAG sequence
    const uint32_t data_bitoffset = ctx->use_fastdata ? 4 : core_count * ctx->param_ir_len_bits + 9;
    uint32_t bitshift;

    // printf("Continuation FASTWRITE:\n");
    // printf("    xfers_left: %d\n", ctx->xfers_left);
    // printf("    xfers_per_bufblk: %d\n", ctx->xfers_per_bufblk);
    // printf("    buffered_length: %d\n", ctx->buffered_length);  

    // If we have less DWORDs awaiting transmission than the premade TDI/TMS sequence, we should
    // truncate the JTAG sequence, change the xfers_per_bufblk, tdi_bitlen records
    if (ctx->xfers_per_bufblk > ctx->xfers_left) {
        ctx->xfers_per_bufblk = ctx->xfers_left;
        
        // Truncate JTAG bit sequence
        ctx->tdi_bitlen = ctx->bits_per_xfer * ctx->xfers_per_bufblk;
        // printf(" -- Truncated, new tdi_bitlen: %d\n", ctx->tdi_bitlen);
    }

    // Ensure we have enough data buffered
    if (ctx->buffered_length < ctx->xfers_per_bufblk * drseq_size) {
        uint32_t avail_len = lsejtag_impl_usbrx_len();
        // printf(" -- avail_len: %d\n", avail_len);

        // Get exactly how many DWORDs we're about to transmit
        if (ctx->buffered_length + avail_len >= ctx->xfers_per_bufblk * drseq_size) {
            // Enough data in USB FIFO
            const uint32_t read_len = ctx->xfers_per_bufblk * drseq_size - ctx->buffered_length;
            // printf(" -- Enough; reading %d bytes; ", read_len);
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, read_len);
            ctx->buffered_length += read_len;
            // printf("buffered %d bytes now\n", read_len);
        } else {
            // Not enough
            // printf(" -- Not enough; reading %d bytes; ", avail_len);
            lsejtag_impl_usbrx_consume(ctx->buffered_cmd + ctx->buffered_length, avail_len);
            ctx->buffered_length += avail_len;
            // printf("buffered %d bytes now\n", ctx->buffered_length);
            return cer_continue;
        }
    }

    // Choose a free JTAG buffer
    if (!ctx->jtagbuf_a.busy) {
        bufblk = &ctx->jtagbuf_a;
    } else if (!ctx->jtagbuf_b.busy) {
        bufblk = &ctx->jtagbuf_b;
    } else {
        return cer_await_jtag_buf;
    }
    tmsbuf = (uint32_t *)&bufblk->tms_data;
    tdibuf = (uint32_t *)&bufblk->tdi_data;

    // Place data at adequate locations in TDI buffer
    for (uint32_t i = 0; i < ctx->xfers_per_bufblk; ++i) {
        bitshift = i * ctx->bits_per_xfer + data_bitoffset;
        for (uint32_t j = 0; j < (ctx->drseq_bitlen / 32); ++j) {
            bitshift += replace_dword(tdibuf, bitshift, *databuf);
            ++databuf;
        }
        if (ctx->drseq_bitlen % 32) {
            replace_bits(tdibuf, bitshift, *databuf, ctx->drseq_bitlen % 32);
            ++databuf;
        }
    }

    // Fill in transfer metadata
    bufblk->tdi_bits = ctx->tdi_bitlen;
    bufblk->tdo_bits = 0;
    void dump_binary_to_console(const uint8_t * const data, uint32_t length);
    // dump_binary_to_console((uint8_t *)tdibuf, (ctx->tdi_bitlen + 7) / 8);
    // dump_binary_to_console((uint8_t *)tmsbuf, (ctx->tdi_bitlen + 7) / 8);

    // Mark buffer as ready
    bufblk->prepared = true;

    // Clear command buffer
    ctx->buffered_length = 0;

    // Check if we reached end of command, if so, set buffer state to clean and return success
    ctx->xfers_left -= ctx->xfers_per_bufblk;
    if (ctx->xfers_left == 0) {
        cmd_buf_clean(ctx);

        // Invalidate continuation context
        ctx->continuation = false;

        return cer_success;
    }

    return cer_continue;
}
