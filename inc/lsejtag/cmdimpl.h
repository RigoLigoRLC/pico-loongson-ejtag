
#pragma once

#include "lsejtag/lsejtag.h"

#define BIT2DWORD(x) (((x) + 31) / 32)
#define DEF_COMMAND(X) lsejtag_cmd_exec_result cmdimpl_##X(lsejtag_ctx *ctx)
#define DEF_COMMAND_ARG(X, ...) lsejtag_cmd_exec_result cmdimpl_##X(lsejtag_ctx *ctx, __VA_ARGS__)

DEF_COMMAND(probe_mem_rw);
DEF_COMMAND(io_manip);
DEF_COMMAND(get_firmware_date);
DEF_COMMAND_ARG(irdr_write, bool isir);
DEF_COMMAND_ARG(fast_mem_write, bool use_fastdata);
DEF_COMMAND_ARG(fast_mem_read, bool use_fastdata);
DEF_COMMAND(continue_fast_mem_write);
