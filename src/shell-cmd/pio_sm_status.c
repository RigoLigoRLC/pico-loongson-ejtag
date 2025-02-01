
#include "shell_port.h"
#include "config.h"
#include "ejtag.h"
#include <hardware/pio.h>
#include <stdio.h>

int pio_sm_status(int argc, char **argv) {
    shellPrint(&shell, " SM0 TDI PC=%02d [FIFO TX=%d STALL=%d]\r\n",
               pio_sm_get_pc(pio0, SM_TDI),
               pio_sm_get_tx_fifo_level(pio0, SM_TDI),
               (pio0->fdebug & (PIO_FDEBUG_TXSTALL_LSB << SM_TDI)) != 0);
    shellPrint(&shell, " SM1 TMS PC=%02d [FIFO TX=%d STALL=%d]\r\n",
               pio_sm_get_pc(pio0, SM_TMS),
               pio_sm_get_tx_fifo_level(pio0, SM_TMS),
               (pio0->fdebug & (PIO_FDEBUG_TXSTALL_LSB << SM_TMS)) != 0);
    shellPrint(&shell, " SM2 TDO PC=%02d [FIFO TX=%d RX=%d STALL=%d]\r\n",
               pio_sm_get_pc(pio0, SM_TDO),
               pio_sm_get_tx_fifo_level(pio0, SM_TDO),
               pio_sm_get_rx_fifo_level(pio0, SM_TDO),
               (pio0->fdebug & (PIO_FDEBUG_TXSTALL_LSB << SM_TDO)) != 0);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), piostatus, pio_sm_status, PIO State machine status);

int cmd_buf_status(int argc, char **argv) {
    const char *states[] = {
        "cbs_clean",
        "cbs_wait_header",
        "cbs_wait_cfg",
        "cbs_wait_payload",
        "cbs_execute",
    };
    shellPrint(&shell, " Command buffer\r\n");
    shellPrint(&shell, "   cmd_buf_state = %s\r\n", states[lsejtag_lib_ctx.cmd_buf_state]);
    shellPrint(&shell, "   buffered_length = %d\r\n", lsejtag_lib_ctx.buffered_length);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), cmdbufstatus, cmd_buf_status, Command buffer status);

int exec_cmd(int argc, char **argv) {
    const char *seq = argv[1];
    uint8_t *cmdPtr = lsejtag_lib_ctx.buffered_cmd;
    if (argc != 2) {
        shellPrint(&shell, "Usage: execcmd <Continuous hex bytes of a command>\r\n");
        return 0;
    }

    if (lsejtag_lib_ctx.cmd_buf_state != cbs_clean) {
        shellPrint(&shell, "Command parser is busy\r\n");
        return 1;
    }

    while (*seq != 0) {
        uint8_t byte;
        sscanf(seq, "%02hhx", &byte);
        *cmdPtr = byte;
        ++cmdPtr;
        seq += 2;
    }
    lsejtag_lib_ctx.cmd_buf_state = cbs_execute;

    lsejtag_execute(&lsejtag_lib_ctx);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN), execcmd, exec_cmd, Execute command);
