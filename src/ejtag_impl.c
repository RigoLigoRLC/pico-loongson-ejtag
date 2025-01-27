
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <tusb.h>
#include "ejtag.h"
#include "config.h"
#include "lsejtag/lsejtag_impl.h"
#include "usb_descriptors.h"

#define BIT2DWORD(x) (((x) + 31) / 32)

void dump_binary_to_console(const uint8_t * const data, uint32_t length)
{
#ifndef DISABLE_DBGPRINT
  char HexBuf[54] = {' ', ' ', 0};
  uint32_t BytesLeft = length, Offset = 0;

  while (BytesLeft)
  {
    /* Print in 16 byte groups */
    uint32_t BytesToPrint = (BytesLeft > 16) ? 16 : BytesLeft;
    HexBuf[2] = '\0';
    for (int i = 0; i < BytesToPrint; i++)
    {
      char Tmp[3];
      sprintf(Tmp, "%02X", data[Offset + i]);
      strcat(HexBuf, Tmp);
      if (i == 7 && length != 7)
        strcat(HexBuf, "-");
      else
        strcat(HexBuf, " ");
    }
    // strcat(HexBuf, "\r\n");

    // uint32_t len = strlen(HexBuf);
    // while (tud_cdc_write_available() < len)
    // {
    //   tud_task();
    // }
    // tud_cdc_write_str(HexBuf);
    puts(HexBuf);

    BytesLeft -= BytesToPrint;
    Offset += BytesToPrint;
  }
#endif
}

uint32_t lsejtag_impl_usbrx_len() {
    return tud_vendor_n_available(ITF_NUM_EJTAG);
}

uint32_t lsejtag_impl_usbtx_free_len() {
    return tud_vendor_n_write_available(ITF_NUM_EJTAG);
}

void lsejtag_impl_usbrx_consume(uint8_t *dest, uint32_t len) {
    tud_vendor_n_read(ITF_NUM_EJTAG, dest, len);
    // tud_cdc_write_str(" - EJTAG RX:\r\n");
    // dump_binary_to_console(dest, len);
    // tud_cdc_write_flush();
}

void lsejtag_impl_usbtx(const uint8_t *data, uint32_t len) {
    tud_vendor_n_write(ITF_NUM_EJTAG, data, len);
    tud_vendor_flush();
}

void lsejtag_impl_io_manip(lsejtag_impl_io io, uint8_t level) {
    switch (io) {
    case impl_io_led:
        *ejtag_ctx.ws2812_write_addr = level ? 0x10200000 : 0x00200000;
        break;
    case impl_io_trst:
        gpio_put(EJTAG_PIN_TRST, level == 1);
        break;
    case impl_io_brst:
        gpio_put(EJTAG_PIN_BRST, level == 1);
        break;
    case impl_io_dint:
        gpio_put(EJTAG_PIN_DINT, level == 1);
        break;
    default:
        break;
    }
}

void lsejtag_impl_run_jtag_setup(uint32_t tdi_bits, uint32_t tdo_bits, uint32_t tdo_skip_bits) {
    pio_set_sm_mask_enabled(pio0, 0x7, false);
    // Because of how PIO program is implemented, the counter values are always minus 1 when written
    // to PIO
    if (tdo_bits) {
        *ejtag_ctx.tdo_write_addr = (tdo_skip_bits * 4 - 1);
        *ejtag_ctx.tdo_write_addr = BIT2DWORD(tdo_bits) * 32 - 1;
    }
    ejtag_ctx.tdo_recv_dword_count = BIT2DWORD(tdo_bits);
    *ejtag_ctx.tdi_write_addr = tdi_bits - 1;
    *ejtag_ctx.tms_write_addr = tdi_bits - 1;
}

// For debug purposes
static int tdiCount = 0, tdoCount = 0, tmsCount = 0;

void lsejtag_impl_run_jtag(const uint32_t *tdi_buf, const uint32_t *tms_buf, uint32_t *tdo_buf,
                           uint32_t tdi_bits, uint32_t tdo_bits, uint32_t tdo_skip_bits) {
    //
    lsejtag_impl_run_jtag_setup(tdi_bits, tdo_bits, tdo_skip_bits);

    dma_hw->ch[ejtag_ctx.tdi_chan].read_addr = (uint32_t)tdi_buf;
    dma_hw->ch[ejtag_ctx.tdi_chan].transfer_count = BIT2DWORD(tdi_bits);
    dma_hw->ch[ejtag_ctx.tms_chan].read_addr = (uint32_t)tms_buf;
    dma_hw->ch[ejtag_ctx.tms_chan].transfer_count = BIT2DWORD(tdi_bits);

    DbgPrint("RunJtag TDI=%d TDO=%d TMS=%d\n", ++tdiCount, ++tdoCount, ++tmsCount);
    DbgPrint("TDI Seq (%08lX):\n", tdi_bits - 1);
    dump_binary_to_console((uint8_t *)tdi_buf, BIT2DWORD(tdi_bits) * 4);
    DbgPrint("TMS Seq (%08lX):\n", tdi_bits - 1);
    dump_binary_to_console((uint8_t *)tms_buf, BIT2DWORD(tdi_bits) * 4);
    if (tdo_bits) {
        DbgPrint("TDO Seq: %08lX  %08lX\n", (tdo_skip_bits * 4 - 1), BIT2DWORD(tdo_bits) * 32 - 1);
    }

    if (tdo_bits) {
        dma_hw->ch[ejtag_ctx.tdo_chan].write_addr = (uint32_t)tdo_buf;
        dma_hw->ch[ejtag_ctx.tdo_chan].transfer_count = BIT2DWORD(tdo_bits);
        dma_start_channel_mask(ejtag_ctx.dma_start_mask);
        pio_enable_sm_mask_in_sync(pio0, 0x7);
    } else {
        dma_start_channel_mask(ejtag_ctx.dma_start_mask_no_tdo);
        pio_enable_sm_mask_in_sync(pio0, 0x3);
    }

    ejtag_ctx.led_turn_on = 1;
}

void isr_dma_irq0() {
    DbgPrint("isr_dma_irq0 triggered\n");
    if (dma_irqn_get_channel_status(0, ejtag_ctx.tdi_chan)) {
        dma_irqn_acknowledge_channel(0, ejtag_ctx.tdi_chan);
        DbgPrint("[% 8d] TDI DMA cplt\n", tdiCount);
    }
    if (dma_irqn_get_channel_status(0, ejtag_ctx.tms_chan)) {
        dma_irqn_acknowledge_channel(0, ejtag_ctx.tms_chan);
        DbgPrint("[% 8d] TMS DMA cplt\n", tmsCount);
    }
}

void isr_dma_irq1() {
    DbgPrint("isr_dma_irq1 triggered\n");
    if (dma_irqn_get_channel_status(1, ejtag_ctx.tdo_chan)) {
        dma_irqn_acknowledge_channel(1, ejtag_ctx.tdo_chan);
        lsejtag_jtag_complete_tdo(&lsejtag_lib_ctx, ejtag_ctx.tdo_recv_dword_count);
        DbgPrint("[% 8d] TDO DMA cplt\n", tdoCount);
        dump_binary_to_console((uint8_t *)lsejtag_lib_ctx.tdo_in_data,ejtag_ctx.tdo_recv_dword_count * 4);
    }
}

void isr_pio0_irq0() {
    // since TDI and TMS are in sync and both transfer same amount of data, we can safely assume
    // their transfer has both completed
    DbgPrint("[% 8d] TDI SM xfer cplt\n", tmsCount);
    lsejtag_jtag_complete_tdi(&lsejtag_lib_ctx);
    lsejtag_jtag_complete_tms(&lsejtag_lib_ctx);
    pio_sm_exec(pio0, 0, pio_encode_irq_clear(false, 0));
}
