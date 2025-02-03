
#include <pico/stdlib.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <tusb.h>
#include <string.h>

#include "shell_port.h"
#include "lsejtag/lsejtag.h"
#include "config.h"
#include "ejtag.h"

#include "ejtag.pio.h"
#include "ws2812.pio.h"

lsejtag_ctx lsejtag_lib_ctx;
ejtag_impl_ctx ejtag_ctx;
Shell shell;
char shellBuffer[512];

void init_hardware();
void init_software();
void init_reset_tap_via_tms();
void init_print_clock();

void led_task();

int main() {
    init_hardware();
    init_software();
    init_reset_tap_via_tms();
    // init_print_clock();

    while (1) {
        tud_task();
        // shellTask(&shell);
        led_task();

        switch (lsejtag_dispatch(&lsejtag_lib_ctx)) {
        case dpr_execute:
            lsejtag_execute(&lsejtag_lib_ctx);
            break;
        case dpr_incomplete:
            break;
        case dpr_flush_tdo:
            lsejtag_flush_tdo(&lsejtag_lib_ctx);
            break;
        case dpr_run_jtag:
            lsejtag_run_jtag(&lsejtag_lib_ctx);
            break;
        case dpr_too_long:
            panic("LSEJTAG command too long");
        case dpr_unknown_cmd:
            break;
        case dpr_corrupt_state:
            printf("!!!!!! dpr_corrput_state !!!!!!\n");
            printf("cmd buf dump:\n");
            // dump_binary_to_console(ctx->buffered_cmd, sizeof(ctx->buffered_cmd));
            printf("buffered_length: %d\r", (int)lsejtag_lib_ctx.buffered_length);
            printf("wait_payload_size: %d\n", (int)lsejtag_lib_ctx.cmd_buf_wait_payload_size);
            printf("state: %d\n",lsejtag_lib_ctx.cmd_buf_state);
            panic("LSEJTAG corrupt state");
        case dpr_clean:
          break;
        }
    }
}

//==============================================================================
// Util
//==============================================================================

static inline uint32_t ws2812_rgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

void isr_hardfault() {
    *ejtag_ctx.ws2812_write_addr = 0x10201000;
    stdio_puts_raw("!! HardFault !!\r\n");
    while (1);
}

//==============================================================================
// Other tasks
//==============================================================================

void led_task() {
    // LED illumination logic
    if (ejtag_ctx.led_turn_on) {
        ejtag_ctx.led_turn_on = 0;
        ejtag_ctx.led_timer_us = time_us_32();
        *ejtag_ctx.ws2812_write_addr = 0x10200000;
        return;
    }

    if (ejtag_ctx.led_timer_us == 0) {
        return;
    }

    // 300ms
    if (time_us_32() - ejtag_ctx.led_timer_us >= 300000) {
        *ejtag_ctx.ws2812_write_addr = 0x00200000;
        ejtag_ctx.led_timer_us = 0;
    }
}

//==============================================================================
// Initialization
//==============================================================================

void init_uart() {
    stdio_uart_init_full(uart0, 115200, 0, 1);

    printf("Pico Loongson EJTAG - By RigoLigo, FW Version " FIRMWARE_VERSION_STR "\n");
    printf("The Loongson EJTAG Probe reimplementation on RP2040.\n");
}

void init_gpio() {
    gpio_init(EJTAG_PIN_TRST);
    gpio_init(EJTAG_PIN_BRST);
    gpio_init(EJTAG_PIN_DINT);
    
    gpio_set_dir(EJTAG_PIN_TRST, true);
    gpio_set_dir(EJTAG_PIN_BRST, true);
    gpio_set_dir(EJTAG_PIN_DINT, true);

    gpio_set_pulls(EJTAG_PIN_TRST, true, false);
    gpio_set_pulls(EJTAG_PIN_BRST, true, false);
    gpio_set_pulls(EJTAG_PIN_DINT, true, false);

    gpio_put(EJTAG_PIN_TRST, true);
    gpio_put(EJTAG_PIN_BRST, true);
    gpio_put(EJTAG_PIN_DINT, true);
}

void init_pio() {
    pio_sm_config smcfg = pio_get_default_sm_config();
    static const char panic_text[] = "%s PIO SM allocation failed. This should not happen.";

    // TDI/TMS SM program
    if (pio_add_program_at_offset(pio0, &ejtag_tdi_program, SM_PC_TDI) < 0) {
        panic(panic_text, "TDI");
    }
    if (pio_add_program_at_offset(pio0, &ejtag_tms_program, SM_PC_TMS) < 0) {
        panic(panic_text, "TMS");
    }
    if (pio_add_program_at_offset(pio0, &ejtag_tdo_program, SM_PC_TDO) < 0) {
        panic(panic_text, "TDO");
    }
    if (pio_add_program_at_offset(pio0, &ws2812_program, SM_PC_WS2812) < 0) {
        panic(panic_text, "WS2812");
    }

    // EJTAG GPIO alternative function selection
    pio_gpio_init(pio0, EJTAG_PIN_TDI);
    pio_gpio_init(pio0, EJTAG_PIN_TDO);
    pio_gpio_init(pio0, EJTAG_PIN_TCK);
    pio_gpio_init(pio0, EJTAG_PIN_TMS);
    pio_gpio_init(pio0, EJTAG_PIN_BRST); // TODO:REMOVE
    pio_gpio_init(pio0, EJTAG_PIN_TDO_SAMPLE_TIME);

    // TDI SM config
    pio_sm_set_consecutive_pindirs(pio0, SM_TDI, EJTAG_PIN_TDI, 1, true);
    pio_sm_set_consecutive_pindirs(pio0, SM_TDI, EJTAG_PIN_TCK, 1, true);
    pio_sm_set_consecutive_pindirs(pio0, SM_TDI, EJTAG_PIN_BRST, 1, true);
    sm_config_set_fifo_join(&smcfg, PIO_FIFO_JOIN_TX);
    sm_config_set_out_pins(&smcfg, EJTAG_PIN_TDI, 1);
    sm_config_set_out_shift(&smcfg, true, false, 0);
    sm_config_set_sideset_pins(&smcfg, EJTAG_PIN_BRST); // TODO:REMOVE
    sm_config_set_sideset(&smcfg, 1, false, false); // TODO:REMOVE
    sm_config_set_wrap(&smcfg, ejtag_tdi_wrap_target + SM_PC_TDI, ejtag_tdi_wrap + SM_PC_TDI);
    sm_config_set_clkdiv(&smcfg, 6.f);
    pio_sm_init(pio0, SM_TDI, SM_PC_TDI, &smcfg);
    ejtag_ctx.tdi_write_addr = &pio0->txf[SM_TDI];

    // TDI SM IRQ (IRQ bit 0, this is for TDI&TMS completion handler since they are in sync)
    irq_set_exclusive_handler(PIO0_IRQ_0, isr_pio0_irq0);
    pio_set_irq0_source_enabled(pio0, pis_interrupt0, true);
    irq_set_enabled(PIO0_IRQ_0, true);

    // TMS SM config
    pio_sm_set_consecutive_pindirs(pio0, SM_TMS, EJTAG_PIN_TMS, 1, true);
    sm_config_set_out_pins(&smcfg, EJTAG_PIN_TMS, 1);
    sm_config_set_sideset_pins(&smcfg, EJTAG_PIN_TCK);
    sm_config_set_sideset(&smcfg, 1, false, false);
    sm_config_set_wrap(&smcfg, ejtag_tms_wrap_target + SM_PC_TMS, ejtag_tms_wrap + SM_PC_TMS);
    pio_sm_init(pio0, SM_TMS, SM_PC_TMS, &smcfg);
    ejtag_ctx.tms_write_addr = &pio0->txf[SM_TMS];

    // TDO SM (PIO0, SM2) config
    pio_sm_set_consecutive_pindirs(pio0, SM_TDO, EJTAG_PIN_TDO, 1, false);
    pio_sm_set_consecutive_pindirs(pio0, SM_TDO, EJTAG_PIN_TDO_SAMPLE_TIME, 1, true);
    sm_config_set_fifo_join(&smcfg, PIO_FIFO_JOIN_NONE);
    sm_config_set_sideset_pins(&smcfg, EJTAG_PIN_TDO_SAMPLE_TIME);
    sm_config_set_in_pins(&smcfg, EJTAG_PIN_TDO);
    sm_config_set_wrap(&smcfg, ejtag_tdo_wrap_target + SM_PC_TDO, ejtag_tdo_wrap + SM_PC_TDO);
    pio_sm_init(pio0, SM_TDO, SM_PC_TDO, &smcfg);
    ejtag_ctx.tdo_write_addr = &pio0->txf[SM_TDO];
    ejtag_ctx.tdo_read_addr = &pio0->rxf[SM_TDO];

    // WS2812 SM (PIO0, SM3) config
    ws2812_program_init(pio0, SM_WS2812, SM_PC_WS2812, WS2812_PIN, 800000, false);
    ejtag_ctx.ws2812_write_addr = &pio0->txf[SM_WS2812];
}

void init_dma() {
    dma_channel_config cfg;

    ejtag_ctx.tdi_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(ejtag_ctx.tdi_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, DREQ_PIO0_TX0 + SM_TDI);
    dma_channel_configure(ejtag_ctx.tdi_chan, &cfg, ejtag_ctx.tdi_write_addr, NULL, 0, false);

    ejtag_ctx.tms_chan = dma_claim_unused_channel(true);
    channel_config_set_dreq(&cfg, DREQ_PIO0_TX0 + SM_TMS);
    channel_config_set_chain_to(&cfg, ejtag_ctx.tms_chan);
    dma_channel_configure(ejtag_ctx.tms_chan, &cfg, ejtag_ctx.tms_write_addr, NULL, 0, false);

    ejtag_ctx.tdo_chan = dma_claim_unused_channel(true);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, DREQ_PIO0_RX0 + SM_TDO);
    channel_config_set_chain_to(&cfg, ejtag_ctx.tdo_chan);
    dma_channel_configure(ejtag_ctx.tdo_chan, &cfg, NULL, ejtag_ctx.tdo_read_addr, 0, false);

    ejtag_ctx.dma_start_mask_no_tdo = (1 << ejtag_ctx.tdi_chan) | (1 << ejtag_ctx.tms_chan);
    ejtag_ctx.dma_start_mask = (1 << ejtag_ctx.tdo_chan) | ejtag_ctx.dma_start_mask_no_tdo;

    // Interrupts
    dma_channel_set_irq0_enabled(ejtag_ctx.tdi_chan, true);
    dma_channel_set_irq0_enabled(ejtag_ctx.tms_chan, true);
    dma_channel_set_irq1_enabled(ejtag_ctx.tdo_chan, true);

    irq_set_exclusive_handler(DMA_IRQ_0, isr_dma_irq0);
    irq_set_exclusive_handler(DMA_IRQ_1, isr_dma_irq1);
    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_enabled(DMA_IRQ_1, true);
}

void init_hardware() {
    init_uart();
    init_gpio();
    init_pio();
    init_dma();
}

void init_task_context() {
    // LED task
    ejtag_ctx.led_timer_us = 0;
    ejtag_ctx.led_turn_on = 0;
    // Make red LED illuminate
    *ejtag_ctx.ws2812_write_addr = 0x00200000;
}

void init_software() {
    lsejtag_init_ctx(&lsejtag_lib_ctx);
    tud_init(BOARD_TUD_RHPORT);
    init_task_context();

    // shell.write = shell_port_write_usb;
    // // shell.read = shell_port_read_usb;
    // shellInit(&shell, shellBuffer, sizeof(shellBuffer));
}

void init_reset_tap_via_tms() {
    *ejtag_ctx.tms_write_addr = 31;
    *ejtag_ctx.tms_write_addr = 0x0FFFFFFF;
    pio_set_sm_mask_enabled(pio0, SM_MASK_TDI_TMS, true);
    sleep_ms(5); // This is more than enough!
    pio_set_sm_mask_enabled(pio0, SM_MASK_JTAG, false);
}

void init_print_clock() {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);

    puts("Clocks debug:");
    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
}
