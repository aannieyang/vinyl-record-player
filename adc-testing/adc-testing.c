#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pcm1808.pio.h"

#define MCLK 13
#define LRCK 19
#define BCK  20
#define DOUT 21

#define NUM_SAMPLES 256

// Circular DMA buffer: 4096 words, 16384 bytes
#define RING_WORDS 4096
#define RING_BITS 14

static volatile uint32_t ring_buffer[RING_WORDS]
    __attribute__((aligned(16384)));

static int32_t samples[NUM_SAMPLES];

int32_t decode_sample(uint32_t raw) {
    uint32_t sample = (raw >> 7) & 0xFFFFFF;

    if (sample & 0x800000)
        sample |= 0xFF000000;

    return (int32_t)sample;
}

int main() {
    set_sys_clock_khz(120000, true);
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // Generate 12 MHz MCLK
    clock_gpio_init(
        MCLK,
        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS,
        10.0f
    );

    // Initialize PIO
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &i2s_rx_program);

    pio_gpio_init(pio, LRCK);
    pio_gpio_init(pio, BCK);
    pio_gpio_init(pio, DOUT);

    pio_sm_config c =
        i2s_rx_program_get_default_config(offset);

    sm_config_set_sideset_pins(&c, LRCK);
    sm_config_set_in_pins(&c, DOUT);

    sm_config_set_in_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, 20.0f);

    pio_sm_init(pio, sm, offset, &c);

    pio_sm_set_consecutive_pindirs(
        pio, sm, LRCK, 2, true
    );

    pio_sm_set_consecutive_pindirs(
        pio, sm, DOUT, 1, false
    );

    // ---------- DMA ----------

    int dma = dma_claim_unused_channel(true);

    dma_channel_config dc =
        dma_channel_get_default_config(dma);

    channel_config_set_transfer_data_size(
        &dc, DMA_SIZE_32
    );

    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);

    // Transfer whenever PIO has received a word
    channel_config_set_dreq(
        &dc, pio_get_dreq(pio, sm, false)
    );

    // Wrap the DMA write address around the buffer
    channel_config_set_ring(&dc, true, RING_BITS);

    dma_channel_configure(
        dma,
        &dc,
        ring_buffer,        // Destination
        &pio->rxf[sm],      // Source
        dma_encode_endless_transfer_count(),
        true
    );

    // Start I2S
    pio_sm_set_enabled(pio, sm, true);

    // Let ADC settle while DMA keeps running
    sleep_ms(1000);

    while (true) {

        // Find where DMA is currently writing
        uintptr_t addr =
            dma_channel_hw_addr(dma)->write_addr;

        uint32_t pos =
            (addr - (uintptr_t)ring_buffer) / 4;

        // Align to a stereo frame
        pos &= ~1u;

        // Select 256 completed frames, with a guard
        uint32_t start =
            (pos - 2 * NUM_SAMPLES - 16)
            & (RING_WORDS - 1);

        // Copy left channel into a separate buffer
        uint64_t copy_start = time_us_64();

        for (int i = 0; i < NUM_SAMPLES; i++) {
            uint32_t idx =
                (start + 2 * i) & (RING_WORDS - 1);

            samples[i] = decode_sample(ring_buffer[idx]);
        }

        // Discard snapshot if copying took too long
        if (time_us_64() - copy_start > 10000)
            continue;

        // Send CSV line to Python
        for (int i = 0; i < NUM_SAMPLES; i++) {
            printf("%ld%c",
                (long)samples[i],
                i == NUM_SAMPLES - 1 ? '\n' : ','
            );
        }

        sleep_ms(30);
    }
}