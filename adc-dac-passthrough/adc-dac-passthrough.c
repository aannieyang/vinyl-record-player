// PCM1808 ADC -> Pico 2 W -> PCM5102A DAC -> LM386 -> speaker
//
// Core 1: real-time audio loop. Reads each ADC frame from the PIO RX FIFO,
//         applies volume, writes the left channel to both DAC channels.
// Core 0: prints 256-sample snapshots of the left channel over USB as CSV
//         lines (same format as adc-testing, so plot_adc.py still works).
//
// The ADC and DAC share LRCK/BCK, so one PIO state machine drives both.

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "i2s_duplex.pio.h"

#define MCLK      13   // -> PCM1808 SCKI
#define LRCK      19   // -> PCM1808 LRCK + PCM5102A LCK
#define BCK       20   // -> PCM1808 BCK  + PCM5102A BCK
#define ADC_DOUT  21   // <- PCM1808 DOUT
#define DAC_DIN   22   // -> PCM5102A DIN

// 120 MHz / 10 = 12 MHz MCLK = 256 * fs, fs = 46.875 kHz (same as adc-testing)
#define SYS_KHZ   120000
#define CLK_DIV   10.0f

// Output volume in Q8: 256 = unity (ADC full scale -> DAC full scale).
// Start low and raise once the LM386 volume is set sensibly.
#define VOLUME_Q8 32   // 1/8 = -18 dB

#define NUM_SAMPLES 256

// 1 = print a status line instead of CSV (frames processed, raw ADC words,
//     FIFO levels). Use with a serial monitor, not plot_adc.py.
#define DEBUG_STATUS 0

// Ring of decoded left-channel samples, written by core 1, read by core 0
#define MON_WORDS 4096
static volatile int32_t mon_ring[MON_WORDS];
static volatile uint32_t mon_head;   // total samples written
static volatile uint32_t last_raw_l, last_raw_r;

static int32_t samples[NUM_SAMPLES];

static PIO pio = pio0;
static uint sm;

static inline int32_t decode_sample(uint32_t raw) {
    uint32_t sample = (raw >> 7) & 0xFFFFFF;
    if (sample & 0x800000)
        sample |= 0xFF000000;
    return (int32_t)sample;
}

// 24-bit signed sample -> TX word with sample in bits 30..7 (see .pio)
static inline uint32_t encode_sample(int32_t s) {
    return (uint32_t)(((int32_t)((uint32_t)s << 8)) >> 1);
}

static void __not_in_flash_func(core1_audio)(void) {
    while (true) {
        uint32_t raw_l = pio_sm_get_blocking(pio, sm);
        uint32_t raw_r = pio_sm_get_blocking(pio, sm);   // right, unused
        last_raw_l = raw_l;
        last_raw_r = raw_r;
        int32_t left = decode_sample(raw_l);

        int64_t v = ((int64_t)left * VOLUME_Q8) >> 8;
        if (v >  8388607) v =  8388607;
        if (v < -8388608) v = -8388608;

        uint32_t out = encode_sample((int32_t)v);
        pio_sm_put_blocking(pio, sm, out);    // DAC left
        pio_sm_put_blocking(pio, sm, out);    // DAC right

        uint32_t h = mon_head;
        mon_ring[h & (MON_WORDS - 1)] = left;
        __dmb();
        mon_head = h + 1;
    }
}

int main() {
    set_sys_clock_khz(SYS_KHZ, true);
    stdio_init_all();

    // 12 MHz master clock for the PCM1808
    clock_gpio_init(MCLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, CLK_DIV);

    sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &i2s_duplex_program);

    pio_gpio_init(pio, LRCK);
    pio_gpio_init(pio, BCK);
    pio_gpio_init(pio, ADC_DOUT);
    pio_gpio_init(pio, DAC_DIN);

    pio_sm_config c = i2s_duplex_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, LRCK);
    sm_config_set_in_pins(&c, ADC_DOUT);
    sm_config_set_out_pins(&c, DAC_DIN, 1);
    sm_config_set_in_shift(&c, false, true, 32);    // MSB first, autopush
    sm_config_set_out_shift(&c, false, true, 32);   // MSB first, autopull
    sm_config_set_clkdiv(&c, CLK_DIV);              // 12 MHz PIO -> 3 MHz BCK
    // FIFOs not joined: both RX and TX are in use

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_consecutive_pindirs(pio, sm, LRCK, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, DAC_DIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, ADC_DOUT, 1, false);
    pio_sm_set_pins(pio, sm, 0);

    // Prefill silence (even count keeps L/R slots aligned)
    for (int i = 0; i < 4; i++)
        pio_sm_put(pio, sm, 0);

    multicore_launch_core1(core1_audio);
    pio_sm_set_enabled(pio, sm, true);

    // Audio is already running; printing waits for the serial monitor
    while (!stdio_usb_connected())
        sleep_ms(100);

    sleep_ms(1000);   // let the ADC settle

#if DEBUG_STATUS
    while (true) {
        printf("frames=%lu raw_L=%08lx raw_R=%08lx rx_lvl=%u tx_lvl=%u pc=%u\n",
               (unsigned long)mon_head,
               (unsigned long)last_raw_l, (unsigned long)last_raw_r,
               pio_sm_get_rx_fifo_level(pio, sm),
               pio_sm_get_tx_fifo_level(pio, sm),
               pio_sm_get_pc(pio, sm) - offset);
        sleep_ms(500);
    }
#endif

    while (true) {
        uint32_t head = mon_head;
        uint32_t start = head - NUM_SAMPLES;

        for (int i = 0; i < NUM_SAMPLES; i++)
            samples[i] = mon_ring[(start + i) & (MON_WORDS - 1)];

        // Discard if core 1 overwrote the snapshot while we copied
        if (mon_head - start > MON_WORDS)
            continue;

        for (int i = 0; i < NUM_SAMPLES; i++) {
            printf("%ld%c", (long)samples[i],
                   i == NUM_SAMPLES - 1 ? '\n' : ',');
        }

        sleep_ms(30);
    }
}
