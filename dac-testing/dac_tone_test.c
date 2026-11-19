// Pico 2 W -> PCM5102A I2S DAC -> LM386 speaker test
// Produces a continuous 1 kHz, low-amplitude sine tone on both DAC channels.
// No ADC, cartridge, or RIAA processing is needed for this test.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "dac_tone_test.pio.h"

#define PIN_LRCK        19u   // Pico physical pin 25 -> DAC LCK/LRCK
#define PIN_BCK         20u   // Pico physical pin 26 -> DAC BCK
#define PIN_DIN         22u   // Pico physical pin 29 -> DAC DIN

#define SAMPLE_RATE     48000u
#define TONE_HZ         1000u
#define TONE_AMPLITUDE  300   // signed 16-bit peak; about 0.9% full scale
#define TABLE_LEN       (SAMPLE_RATE / TONE_HZ) // 48 samples/cycle

static uint32_t stereo_words[TABLE_LEN];

static void fill_tone_table(void) {
    for (uint32_t i = 0; i < TABLE_LEN; ++i) {
        const float phase = 2.0f * 3.14159265358979323846f * (float)i / (float)TABLE_LEN;
        const int16_t s = (int16_t)lroundf((float)TONE_AMPLITUDE * sinf(phase));
        // Two's-complement signed PCM, left in upper half, right in lower half.
        const uint16_t bits = (uint16_t)s;
        stereo_words[i] = ((uint32_t)bits << 16) | bits;
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(300);

    fill_tone_table();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &dac_i2s_program);

    pio_gpio_init(pio, PIN_LRCK);
    pio_gpio_init(pio, PIN_BCK);
    pio_gpio_init(pio, PIN_DIN);

    pio_sm_config c = dac_i2s_program_get_default_config(offset);
    sm_config_set_out_pins(&c, PIN_DIN, 1);
    sm_config_set_sideset_pins(&c, PIN_LRCK);
    sm_config_set_out_shift(&c, false, true, 32); // shift MSB first, autopull 32 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // 16 bits x 2 channels x 2 PIO cycles per I2S bit = 64 PIO cycles/frame.
    const float pio_instruction_hz = (float)SAMPLE_RATE * 64.0f;
    const float divider = (float)clock_get_hz(clk_sys) / pio_instruction_hz;
    sm_config_set_clkdiv(&c, divider);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_LRCK, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_DIN, 1, true);
    pio_sm_set_pins(pio, sm, 0);

    // Start at the PIO program's I2S frame entry point.
    pio_sm_exec(pio, sm, pio_encode_jmp(offset + dac_i2s_offset_entry_point));
    pio_sm_set_enabled(pio, sm, true);

    printf("PCM5102A tone test: 1 kHz, 48 kHz sample rate, low amplitude.\n");
    printf("LRCK=GP19, BCK=GP20, DIN=GP22.\n");

    uint32_t idx = 0;
    while (true) {
        // The TX FIFO blocks at the correct clock rate; no timer or sleep needed.
        pio_sm_put_blocking(pio, sm, stereo_words[idx]);
        idx = (idx + 1u) % TABLE_LEN;
    }
}
