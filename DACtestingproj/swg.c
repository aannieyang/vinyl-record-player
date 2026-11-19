
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "i2s.pio.h"

// -------------------------------
// Configuration
// -------------------------------

#define SAMPLE_RATE 48000
#define SINE_FREQ   5000.0f
#define AMPLITUDE   0.5f

#define BCLK_PIN    16
#define LRCLK_PIN   17
#define DATA_PIN    18

#define PI 3.14159265358979323846f

// -------------------------------
// Main
// -------------------------------

int main()
{
    stdio_init_all();

    // Initialize PIO
    PIO pio = pio0;
    uint sm = 0;

    uint offset = pio_add_program(pio, &audio_i2s_program);

    pio_sm_config config =
        audio_i2s_program_get_default_config(offset);

    // Data output pin
    sm_config_set_out_pins(&config, DATA_PIN, 1);

    // BCLK and LRCLK side-set pins
    sm_config_set_sideset_pins(&config, BCLK_PIN);

    // MSB first, shift left, autopull every 32 bits
    sm_config_set_out_shift(&config, false, true, 32);

    // Join TX FIFO for additional buffering
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);

    // Set pin directions
    pio_gpio_init(pio, BCLK_PIN);
    pio_gpio_init(pio, LRCLK_PIN);
    pio_gpio_init(pio, DATA_PIN);

    pio_sm_set_consecutive_pindirs(
        pio, sm, BCLK_PIN, 3, true
    );

    // Configure clock divider
    float divider =
        (float)clock_get_hz(clk_sys) /
        (SAMPLE_RATE * 64.0f);

    sm_config_set_clkdiv(&config, divider);

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &config);

    // Start at the I2S entry point
    pio_sm_exec(
        pio, sm,
        pio_encode_jmp(offset + audio_i2s_offset_entry_point)
    );

    int16_t square_table[48];
    const int16_t level = (int16_t)(AMPLITUDE * 32767.0f);

    for (unsigned i = 0; i < 48; ++i) {
        unsigned phase = (5u * i) % 48u;
        square_table[i] = (phase < 24u) ? level : -level;
    }

    int16_t sine_table[48];

    for (unsigned i = 0; i < 48; ++i) {
        float phase = 2.0f * PI * 5.0f * (float)i / 48.0f;
        sine_table[i] = (int16_t)(sinf(phase) * AMPLITUDE * 32767.0f);
    }

    pio_sm_set_enabled(pio, sm, true);

    // -------------------------------
    // Continuously transmit audio
    // -------------------------------

    uint32_t sample_index = 0;

    unsigned i = 0;

    while (true) {
        uint32_t sample = (uint16_t)sine_table[i];
        pio_sm_put_blocking(pio, sm, (sample << 16) | sample);

        if (++i == 48) {
            i = 0;
        }
    }   

    return 0;
}
