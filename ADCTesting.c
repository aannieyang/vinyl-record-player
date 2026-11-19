#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pcm1808_rx.pio.h"

enum {
    BCK_PIN = 16,
    LRCK_PIN = 17,
    DATA_PIN = 18,
    WINDOW_SAMPLES = 2400   // 50 ms at 48 kHz
};

// Nominal conversion at the PCM1808 input, before any breakout gain.
static const double VOLTS_PER_COUNT = 3.0 / 16777216.0;

int main(void)
{
    // Set clk_sys to 61.44 MHz: 1536 MHz / 5 / 5.
    set_sys_clock_pll(1536000000u, 5, 5);

    // Output clk_sys / 5 = 12.288 MHz on GP21.
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 5.0f);

    stdio_init_all();

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &pcm1808_rx_program);

    pio_gpio_init(pio, BCK_PIN);
    pio_gpio_init(pio, LRCK_PIN);
    pio_gpio_init(pio, DATA_PIN);

    pio_sm_config config =
        pcm1808_rx_program_get_default_config(offset);

    sm_config_set_in_pins(&config, DATA_PIN);
    sm_config_set_in_shift(&config, false, true, 24);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, sm, offset, &config);
    pio_sm_set_consecutive_pindirs(pio, sm, BCK_PIN, 3, false);

    while (true) {
        // Start at the beginning of the PIO program for a fresh sample.
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
        pio_sm_exec(pio, sm, pio_encode_jmp(offset));
        pio_sm_set_enabled(pio, sm, true);

        uint64_t deadline = time_us_64() + 100000u;  // 100 ms timeout
        while (pio_sm_is_rx_fifo_empty(pio, sm) &&
            time_us_64() < deadline) {
            tight_loop_contents();
        }

        pio_sm_set_enabled(pio, sm, false);

        if (pio_sm_is_rx_fifo_empty(pio, sm)) {
            printf("No sample received; check BCK and LRC.\n");
            sleep_ms(500);
            continue;
        }

        uint32_t raw = pio_sm_get(pio, sm) & 0xFFFFFFu;
        int32_t sample = (int32_t)raw;
        if (raw & 0x800000u) {
            sample -= 0x1000000;
        }

        double volts = sample * VOLTS_PER_COUNT;
        printf("Left: %+.3f V\n", volts);

        sleep_ms(50);
    }
}