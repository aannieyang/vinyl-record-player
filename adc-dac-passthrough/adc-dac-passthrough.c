// PCM1808 -> Pico 2 W -> PCM5102A: continuous left-channel passthrough.
// ADC is BCK/LRCK master (MD1=MD0=HIGH, FMT=LOW). Pico supplies MCLK.
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "i2s_duplex.pio.h"

#define MCLK      21   // physical pin 27 -> ADC SCKI
#define BCK       16   // physical pin 21 <- ADC BCK; also connected to DAC BCK
#define LRCK      17   // physical pin 22 <- ADC LRCK; also connected to DAC LCK
#define ADC_DOUT  18   // physical pin 24 <- ADC DOUT
#define DAC_DIN   19   // physical pin 25 -> DAC DIN
#define VOLUME_DIVISOR 8  // retain previous 1/8 volume; 1 = unity gain

// PIO WAIT instructions use these fixed GPIO numbers.
_Static_assert(BCK == 16 && LRCK == 17, "Update PIO WAIT pins with clock pins");
_Static_assert(VOLUME_DIVISOR >= 1, "Use attenuation or unity gain");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "Audio requires lock-free word atomics");

static PIO pio = pio0;
static uint sm;
// One latest sample for USB display. INT32_MIN cannot be a signed PCM24 sample.
static _Atomic int32_t latest_left = INT32_MIN;

static inline int32_t decode_sample(uint32_t raw) {
    uint32_t sample = (raw >> 7) & 0xFFFFFFu;
    return (int32_t)(sample ^ 0x800000u) - 0x800000;
}

static inline int32_t scale_sample(int32_t sample) {
    return sample / VOLUME_DIVISOR;
}

static inline uint32_t encode_sample(int32_t sample) {
    // This PIO places the I2S delay bit in bit31, then PCM24 in bits30..7.
    return ((uint32_t)sample & 0xFFFFFFu) << 7;
}

static void __not_in_flash_func(audio_loop)(void) {
    while (true) {
        int32_t left = decode_sample(pio_sm_get_blocking(pio, sm));
        (void)pio_sm_get_blocking(pio, sm); // consume right channel, unused
        uint32_t output = encode_sample(scale_sample(left));
        pio_sm_put_blocking(pio, sm, output); // DAC left
        pio_sm_put_blocking(pio, sm, output); // DAC right: same mono signal
        atomic_store_explicit(&latest_left, left, memory_order_relaxed);
    }
}

int main(void) {
    // Keep existing clock rate: MCLK=12 MHz; ADC produces fs=46,875 Hz.
    set_sys_clock_khz(120000, true);
    stdio_init_all();
    clock_gpio_init(MCLK, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 10.0f);

    sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &i2s_duplex_program);
    pio_gpio_init(pio, BCK);
    pio_gpio_init(pio, LRCK);
    pio_gpio_init(pio, ADC_DOUT);
    pio_gpio_init(pio, DAC_DIN);

    pio_sm_config config = i2s_duplex_program_get_default_config(offset);
    sm_config_set_in_pins(&config, ADC_DOUT);
    sm_config_set_out_pins(&config, DAC_DIN, 1);
    sm_config_set_in_shift(&config, false, true, 32);
    sm_config_set_out_shift(&config, false, true, 32);
    sm_config_set_clkdiv(&config, 1.0f); // WAIT instructions follow ADC clocks
    pio_sm_init(pio, sm, offset, &config);
    pio_sm_set_consecutive_pindirs(pio, sm, BCK, 3, false); // BCK, LRCK, DOUT
    pio_sm_set_consecutive_pindirs(pio, sm, DAC_DIN, 1, true);
    pio_sm_set_pins_with_mask(pio, sm, 0, 1u << DAC_DIN);

    // Seed two silent frames before the receive-then-transmit loop starts.
    for (int i = 0; i < 4; ++i)
        pio_sm_put(pio, sm, 0);
    multicore_launch_core1(audio_loop);
    pio_sm_set_enabled(pio, sm, true);

    // Like ADCTesting.c, print input signal voltage. USB never blocks audio.
    sleep_ms(1000);
    while (true) {
        sleep_ms(500);
        int32_t sample = atomic_exchange_explicit(&latest_left, INT32_MIN,
                                                  memory_order_relaxed);
        if (!stdio_usb_connected())
            continue;
        if (sample == INT32_MIN) {
            printf("No sample received; check BCK and LRCK.\n");
        } else {
            // Nominal AC-equivalent ADC input voltage, before digital volume.
            double volts = sample * (3.0 / 16777216.0);
            printf("Left: %+.3f V\n", volts);
        }
    }
}
