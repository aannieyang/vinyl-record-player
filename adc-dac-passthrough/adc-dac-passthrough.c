// PCM1808 -> Pico 2 W -> PCM5102A: continuous left-channel passthrough
// with RIAA playback EQ.
// ADC is BCK/LRCK master (MD1=MD0=HIGH, FMT=LOW). Pico supplies MCLK.
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdatomic.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "i2s_duplex.pio.h"

#define MCLK      13   // physical pin 17 -> ADC SCK
#define BCK       20   // physical pin 26 <- ADC BCK; also DAC BCK
#define LRCK      19   // physical pin 25 <- ADC LRC; also DAC LCK
#define ADC_DOUT  21   // physical pin 27 <- ADC OUT
#define DAC_DIN   22   // physical pin 29 -> DAC DIN
#define VOLUME_DIVISOR 8  // retain previous 1/8 volume; 1 = unity gain
#ifndef RIAA_ENABLE
#define RIAA_ENABLE 1   // 1 = RIAA playback EQ, 0 = flat passthrough
#endif
#define SAMPLE_RATE 46875.0 // ADC fs from 12 MHz MCLK (256 * fs)

// PIO WAIT instructions use these fixed GPIO numbers.
_Static_assert(BCK == 20 && LRCK == 19, "Update PIO WAIT pins with clock pins");
_Static_assert(VOLUME_DIVISOR >= 1, "Use attenuation or unity gain");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "Audio requires lock-free word atomics");

static PIO pio = pio0;
static uint sm;
// Left-channel min/max since the last USB print. Sentinels mean "no sample".
static _Atomic int32_t peak_min = INT32_MAX;
static _Atomic int32_t peak_max = INT32_MIN;

static inline int32_t decode_sample(uint32_t raw) {
    uint32_t sample = (raw >> 7) & 0xFFFFFFu;
    return (int32_t)(sample ^ 0x800000u) - 0x800000;
}

// RIAA playback (de-emphasis) EQ: time constants 3180 us, 318 us, 75 us.
// H(s) = (1 + s*318us) / ((1 + s*3180us)(1 + s*75us)), as one biquad.
// The 3180/318 us corners use the prewarped bilinear transform; the 75 us
// pole uses the matched-z transform, which avoids bilinear's extra treble
// rolloff near Nyquist. Normalized to 0 dB at 1 kHz: bass boosted ~+19 dB at
// 20 Hz. Matches ideal RIAA within 0.7 dB to 10 kHz, +2.8 dB at 20 kHz.
static float riaa_b0, riaa_b1, riaa_b2, riaa_a1, riaa_a2;

static void riaa_init(void) {
    // Bilinear: (1 + s*tau) -> (c + d z^-1) / (1 + z^-1), corner prewarped.
    const double k = 2.0 * SAMPLE_RATE;
    double az = 1.0 / tan(1.0 / (318e-6 * k)), ap = 1.0 / tan(1.0 / (3180e-6 * k));
    double c0 = 1.0 + az, d0 = 1.0 - az; // 318 us zero
    double c1 = 1.0 + ap, d1 = 1.0 - ap; // 3180 us pole
    double p = exp(-1.0 / (75e-6 * SAMPLE_RATE)); // 75 us pole: 1 / (1 - p z^-1)
    // The two (1 + z^-1) factors cancel, leaving:
    double b[3] = {c0, d0, 0.0};
    double den[3] = {c1, d1 - c1 * p, -d1 * p};

    // Gain at 1 kHz, to normalize.
    double th = 2.0 * M_PI * 1000.0 / SAMPLE_RATE;
    double nr = b[0] + b[1] * cos(th) + b[2] * cos(2 * th);
    double ni = -(b[1] * sin(th) + b[2] * sin(2 * th));
    double dr = den[0] + den[1] * cos(th) + den[2] * cos(2 * th);
    double di = -(den[1] * sin(th) + den[2] * sin(2 * th));
    double g = sqrt((dr * dr + di * di) / (nr * nr + ni * ni));

    riaa_b0 = (float)(g * b[0] / den[0]);
    riaa_b1 = (float)(g * b[1] / den[0]);
    riaa_b2 = (float)(g * b[2] / den[0]);
    riaa_a1 = (float)(den[1] / den[0]);
    riaa_a2 = (float)(den[2] / den[0]);
}

// Direct Form I biquad (robust in float). Runs only on core 1.
static inline float riaa_process(float x) {
    static float x1, x2, y1, y2;
    float y = riaa_b0 * x + riaa_b1 * x1 + riaa_b2 * x2 - riaa_a1 * y1 - riaa_a2 * y2;
    x2 = x1; x1 = x;
    y2 = y1; y1 = y;
    return y;
}

static inline int32_t scale_sample(float sample) {
    // Volume, then clamp: the RIAA bass boost can exceed PCM24 full scale.
    float v = sample / VOLUME_DIVISOR;
    if (v > 8388607.0f) v = 8388607.0f;
    if (v < -8388608.0f) v = -8388608.0f;
    return (int32_t)v;
}

static inline uint32_t encode_sample(int32_t sample) {
    // This PIO places the I2S delay bit in bit31, then PCM24 in bits30..7.
    return ((uint32_t)sample & 0xFFFFFFu) << 7;
}

static void __not_in_flash_func(audio_loop)(void) {
    while (true) {
        int32_t left = decode_sample(pio_sm_get_blocking(pio, sm));
        (void)pio_sm_get_blocking(pio, sm); // consume right channel, unused
#if RIAA_ENABLE
        uint32_t output = encode_sample(scale_sample(riaa_process((float)left)));
#else
        uint32_t output = encode_sample(scale_sample((float)left));
#endif
        pio_sm_put_blocking(pio, sm, output); // DAC left
        pio_sm_put_blocking(pio, sm, output); // DAC right: same mono signal
        if (left < atomic_load_explicit(&peak_min, memory_order_relaxed))
            atomic_store_explicit(&peak_min, left, memory_order_relaxed);
        if (left > atomic_load_explicit(&peak_max, memory_order_relaxed))
            atomic_store_explicit(&peak_max, left, memory_order_relaxed);
    }
}

int main(void) {
    // Keep existing clock rate: MCLK=12 MHz; ADC produces fs=46,875 Hz.
    set_sys_clock_khz(120000, true);
    riaa_init();
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
    pio_sm_set_consecutive_pindirs(pio, sm, LRCK, 3, false); // LRCK, BCK, DOUT
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
        int32_t lo = atomic_exchange_explicit(&peak_min, INT32_MAX, memory_order_relaxed);
        int32_t hi = atomic_exchange_explicit(&peak_max, INT32_MIN, memory_order_relaxed);
        if (!stdio_usb_connected())
            continue;
        if (hi == INT32_MIN) {
            printf("No sample received; check BCK and LRCK.\n");
        } else {
            // ADC input range over the last 500 ms, before RIAA and volume.
            // Silent input: a few mV is analog noise; big spikes are glitches.
            const double v = 3.0 / 16777216.0;
            printf("Left min %+.4f V  max %+.4f V  p-p %.4f V\n",
                   lo * v, hi * v, (double)(hi - lo) * v);
        }
    }
}
