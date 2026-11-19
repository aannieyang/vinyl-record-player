// Raspberry Pi Pico 2 W: PCM1808 -> RIAA EQ -> PCM5102A -> LM386.
// Proof-of-concept: use only left ADC channel and duplicate mono to L+R DAC.
// No PWM / built-in Pico ADC needed. No prints inside audio loop.
// Hardware assumptions and limitations: see README.md.

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "audio_full_duplex.pio.h"

#define PIN_MCLK   13u // physical 17 -> PCM1808 SCKI (input)
#define PIN_LRCK   19u // physical 25 -> PCM1808 LRCK + PCM5102A LCK
#define PIN_BCK    20u // physical 26 -> PCM1808 BCK + PCM5102A BCK
#define PIN_ADC_DO 21u // physical 27 <- PCM1808 DOUT
#define PIN_DAC_DI 22u // physical 29 -> PCM5102A DIN

#define SAMPLE_RATE_HZ 48000u
#define SYS_CLOCK_KHZ 120000u
// 120 MHz / 9.765625 = 12.288 MHz = 256 * 48 kHz
#define CLOCK_DIVIDER 9.765625f
// 64 bits/frame, 4 PIO instructions/bit = 256 instructions/frame
#define PIO_DIVIDER CLOCK_DIVIDER

#define RIAA_ENABLED 1       // set 0 to listen with flat response
#define OUTPUT_GAIN 0.020f   // 2 percent of full scale, start quiet
#define INPUT_LEFT_SECOND 1 // first RX word is nominal right, second left

// Original coefficients from user's audio-proc.c, designed for exactly 48 kHz.
#define RIAA_B0  0.214828211f
#define RIAA_B1 -0.168766096f
#define RIAA_B2 -0.030381978f
#define RIAA_A1 -1.750935179f
#define RIAA_A2  0.752518920f

typedef struct {
    float x1, x2, y1, y2;
} biquad_state_t;

static inline float riaa_filter(float x, biquad_state_t *s) {
    float y = RIAA_B0*x + RIAA_B1*s->x1 + RIAA_B2*s->x2
              - RIAA_A1*s->y1 - RIAA_A2*s->y2;
    s->x2 = s->x1;
    s->x1 = x;
    s->y2 = s->y1;
    s->y1 = y;
    return y;
}

static inline uint32_t pcm32_from_float(float v) {
    // Clamp to keep DAC output predictable. Keep headroom for EQ peaks.
    if (v > 0.95f) v = 0.95f;
    if (v < -0.95f) v = -0.95f;
    // PCM1808 data is 24-bit MSB-justified in 32-bit slots;
    // DAC receives signed PCM24 left-justified in 32-bit slots as well.
    int32_t signed24 = (int32_t)(v * 8388607.0f);
    return ((uint32_t)signed24) << 8; // preserve sign/two's complement
}

int main(void) {
    // Use the same fractional divider for MCLK and PIO audio clocks.
    // Check real clocks with scope: MCLK 12.288MHz, LRCK 48kHz, BCK 3.072MHz.
    if (!set_sys_clock_khz(SYS_CLOCK_KHZ, true)) {
        panic("Cannot set 120 MHz clock");
    }
    stdio_init_all();

    // PCM1808 must have slave mode (MD0=0, MD1=0) and I2S (FMT=0).
    // Feed the ADC its 256*fs master clock.
    clock_gpio_init(PIN_MCLK,
        CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, CLOCK_DIVIDER);

    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &audio_full_duplex_program);

    pio_gpio_init(pio, PIN_LRCK);
    pio_gpio_init(pio, PIN_BCK);
    pio_gpio_init(pio, PIN_ADC_DO);
    pio_gpio_init(pio, PIN_DAC_DI);

    pio_sm_config c = audio_full_duplex_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, PIN_LRCK);
    sm_config_set_out_pins(&c, PIN_DAC_DI, 1);
    sm_config_set_in_pins(&c, PIN_ADC_DO);
    sm_config_set_out_shift(&c, false, true, 32); // MSB-first TX, pull each word
    sm_config_set_in_shift(&c, false, true, 32);  // MSB-first RX, push each word
    sm_config_set_clkdiv(&c, PIO_DIVIDER);
    // Do not join FIFOs: both RX and TX are used.
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_LRCK, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_DAC_DI, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_ADC_DO, 1, false);

    // Preload silence. Each stereo frame consists of right then left words.
    for (int i = 0; i < 4; ++i) pio_sm_put(pio, sm, 0);
    pio_sm_set_enabled(pio, sm, true);

    biquad_state_t eq = {0};
    uint32_t frame_count = 0;
    while (true) {
        // Read a complete stereo ADC frame, then produce a stereo DAC frame.
        uint32_t first_rx = pio_sm_get_blocking(pio, sm);
        uint32_t second_rx = pio_sm_get_blocking(pio, sm);
        uint32_t selected = INPUT_LEFT_SECOND ? second_rx : first_rx;
        // PCM1808 24-bit I2S: 24 signed MSB-first bits + 8 padding bits.
        // As a signed 32-bit word, scale by 2^31 to roughly [-1, 1).
        float x = (float)(int32_t)selected * (1.0f / 2147483648.0f);
#if RIAA_ENABLED
        x = riaa_filter(x, &eq);
#endif
        uint32_t out = pcm32_from_float(x * OUTPUT_GAIN);
        pio_sm_put_blocking(pio, sm, out); // right DAC slot
        pio_sm_put_blocking(pio, sm, out); // left DAC slot
        ++frame_count;
        // No printf/sleep in the real-time audio path.
    }
}
