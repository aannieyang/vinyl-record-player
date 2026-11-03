#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

// GPIO numbers, not physical header pins
#define STEREO_PIN 26     // pin 31 = ADC0, op amp output (via 0.1uF + 47k/47k bias) in
#define AUDIO_OUT_PIN 16  // pin 21, PWM -> 0.1uF -> LM386 module IN

#define SAMPLE_RATE_HZ 48000
#define PWM_WRAP 1023        // 10-bit output: 150 MHz / 1024 = ~146 kHz carrier, well above audio
#define PWM_MID ((PWM_WRAP + 1) / 2)
#define CAPTURE_LEN 1024     // ~21 ms of audio per capture sent to scope.py

#define DIGITAL_GAIN 1       // raise if too quiet, lower if distorted
#define BEEP_HZ 440
#define BEEP_MS 1000

static uint16_t capture_buf[CAPTURE_LEN];
static volatile uint capture_count = 0;

// runs once per ADC sample: remove DC bias, apply gain, send to the amp, record for scope.py
static void adc_irq_handler(void)
{
    static int32_t dc = 2048 << 8;   // running average of the bias point, 8 fractional bits
    uint16_t sample = adc_fifo_get(); // 12-bit

    dc += (((int32_t)sample << 8) - dc) >> 10;
    int32_t s = ((int32_t)sample - (dc >> 8)) * DIGITAL_GAIN;

    if (s > 2047) s = 2047;
    if (s < -2048) s = -2048;

    pwm_set_gpio_level(AUDIO_OUT_PIN, (uint16_t)((s + 2048) >> 2)); // scale to 10-bit PWM

    if (capture_count < CAPTURE_LEN) {
        capture_buf[capture_count++] = sample;
    }
}

// square-wave beep straight out the PWM to check the output side on its own
static void play_beep(void)
{
    uint32_t half_period_us = 1000000u / (2u * BEEP_HZ);
    uint32_t cycles = (BEEP_HZ * BEEP_MS) / 1000u;
    for (uint32_t i = 0; i < cycles; i++) {
        pwm_set_gpio_level(AUDIO_OUT_PIN, PWM_MID + 150);
        sleep_us(half_period_us);
        pwm_set_gpio_level(AUDIO_OUT_PIN, PWM_MID - 150);
        sleep_us(half_period_us);
    }
    pwm_set_gpio_level(AUDIO_OUT_PIN, PWM_MID);
}

int main()
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("cyw43 init failed\n");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // audio output
    gpio_set_function(AUDIO_OUT_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(AUDIO_OUT_PIN);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_gpio_level(AUDIO_OUT_PIN, PWM_MID); // start at mid-scale (silence)
    pwm_set_enabled(slice, true);

    sleep_ms(500);
    play_beep();
    sleep_ms(300);

    // audio input: ADC free-runs at SAMPLE_RATE_HZ and interrupts on every sample
    adc_init();
    adc_gpio_init(STEREO_PIN);
    adc_select_input(0);
    adc_fifo_setup(true, false, 1, false, false); // FIFO on, no DMA, IRQ at 1 sample, keep 12 bits
    adc_set_clkdiv(48000000.0f / SAMPLE_RATE_HZ - 1); // ADC clock is 48 MHz
    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
    adc_irq_set_enabled(true);
    irq_set_enabled(ADC_IRQ_FIFO, true);
    adc_run(true);

    // main loop: whenever a capture fills up, send it to scope.py and start the next one
    while (1) {
        if (capture_count == CAPTURE_LEN) {
            // format: W:<rate>:s0,s1,s2,... (raw 12-bit counts)
            printf("W:%d:", SAMPLE_RATE_HZ);
            for (int i = 0; i < CAPTURE_LEN; i++) {
                printf(i ? ",%u" : "%u", capture_buf[i]);
            }
            printf("\n");
            sleep_ms(50);
            capture_count = 0;
        }
    }
}