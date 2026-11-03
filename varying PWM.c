#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

#define PWM_PIN 15

int main() {
    stdio_init_all();

    // Configure GPIO 15 for PWM
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Set PWM frequency to approximately 1 kHz
    pwm_set_clkdiv(slice_num, 125.0f);
    pwm_set_wrap(slice_num, 999);

    // Start at 100%
    int pwm_value = 1000;

    pwm_set_gpio_level(PWM_PIN, pwm_value);
    pwm_set_enabled(slice_num, true);

    // Wait for USB serial connection to initialize
    sleep_ms(2000);

    while (true) {
        printf("PWM Duty Cycle: %d%%\n", pwm_value / 10);

        sleep_ms(2000);

        if (pwm_value > 0) {
            pwm_value -= 100;  // Decrease by 10 percentage points
            pwm_set_gpio_level(PWM_PIN, pwm_value);
        } else {
            printf("PWM reached 0%%. Stopping.\n");
            break;
        }
    }

    return 0;
}