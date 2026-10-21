// Motor control with XY-MOS MOSFET module on Raspberry Pi Pico 2 W (Pico SDK, C)
//
// Wiring:
//   Pico GP15  -> XY-MOS PWM+ (TRIG/PWM)
//   Pico GND   -> XY-MOS PWM- (GND)   <- common ground is required
//   Motor PSU+ -> XY-MOS VIN+ / DC+
//   Motor PSU- -> XY-MOS VIN- / DC-
//   Motor      -> XY-MOS OUT+ and OUT-
//
// A flyback diode across the motor (cathode to OUT+) is strongly recommended.

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define MOTOR_PIN 15
#define PWM_FREQ_HZ 1000  // XY-MOS handles up to ~20 kHz; raise if the motor whines

static uint slice_num;
static uint16_t pwm_wrap;

static void motor_init(uint32_t freq_hz) {
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);

    uint32_t sys_hz = clock_get_hz(clk_sys);

    // Integer divider chosen so wrap + 1 always fits in 16 bits
    uint32_t divider = sys_hz / (freq_hz * 65535u) + 1;   // 150 MHz, 1 kHz -> 3
    pwm_wrap = (uint16_t)(sys_hz / (divider * freq_hz) - 1);  // -> 49999

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, divider);
    pwm_config_set_wrap(&cfg, pwm_wrap);
    pwm_init(slice_num, &cfg, false);

    pwm_set_gpio_level(MOTOR_PIN, 0);
    pwm_set_enabled(slice_num, true);
}

static void motor_set_speed(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    uint32_t level = ((uint32_t)percent * (pwm_wrap + 1u)) / 100u;
    pwm_set_gpio_level(MOTOR_PIN, (uint16_t)level);
}

// Smoothly change speed from start% to end%.
static void motor_ramp(int start, int end, uint32_t step_delay_ms) {
    int step = (end >= start) ? 1 : -1;
    for (int p = start; p != end + step; p += step) {
        motor_set_speed(p);
        sleep_ms(step_delay_ms);
    }
}

static void motor_stop(void) {
    pwm_set_gpio_level(MOTOR_PIN, 0);
}

#define MOTOR_SPEED 100  // percent, 0–100

int main(void) {
    stdio_init_all();
    motor_init(PWM_FREQ_HZ);

    // Soft start to avoid a big current spike, then hold speed forever
    motor_ramp(0, MOTOR_SPEED, 20);
    printf("Running at %d%%\n", MOTOR_SPEED);

    while (true) {
        tight_loop_contents();
    }
}