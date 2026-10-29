// Motor control with XY-MOS MOSFET module on Raspberry Pi Pico 2 W (Pico SDK, C)
//
// Wiring:
//   Pico GP15  -> XY-MOS PWM+ (TRIG/PWM)
//   Pico GP16  -> Encoder Channel A Output
//   Pico GP17  -> Encoder Channel B Output
//   Pico GND   -> XY-MOS PWM- (GND)   <- common ground is required
//   Motor PSU+ -> XY-MOS VIN+ / DC+
//   Motor PSU- -> XY-MOS VIN- / DC-
//   Motor      -> XY-MOS OUT+ and OUT-
//
// A flyback diode across the motor (cathode to OUT+) is strongly recommended.

#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define MOTOR_PIN 15
#define ENCODER_A_PIN 16
#define ENCODER_B_PIN 17
#define PWM_FREQ_HZ 1000  // XY-MOS handles up to ~20 kHz; raise if the motor whines
#define MOTOR_SPEED 100  // percent, 0–100

// Encoder & RPM
#define TARGET_RPM (100.0f / 3.0f)
#define ENCODER_PPR 7.0f
#define GEAR_RATIO 298.0f
#define PULSES_PER_REV (ENCODER_PPR * GEAR_RATIO)

#define KP 0.5f
#define KI 0.15f

#define CONTROL_INTERVAL_MS 500
#define BASE_PWM 67.0f

static uint slice_num;
static uint16_t pwm_wrap;
static volatile uint32_t a_count = 0;
static volatile uint32_t b_count = 0;


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

//
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

//
static void motor_stop(void) {
    pwm_set_gpio_level(MOTOR_PIN, 0);
}

static void encoder_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == ENCODER_A_PIN) {
            a_count++;
        } else if (gpio == ENCODER_B_PIN) {
            b_count++;
        }
    }
}

// Initialize encoder pins
static void encoder_init(void){
    // Set encoder A and B as inputs
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);
    
    gpio_set_irq_enabled_with_callback(
        ENCODER_A_PIN,
        GPIO_IRQ_EDGE_RISE,
        true,
        encoder_callback
    );

    gpio_set_irq_enabled(
        ENCODER_B_PIN,
        GPIO_IRQ_EDGE_RISE,
        true
    );
}

int main(void) {
    stdio_init_all();

    motor_init(PWM_FREQ_HZ);
    encoder_init();

    float integral = 0.0f;
    float pwm = BASE_PWM;

    uint32_t last_count = a_count;

    // Soft start to avoid a big current spike, then hold speed forever
    motor_ramp(0, MOTOR_SPEED, 20);
    // printf("Running at %d%%\n", MOTOR_SPEED);

    while (true) {
        // wait one second
        sleep_ms(CONTROL_INTERVAL_MS);

        // Measure actual motor speed
        uint32_t current_count = a_count;
        uint32_t pulses = current_count - last_count;
        last_count = current_count;

        float dt = CONTROL_INTERVAL_MS / 1000.0f;
        float rpm = (pulses * 60.0f) / PULSES_PER_REV;

        // Calculate error
        float error = TARGET_RPM - rpm; 

        // Integral term 
        float candidate_integral = integral + error * dt; 

        // Keep integral correction within 30 PWM points for stability
        if (candidate_integral > 30.0f / KI)
            candidate_integral = 30.0f / KI;

        if (candidate_integral < -30.0f / KI)
            candidate_integral = -30.0f / KI;

        // Calculate PWM correction
        float new_pwm = BASE_PWM + KP * error + KI * candidate_integral; 
        
        if (!((new_pwm > 100.0f && error > 0) || (new_pwm < 0.0f && error < 0))) 
        {
            integral = candidate_integral;
        }

        pwm = BASE_PWM + KP * error + KI * integral;

        // Limit PWM to valid range
        if (pwm > 100.0f) pwm = 100.0f;
        if (pwm < 0.0f) pwm = 0;

        motor_set_speed((int)(pwm + 0.5f));
        
        // Print stats
        printf("Target: %.2f | Actual: %.2f | PWM: %.1f%%\n",
               TARGET_RPM, rpm, pwm);
    }
}