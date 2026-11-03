// Motor control with XY-MOS MOSFET module on Raspberry Pi Pico 2 W
//
// Features:
//   - PWM motor speed control on GP15
//   - Encoder interrupts on GP16 and GP17
//   - PI speed control updated after a specified number of pulses
//   - Runtime adjustment of KP and KI using keyboard arrow keys
//
// Keyboard controls (USB serial terminal):
//   UP     = Increase KP
//   DOWN   = Decrease KP
//   RIGHT  = Increase KI
//   LEFT   = Decrease KI
//
// Wiring:
//   Pico GP15  -> XY-MOS PWM+
//   Pico GP16  -> Encoder Channel A
//   Pico GP17  -> Encoder Channel B
//   Pico GND   -> XY-MOS PWM-
//   Motor PSU+ -> XY-MOS VIN+
//   Motor PSU- -> XY-MOS VIN-
//   Motor      -> XY-MOS OUT+ and OUT-
//
// IMPORTANT:
//   Use an appropriate motor power supply.
//   Do not power the motor directly from a Pico GPIO pin.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

// --------------------------------------------------
// Motor configuration
// --------------------------------------------------

#define MOTOR_PIN 15
#define ENCODER_A_PIN 16
#define ENCODER_B_PIN 17

#define PWM_FREQ_HZ 1000
#define MOTOR_SPEED 100

// --------------------------------------------------
// Encoder and speed configuration
// --------------------------------------------------

#define TARGET_RPM (100.0f / 3.0f)

#define ENCODER_PPR 7.0f
#define GEAR_RATIO 298.0f

#define PULSES_PER_REV (ENCODER_PPR * GEAR_RATIO)

// Number of encoder pulses per control update
#define CONTROL_PULSES 20

// --------------------------------------------------
// PI controller configuration
// --------------------------------------------------

static volatile float KP = 0.5f;
static volatile float KI = 0.15f;

#define KP_STEP 0.05f
#define KI_STEP 0.01f

#define KP_MIN 0.0f
#define KP_MAX 10.0f

#define KI_MIN 0.0f
#define KI_MAX 5.0f

#define BASE_PWM 67.0f

// --------------------------------------------------
// Global variables
// --------------------------------------------------

static uint slice_num;
static uint16_t pwm_wrap;

static volatile uint32_t a_count = 0;
static volatile uint32_t b_count = 0;

// Pulse-window synchronization
static volatile uint32_t pulse_window_count = 0;
static volatile bool control_ready = false;
static volatile uint32_t control_timestamp_us = 0;

static volatile uint32_t window_dt_us = 0;
static uint32_t window_start_us = 0;   // ISR-only


// --------------------------------------------------
// Motor initialization
// --------------------------------------------------

static void motor_init(uint32_t freq_hz)
{
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);

    uint32_t sys_hz = clock_get_hz(clk_sys);

    uint32_t divider =
        sys_hz / (freq_hz * 65535u) + 1;

    pwm_wrap =
        (uint16_t)(sys_hz / (divider * freq_hz) - 1);

    pwm_config cfg = pwm_get_default_config();

    pwm_config_set_clkdiv_int(&cfg, divider);
    pwm_config_set_wrap(&cfg, pwm_wrap);

    pwm_init(slice_num, &cfg, false);

    pwm_set_gpio_level(MOTOR_PIN, 0);
    pwm_set_enabled(slice_num, true);
}


// --------------------------------------------------
// Set motor PWM duty cycle
// --------------------------------------------------

static void motor_set_speed(int percent)
{
    if (percent < 0)
        percent = 0;

    if (percent > 100)
        percent = 100;

    uint32_t level =
        ((uint32_t)percent * (pwm_wrap + 1u)) / 100u;

    pwm_set_gpio_level(MOTOR_PIN, (uint16_t)level);
}


// --------------------------------------------------
// Soft-start motor
// --------------------------------------------------

static void motor_ramp(int start, int end, uint32_t step_delay_ms)
{
    int step = (end >= start) ? 1 : -1;

    for (int p = start; p != end + step; p += step)
    {
        motor_set_speed(p);
        sleep_ms(step_delay_ms);
    }
}


// --------------------------------------------------
// Stop motor
// --------------------------------------------------

static void motor_stop(void)
{
    pwm_set_gpio_level(MOTOR_PIN, 0);
}


// --------------------------------------------------
// Encoder interrupt callback
// --------------------------------------------------

static void encoder_callback(uint gpio, uint32_t events)
{
    if (!(events & GPIO_IRQ_EDGE_RISE))
        return;

    if (gpio == ENCODER_A_PIN)
    {
        a_count++;
        if (++pulse_window_count >= CONTROL_PULSES) {
            uint32_t now = time_us_32();
            window_dt_us = now - window_start_us;
            window_start_us = now;
            pulse_window_count = 0;
            control_ready = true;   // main just consumes window_dt_us
        }
    }
    else if (gpio == ENCODER_B_PIN)
    {
        b_count++;
    }
}


// --------------------------------------------------
// Encoder initialization
// --------------------------------------------------

static void encoder_init(void)
{
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


// --------------------------------------------------
// Keyboard input handler
// --------------------------------------------------

// Standard ANSI arrow-key sequences:
//   ESC [ A = Up
//   ESC [ B = Down
//   ESC [ C = Right
//   ESC [ D = Left

static void handle_serial_input(void)
{
    static int escape_state = 0;

    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (escape_state == 0)
        {
            if (c == 27)
            {
                escape_state = 1;
            }
        }
        else if (escape_state == 1)
        {
            if (c == '[')
            {
                escape_state = 2;
            }
            else
            {
                escape_state = 0;
            }
        }
        else if (escape_state == 2)
        {
            switch (c)
            {
                case 'A':  // UP: increase KP
                    KP += KP_STEP;

                    if (KP > KP_MAX)
                        KP = KP_MAX;

                    printf(
                        "\nKP = %.3f | KI = %.3f\n",
                        (double)KP,
                        (double)KI
                    );
                    break;

                case 'B':  // DOWN: decrease KP
                    KP -= KP_STEP;

                    if (KP < KP_MIN)
                        KP = KP_MIN;

                    printf(
                        "\nKP = %.3f | KI = %.3f\n",
                        (double)KP,
                        (double)KI
                    );
                    break;

                case 'C':  // RIGHT: increase KI
                    KI += KI_STEP;

                    if (KI > KI_MAX)
                        KI = KI_MAX;

                    printf(
                        "\nKP = %.3f | KI = %.3f\n",
                        (double)KP,
                        (double)KI
                    );
                    break;

                case 'D':  // LEFT: decrease KI
                    KI -= KI_STEP;

                    if (KI < KI_MIN)
                        KI = KI_MIN;

                    printf(
                        "\nKP = %.3f | KI = %.3f\n",
                        (double)KP,
                        (double)KI
                    );
                    break;

                default:
                    break;
            }

            escape_state = 0;
        }
    }
}


// --------------------------------------------------
// Main program
// --------------------------------------------------

int main(void)
{
    stdio_init_all();

    motor_init(PWM_FREQ_HZ);
    encoder_init();

    float integral = 0.0f;
    float pwm = BASE_PWM;

    // Soft start
    motor_ramp(0, MOTOR_SPEED, 20);

    // Reset pulse window after soft start.
    uint32_t irq_state = save_and_disable_interrupts();

    pulse_window_count = 0;
    control_ready = false;

    uint32_t last_timestamp_us = time_us_32();

    restore_interrupts(irq_state);

    printf("\nMotor controller started.\n");
    printf("UP/DOWN: adjust KP.\n");
    printf("LEFT/RIGHT: adjust KI.\n");
    printf("Initial KP = %.3f | KI = %.3f\n",
           (double)KP, (double)KI);

    while (true)
    {
        // Wait for encoder pulses while continuing to
        // process keyboard input.
        while (!control_ready)
        {
            handle_serial_input();
            tight_loop_contents();
        }

        // Atomically retrieve the completed pulse window.
        irq_state = save_and_disable_interrupts();

        uint32_t pulses = pulse_window_count;
        uint32_t current_timestamp_us = control_timestamp_us;

        pulse_window_count = 0;
        control_ready = false;

        restore_interrupts(irq_state);

        // Elapsed time in seconds.
        uint32_t elapsed_us =
            current_timestamp_us - last_timestamp_us;

        last_timestamp_us = current_timestamp_us;

        float dt = elapsed_us / 1000000.0f;

        if (dt <= 0.0f)
            continue;

        // Calculate actual RPM.
        float rpm =
            (pulses * 60.0f) / (PULSES_PER_REV * dt);

        // PI controller error.
        float error = TARGET_RPM - rpm;

        float candidate_integral =
            integral + error * dt;

        // Integral anti-windup limit.
        if (candidate_integral > 30.0f / KI && KI > 0.0f)
        {
            candidate_integral = 30.0f / KI;
        }

        if (candidate_integral < -30.0f / KI && KI > 0.0f)
        {
            candidate_integral = -30.0f / KI;
        }

        // Candidate PWM.
        float new_pwm =
            BASE_PWM + KP * error + KI * candidate_integral;

        // Anti-windup.
        if (!((new_pwm > 100.0f && error > 0.0f) ||
              (new_pwm < 0.0f && error < 0.0f)))
        {
            integral = candidate_integral;
        }

        // Final PWM command.
        pwm = BASE_PWM + KP * error + KI * integral;

        if (pwm > 100.0f)
            pwm = 100.0f;

        if (pwm < 0.0f)
            pwm = 0.0f;

        motor_set_speed((int)(pwm + 0.5f));

        // Print motor status and current controller gains.
        printf(
            "Pulses: %lu | dt: %.4f s | "
            "Target: %.2f RPM | Actual: %.2f RPM | "
            "PWM: %.1f%% | KP: %.3f | KI: %.3f\n",
            (unsigned long)pulses,
            (double)dt,
            (double)TARGET_RPM,
            (double)rpm,
            (double)pwm,
            (double)KP,
            (double)KI
        );
    }

    return 0;
}