
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
// Encoder configuration
// --------------------------------------------------

#define ENCODER_PPR 7.0f
#define GEAR_RATIO 298.0f

#define PULSES_PER_REV (ENCODER_PPR * GEAR_RATIO)
#define CONTROL_PULSES 20

// --------------------------------------------------
// Controller configuration
// --------------------------------------------------

static volatile float KP = 0.5f;
static volatile float KI = 0.0f;

static volatile float target_rpm = 100.0f / 3.0f;

#define BASE_PWM 67.0f

// --------------------------------------------------
// Global variables
// --------------------------------------------------

static uint slice_num;
static uint16_t pwm_wrap;

static volatile uint32_t a_count = 0;
static volatile uint32_t b_count = 0;

static volatile uint32_t pulse_window_count = 0;
static volatile bool control_ready = false;
static volatile uint32_t control_timestamp_us = 0;

// Serial command buffer
static char serial_buffer[64];
static size_t serial_length = 0;

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
// Set motor PWM
// --------------------------------------------------

static void motor_set_speed(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t level =
        ((uint32_t)percent * (pwm_wrap + 1u)) / 100u;

    pwm_set_gpio_level(MOTOR_PIN, (uint16_t)level);
}

// --------------------------------------------------
// Soft start
// --------------------------------------------------

static void motor_ramp(int start, int end, uint32_t delay_ms)
{
    int step = (end >= start) ? 1 : -1;

    for (int p = start; p != end + step; p += step)
    {
        motor_set_speed(p);
        sleep_ms(delay_ms);
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
// Encoder interrupt
// --------------------------------------------------

static void encoder_callback(uint gpio, uint32_t events)
{
    if (!(events & GPIO_IRQ_EDGE_RISE))
        return;

    if (gpio == ENCODER_A_PIN)
    {
        a_count++;

        if (!control_ready)
        {
            pulse_window_count++;

            if (pulse_window_count >= CONTROL_PULSES)
            {
                control_timestamp_us = time_us_32();
                control_ready = true;
            }
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
// Process a complete serial command
//
// Commands:
//   KP 0.75
//   KI 0.02
//   SETPOINT 33.33
// --------------------------------------------------

static void process_serial_command(char *line)
{
    char command[16];
    float value;

    if (sscanf(line, "%15s %f", command, &value) != 2)
        return;

    if (!isfinite(value))
        return;

    if (strcmp(command, "KP") == 0)
    {
        if (value >= 0.0f)
        {
            KP = value;
            printf("# KP updated: %.5f\n", (double)KP);
        }
    }
    else if (strcmp(command, "KI") == 0)
    {
        if (value >= 0.0f)
        {
            KI = value;
            printf("# KI updated: %.5f\n", (double)KI);
        }
    }
    else if (strcmp(command, "SETPOINT") == 0)
    {
        if (value >= 0.0f && value <= 300.0f)
        {
            target_rpm = value;
            printf("# Setpoint updated: %.3f RPM\n",
                   (double)target_rpm);
        }
    }
}

// --------------------------------------------------
// Read serial input without blocking
// --------------------------------------------------

static void handle_serial_input(void)
{
    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (c == '\r' || c == '\n')
        {
            if (serial_length > 0)
            {
                serial_buffer[serial_length] = '\0';
                process_serial_command(serial_buffer);
                serial_length = 0;
            }
        }
        else if (c == '\b' || c == 127)
        {
            if (serial_length > 0)
                serial_length--;
        }
        else if (serial_length < sizeof(serial_buffer) - 1)
        {
            serial_buffer[serial_length++] = (char)c;
        }
    }
}

// --------------------------------------------------
// Main
// --------------------------------------------------

int main(void)
{
    stdio_init_all();

    motor_init(PWM_FREQ_HZ);
    encoder_init();

    float integral = 0.0f;
    float pwm = BASE_PWM;

    // Give USB serial time to initialize.
    sleep_ms(1500);

    printf("# Motor controller started\n");
    printf("# Commands: KP <value>, KI <value>, SETPOINT <rpm>\n");

    // Soft start
    motor_ramp(0, MOTOR_SPEED, 20);

    // Reset pulse window after startup.
    uint32_t irq_state = save_and_disable_interrupts();

    pulse_window_count = 0;
    control_ready = false;

    uint32_t last_timestamp_us = time_us_32();

    restore_interrupts(irq_state);

    while (true)
    {
        // Wait for encoder pulses while still accepting
        // commands from the computer.
        while (!control_ready)
        {
            handle_serial_input();
            tight_loop_contents();
        }

        // Process any commands that arrived at the boundary.
        handle_serial_input();

        // Atomically retrieve the completed pulse window.
        irq_state = save_and_disable_interrupts();

        uint32_t pulses = pulse_window_count;
        uint32_t current_timestamp_us = control_timestamp_us;

        pulse_window_count = 0;
        control_ready = false;

        restore_interrupts(irq_state);

        // Calculate elapsed time.
        uint32_t elapsed_us =
            current_timestamp_us - last_timestamp_us;

        last_timestamp_us = current_timestamp_us;

        float dt = elapsed_us / 1000000.0f;

        if (dt <= 0.0f)
            continue;

        // Calculate RPM.
        float rpm =
            (pulses * 60.0f) / (PULSES_PER_REV * dt);

        // PI controller.
        float error = target_rpm - rpm;

        float candidate_integral =
            integral + error * dt;

        // Integral anti-windup.
        if (KI > 0.0f)
        {
            float limit = 30.0f / KI;

            if (candidate_integral > limit)
                candidate_integral = limit;

            if (candidate_integral < -limit)
                candidate_integral = -limit;
        }

        float new_pwm =
            BASE_PWM + KP * error + KI * candidate_integral;

        // Prevent integral windup when PWM saturates.
        if (!((new_pwm > 100.0f && error > 0.0f) ||
              (new_pwm < 0.0f && error < 0.0f)))
        {
            integral = candidate_integral;
        }

        pwm = BASE_PWM + KP * error + KI * integral;

        if (pwm > 100.0f) pwm = 100.0f;
        if (pwm < 0.0f) pwm = 0.0f;

        motor_set_speed((int)(pwm + 0.5f));

        // Send telemetry to the Python GUI.
        // Format:
        // DATA,time,rpm,target,pwm,kp,ki

        printf(
            "DATA,%.3f,%.3f,%.3f,%.1f,%.5f,%.5f\n",
            (double)(time_us_64() / 1000000.0),
            (double)rpm,
            (double)target_rpm,
            (double)pwm,
            (double)KP,
            (double)KI
        );
    }

    return 0;
}
