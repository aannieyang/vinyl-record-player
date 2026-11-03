// Turntable motor controller - Raspberry Pi Pico 2 W + XY-MOS MOSFET module
//
// Changes vs. recordplayercode.c:
//   - Fixed-rate control loop (CONTROL_PERIOD_MS). The encoder ISR never stops
//     counting, so no pulses are lost while the main loop is busy printing.
//   - RPM is measured edge-to-edge (edges / time between first and last edge),
//     so it stays accurate with only a handful of edges per sample.
//   - Float duty using the full PWM resolution; PWM at 20 kHz (inaudible).
//   - PI integral is stored as a duty term (i_term), so changing KI never
//     kicks the output, and switching open loop -> PI is bumpless.
//   - Line-based text commands + one CSV telemetry line per sample, for tuner.py.
//
// Commands (newline-terminated, e.g. from tuner.py or a serial terminal):
//   duty <0-100>     open loop at this duty (%)
//   pi               closed-loop PI (bumpless from the current duty)
//   stop             open loop, duty 0
//   target <rpm>     PI setpoint
//   kp <v>           proportional gain   [% duty per RPM]
//   ki <v>           integral gain       [% duty per RPM*s]
//   ff <duty%>       feedforward duty added to the PI output
//   status / help
//
// Telemetry, one line per control sample:
//   D,t_ms,mode,duty,rpm,target,kp,ki,ff        mode: O = open loop, P = PI
// Info lines start with '#'.
//
// Wiring:
//   Pico GP15  -> XY-MOS PWM+          Pico GND   -> XY-MOS PWM-
//   Pico GP16  -> Encoder channel A
//   Motor PSU  -> XY-MOS VIN+/VIN-     Motor      -> XY-MOS OUT+/OUT-
//
// CMakeLists.txt needs: target_link_libraries(motor pico_stdlib hardware_pwm)
//                       pico_enable_stdio_usb(motor 1)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

// --------------------------------------------------
// Configuration
// --------------------------------------------------

#define MOTOR_PIN           15
#define ENCODER_A_PIN       16

#define PWM_FREQ_HZ         1000u

#define ENCODER_PPR         3.0f     // rising edges on channel A per MOTOR rev
#define GEAR_RATIO          210.59f  // from 5117 datasheet
#define PULSES_PER_REV      (ENCODER_PPR * GEAR_RATIO)

#define CONTROL_PERIOD_MS   20       // 50 Hz control + telemetry
#define STALL_TIMEOUT_US    250000u  // no edges for this long -> report 0 RPM

#define DEFAULT_TARGET_RPM  (100.0f / 3.0f)
#define DEFAULT_KP          0.5f
#define DEFAULT_KI          0.15f
#define DEFAULT_FF          67.0f
#define I_TERM_LIMIT        40.0f    // max |integral contribution| in % duty

#define AUTOSTART_PI        0        // 1 = start spinning in PI mode on boot

// --------------------------------------------------
// State
// --------------------------------------------------

typedef enum { MODE_OPEN, MODE_PI } ctrl_mode_t;

static uint16_t pwm_wrap;

static volatile uint32_t edge_count = 0;     // written only by the ISR
static volatile uint32_t last_edge_us = 0;   // written only by the ISR

static ctrl_mode_t mode = MODE_OPEN;
static float duty_pct = 0.0f;
static float rpm = 0.0f;
static float target_rpm = DEFAULT_TARGET_RPM;
static float kp = DEFAULT_KP;
static float ki = DEFAULT_KI;
static float ff = DEFAULT_FF;
static float i_term = 0.0f;

// --------------------------------------------------
// Motor PWM
// --------------------------------------------------

static void motor_init(void)
{
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_PIN);

    uint32_t sys_hz = clock_get_hz(clk_sys);
    uint32_t div = sys_hz / (PWM_FREQ_HZ * 65536u) + 1u;
    pwm_wrap = (uint16_t)(sys_hz / (div * PWM_FREQ_HZ) - 1u);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, div);
    pwm_config_set_wrap(&cfg, pwm_wrap);
    pwm_init(slice, &cfg, false);

    pwm_set_gpio_level(MOTOR_PIN, 0);
    pwm_set_enabled(slice, true);
}

static void motor_set_duty(float pct)
{
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    duty_pct = pct;

    uint32_t level = (uint32_t)(pct * (float)(pwm_wrap + 1u) / 100.0f + 0.5f);
    if (level > 65535u) level = 65535u;
    pwm_set_gpio_level(MOTOR_PIN, (uint16_t)level);
}

// --------------------------------------------------
// Encoder: count every rising edge on A, remember when the last one happened
// --------------------------------------------------

static void encoder_isr(uint gpio, uint32_t events)
{
    if (gpio == ENCODER_A_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        last_edge_us = time_us_32();
        edge_count++;
    }
}

static void encoder_init(void)
{
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN, GPIO_IRQ_EDGE_RISE,
                                       true, encoder_isr);
}

// Edge-to-edge speed measurement. Uses the time between the last edge of the
// previous sample and the last edge of this one, so there is no +/-1 pulse
// quantization error and nothing is ever dropped.
static uint32_t ref_count = 0;
static uint32_t ref_edge_us = 0;
static bool have_ref = false;

static void update_rpm(void)
{
    uint32_t irq = save_and_disable_interrupts();
    uint32_t count  = edge_count;
    uint32_t edge_t = last_edge_us;
    restore_interrupts(irq);

    uint32_t new_edges = count - ref_count;

    if (new_edges > 0) {
        if (have_ref) {
            uint32_t span_us = edge_t - ref_edge_us;
            if (span_us > 0)
                rpm = ((float)new_edges * 60.0e6f) / (PULSES_PER_REV * (float)span_us);
        }
        ref_count = count;
        ref_edge_us = edge_t;
        have_ref = true;
    } else if (have_ref) {
        // No edges this sample: speed can't be more than one edge's worth
        // over the time since the last edge.
        uint32_t since_us = time_us_32() - ref_edge_us;
        if (since_us > STALL_TIMEOUT_US) {
            rpm = 0.0f;
            have_ref = false;
        } else if (since_us > 0) {
            float bound = 60.0e6f / (PULSES_PER_REV * (float)since_us);
            if (rpm > bound) rpm = bound;
        }
    }
}

// --------------------------------------------------
// PI controller
// --------------------------------------------------

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void start_pi(void)
{
    if (rpm < 0.5f) {
        i_term = 0.0f;   // starting from rest: begin at ff + kp*error
    } else {
        // bumpless: choose i_term so the output equals the current duty
        i_term = duty_pct - ff - kp * (target_rpm - rpm);
        i_term = clampf(i_term, -I_TERM_LIMIT, I_TERM_LIMIT);
    }
    mode = MODE_PI;
}

static void control_step(float dt)
{
    if (mode != MODE_PI)
        return;

    float err = target_rpm - rpm;
    float i_new = clampf(i_term + ki * err * dt, -I_TERM_LIMIT, I_TERM_LIMIT);
    float u = ff + kp * err + i_new;

    // Conditional integration: don't keep integrating into saturation.
    if (!((u > 100.0f && err > 0.0f) || (u < 0.0f && err < 0.0f)))
        i_term = i_new;

    motor_set_duty(ff + kp * err + i_term);
}

// --------------------------------------------------
// Serial commands
// --------------------------------------------------

static void print_status(void)
{
    printf("# mode=%s target=%.3f kp=%.4f ki=%.4f ff=%.2f duty=%.2f rpm=%.3f "
           "pwm_hz=%u wrap=%u period_ms=%d\n",
           mode == MODE_PI ? "PI" : "OPEN",
           (double)target_rpm, (double)kp, (double)ki, (double)ff,
           (double)duty_pct, (double)rpm,
           (unsigned)PWM_FREQ_HZ, (unsigned)pwm_wrap, CONTROL_PERIOD_MS);
}

static void print_help(void)
{
    printf("# commands: duty <%%> | pi | stop | target <rpm> | kp <v> | ki <v> | "
           "ff <%%> | status | help\n");
}

static void handle_command(char *line)
{
    char cmd[16];
    float v = 0.0f;
    int n = sscanf(line, "%15s %f", cmd, &v);
    if (n < 1)
        return;

    if (strcmp(cmd, "duty") == 0 && n == 2) {
        mode = MODE_OPEN;
        motor_set_duty(v);
        printf("# ok duty %.2f\n", (double)duty_pct);
    } else if (strcmp(cmd, "pi") == 0) {
        start_pi();
        printf("# ok pi\n");
    } else if (strcmp(cmd, "stop") == 0) {
        mode = MODE_OPEN;
        motor_set_duty(0.0f);
        printf("# ok stop\n");
    } else if (strcmp(cmd, "target") == 0 && n == 2 && v >= 0.0f) {
        target_rpm = v;
        printf("# ok target %.3f\n", (double)target_rpm);
    } else if (strcmp(cmd, "kp") == 0 && n == 2 && v >= 0.0f) {
        kp = v;
        printf("# ok kp %.4f\n", (double)kp);
    } else if (strcmp(cmd, "ki") == 0 && n == 2 && v >= 0.0f) {
        ki = v;
        printf("# ok ki %.4f\n", (double)ki);
    } else if (strcmp(cmd, "ff") == 0 && n == 2) {
        ff = clampf(v, 0.0f, 100.0f);
        printf("# ok ff %.2f\n", (double)ff);
    } else if (strcmp(cmd, "status") == 0) {
        print_status();
    } else if (strcmp(cmd, "help") == 0) {
        print_help();
    } else {
        printf("# err unknown or bad command: %s\n", line);
    }
}

static void handle_serial(void)
{
    static char buf[48];
    static int len = 0;
    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                buf[len] = '\0';
                handle_command(buf);
                len = 0;
            }
        } else if (len < (int)sizeof(buf) - 1) {
            buf[len++] = (char)c;
        }
    }
}

// --------------------------------------------------
// Main
// --------------------------------------------------

int main(void)
{
    stdio_init_all();
    motor_init();
    encoder_init();

#if AUTOSTART_PI
    start_pi();
#endif

    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    absolute_time_t next = get_absolute_time();

    while (true) {
        next = delayed_by_ms(next, CONTROL_PERIOD_MS);
        while (!time_reached(next)) {
            handle_serial();
            tight_loop_contents();
        }

        update_rpm();
        control_step(dt);

        printf("D,%lu,%c,%.2f,%.3f,%.3f,%.4f,%.4f,%.2f\n",
               (unsigned long)to_ms_since_boot(get_absolute_time()),
               mode == MODE_PI ? 'P' : 'O',
               (double)duty_pct, (double)rpm, (double)target_rpm,
               (double)kp, (double)ki, (double)ff);

        // If we fell more than a period behind (e.g. USB stalled), resync
        // instead of running a burst of back-to-back samples.
        if (absolute_time_diff_us(next, get_absolute_time()) > CONTROL_PERIOD_MS * 1000)
            next = get_absolute_time();
    }
}