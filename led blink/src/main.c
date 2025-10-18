#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <zephyr/display/cfb.h>
#include <string.h>

LOG_MODULE_REGISTER(cruise_control, LOG_LEVEL_INF);

// -------------------- PIN DEFINITIONS --------------------
#define SPEED_IN_PIN    2   // GP2 (pin 4) - Input square wave (actual speed)
#define SPEED_SIM_PIN   16  // GP16 (pin 21) - Simulated speed output for testing
#define PWM_PIN         15  // GP15 (pin 20) - PWM output pin
#define PWM_CHANNEL     0   // Channel for pwm-leds binding
#define PWM_FLAGS       0   // PWM flags

// -------------------- TIMING --------------------
#define PWM_PERIOD_NSEC     20000000  // 20ms period (50 Hz)
#define CONTROL_PERIOD_MS   1000      // Run PI controller every 1 second
#define SPEED_SIM_PERIOD_MS 50        // Toggle simulated speed every 50ms
#define DISPLAY_UPDATE_MS   200       // Update display every 200ms
#define LOGO_DISPLAY_MS     3000      // Show logo for 3 seconds

// -------------------- PI CONTROLLER PARAMETERS --------------------
#define REF_SPEED_HZ    15.0f  // Reference speed in Hz (pulses per second)

// Base PI gains (used in normal operation)
#define KP_BASE         30.0f  // Base proportional gain
#define KI_BASE         5.0f   // Base integral gain

// Adaptive control parameters
#define KP_SATURATED    10.0f  // Reduced KP when saturated (less aggressive)
#define KI_SATURATED    1.0f   // Reduced KI when saturated (less windup)
#define GAIN_BLEND      0.3f   // How quickly gains transition (0-1)

// Anti-windup limits for integral term
#define INTEGRAL_MAX    50.0f
#define INTEGRAL_MIN   -50.0f

// Integral decay when saturated (anti-windup back-calculation)
#define INTEGRAL_DECAY  0.95f  // Multiply integral by this when saturated

// PWM output limits (in microseconds, then converted to nanoseconds)
#define PWM_MIN_US      1000   // 1ms pulse width
#define PWM_MAX_US      2000   // 2ms pulse width
#define PWM_NEUTRAL_US  1500   // 1.5ms neutral position

// Saturation detection thresholds
#define SATURATION_THRESHOLD_US  50  // Consider saturated if within 50us of limits

// -------------------- DISPLAY DEFINITIONS --------------------
#define YELLOW_ROWS     16    // Top 16 rows are yellow (128x64 display)
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

// -------------------- GLOBAL VARIABLES --------------------
static const struct device *gpio_dev;
static const struct device *pwm_dev;
static const struct device *display_dev;

static volatile uint32_t pulse_count = 0;
static float integral = 0.0f;
static float last_error = 0.0f;

// Adaptive control state
static float current_kp = KP_BASE;
static float current_ki = KI_BASE;
static bool is_saturated = false;
static uint32_t saturation_count = 0;

// Shared variables for display (thread-safe through atomic updates)
static volatile int32_t display_measured_speed = 0;
static volatile int32_t display_pwm_us = PWM_NEUTRAL_US;
static volatile bool show_logo = true;

// -------------------- GPIO INTERRUPT HANDLER --------------------
static struct gpio_callback speed_cb;

void speed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    pulse_count++;
}

// -------------------- DISPLAY FUNCTIONS --------------------
void draw_centered_text(const struct device *dev, const char *text, uint16_t row) {
    size_t len = strlen(text);
    uint16_t text_width = len * 6; // Assuming 6 pixels per char
    uint16_t x_pos = (DISPLAY_WIDTH - text_width) / 2;
    
    cfb_print(dev, text, x_pos, row);
}

void draw_horizontal_line(const struct device *dev, uint16_t y, uint16_t spacing) {
    struct cfb_position pos;
    for (uint16_t i = 0; i < DISPLAY_WIDTH; i += spacing) {
        pos.x = i;
        pos.y = y;
        cfb_draw_point(dev, &pos);
    }
}

void draw_logo_screen(const struct device *dev) {
    cfb_framebuffer_clear(dev, false);
    
    // Draw stylized car icon using text
    draw_centered_text(dev, "   ___  ", 1);
    draw_centered_text(dev, " _|___|_", 2);
    draw_centered_text(dev, "|_______|", 3);
    draw_centered_text(dev, "  O   O", 4);
    
    // Draw decorative line
    draw_horizontal_line(dev, 40, 4);
    
    // Draw "EEE-B" at bottom with "ADAPTIVE" label
    draw_centered_text(dev, "ADAPTIVE PI", 6);
    draw_centered_text(dev, "E E E - B", 7);
    
    cfb_framebuffer_finalize(dev);
}

void draw_running_screen(const struct device *dev) {
    char buf[32];
    
    cfb_framebuffer_clear(dev, false);
    
    // Title in yellow area (top)
    if (is_saturated) {
        draw_centered_text(dev, "**SATURATED**", 0);
    } else {
        draw_centered_text(dev, "CRUISE CTRL", 0);
    }
    
    // Draw separator line at yellow/blue boundary
    draw_horizontal_line(dev, 15, 2);
    
    // Speed information in blue area (bottom)
    snprintf(buf, sizeof(buf), "REF:%dHz ACT:%dHz", (int)REF_SPEED_HZ, (int)display_measured_speed);
    cfb_print(dev, buf, 0, 2);
    
    float error = REF_SPEED_HZ - (float)display_measured_speed;
    snprintf(buf, sizeof(buf), "ERR:%.1f INT:%.1f", (double)error, (double)integral);
    cfb_print(dev, buf, 0, 3);
    
    snprintf(buf, sizeof(buf), "KP:%.1f KI:%.1f", (double)current_kp, (double)current_ki);
    cfb_print(dev, buf, 0, 4);
    
    snprintf(buf, sizeof(buf), "PWM: %d us", (int)display_pwm_us);
    cfb_print(dev, buf, 0, 5);
    
    // Mode indicator
    if (is_saturated) {
        cfb_print(dev, "MODE: ADAPTIVE", 0, 6);
    } else {
        cfb_print(dev, "MODE: NORMAL", 0, 6);
    }
    
    // Draw throttle bar indicator (visual representation)
    int bar_width = ((display_pwm_us - PWM_MIN_US) * (DISPLAY_WIDTH - 20)) / (PWM_MAX_US - PWM_MIN_US);
    
    // Draw bar outline
    struct cfb_position start, end;
    start.x = 10;
    start.y = 54;
    end.x = DISPLAY_WIDTH - 10;
    end.y = 61;
    cfb_draw_rect(dev, &start, &end);
    
    // Fill bar based on throttle position
    struct cfb_position pixel;
    for (int y = 55; y < 61; y++) {
        for (int x = 11; x < 11 + bar_width && x < DISPLAY_WIDTH - 11; x++) {
            pixel.x = x;
            pixel.y = y;
            cfb_draw_point(dev, &pixel);
        }
    }
    
    cfb_framebuffer_finalize(dev);
}

// -------------------- DISPLAY UPDATE THREAD --------------------
void display_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    if (!display_dev) {
        LOG_ERR("Display device not initialized");
        return;
    }
    
    // Show logo screen
    draw_logo_screen(display_dev);
    k_msleep(LOGO_DISPLAY_MS);
    show_logo = false;
    
    // Main display loop
    while (1) {
        draw_running_screen(display_dev);
        k_msleep(DISPLAY_UPDATE_MS);
    }
}

// -------------------- SPEED SIMULATOR THREAD --------------------
void speed_simulator_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        gpio_pin_toggle(gpio_dev, SPEED_SIM_PIN);
        k_msleep(SPEED_SIM_PERIOD_MS);
    }
}

// -------------------- ADAPTIVE PI CONTROLLER THREAD --------------------
void pi_controller_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_msleep(CONTROL_PERIOD_MS);

        // Get measured speed (pulses per second)
        uint32_t measured_speed = pulse_count;
        pulse_count = 0;

        // Calculate error
        float error = REF_SPEED_HZ - (float)measured_speed;

        // Update integral term
        integral += error;
        
        // Apply integral limits (anti-windup)
        if (integral > INTEGRAL_MAX) {
            integral = INTEGRAL_MAX;
        } else if (integral < INTEGRAL_MIN) {
            integral = INTEGRAL_MIN;
        }

        // PI Controller output with current gains
        float control_output = (current_kp * error) + (current_ki * integral);

        // Convert control output to PWM duty cycle (pulse width in microseconds)
        int32_t pulse_width_us = PWM_NEUTRAL_US + (int32_t)control_output;

        // Check for saturation BEFORE clamping
        bool will_saturate = false;
        if (pulse_width_us < PWM_MIN_US + SATURATION_THRESHOLD_US) {
            will_saturate = true;
        } else if (pulse_width_us > PWM_MAX_US - SATURATION_THRESHOLD_US) {
            will_saturate = true;
        }

        // Clamp to valid PWM range
        int32_t unclamped_pwm = pulse_width_us;
        if (pulse_width_us < PWM_MIN_US) {
            pulse_width_us = PWM_MIN_US;
        } else if (pulse_width_us > PWM_MAX_US) {
            pulse_width_us = PWM_MAX_US;
        }

        // ========== ADAPTIVE CONTROL LOGIC ==========
        
        if (will_saturate) {
            // We're saturated or about to saturate
            saturation_count++;
            
            // Apply integral decay (anti-windup back-calculation)
            // This helps the integral "forget" some of its history when saturated
            integral *= INTEGRAL_DECAY;
            
            // Smoothly transition to reduced gains
            float target_kp = KP_SATURATED;
            float target_ki = KI_SATURATED;
            
            current_kp = current_kp * (1.0f - GAIN_BLEND) + target_kp * GAIN_BLEND;
            current_ki = current_ki * (1.0f - GAIN_BLEND) + target_ki * GAIN_BLEND;
            
            is_saturated = true;
            
            LOG_WRN("SATURATED! Reducing gains - KP: %.1f → %.1f, KI: %.1f → %.1f",
                    (double)KP_BASE, (double)current_kp,
                    (double)KI_BASE, (double)current_ki);
        } else {
            // Normal operation - not saturated
            if (is_saturated && saturation_count > 2) {
                // We were saturated but recovered - gradually restore gains
                LOG_INF("Recovered from saturation. Restoring gains.");
                saturation_count = 0;
            }
            
            // Smoothly transition back to base gains
            float target_kp = KP_BASE;
            float target_ki = KI_BASE;
            
            current_kp = current_kp * (1.0f - GAIN_BLEND) + target_kp * GAIN_BLEND;
            current_ki = current_ki * (1.0f - GAIN_BLEND) + target_ki * GAIN_BLEND;
            
            is_saturated = false;
        }

        // Convert to nanoseconds for PWM API
        uint32_t pulse_width_ns = pulse_width_us * 1000;

        // Set PWM output
        int ret = pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD_NSEC, pulse_width_ns, PWM_FLAGS);
        if (ret < 0) {
            LOG_ERR("PWM set failed: %d", ret);
        }

        // Update display variables
        display_measured_speed = measured_speed;
        display_pwm_us = pulse_width_us;

        // Log controller state
        if (is_saturated) {
            LOG_WRN("SATURATED | Ref: %.1f Hz | Act: %d Hz | Err: %.2f | Int: %.2f | PWM: %d us | KP: %.1f | KI: %.1f",
                    (double)REF_SPEED_HZ, measured_speed, (double)error, (double)integral, 
                    pulse_width_us, (double)current_kp, (double)current_ki);
        } else {
            LOG_INF("Ref: %.1f Hz | Act: %d Hz | Err: %.2f | Int: %.2f | PWM: %d us | KP: %.1f | KI: %.1f",
                    (double)REF_SPEED_HZ, measured_speed, (double)error, (double)integral, 
                    pulse_width_us, (double)current_kp, (double)current_ki);
        }

        last_error = error;
    }
}

// -------------------- THREAD DEFINITIONS --------------------
K_THREAD_DEFINE(speed_sim_tid, 512, speed_simulator_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(pi_ctrl_tid, 1024, pi_controller_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(display_tid, 2048, display_thread, NULL, NULL, NULL, 6, 0, 0);

// -------------------- MAIN --------------------
int main(void) {
    int ret;

    LOG_INF("=== Pico 2W Adaptive Cruise Control ===");
    LOG_INF("Reference Speed: %.1f Hz", (double)REF_SPEED_HZ);
    LOG_INF("Base Gains - KP: %.2f, KI: %.2f", (double)KP_BASE, (double)KI_BASE);
    LOG_INF("Saturated Gains - KP: %.2f, KI: %.2f", (double)KP_SATURATED, (double)KI_SATURATED);
    LOG_INF("Integral Decay Factor: %.2f", (double)INTEGRAL_DECAY);

    // -------------------- GPIO SETUP --------------------
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -1;
    }

    // Configure speed input pin (GP2) with interrupt
    ret = gpio_pin_configure(gpio_dev, SPEED_IN_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure speed input pin: %d", ret);
        return -1;
    }

    ret = gpio_pin_interrupt_configure(gpio_dev, SPEED_IN_PIN, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt on speed input: %d", ret);
        return -1;
    }

    gpio_init_callback(&speed_cb, speed_isr, BIT(SPEED_IN_PIN));
    gpio_add_callback(gpio_dev, &speed_cb);
    LOG_INF("Speed input configured on GP%d", SPEED_IN_PIN);

    // Configure simulated speed output pin (GP16)
    ret = gpio_pin_configure(gpio_dev, SPEED_SIM_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure simulated speed output: %d", ret);
        return -1;
    }
    LOG_INF("Simulated speed output on GP%d (10 Hz square wave)", SPEED_SIM_PIN);

    // -------------------- PWM SETUP --------------------
    pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm));
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("PWM device not ready");
        LOG_ERR("Make sure:");
        LOG_ERR("  1. rpi_pico2.overlay file exists in project root");
        LOG_ERR("  2. CONFIG_PWM=y is in prj.conf");
        LOG_ERR("  3. Clean build: west build -t pristine");
        return -1;
    }
    LOG_INF("PWM device ready");

    // Initialize PWM to neutral position
    uint32_t neutral_ns = PWM_NEUTRAL_US * 1000;
    ret = pwm_set(pwm_dev, PWM_CHANNEL, PWM_PERIOD_NSEC, neutral_ns, PWM_FLAGS);
    if (ret < 0) {
        LOG_ERR("Failed to initialize PWM: %d", ret);
        return -1;
    }
    LOG_INF("PWM throttle output on GP%d (pin 20) - 1-2ms pulse, 20ms period", PWM_PIN);

    // -------------------- DISPLAY SETUP --------------------
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_WRN("Display device not ready - continuing without display");
        display_dev = NULL;
    } else {
        LOG_INF("Display device ready");
        
        // Initialize character framebuffer
        ret = cfb_framebuffer_init(display_dev);
        if (ret) {
            LOG_ERR("Failed to initialize framebuffer: %d", ret);
            display_dev = NULL;
        } else {
            cfb_framebuffer_clear(display_dev, true);
            
            // Set font
            cfb_framebuffer_set_font(display_dev, 0);
            
            LOG_INF("Display: SSD1306 128x64 OLED initialized");
            LOG_INF("Top %d rows: YELLOW, Bottom rows: BLUE", YELLOW_ROWS);
        }
    }

    LOG_INF("===========================================");
    LOG_INF("Adaptive Control System initialized!");
    LOG_INF("Monitoring speed and adjusting throttle...");
    LOG_INF("===========================================");

    // Main thread goes idle - worker threads handle everything
    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}