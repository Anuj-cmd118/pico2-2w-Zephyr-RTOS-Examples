/*
 * Zephyr RTOS Power Monitor for Raspberry Pi Pico 2W
 * Features: Over/Under Voltage and Current Protection with SSD1306 Display
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(power_monitor, LOG_LEVEL_INF);

/* Configuration Parameters - Adjust these as needed */
#define VOLTAGE_MIN          10.5f   // Minimum voltage threshold (V)
#define VOLTAGE_MAX          14.5f   // Maximum voltage threshold (V)
#define CURRENT_MIN          0.1f    // Minimum current threshold (A)
#define CURRENT_MAX          2.0f    // Maximum current threshold (A)

/* ADC Configuration */
#define ADC_NODE             DT_NODELABEL(adc)
#define ADC_VOLTAGE_CHANNEL  0
#define ADC_CURRENT_CHANNEL  1
#define ADC_RESOLUTION       12
#define ADC_GAIN             ADC_GAIN_1
#define ADC_REFERENCE        ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_VREF_MV          3300    // 3.3V reference

/* Relay GPIO Configuration */
#define RELAY_NODE           DT_NODELABEL(gpio0)
#define RELAY_PIN            16

/* I2C and SSD1306 Configuration */
#define I2C_NODE             DT_NODELABEL(i2c0)
#define SSD1306_ADDR         0x3C
#define SSD1306_WIDTH        128
#define SSD1306_HEIGHT       64

/* Sample timing */
#define SAMPLE_INTERVAL_MS   100
#define DISPLAY_UPDATE_MS    200

/* SSD1306 Commands */
#define SSD1306_SETCONTRAST          0x81
#define SSD1306_DISPLAYALLON_RESUME  0xA4
#define SSD1306_DISPLAYALLON         0xA5
#define SSD1306_NORMALDISPLAY        0xA6
#define SSD1306_INVERTDISPLAY        0xA7
#define SSD1306_DISPLAYOFF           0xAE
#define SSD1306_DISPLAYON            0xAF
#define SSD1306_SETDISPLAYOFFSET     0xD3
#define SSD1306_SETCOMPINS           0xDA
#define SSD1306_SETVCOMDETECT        0xDB
#define SSD1306_SETDISPLAYCLOCKDIV   0xD5
#define SSD1306_SETPRECHARGE         0xD9
#define SSD1306_SETMULTIPLEX         0xA8
#define SSD1306_SETLOWCOLUMN         0x00
#define SSD1306_SETHIGHCOLUMN        0x10
#define SSD1306_SETSTARTLINE         0x40
#define SSD1306_MEMORYMODE           0x20
#define SSD1306_COLUMNADDR           0x21
#define SSD1306_PAGEADDR             0x22
#define SSD1306_COMSCANINC           0xC0
#define SSD1306_COMSCANDEC           0xC8
#define SSD1306_SEGREMAP             0xA0
#define SSD1306_CHARGEPUMP           0x8D

/* Global variables */
static const struct device *adc_dev;
static const struct device *relay_dev;
static const struct device *i2c_dev;
static struct gpio_dt_spec relay_spec;

static float voltage = 0.0f;
static float current = 0.0f;
static bool relay_state = false;
static bool fault_state = false;
static char fault_msg[32] = "OK";

/* Display buffer (1024 bytes for 128x64 monochrome) */
static uint8_t display_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* 5x7 font (basic ASCII 32-127) */
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

/* Function prototypes */
static int ssd1306_init(void);
static void ssd1306_command(uint8_t cmd);
static void ssd1306_clear(void);
static void ssd1306_draw_char(int16_t x, int16_t y, char c, uint8_t size);
static void ssd1306_draw_string(int16_t x, int16_t y, const char *str, uint8_t size);
static void ssd1306_display(void);
static void update_display(void);
static float read_adc_channel(uint8_t channel);
static void check_thresholds(void);
static void set_relay(bool state);

/* I2C write helper */
static int i2c_write_bytes(uint8_t addr, const uint8_t *data, size_t len)
{
    return i2c_write(i2c_dev, data, len, addr);
}

/* SSD1306 command sender */
static void ssd1306_command(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_bytes(SSD1306_ADDR, buf, 2);
}

/* Initialize SSD1306 display */
static int ssd1306_init(void)
{
    k_msleep(100);
    
    ssd1306_command(SSD1306_DISPLAYOFF);
    ssd1306_command(SSD1306_SETDISPLAYCLOCKDIV);
    ssd1306_command(0x80);
    ssd1306_command(SSD1306_SETMULTIPLEX);
    ssd1306_command(SSD1306_HEIGHT - 1);
    ssd1306_command(SSD1306_SETDISPLAYOFFSET);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SETSTARTLINE | 0x00);
    ssd1306_command(SSD1306_CHARGEPUMP);
    ssd1306_command(0x14);
    ssd1306_command(SSD1306_MEMORYMODE);
    ssd1306_command(0x00);
    ssd1306_command(SSD1306_SEGREMAP | 0x01);
    ssd1306_command(SSD1306_COMSCANDEC);
    ssd1306_command(SSD1306_SETCOMPINS);
    ssd1306_command(0x12);
    ssd1306_command(SSD1306_SETCONTRAST);
    ssd1306_command(0xCF);
    ssd1306_command(SSD1306_SETPRECHARGE);
    ssd1306_command(0xF1);
    ssd1306_command(SSD1306_SETVCOMDETECT);
    ssd1306_command(0x40);
    ssd1306_command(SSD1306_DISPLAYALLON_RESUME);
    ssd1306_command(SSD1306_NORMALDISPLAY);
    ssd1306_command(SSD1306_DISPLAYON);
    
    ssd1306_clear();
    return 0;
}

/* Clear display buffer */
static void ssd1306_clear(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
}

/* Draw a character */
static void ssd1306_draw_char(int16_t x, int16_t y, char c, uint8_t size)
{
    if (c < 32 || c > 90) c = 32;
    
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[c - 32][i];
        for (uint8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1) {
                for (uint8_t sx = 0; sx < size; sx++) {
                    for (uint8_t sy = 0; sy < size; sy++) {
                        int16_t px = x + i * size + sx;
                        int16_t py = y + j * size + sy;
                        if (px >= 0 && px < SSD1306_WIDTH && py >= 0 && py < SSD1306_HEIGHT) {
                            display_buffer[px + (py / 8) * SSD1306_WIDTH] |= (1 << (py & 7));
                        }
                    }
                }
            }
        }
    }
}

/* Draw a string */
static void ssd1306_draw_string(int16_t x, int16_t y, const char *str, uint8_t size)
{
    while (*str) {
        ssd1306_draw_char(x, y, *str++, size);
        x += 6 * size;
    }
}

/* Send buffer to display */
static void ssd1306_display(void)
{
    ssd1306_command(SSD1306_COLUMNADDR);
    ssd1306_command(0);
    ssd1306_command(SSD1306_WIDTH - 1);
    ssd1306_command(SSD1306_PAGEADDR);
    ssd1306_command(0);
    ssd1306_command((SSD1306_HEIGHT / 8) - 1);
    
    for (uint16_t i = 0; i < sizeof(display_buffer); i += 16) {
        uint8_t buf[17];
        buf[0] = 0x40;
        memcpy(&buf[1], &display_buffer[i], 16);
        i2c_write_bytes(SSD1306_ADDR, buf, 17);
    }
}

/* Read ADC channel and convert to meaningful value */
static float read_adc_channel(uint8_t channel)
{
    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = channel,
        .differential = 0
    };
    
    adc_channel_setup(adc_dev, &ch_cfg);
    
    uint16_t sample_buffer;
    struct adc_sequence sequence = {
        .channels = BIT(channel),
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };
    
    int ret = adc_read(adc_dev, &sequence);
    if (ret != 0) {
        return 0.0f;
    }
    
    float adc_voltage = (float)sample_buffer * ADC_VREF_MV / 4096.0f / 1000.0f;
    
    if (channel == ADC_VOLTAGE_CHANNEL) {
        // Assuming voltage divider: adjust multiplier for your circuit
        return adc_voltage * 5.0f;  // Example: 5:1 voltage divider
    } else {
        // Current sensor (e.g., ACS712): adjust for your sensor
        // ACS712-05B: 185mV/A, centered at 2.5V
        return (adc_voltage - 2.5f) / 0.185f;
    }
}

/* Check voltage and current thresholds */
static void check_thresholds(void)
{
    bool fault = false;
    
    if (voltage < VOLTAGE_MIN) {
        snprintf(fault_msg, sizeof(fault_msg), "UNDER VOLTAGE");
        fault = true;
    } else if (voltage > VOLTAGE_MAX) {
        snprintf(fault_msg, sizeof(fault_msg), "OVER VOLTAGE");
        fault = true;
    } else if (current < CURRENT_MIN) {
        snprintf(fault_msg, sizeof(fault_msg), "UNDER CURRENT");
        fault = true;
    } else if (current >= CURRENT_MAX) {
        snprintf(fault_msg, sizeof(fault_msg), "OVER CURRENT");
        fault = true;
    } else {
        snprintf(fault_msg, sizeof(fault_msg), "OK");
    }
    
    if (fault && !fault_state) {
        set_relay(false);
        fault_state = true;
        LOG_WRN("Fault detected: %s", fault_msg);
    } else if (!fault && fault_state) {
        set_relay(true);
        fault_state = false;
        LOG_INF("Fault cleared");
    }
}

/* Control relay */
static void set_relay(bool state)
{
    gpio_pin_set_dt(&relay_spec, state ? 1 : 0);
    relay_state = state;
}

/* Update display with current readings */
static void update_display(void)
{
    char buf[32];
    
    ssd1306_clear();
    
    // Title
    ssd1306_draw_string(10, 2, "POWER MONITOR", 1);
    
    // Voltage
    snprintf(buf, sizeof(buf), "V: %.2fV", voltage);
    ssd1306_draw_string(5, 18, buf, 2);
    
    // Current
    snprintf(buf, sizeof(buf), "I: %.2fA", current);
    ssd1306_draw_string(5, 34, buf, 2);
    
    // Status
    ssd1306_draw_string(5, 50, relay_state ? "RELAY: ON" : "RELAY: OFF", 1);
    
    // Fault message
    if (fault_state) {
        ssd1306_draw_string(5, 58, fault_msg, 1);
    }
    
    ssd1306_display();
}

/* Main application */
int main(void)
{
    int ret;
    
    LOG_INF("Power Monitor Starting...");
    
    // Initialize ADC
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -1;
    }
    
    // Initialize relay GPIO
    relay_dev = DEVICE_DT_GET(RELAY_NODE);
    relay_spec.port = relay_dev;
    relay_spec.pin = RELAY_PIN;
    relay_spec.dt_flags = GPIO_OUTPUT_INACTIVE;
    
    ret = gpio_pin_configure_dt(&relay_spec, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure relay GPIO");
        return -1;
    }
    
    // Initialize I2C
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -1;
    }
    
    // Initialize display
    ret = ssd1306_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize display");
    }
    
    LOG_INF("Initialization complete");
    
    // Initial relay state
    set_relay(true);
    
    uint32_t last_display_update = 0;
    
    while (1) {
        // Read sensors
        voltage = read_adc_channel(ADC_VOLTAGE_CHANNEL);
        current = read_adc_channel(ADC_CURRENT_CHANNEL);
        
        // Check thresholds
        check_thresholds();
        
        // Update display periodically
        uint32_t now = k_uptime_get_32();
        if (now - last_display_update >= DISPLAY_UPDATE_MS) {
            update_display();
            last_display_update = now;
        }
        
        k_msleep(SAMPLE_INTERVAL_MS);
    }
    
    return 0;
}