/*
 * Blink external LED on Raspberry Pi Pico 2 (GPIO 15) using Zephyr
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* Use GPIO 15 directly */
#define LED_PIN 15

int main(void)
{
    const struct device *gpio_dev;
    int ret;
    bool led_state = true;

    /* Get GPIO controller (gpio0 on RP2350/Pico2) */
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        printf("Error: GPIO device not ready\n");
        return 0;
    }

    /* Configure pin as output */
    ret = gpio_pin_configure(gpio_dev, LED_PIN, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("Error: failed to configure pin\n");
        return 0;
    }

    while (1) {
        ret = gpio_pin_toggle(gpio_dev, LED_PIN);
        if (ret < 0) {
            printf("Error: failed to toggle pin\n");
            return 0;
        }

        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");
        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
