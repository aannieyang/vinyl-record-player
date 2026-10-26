#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

// GPIO numbers, not physical header pins (GP26 = pin 31 = ADC0, GP27 = pin 32 = ADC1)
#define STEREO_LEFT_PIN 26
#define STEREO_RIGHT_PIN 27

int main()
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("cyw43 init failed\n");
        return -1;
    }
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); 

    adc_init();
    adc_gpio_init(STEREO_LEFT_PIN);
    adc_gpio_init(STEREO_RIGHT_PIN);
    const float conversion_factor = 3.3f / (1 << 12);

    // main loop
    while(1){
        adc_select_input(0); 
        float left = adc_read() * conversion_factor;
        adc_select_input(1); 
        float right = adc_read() * conversion_factor;
        printf(">left:%.3f\n>right:%.3f\n", left, right); // teleplot; control panel > open session, add serial
        sleep_ms(20);
    }

}
