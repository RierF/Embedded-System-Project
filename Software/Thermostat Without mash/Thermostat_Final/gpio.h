

#ifndef GPIO_H
#define GPIO_H

void gpio_init(void);

#define nrf_gpio 0 //if NULL Pins are mapped for SDK if 1 Pins are mapped for Thermostat PCB (Has to be changed in ssd1306.c and gpio.c File as well)

#if nrf_gpio

#define PIN_IN_A  3
#define PIN_IN_B  4
#define PIN_IN_C  20
#define PIN_OUT   16

#else

#define PIN_IN_A  30
#define PIN_IN_B  31
#define PIN_IN_C  BUTTON_1
#define PIN_OUT   BSP_LED_0

#endif

#endif //GPIO_H
