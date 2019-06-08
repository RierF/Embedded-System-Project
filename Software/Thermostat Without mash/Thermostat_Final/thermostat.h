

#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


void get_is_val(int32_t temp);
void twi_ssd1306_init (void);
void cnt_updown(void);
void testdrawchar(void);



#endif //THERMOSTAT_H
