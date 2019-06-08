/**
 *
 * Bluetooth Low Energy MESH Example for Embedded Systems Project 2018
 * Author: Rier Fabian, Bsc
 * Some of the used Drivers Source Code was taken and modified from Nordic Semiconductor ASA
 *
 * The main function below containes initialtzatoin for the I2C and Gpiote stack
 *
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 */

#include "bme.h"
#include "thermostat.h"
#include "gpio.h"
#include "ssd1306.c"
#include "ssd1306.h"

#define nrf_bme 0  //if NULL Pins are mapped for SDK if 1 Pins are mapped for Thermostat PCB (Has to be changed in ssd1306.c and gpio.c File as well)

#if nrf_bme

#define Pin_SDA_BME 19
#define Pin_SCL_BME 17

#else

#define Pin_SDA_BME 2
#define Pin_SCL_BME 27

#endif



/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);


int32_t curr_temp = 0;
static volatile bool   m_xfer_done = false;

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

		// Forward the TWI event to the driver
		bme280_twi_evt_handler(p_event, p_context);

}

void bme280_handler(bme280_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type) {
		case BME280_TWI_MEASUREMENT_FETCHED:
			m_xfer_done = true;
			break;
		default:
			break;
	}
}
/*
 * Init of BME Bosh Temperature Sensor I2C bus
*/
void twi_init(void)
{
	const nrf_drv_twi_config_t twi_config = {
		.scl                = Pin_SCL_BME,
		.sda                = Pin_SDA_BME ,
		.frequency          = NRF_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init     = false
	};
	nrf_drv_twi_uninit(&m_twi);
	ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}
/*
 * Init of Bosh BME280 Sensor
*/
void bme280_init(void) {

	const bme280_twi_config_t bme280_twi_config = {
		.addr = BME280_TWI_ADDR_0,
		.standby = BME280_TWI_STANDBY_250_MS,
		.filter = BME280_TWI_FILTER_OFF,
		.temp_oversampling = BME280_TWI_OVERSAMPLING_X4,
	};

	bme280_twi_init(&m_twi, &bme280_twi_config, bme280_handler, NULL);
	bme280_twi_enable();
}

/*
 * Log Temp function sends current temperature upgrade to Display function
 *Log Temp function is called form Main Function every XX Seconds after delay and fetching of Measurments from i2c bus
 */

static void log_temp(void)
{	
	bme280_twi_data_t data;
	bme280_twi_temperature_get(&data);

	//Write current temperature to display write function
	write_is_temp(data.temp);


}
/*
 * Get is val is called from gpio.c library whenever a Interrupt from the roraty encoder is triggered and the set value was changed
 *
 */

void get_is_val(int32_t temp)
{
	bme280_twi_data_t data;

	//get current temp from bme280 i2c
	m_xfer_done = false;
	bme280_twi_measurement_fetch();
	do {
		__WFE();
	} while (m_xfer_done == false);

	log_temp(&data);

	curr_temp = data.temp / 10;

	//If current set_temp>current temp a led will be switched on
	if(temp > curr_temp)
	{
		nrf_drv_gpiote_out_set(PIN_OUT);
	}else
	{
		nrf_drv_gpiote_out_clear(PIN_OUT);
	}
}

int main(void)
{
	//init i2c bus for bme280
	twi_init();
	//Init BME280 (see bme.c)
	bme280_init();
	//Init Gpio for Rotary Encoder and button Interrupt and LED (see gpio.c)
	gpio_init();
	nrf_delay_ms(1000);
	//Init i2c bus for ssd1306 display (see ssd1306.c)
	twi_init_disp();
	//Init of ssd1306 display (see ssd1306.c)
	ssd1306_begin(0x2, 0x3C, false);
    nrf_delay_ms(1000);
    //clear Display
    ssd1306_clear_display();
    //Write to display
    ssd1306_display();
	nrf_delay_ms(1000);
	//Write data_buffer where current and set temperature will be stored
	testdrawchar();


	while (true) {

		nrf_delay_ms(3000);


		//Switch off display every xx seconds for energy savings
		display_off();
		//Get current temperature every XX seconds
		m_xfer_done = false;
		bme280_twi_measurement_fetch();

		do {
			__WFE();
		} while (m_xfer_done == false);

		log_temp();

	}
}

/** @} */
