/*
Copyright 2017 Knut Auvor Grythe

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "bme.h"
#include "sdk_errors.h"
#include "app_error.h"
#include "thermostat.h"


static int32_t  m_temp_raw;
static int32_t  m_pres_raw;
static double  m_hum_raw;
static volatile bool   m_xfer_done = false;

static const nrf_drv_twi_t *m_twi_ptr;
static bme280_twi_evt_handler_t m_handler;
static uint8_t	m_addr;
static void *   m_context;
static uint8_t  m_buf[30];
uint8_t m_rx_reg;

struct bme280_driver bme280;


static uint8_t  m_ctrl_meas;



__STATIC_INLINE void _send_event(bme280_twi_evt_type_t event_type)
{
	bme280_twi_evt_t event = {
		.type = event_type
	};
	m_handler(&event, m_context);
}

static void _compensate_temp(int32_t adc_T, int32_t *temp)
{
	int32_t var1, var2;

	var1 = (int32_t)((adc_T / 8) - ((int32_t)bme280.cp.dig_T1 * 2));
	var1 = (var1 * ((int32_t)bme280.cp.dig_T2)) / 2048;
	var2 = (int32_t)((adc_T / 16) - ((int32_t)bme280.cp.dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)bme280.cp.dig_T3)) / 16384;

	bme280.t_fine = var1 + var2;

	*temp = (bme280.t_fine * 5 + 128 ) / 256;
}

static void _compensate_hum(double adc_H, double *hum)
{

		double var1;
		double var2;
		double var3;
		double var4;
		double var5;
		double var6;

		var1 = bme280.t_fine - 76800.0;
		var2 = (((double)bme280.cp.dig_H4) * 64.0 + (((double)bme280.cp.dig_H5) / 16384.0) * var1);
		var3 = adc_H - var2;
		var4 = ((double)bme280.cp.dig_H2) / 65536.0;
		var5 = (1.0 + (((double)bme280.cp.dig_H3) / 67108864.0) * var1);
		var6 = 1.0 + (((double)bme280.cp.dig_H6) / 67108864.0) * var1 * var5;
		var6 = var3 * var4 * (var5 * var6);
		*hum = var6 * (1.0 - ((double)bme280.cp.dig_H1) * var6 / 524288.0);


}

static void _compensate_pres(int32_t adc_P, int32_t *pres)
{

		int32_t var1;
		int32_t var2;
		int32_t var3;
		int32_t var4;
		uint32_t var5;
		uint32_t pressure;


		var1 = (((int32_t)bme280.t_fine) / 2) - (int32_t)64000;
		var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)bme280.cp.dig_P6);
		var2 = var2 + ((var1 * ((int32_t)bme280.cp.dig_P5)) * 2);
		var2 = (var2 / 4) + (((int32_t)bme280.cp.dig_P4) * 65536);
		var3 = (bme280.cp.dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
		var4 = (((int32_t)bme280.cp.dig_P2) * var1) / 2;
		var1 = (var3 + var4) / 262144;
		var1 = (((32768 + var1)) * ((int32_t)bme280.cp.dig_P1)) / 32768;
		 /* avoid exception caused by division by zero */
		if (var1) {
			var5 = (uint32_t)((uint32_t)1048576) - adc_P;
			pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
			if (pressure < 0x80000000)
				pressure = (pressure << 1) / ((uint32_t)var1);
			else
				pressure = (pressure / (uint32_t)var1) * 2;

			var1 = (((int32_t)bme280.cp.dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
			var2 = (((int32_t)(pressure / 4)) * ((int32_t)bme280.cp.dig_P8)) / 8192;
			*pres = (uint32_t)((int32_t)pressure + ((var1 + var2 + bme280.cp.dig_P7) / 16));

		}
}

void write(uint8_t reg, uint8_t data, uint8_t addr)
{
	m_xfer_done = false;
	ret_code_t err_code;
	uint8_t buf[] = {reg, data};
	err_code = nrf_drv_twi_tx(m_twi_ptr, addr, buf, sizeof(buf), false);
	APP_ERROR_CHECK(err_code);
}

void read(uint8_t reg, uint8_t *buf, int len, uint8_t addr)
{
	m_xfer_done = false;
	m_rx_reg = reg;
	nrf_drv_twi_xfer_desc_t desc = {
		.type = NRF_DRV_TWI_XFER_TXRX,
		.address = addr,
		.primary_length = sizeof(reg),
		.secondary_length = sizeof(*buf) * len,
		.p_primary_buf = &reg,
		.p_secondary_buf = buf
	};
	ret_code_t err_code = nrf_drv_twi_xfer(m_twi_ptr, &desc, 0);
	APP_ERROR_CHECK(err_code);
}


 void _write_blocking(uint8_t reg, uint8_t data) {

		write(reg, data, m_addr);
		do {
			__WFE();
		} while (m_xfer_done == false);

}

void _read_blocking(uint8_t reg, uint8_t *buf, int len) {

		read(reg, buf, len, m_addr);
		do {
			__WFE();
		} while (m_xfer_done == false);

}

void bme280_twi_evt_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{

	uint32_t data_xlsb;
	uint32_t data_msb;
	uint32_t data_lsb;

	switch (p_event->type) {
		case NRF_DRV_TWI_EVT_DONE:
			if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX) {
				switch (m_rx_reg) {
					case _REG_TEMP_MSB:

						data_msb = (uint32_t)m_buf[0] << 12;
						data_lsb = (uint32_t)m_buf[1] << 4;
						data_xlsb = (uint32_t)m_buf[2] >> 4;
						m_pres_raw = data_msb | data_lsb | data_xlsb;

						data_msb = (uint32_t)m_buf[3] << 12;
						data_lsb = (uint32_t)m_buf[4] << 4;
						data_xlsb = (uint32_t)m_buf[5] >> 4;
						m_temp_raw = data_msb | data_lsb | data_xlsb;

						data_lsb = (uint32_t)m_buf[6] << 8;
						data_msb = (uint32_t)m_buf[7];
						m_hum_raw = data_msb | data_lsb;

						_send_event(BME280_TWI_MEASUREMENT_FETCHED);

						break;
				}
				m_rx_reg = 0;
			}
			m_xfer_done = true;
			break;
		default:
			break;
	}
}

void bme280_twi_init(nrf_drv_twi_t const *       p_twi,
                     bme280_twi_config_t const * p_config,
                     bme280_twi_evt_handler_t    event_handler,
                     void *                      p_context)
{
	m_twi_ptr = p_twi;
	m_context = p_context;
	m_handler = event_handler;
	m_addr = p_config->addr;

	int16_t dig_H4_lsb;
	int16_t dig_H4_msb;
	int16_t dig_H5_lsb;
	int16_t dig_H5_msb;

	_read_blocking(_REG_DIG_T1_MSB, m_buf, 30);
	bme280.cp.dig_T1 = BME280_Con_Bytes(m_buf[0], m_buf[1]);
	bme280.cp.dig_T2 = (int16_t)BME280_Con_Bytes(m_buf[2], m_buf[3]);
	bme280.cp.dig_T3 = (int16_t)BME280_Con_Bytes(m_buf[4], m_buf[5]);
	bme280.cp.dig_P1 = BME280_Con_Bytes(m_buf[6], m_buf[7]);
	bme280.cp.dig_P2 = (int16_t)BME280_Con_Bytes(m_buf[8], m_buf[9]);
	bme280.cp.dig_P3 = (int16_t)BME280_Con_Bytes(m_buf[10], m_buf[11]);
	bme280.cp.dig_P4 = (int16_t)BME280_Con_Bytes(m_buf[12], m_buf[13]);
	bme280.cp.dig_P5 = (int16_t)BME280_Con_Bytes(m_buf[14], m_buf[15]);
	bme280.cp.dig_P6 = (int16_t)BME280_Con_Bytes(m_buf[16], m_buf[17]);
	bme280.cp.dig_P7 = (int16_t)BME280_Con_Bytes(m_buf[18], m_buf[19]);
	bme280.cp.dig_P8 = (int16_t)BME280_Con_Bytes(m_buf[20], m_buf[21]);
	bme280.cp.dig_P9 = (int16_t)BME280_Con_Bytes(m_buf[22], m_buf[23]);
	bme280.cp.dig_H1 = (m_buf[25]);

	bme280.cp.dig_H2 = (int16_t)BME280_Con_Bytes(m_buf[0], m_buf[1]);
	bme280.cp.dig_H3 = (m_buf[2]);

	dig_H4_msb = (int16_t)(int8_t)m_buf[3] * 16;
	dig_H4_lsb = (int16_t)(m_buf[4] & 0x0F);
	bme280.cp.dig_H4 = dig_H4_msb | dig_H4_lsb;

	dig_H5_msb = (int16_t)(int8_t)m_buf[5] * 16;
	dig_H5_lsb = (int16_t)(m_buf[4] >> 4);
	bme280.cp.dig_H5 = dig_H5_msb | dig_H5_lsb;

	bme280.cp.dig_H6 = (int8_t)m_buf[6];


	// Write CONFIG first, because it is only guaranteed to take effect in sleep mode.
	_write_blocking(_REG_CONFIG, ((p_config->standby << _SHIFT_T_SB) | (p_config->filter << _SHIFT_FILTER)));

	// Write CTRL_HUM next, because it only takes effect after writing CTRL_MEAS.
	_write_blocking(_REG_CTRL_HUM, BME280_TWI_OVERSAMPLING_SKIPPED);

	// Calculate, but don't write CTRL_MEAS yet. It will be written by bme280_twi_enable.
	m_ctrl_meas = ((p_config->temp_oversampling << _SHIFT_OSRS_T)
			| (BME280_TWI_OVERSAMPLING_SKIPPED << _SHIFT_OSRS_P)
			| BME280_TWI_MODE_NORMAL);
}

void bme280_twi_enable(void)
{
	// Write CTRL_MEAS to start the show
	_write_blocking(_REG_CTRL_MEAS, m_ctrl_meas);
}

void bme280_twi_measurement_fetch(void)
{
	read(_REG_TEMP_MSB, m_buf, 26, m_addr);
}

void bme280_twi_temperature_get(bme280_twi_data_t *data)
{
	_compensate_temp(m_temp_raw, &data->temp);
}

void bme280_twi_pressure_get(bme280_twi_data_t *data)
{
	_compensate_pres(m_pres_raw, &data->pres);
}

void bme280_twi_humidity_get(bme280_twi_data_t *data)
{
	_compensate_hum(m_hum_raw, &data->hum);
}
