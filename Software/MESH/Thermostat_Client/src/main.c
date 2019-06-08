/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
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
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "sdk_config.h"
#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "proxy.h"
#include "nrf_drv_gpiote.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_onoff_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"

#include "bme.h"
#include "thermostat.h"
#include "gpio.h"
#include "ssd1306.c"
#include "ssd1306.h"

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)


#define nrf_bme 1  //if NULL Pins are mapped for SDK if 1 Pins are mapped for Thermostat PCB (Has to be changed in ssd1306.c and gpio.c File as well)

#if nrf_bme

#define Pin_SDA_BME 19
#define Pin_SCL_BME 17

#else

#define Pin_SDA_BME 2
#define Pin_SCL_BME 27

#endif

#define DEVICE_NAME                     "nRF5x Mesh Switch"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

 int32_t curr_temp = 0;
 static volatile bool   m_xfer_done = false;

static void gap_params_init(void);
static void conn_params_init(void);
void button_action_handler(uint32_t button_number);

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);

static generic_onoff_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

  } 

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else 
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }
}

static void node_reset(void)
{
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
	{
		node_reset();
	}
}

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


void button_action_handler(uint32_t button_number)
{
   
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    /* Button 1: ON, Button 2: Off, Client[0]
     * Button 2: ON, Button 3: Off, Client[1]
     */

    switch(button_number)
    {
        case 0:
        case 2:
            set_params.on_off = APP_STATE_ON;
            break;

        case 1:
        case 3:
            set_params.on_off = APP_STATE_OFF;
            break;
    }

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: ONOFF SET %d\n", set_params.on_off);

    switch (button_number)
    {
        case 0:
        case 1:
            /* Demonstrate acknowledged transaction, using 1st client model instance */
            /* In this examples, users will not be blocked if the model is busy */
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
            //hal_led_pin_set(BSP_LED_0, set_params.on_off);
            break;

        case 2:
        case 3:
            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            //hal_led_pin_set(BSP_LED_1, set_params.on_off);
            break;
      }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            //hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}


static void models_init_cb(void)
{
  
    
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = CLIENT_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());


    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    /* Set the default configuration (as defined through sdk_config.h). */
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    gap_params_init();
    conn_params_init();

    mesh_init();
}

static void app_model_init(void)
{
	for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
	    {
	        m_clients[i].settings.p_callbacks = &client_cbs;
	        m_clients[i].settings.timeout = 0;
	        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
	        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

	        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
	    }

}

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

	bme280_twi_temperature_get(&data);
	write_is_temp(data.temp);

	curr_temp = data.temp / 10;

	//If current set_temp>current temp a led will be switched on
	if(temp > curr_temp)
	{
		button_action_handler(3);
		nrf_drv_gpiote_out_set(PIN_OUT);
	}else
	{
		button_action_handler(2);
		nrf_drv_gpiote_out_clear(PIN_OUT);
	}
}

static void start(void)
{

    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

}

int main(void)
{
    initialize();
    app_model_init();
    execution_start(start);
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

    while(true)
    {

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

    	for (;;)
    	{
    		(void)sd_app_evt_wait();
    	 }

    }
	

}