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
#include <stdbool.h>

/* HAL */
#include "app_timer.h"
#include "nrf_delay.h"


#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"

/* Core */
#include "sdk_config.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "net_state.h"
#include "mesh_adv.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_conn_params.h"
#include "ble_hci.h"
#include "proxy.h"
#include "mesh_opt_gatt.h"
#include "mesh_config.h"
#include "rand.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "app_level.h"

//NFC
#include "app_error.h"
#include "app_scheduler.h"
#include "nfc_t4t_lib.h"
#include "ndef_file_m.h"
#include "nfc_ndef_msg.h"
#include "nfc_uri_msg.h"
#include "flash_manager.h"

#define PIN_OUT 18

#define DEVICE_NAME                     "Heizung"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(150,  UNIT_1_25_MS)           /**< Minimum acceptable connection interval. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(250,  UNIT_1_25_MS)           /**< Maximum acceptable connection interval. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(100)                        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called. */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */


// Variables for BLE Mesh Models
bool SWITCH_STATE_1 = 0;
bool SWITCH_STATE_2 = 0;
bool SWITCH_STATE_3 = 0;
bool SWITCH_STATE_4 = 0;

int16_t Level_Output_1 = 0;
int16_t Level_Output_2 = 0;
int16_t Level_Output_3 = 0;
int16_t Level_Output_4 = 0;

static bool m_device_provisioned;



//	Node Hardware Output - Configuration 8 Bit
//	|    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
//                                            Kanal W   Kanal B   Kanal G   Kanal R
//	0 ... Lowside
//  1 ... Highside

uint8_t node_output_configuration = 0b00000000;
uint8_t Configuration_Out_1 = 0;
uint8_t Configuration_Out_2 = 0;
uint8_t Configuration_Out_3 = 0;
uint8_t Configuration_Out_4 = 0;



// Variables for the static provisioning
nrf_mesh_prov_provisioning_data_t prov_data;


uint8_t networkkey_configuration[NRF_MESH_KEY_SIZE];
uint8_t applicationkey_configuration[NRF_MESH_KEY_SIZE];
uint8_t devicekey_configuration[NRF_MESH_KEY_SIZE];

uint16_t node_address_configuration = 0;
uint8_t netkey_index = 0;
uint8_t appkey_index = 0;
uint8_t iv_index = 0;
static dsm_handle_t m_appkey_handles[DSM_APP_MAX];
static dsm_handle_t m_netkey_handles[DSM_SUBNET_MAX];


//NFC
#define APP_SCHED_MAX_EVENT_SIZE 0                  /**< Maximum size of scheduler events. */
#define APP_SCHED_QUEUE_SIZE     4                  /**< Maximum number of events in the scheduler queue. */
#define APP_DEFAULT_BTN          BSP_BOARD_BUTTON_0 /**< Button used to set default NDEF message. */

//Hardware configuration data
#define CUSTOM_HARDWARE_CONFIG_ENTRY_HANDLE		0x0001
#define CUSTOM_DATA_FLASH_PAGE_COUNT                 1
#define CUSTOM_DATA_WORDS						     2
flash_manager_config_t custom_data_manager_config;
typedef struct
{
  uint32_t data[CUSTOM_DATA_WORDS];
}custom_data_format_t; // Format for the custom data
static flash_manager_t m_custom_data_flash_manager; // flash manager instance


static uint8_t ndef_msg_buf[NDEF_FILE_SIZE];      /**< Buffer for NDEF file. */
static uint8_t welcome_msg[] = {'L','i','c','h','t','s','t','e','u','e','r','u','n','g'};



static void gap_params_init(void);
static void conn_params_init(void);
//static void init_pwm_output(void);
static void read_configuration(void);
static void write_provisioning_configuration(void);
static void write_hw_configuration(void);
static void unprovisioning(void);
static void init_NFC(void);


static void app_onoff_server_0_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_0_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);



/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_CONFIG_FORCE_SEGMENTATION,
                     APP_CONFIG_MIC_SIZE,
                     app_onoff_server_0_set_cb,
                     app_onoff_server_0_get_cb)



static void app_onoff_server_0_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Red: %d\n", onoff)
	SWITCH_STATE_1 = onoff;

    if(SWITCH_STATE_1 == 0)
    {
		nrf_drv_gpiote_out_clear(PIN_OUT);
    	m_onoff_server_0.state.present_onoff = 0;
    	app_onoff_status_publish(&m_onoff_server_0);
    }
    if(SWITCH_STATE_1 == 1)
	{
		nrf_drv_gpiote_out_set(PIN_OUT);      	
    	m_onoff_server_0.state.present_onoff = 1;
    	app_onoff_status_publish(&m_onoff_server_0);
	}

}


static void app_onoff_server_0_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    *p_present_onoff = nrf_gpio_pin_read(PIN_OUT);
}


static void app_model_init(void)
{
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, 0));
}



static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    (void) nrf_mesh_on_sd_evt(sd_evt);
}

NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL); // @suppress("Unused variable declaration in file scope")


static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
    	__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset\n")
    	mesh_stack_device_reset();
    }
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



static void provisioning_complete_cb(void)
{
    /* Restores the application parameters after switching from the Provisioning service to the Proxy  */
    gap_params_init();
    conn_params_init();

}



static void models_init_cb(void)
{

}



static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = SERVER_NODE_UUID_PREFIX;

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
	uint32_t ret_code;

	__LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

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



    custom_data_manager_config.write_complete_cb = NULL;
    custom_data_manager_config.invalidate_complete_cb = NULL;
    custom_data_manager_config.remove_complete_cb = NULL;
    custom_data_manager_config.min_available_space = WORD_SIZE;
    custom_data_manager_config.p_area = (const flash_manager_page_t *) (((const uint8_t *) dsm_flash_area_get()) - (ACCESS_FLASH_PAGE_COUNT * PAGE_SIZE) - (NET_FLASH_PAGE_COUNT * PAGE_SIZE) );
    custom_data_manager_config.page_count = CUSTOM_DATA_FLASH_PAGE_COUNT;
    ret_code = flash_manager_add(&m_custom_data_flash_manager, &custom_data_manager_config);
    if (NRF_SUCCESS != ret_code)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Flash error: no memory\n",ret_code);

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

}




/*static void init_pwm_output(void)
{
	  uint8_t ch1 = 0;
	  uint8_t ch2 = 0;
	  uint8_t ch3 = 0;
	  uint8_t ch4 = 0;

	  if(Configuration_Out_1 == 0)
	  {
		  ch1 = LED_R_Debug;
	  }
	  else
	  {
		  ch1 = LED_R_Pin_HS;
	  }

	  if(Configuration_Out_2 == 0)
	  {
		  ch2 = LED_G_Debug;
	  }
	  else
	  {
		  ch2 = LED_G_Pin_HS;
	  }

	  if(Configuration_Out_3 == 0 )
	  {
		  ch3 = LED_B_Debug;
	  }
	  else
	  {
		  ch3 = LED_B_Pin_HS;
	  }

	  if(Configuration_Out_4 == 0 )
	  {
		  ch4 = LED_W_Pin_LS;
	  }
	  else
	  {
		  ch4 = LED_W_Pin_HS;
	  }

	  PWM_init(ch1, Configuration_Out_1, ch2, Configuration_Out_2, ch3, Configuration_Out_3, ch4, Configuration_Out_4);
	  set_PWM_Duty_Cycle(RED,    0);
	  set_PWM_Duty_Cycle(GREEN,  0);
	  set_PWM_Duty_Cycle(BLUE,   0);
	  set_PWM_Duty_Cycle(WHITE,  0);
}*/



static void read_configuration(void)
{
	dsm_local_unicast_address_t node_address;

	node_output_configuration = 0;

	dsm_local_unicast_addresses_get(&node_address);
	__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
	__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Counter: %d\n", node_address.count);
	__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning: %d\n", mesh_stack_is_device_provisioned());

	const fm_entry_t * p_read_raw = flash_manager_entry_get(&m_custom_data_flash_manager, CUSTOM_HARDWARE_CONFIG_ENTRY_HANDLE);
	const custom_data_format_t * p_read_data = (const custom_data_format_t *) p_read_raw->data;

	if(p_read_data->data[1] == 1)
	{
		node_output_configuration = p_read_data->data[0];
	}
	else
	{
		node_output_configuration = 0;
	}

	Configuration_Out_1 = node_output_configuration & 0b00000001;
	Configuration_Out_2 = (node_output_configuration & 0b00000010) >> 1;
	Configuration_Out_3 = (node_output_configuration & 0b00000100) >> 2;
	Configuration_Out_4 = (node_output_configuration & 0b00001000) >> 3;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "read HW config: %x\n",node_output_configuration);

}


static void unprovisioning(void)
{
	(void) proxy_stop();
	mesh_stack_config_clear();
	mesh_stack_device_reset();
}


static void write_provisioning_configuration(void)
{
	prov_data.address = node_address_configuration;
	prov_data.iv_index = iv_index;
	memcpy(prov_data.netkey, networkkey_configuration, NRF_MESH_KEY_SIZE);
	prov_data.netkey_index = netkey_index;

	rand_hw_rng_get(devicekey_configuration, NRF_MESH_KEY_SIZE);

	mesh_stack_provisioning_data_store(&prov_data, devicekey_configuration);


	m_netkey_handles[netkey_index] = dsm_net_key_index_to_subnet_handle(netkey_index);
	ERROR_CHECK(dsm_appkey_add(0, m_netkey_handles[netkey_index], applicationkey_configuration, &m_appkey_handles[appkey_index]));

	ERROR_CHECK(access_model_application_bind(m_onoff_server_0.server.model_handle, m_appkey_handles[appkey_index]));
	ERROR_CHECK(access_model_publish_application_set(m_onoff_server_0.server.model_handle, m_appkey_handles[appkey_index]));

	provisioning_complete_cb();
	mesh_stack_device_reset();

}


static void write_hw_configuration(void)
{

	uint32_t current_time, starttime;
	__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Schreibe HW config %x\n", node_output_configuration);

	fm_entry_t * p_entry = flash_manager_entry_alloc(&m_custom_data_flash_manager, CUSTOM_HARDWARE_CONFIG_ENTRY_HANDLE, sizeof(custom_data_format_t));
	if (p_entry == NULL)
	{
		__LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "No memory available\n");
	}
	else
	{

	  custom_data_format_t * p_custom_data = (custom_data_format_t *) p_entry->data;
	  p_custom_data->data[0] = node_output_configuration;
	  p_custom_data->data[1] = 1;

	  // b) write to flash
	  flash_manager_entry_commit(p_entry);
	  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "write:%x\n",p_entry->data[0]);

	}
    flash_manager_wait();

	mesh_stack_device_reset();
}



//NFC Part
/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void * context, nfc_t4t_event_t event, const uint8_t * data, size_t dataLength, uint32_t flags)
{
	(void)context;
	uint8_t highbyte = 0, lowbyte = 0;
	node_address_configuration = 0;
	uint32_t size = 0;

    switch (event)
    {
        case NFC_T4T_EVENT_FIELD_ON:

        	break;

        case NFC_T4T_EVENT_FIELD_OFF:

            break;

        case NFC_T4T_EVENT_NDEF_READ:

            break;

        case NFC_T4T_EVENT_NDEF_UPDATED:
        	if (dataLength > 0)
        	{

        		// Unprovisioning
				if( (dataLength==10) && (ndef_msg_buf[9] == '0') && (ndef_msg_buf[10] == '0') && (ndef_msg_buf[11] == ';'))
				{
					unprovisioning();
				}

				// Provisioning over NFC
				else if( (dataLength==84) && (ndef_msg_buf[9] == '0') && (ndef_msg_buf[10] == '1') && (ndef_msg_buf[85] == ';'))
				{

					for(uint8_t i = 13; i < 17; i++)
					{
						node_address_configuration = node_address_configuration * 16;

						if( ndef_msg_buf[i] <= 57)
						{
							node_address_configuration = node_address_configuration + (ndef_msg_buf[i] - 48);
						}

						if( ndef_msg_buf[i] >= 65)
						{
							node_address_configuration = node_address_configuration + (ndef_msg_buf[i] - 55);
						}
					}

					for(uint8_t i = 19; i < 51; i = i + 2)
					{
						if( ndef_msg_buf[i] <= 57)
						{
							highbyte = (ndef_msg_buf[i] - 48) * 16;
						}

						if( ndef_msg_buf[i] >= 65)
						{
							highbyte = (ndef_msg_buf[i] - 55) * 16;
						}

						if( ndef_msg_buf[i+1] <= 57)
						{
							lowbyte = ndef_msg_buf[i+1] - 48;
						}

						if( ndef_msg_buf[i+1] >= 65)
						{
							lowbyte = ndef_msg_buf[i+1] - 55;
						}

						networkkey_configuration[(i-19)/2] = highbyte + lowbyte;
					}

					for(uint8_t i = 53; i < 85; i = i + 2)
					{
						if( ndef_msg_buf[i] <= 57)
						{
							highbyte = (ndef_msg_buf[i] - 48) * 16;
						}

						if( ndef_msg_buf[i] >= 65)
						{
							highbyte = (ndef_msg_buf[i] - 55) * 16;
						}

						if( ndef_msg_buf[i+1] <= 57)
						{
							lowbyte = ndef_msg_buf[i+1] - 48;
						}

						if( ndef_msg_buf[i+1] >= 65)
						{
							lowbyte = ndef_msg_buf[i+1] - 55;
						}

						applicationkey_configuration[(i-53)/2] = highbyte + lowbyte;
					}
					write_provisioning_configuration();
				}

				// HW config speichern
				else if( (dataLength==13) && (ndef_msg_buf[9] == '1') && (ndef_msg_buf[10] == '1') && (ndef_msg_buf[14] == ';'))
				{
					if( ndef_msg_buf[13] <= 57)
					{
						node_output_configuration = ndef_msg_buf[13] - 48;
					}

					if( ndef_msg_buf[13] >= 65)
					{
						node_output_configuration = ndef_msg_buf[13] - 55;
					}
					write_hw_configuration();
				}
				else
				{
					size = sizeof(ndef_msg_buf);
					nfc_uri_msg_encode(NFC_URI_NONE, welcome_msg, sizeof(welcome_msg), ndef_msg_buf, &size);
				}
        	}

            break;

        default:
            break;
    }
}

static void init_gpio(void)
{
	ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_init(PIN_OUT, &out_config);


}


static void init_NFC(void)
{
	uint32_t size = 0;


	size = sizeof(ndef_msg_buf);
	nfc_uri_msg_encode(NFC_URI_NONE, welcome_msg, sizeof(welcome_msg), ndef_msg_buf, &size);

    /* Set up NFC */
    nfc_t4t_setup(nfc_callback, NULL);

    /* Run Read-Write mode for Type 4 Tag platform */
    nfc_t4t_ndef_rwpayload_set(ndef_msg_buf, sizeof(ndef_msg_buf));


    /* Start sensing NFC field */
    nfc_t4t_emulation_start();
}




int main(void)
{

	initialize();

	read_configuration();
	app_model_init();
	//init_pwm_output();
	init_gpio();

	execution_start(start);

	init_NFC();

    for (;;)
    {
    	(void)sd_app_evt_wait();
    }
}
