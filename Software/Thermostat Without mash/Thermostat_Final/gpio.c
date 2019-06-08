
#include "gpio.h"
#include "bme.h"
#include "thermostat.h"
#include "ssd1306.h"


int set_mode = 0;
int lastEncoded = 0;
int FLAG = 0;
int32_t set_temp = 200;


/*
 * This function triggers the Rotary Encoder movements and evaluates if CCW or CW rotation was inserted
 *Function is called by Interrupt Handler from Rotary Encoder Pin A or B
 */
void cnt_updown(void)
{

	int signalA = nrf_gpio_pin_read(PIN_IN_A);

	int signalB = nrf_gpio_pin_read(PIN_IN_B);
	//read sig A and B fetched into one int
	int encoded = (signalB << 1) | signalA;
	//old and new value are fetched into one int
	int sum = (lastEncoded << 2) | encoded;

	//If current and old gpio sig are 0bXXXX the rotation was CCW
	if ((sum == 0b1011 || sum == 0b1110 || sum == 0b1000 || sum == 0b0001) && FLAG == 0)
	{
		set_temp--;
		FLAG = 1;
		//Get current temp
		get_is_val(set_temp);
		//write updated set temp to display buffer
		write_set_temp(set_temp);
		//switch on display
		activate_display();


	}//If current and old gpio sig are 0bXXXX the rotation was CW
	else if ((sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b0111) && FLAG == 0)
	{
		set_temp++;
		FLAG = 1;
		//Get current temp
		get_is_val(set_temp);
		//Write updated set temp to display buffer
		write_set_temp(set_temp);
		//switch on display
		activate_display();

	}else
		//Make sure to never falsely detect double interrupt
		FLAG = 0;

	lastEncoded = encoded;

}

void activate_display(void)
{

	//Init display
    ssd1306_begin(0x2, 0x3C, false);
    //Calculate display buffer
	testdrawchar();
	//Display Output via i2c bus
	ssd1306_display();


}
/*
 *Interrupt triggered by Rotary Encoder Channel A or B
 */
void gpiote_rotary_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	cnt_updown();
}
/*
 * Interrupt triggered by Button to activate Display
 */
void gpiote_button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	activate_display();
}
/*
 * Init of Gpio Pins for Button LED and Rotary Encoder
 */
void gpio_init(void)
{

    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);





    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;


    err_code = nrf_drv_gpiote_in_init(PIN_IN_A, &in_config, gpiote_rotary_handler);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_gpiote_in_init(PIN_IN_B, &in_config, gpiote_rotary_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_in_init(PIN_IN_C, &in_config, gpiote_button_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN_A, true);
    nrf_drv_gpiote_in_event_enable(PIN_IN_B, true);
    nrf_drv_gpiote_in_event_enable(PIN_IN_C, true);

    nrf_drv_gpiote_out_init(PIN_OUT, &out_config);

}


