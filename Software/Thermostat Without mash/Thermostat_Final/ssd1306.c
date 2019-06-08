#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "ssd1306.h"
#include "thermostat.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "bme.h"


#define nrf_ssd 0 //if NULL Pins are mapped for SDK if 1 Pins are mapped for Thermostat PCB (Has to be changed in main.c and gpio.c File as well)

#if nrf_ssd

#define Pin_SDA_SSD  2
#define Pin_SCL_SSD  31

#else

#define Pin_SDA_SSD 23
#define Pin_SCL_SSD 22

#endif



static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
    0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x80, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xF8, 0xE0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
    0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF,
#if (SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH > 96*16)
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,
    0x80, 0xFF, 0xFF, 0x80, 0x80, 0x00, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x8C, 0x8E, 0x84, 0x00, 0x00, 0x80, 0xF8,
    0xF8, 0xF8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xE0, 0xC0, 0x80,
    0x00, 0xE0, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xC7, 0x01, 0x01,
    0x01, 0x01, 0x83, 0xFF, 0xFF, 0x00, 0x00, 0x7C, 0xFE, 0xC7, 0x01, 0x01, 0x01, 0x01, 0x83, 0xFF,
    0xFF, 0xFF, 0x00, 0x38, 0xFE, 0xC7, 0x83, 0x01, 0x01, 0x01, 0x83, 0xC7, 0xFF, 0xFF, 0x00, 0x00,
    0x01, 0xFF, 0xFF, 0x01, 0x01, 0x00, 0xFF, 0xFF, 0x07, 0x01, 0x01, 0x01, 0x00, 0x00, 0x7F, 0xFF,
    0x80, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0xFF,
    0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x03, 0x0F, 0x3F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xC7, 0xC7, 0x8F,
    0x8F, 0x9F, 0xBF, 0xFF, 0xFF, 0xC3, 0xC0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC, 0xFC,
    0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0xC0, 0x00, 0x01, 0x03, 0x03, 0x03,
    0x03, 0x03, 0x01, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x03, 0x03, 0x00, 0x00,
    0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x03,
    0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

#endif
};

static uint8_t buffer_temp[255];

static uint8_t text_big_buf[17] = {

		50, 49, 46, 53, 246, 67, 0,0,0,0, 50, 49, 46, 53, 246, 67, 0  //XX.X°C/XX.X°C

};

/*static uint8_t text_buf[84] = {
	0,67, 117, 114, 114, 101, 110, 116,  //C u r r e n t
	0,								   //Space
	84, 101, 109, 112, 58, 50, 49, 46, 53, 246, 67, // Temp:XX.X°C
	0,0,0,0,83,101,116,0,84,101,109,112,58,
	50, 49,								//XX
	46,									//.
	53,									//X
	246, 67,0,0,							//°C [42; 0->41]
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

	};*/


// TWI instance ID.
#define TWI_INSTANCE_ID    1
static const nrf_drv_twi_t mtwi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

#define ssd1306_swap(a, b) { int16_t t = a; a = b; b = t; }


static uint8_t _i2caddr = 0x3C;
static int16_t _width, _height, WIDTH, HEIGHT, cursor_x, cursor_y;
static uint8_t textsize, rotation;
static uint16_t textcolor, textbgcolor;

bool wrap,   // If set, 'wrap' text at right edge of display
     _cp437; // If set, use correct CP437 charset (default is off)


void twi_init_disp(void)
{
	ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config_disp = {
		.scl                = Pin_SCL_SSD,
		.sda                = Pin_SDA_SSD ,
		.frequency          = NRF_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init     = false
	};

	err_code = nrf_drv_twi_init(&mtwi, &twi_config_disp, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&mtwi);
}


int16_t ssd1306_width(void)
{
    return _width;
}

int16_t ssd1306_height(void)
{
    return _height;
}

void set_rotation(uint8_t x)
{
    rotation = (x & 3);
    switch (rotation) {
    case 0:
    case 2:
        _width  = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width  = HEIGHT;
        _height = WIDTH;
        break;
    }
}

void ssd1306_set_textsize(uint8_t s)
{
    textsize = (s > 0) ? s : 1;
}

void ssd1306_set_textcolor(uint16_t c)
{
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    textcolor = textbgcolor = c;
}

void ssd1306_set_textcolor_bg(uint16_t c, uint16_t b)
{
    textcolor   = c;
    textbgcolor = b;
}

void ssd1306_set_cursor(int16_t x, int16_t y)
{
    cursor_x = x;
    cursor_y = y;
}

void display_off()
{
	ssd1306_command(0xAE);
}

void ssd1306_begin(uint8_t vccstate, uint8_t i2caddr, bool reset)
{
	
	cursor_y  = cursor_x    = 0;
    textsize  = 1;
    textcolor = textbgcolor = 0xFFFF;
    wrap      = true;
    _cp437    = false;

    _width = WIDTH = SSD1306_LCDWIDTH;
    _height = HEIGHT = SSD1306_LCDHEIGHT;
    rotation  = 0;
	
	uint8_t init1[] = {
		0,
	    //SSD1306_DISPLAYOFF,                   // 0xAE
	    SSD1306_SETDISPLAYCLOCKDIV,           // 0xD5
	    0x80,                                 // the suggested ratio 0x80
		SSD1306_SETMULTIPLEX                  // 0xA8
	};
	ssd1306_commandlist(init1,sizeof(init1));
    
    ssd1306_command(0x1F);
    
    uint8_t init2[] = {
		0,
		SSD1306_SETDISPLAYOFFSET,             // 0xD3
		0x0,                                  // no offset
		SSD1306_SETSTARTLINE | 0x0,           // line #0
		SSD1306_CHARGEPUMP };                 // 0x8D
    
    ssd1306_commandlist(init2,sizeof(init2));
    ssd1306_command(0x14);
    
    uint8_t init3[] = {
		0,
		SSD1306_MEMORYMODE,                   // 0x20
		0x00,                                 // 0x0 act like ks0108
		SSD1306_SEGREMAP | 0x1,
		SSD1306_COMSCANDEC };
		ssd1306_commandlist(init3, sizeof(init3));
    
        
	uint8_t init4a[] = {
		0,
		SSD1306_SETCOMPINS,                 // 0xDA
		0x02,
		SSD1306_SETCONTRAST,                // 0x81
		0x8F };
      
    ssd1306_commandlist(init4a, sizeof(init4a));
    ssd1306_command(SSD1306_SETPRECHARGE);                  // 0xd9
    ssd1306_command(0xF1);
    
    uint8_t init5[] = {
		0,
		SSD1306_SETVCOMDETECT,               // 0xDB
		0x40,
		SSD1306_DISPLAYALLON_RESUME,         // 0xA4
		SSD1306_NORMALDISPLAY,               // 0xA6
		SSD1306_DEACTIVATE_SCROLL,
		SSD1306_DISPLAYON };                 // Main screen turn on
		
	ssd1306_commandlist(init5, sizeof(init5));

	memset(buffer,0x00,sizeof(buffer));
}


void ssd1306_commandlist(uint8_t *c, uint8_t len)
{
        ret_code_t ret;
        ret = nrf_drv_twi_tx(&mtwi, _i2caddr, c, len, false);
        UNUSED_VARIABLE(ret);
} 


void ssd1306_command(uint8_t c)
{
        ret_code_t ret;
        uint8_t dta_send[] = {0x00, c};
        ret = nrf_drv_twi_tx(&mtwi, _i2caddr, dta_send, 2, false);
        UNUSED_VARIABLE(ret);
} 

void write_set_temp(int32_t stemp)
{

	char stemp_array[5];

	sprintf(stemp_array, "%ld", stemp);
    /* //Small Full Text
	text_buf[33] = stemp_array[0];
	text_buf[34] = stemp_array[1];
	text_buf[36] = stemp_array[2];*/

	//Big text
	text_big_buf[0] = stemp_array[0];
	text_big_buf[1] = stemp_array[1];
	text_big_buf[3] = stemp_array[2];

}

void write_is_temp(int32_t istemp)
{
	char istemp_array[5];

	sprintf(istemp_array, "%ld", istemp);
	/*//Small full text
	text_buf[14] = istemp_array[0];
	text_buf[15] = istemp_array[1];
	text_buf[17] = istemp_array[2];*/

	//Big text
	text_big_buf[10] = istemp_array[0];
	text_big_buf[11] = istemp_array[1];
	text_big_buf[13] = istemp_array[2];
}


void testdrawchar(void)
{
    //ssd1306_clear_display();
    ssd1306_set_textsize(2);
    ssd1306_set_textcolor(WHITE);
    ssd1306_set_cursor(0, 0);

    /*//Full small text
    if(text_buf[33] > text_buf[14])
    {
    	text_buf[40] = '+';
    }else if(text_buf[33] == text_buf[14] && text_buf[34] > text_buf[15])
    {
    	text_buf[40] = '+';
    }else if(text_buf[33] == text_buf[14] && text_buf[34] == text_buf[15] && text_buf[36] > text_buf[17])
    {
    	text_buf[40] = '+';
    }else
    	text_buf[40] = '\0';*/



    if(text_big_buf[0] > text_big_buf[10])
    {
    	text_big_buf[16] = '+';
    }else if(text_big_buf[0] == text_big_buf[10] && text_big_buf[1] > text_big_buf[11])
    {
    	text_big_buf[16] = '+';
    }else if(text_big_buf[0] == text_big_buf[10] && text_big_buf[1] == text_big_buf[11] && text_big_buf[3] > text_big_buf[13])
    {
    	text_big_buf[16] = '+';
    }else
    	text_big_buf[16] = '\0';



    /*for full small text
    for (uint8_t i = 0; i < 84; i++) {
        //if (i == '\n') continue;
    	//if(i == )

        ssd1306_write(text_buf[i]);
        if ((i > 0) && (i % 21 == 0))
        	ssd1306_write('\n');

    }*/

    //for big text
    for (uint8_t i = 0; i < 17; i++) {

    		/*if(i == 6)
    		{
    			ssd1306_set_textsize(1);
    		}*/
           ssd1306_write(text_big_buf[i]);
           if ((i > 0) && (i % 21 == 0))
           	ssd1306_write('\n');

       }

    ssd1306_display();
}

void ssd1306_write(uint8_t c)
{
    if (c == '\n') {
        cursor_y += textsize * 8;
        cursor_x  = 0;
    }
    else if (c == '\r') {
        // skip em
    }
    else {
        ssd1306_draw_char(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
        cursor_x += textsize * 6;
        if (wrap && (cursor_x > (_width - textsize * 6))) {
            cursor_y += textsize * 8;
            cursor_x = 0;
        }
    }

}


void ssd1306_draw_char(int16_t x, int16_t y, uint8_t c, uint16_t color, uint16_t bg, uint8_t size)
{

    if ((x >= _width)            || // Clip right
            (y >= _height)           || // Clip bottom
            ((x + 6 * size - 1) < 0) || // Clip left
            ((y + 8 * size - 1) < 0))   // Clip top
        return;

    if (!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    for (int8_t i = 0; i < 6; i++ ) {
        uint8_t line;
        if (i == 5)
            line = 0x0;
        else
            line = font [(c * 5) + i];
        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, color);
                else {  // big size
                    ssd1306_fill_rect(x + (i * size), y + (j * size), size, size, color);
                }
            }
            else if (bg != color) {
                if (size == 1) // default size
                    ssd1306_draw_pixel(x + i, y + j, bg);
                else {  // big size
                    ssd1306_fill_rect(x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
}


void ssd1306_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    // Update in subclasses if desired!
    for (int16_t i = x; i < x + w; i++) {
        ssd1306_draw_fast_vline(i, y, h, color);
    }
}

void ssd1306_draw_fast_vline(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    bool __swap = false;
    switch (rotation) {
    case 0:
        break;
    case 1:
        // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
        __swap = true;
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        x -= (h - 1);
        break;
    case 2:
        // 180 degree rotation, invert x and y - then shift y around for height.
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        y -= (h - 1);
        break;
    case 3:
        // 270 degree rotation, swap x & y for rotation, then invert y
        __swap = true;
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    if (__swap) {
        ssd1306_draw_fast_hline_internal(x, y, h, color);
    }
    else {
        ssd1306_draw_fast_vline_internal(x, y, h, color);
    }
}

void ssd1306_draw_fast_hline_internal(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    // Do bounds/limit checks
    if (y < 0 || y >= HEIGHT) {
        return;
    }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x;
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ( (x + w) > WIDTH) {
        w = (WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) {
        return;
    }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y & 7);

    switch (color) {
    case WHITE:
        while (w--) {
            *pBuf++ |= mask;
        };
        break;
    case BLACK:
        mask = ~mask;
        while (w--) {
            *pBuf++ &= mask;
        };
        break;
    case INVERSE:
        while (w--) {
            *pBuf++ ^= mask;
        };
        break;
    }
}


void ssd1306_draw_fast_vline_internal(int16_t x, int16_t __y, int16_t __h, uint16_t color)
{

    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= WIDTH) {
        return;
    }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;

    }

    // make sure we don't go past the height of the display
    if ( (__y + __h) > HEIGHT) {
        __h = (HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
        return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;


    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y / 8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y & 7);
    if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if ( h < mod) {
            mask &= (0XFF >> (mod - h));
        }

        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }

        // fast exit if we're done here!
        if (h < mod) {
            return;
        }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH;
    }


    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE)  {          // separate copy of the code so we don't impact performance of the black/white write version with an extra comparison per loop
            do  {
                *pBuf = ~(*pBuf);

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
        else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do  {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH;

                // adjust h & y (there's got to be a faster way for me to do this, but this should still help a fair bit for now)
                h -= 8;
            }
            while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color) {
        case WHITE:
            *pBuf |=  mask;
            break;
        case BLACK:
            *pBuf &= ~mask;
            break;
        case INVERSE:
            *pBuf ^=  mask;
            break;
        }
    }
}



void ssd1306_clear_display(void)
{
    memset(buffer, 0x00, (SSD1306_LCDWIDTH * SSD1306_LCDHEIGHT / 8));
}


void ssd1306_draw_pixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT))
        return;

    // check rotation, move pixel around if necessary
    switch (rotation) {
    case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
    case 2:
        x = WIDTH - x - 1;
        y = HEIGHT - y - 1;
        break;
    case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }

    // x is which column
    switch (color) {
    case WHITE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] |=  (1 << (y & 7));
        break;
    case BLACK:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] &= ~(1 << (y & 7));
        break;
    case INVERSE:
        buffer[x + (y / 8)*SSD1306_LCDWIDTH] ^=  (1 << (y & 7));
        break;
    }
}


void ssd1306_invert_display(uint8_t i)
{
    if (i) {
        ssd1306_command(SSD1306_INVERTDISPLAY);
    }
    else {
        ssd1306_command(SSD1306_NORMALDISPLAY);
    }
}


void ssd1306_display(void)
{
	
	uint8_t dlist1[] = {
			0,
			SSD1306_PAGEADDR,
			0,                         // Page start address
			0xFF,                      // Page end (not really, but works here)
			SSD1306_COLUMNADDR,
			0,127 };

	ssd1306_commandlist(dlist1, sizeof(dlist1));

	for(uint8_t i = 0; i<32; i++)
	{
		//setup single line buffer
		buffer_temp[0] = 0x40;
		memcpy(&buffer_temp[1],&buffer[i*16],16);
		nrf_drv_twi_tx(&mtwi, _i2caddr, buffer_temp, 17, false);
	}
}

/** @} */
