/*****************************************************************************
* | File      	:   OLED_0in96.c
* | Author      :   Waveshare team
* | Function    :   OLED_0in96 OLED Module Drive function
* | Info        :
*----------------
* |	This version:   V2.0
* | Date        :   2020-08-14
* | Info        :
* -----------------------------------------------------------------------------
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "OLED_0in96.h"
#include "stdio.h"
#include "esp_log.h"
#include "i2cconfig.h"

#define vccstate SSD1306_SWITCHCAPVCC


/**
 * @description: OLED 发送一个字节
 * @return       错误信息
 * @param {uint8_t} data 需要发送的内容，数据或者命令
 * @param {uint8_t} cmd_ 1:发送数据 0:发送命令
 */
// static esp_err_t OLED_WR_Byte(uint8_t data, uint8_t cmd_)
// {
//     int ret;

//     uint8_t write_buf[2] = {((cmd_ == 1) ? (0x40) : (0x00)), data};

//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

//     return ret;
// }


/*******************************************************************************
function:
			Write register address and data
*******************************************************************************/
void OLED_0in96_WriteReg(uint8_t Reg)
{
		uint8_t write_buf[2] = {0x00, Reg};
		i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/*******************************************************************************
function:
			Common register initialization
*******************************************************************************/
void OLED_0in96_WriteData(uint8_t Data)
{	
		uint8_t write_buf[2] = {0x40, Data};
		i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static void OLED_0in96_InitReg()
{
	OLED_0in96_WriteReg(SSD1306_DISPLAYOFF);
	OLED_0in96_WriteReg(SSD1306_SETDISPLAYCLOCKDIV);
	OLED_0in96_WriteReg(0x80);                              // the suggested ratio 0x80

	OLED_0in96_WriteReg(SSD1306_SETMULTIPLEX);
	OLED_0in96_WriteReg(0x3F);
	OLED_0in96_WriteReg(SSD1306_SETDISPLAYOFFSET);
	OLED_0in96_WriteReg(0x0);                               // no offset
	OLED_0in96_WriteReg(SSD1306_SETSTARTLINE | 0x0);        // line #0
	OLED_0in96_WriteReg(SSD1306_CHARGEPUMP);
	OLED_0in96_WriteReg((vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

	OLED_0in96_WriteReg(SSD1306_MEMORYMODE);
	OLED_0in96_WriteReg(0x00);                              // 0x0 act like ks0108

	OLED_0in96_WriteReg(SSD1306_SEGREMAP | 0x1);
	OLED_0in96_WriteReg(SSD1306_COMSCANDEC);
	OLED_0in96_WriteReg(SSD1306_SETCOMPINS);
	OLED_0in96_WriteReg(0x12);           // TODO - calculate based on _rawHieght ?
	OLED_0in96_WriteReg(SSD1306_SETCONTRAST);
	OLED_0in96_WriteReg((vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
	OLED_0in96_WriteReg(SSD1306_SETPRECHARGE);
	OLED_0in96_WriteReg((vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
	OLED_0in96_WriteReg(SSD1306_SETVCOMDETECT);
	OLED_0in96_WriteReg(0x40);
	OLED_0in96_WriteReg(SSD1306_DISPLAYALLON_RESUME);
	OLED_0in96_WriteReg(SSD1306_NORMALDISPLAY);


	
    // OLED_0in96_WriteReg(0xAE); //--display off
    // OLED_0in96_WriteReg(0x00); //---set low column address
    // OLED_0in96_WriteReg(0x10); //---set high column address
    // OLED_0in96_WriteReg(0x40); //--set start line address
    // OLED_0in96_WriteReg(0xB0); //--set page address
    // OLED_0in96_WriteReg(0x81); // contract control
    // OLED_0in96_WriteReg(0xFF); //--128
    // OLED_0in96_WriteReg(0xA0); // set segment remap
    // OLED_0in96_WriteReg(0xA6); //--normal / reverse
    // OLED_0in96_WriteReg(0xA8); //--set multiplex ratio(1 to 64)
    // OLED_0in96_WriteReg(0x3F); //--1/32 duty
    // OLED_0in96_WriteReg(0xC0); // Com scan direction
    // OLED_0in96_WriteReg(0xD3); //-set display offset
    // OLED_0in96_WriteReg(0x00); //
    // OLED_0in96_WriteReg(0xD5); // set osc division
    // OLED_0in96_WriteReg(0x80); //
    // OLED_0in96_WriteReg(0xD8); // set area color mode off
    // OLED_0in96_WriteReg(0x05); //
    // OLED_0in96_WriteReg(0xD9); // Set Pre-Charge Period
    // OLED_0in96_WriteReg(0xF1); //
    // OLED_0in96_WriteReg(0xDA); // set com pin configuartion
    // OLED_0in96_WriteReg(0x12); //
    // OLED_0in96_WriteReg(0xDB); // set Vcomh
    // OLED_0in96_WriteReg(0x30); //
    // OLED_0in96_WriteReg(0x8D); // set charge pump enable
    // OLED_0in96_WriteReg(0x14); //
    // OLED_0in96_WriteReg(0xAF); //--turn on oled panel

}

/********************************************************************************
function:
			initialization
********************************************************************************/
void OLED_0in96_Init()
{
    //Hardware reset
    //OLED_0in96_Reset();

    //Set the initialization register
    OLED_0in96_InitReg();
    //Driver_Delay_ms(200);

    //Turn on the OLED display
    OLED_0in96_WriteReg(0xaf);
}

/********************************************************************************
function:
			Clear screen
********************************************************************************/
void OLED_0in96_clear()
{
    UWORD j;
	OLED_0in96_WriteReg(SSD1306_COLUMNADDR);
	OLED_0in96_WriteReg(20);         //cloumn start address
	OLED_0in96_WriteReg(OLED_0in96_HEIGHT -1); //cloumn end address
	OLED_0in96_WriteReg(SSD1306_PAGEADDR);
	OLED_0in96_WriteReg(2);         //page start address
	OLED_0in96_WriteReg(OLED_0in96_WIDTH/8 -1); //page end address
    
    for (j = 0; j < 5; j++) 
    {
	        OLED_0in96_WriteData(0xFF);
    }
}

/********************************************************************************
function:
			Update all memory to OLED
********************************************************************************/
void OLED_0in96_display(const UBYTE *Image)
{
    UWORD j, i, temp;
	OLED_0in96_WriteReg(SSD1306_COLUMNADDR);
	OLED_0in96_WriteReg(0);         //cloumn start address
	OLED_0in96_WriteReg(OLED_0in96_HEIGHT -1); //cloumn end address
	OLED_0in96_WriteReg(SSD1306_PAGEADDR);
	OLED_0in96_WriteReg(0);         //page atart address
	OLED_0in96_WriteReg(OLED_0in96_WIDTH/8 -1); //page end address
    
    for (j = 0; j < 8; j++) {
        for(i = 0; i < 128; i++) {
            temp = Image[7-j + i*8];
            OLED_0in96_WriteData(temp);
        }
    }
}
/*
 * @description: OLED 屏幕 设置坐标
 * @return       无
 * @param {uint8_t} x 坐标x轴，范围0~127
 * @param {uint8_t} y 坐标y轴，范围0~63
 */
void OLED_0in96_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_0in96_WriteReg(0xb0 + y);
    OLED_0in96_WriteReg(((x & 0xf0) >> 4) | 0x10);
    OLED_0in96_WriteReg((x & 0x0f));
}