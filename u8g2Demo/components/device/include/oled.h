#ifndef _OLED_H_
#define _OLED_H_

#include <stdio.h>
#include "u8g2.h"
#include "u8x8.h"



#define OLED_ADDR 0x3C // OLED的IIC地址，逻辑分析仪读出的

#define OLED_CMD 0
#define OLED_DATA 1


void OLED_Init(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
void OLED_Clear(void);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size2);
uint32_t oled_pow(uint8_t m, uint8_t n);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
void u8g2Init(u8g2_t *u8g2);
#endif