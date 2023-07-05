/*
 * display.c
 *
 *  Created on: Jul 2, 2023
 *      Author: mfact
 */

#include <lib/i2c_lcd.h>
#include <modules/display.h>
#include <stdio.h>

static i2cLcd_HandleTypeDef h_lcd;

static const char *blank_literal = "                    ";
static const uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

static void Display_SetCursor(uint8_t row, uint8_t col) {

	(void)i2cLcd_SetCursorPosition(&h_lcd, col + row_offsets[row]);
	//TODO fix for row and col
}

void Display_Init(I2C_HandleTypeDef *h_i2c) {
	(void) i2cLcd_CreateHandle(&h_lcd, h_i2c, 0x4E);
	(void) i2cLcd_Init(&h_lcd);
	(void) i2cLcd_ClearDisplay(&h_lcd);
	(void) i2cLcd_Blink(&h_lcd, 0);
	(void) i2cLcd_Cursor(&h_lcd, 0);
}

void Display_PrintText(const char *text) {
	uint8_t i = 0;
	while (text[i]) {
		(void)i2cLcd_SendChar(&h_lcd, text[i]);
		i++;
	}
}

void Display_PrintTextPosition(uint8_t row, uint8_t col, const char *text) {
	Display_SetCursor(row, col);
	Display_PrintText(text);
}

void Display_PrintValue(uint16_t value) {
	char buffer[6];
	const uint8_t len = sprintf(buffer, "%u", value);
	Display_PrintText(buffer);
	Display_PrintText(blank_literal + len + 15); //fill with blanks
}

void Display_PrintValuePosition(uint8_t row, uint8_t col, uint16_t value) {
	Display_SetCursor(row, col);
	Display_PrintValue(value);
}

void Display_PrintTextCenter(uint8_t row, const char *text, uint8_t len) {
	Display_PrintTextPosition(0, 10-len/2, text);
}

void Display_PrintLinePosition(uint8_t row, uint8_t col, const char *text, uint8_t len) {
	Display_PrintTextPosition(row, col, text);
	Display_PrintText(blank_literal + col + len);
}

void Display_Clear(void) {
	(void)i2cLcd_ClearDisplay(&h_lcd);
}
