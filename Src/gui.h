/*
 * gui.h
 *
 *  Created on: 16 ���. 2017 �.
 *      Author: koko
 */

#ifndef GUI_H_
#define GUI_H_


#define SIMBOL_LCD_WIDTH	24
#define SIMBOL_LCD_ROWS		2



typedef struct{
	uint8_t x;
	uint8_t y;
	char txt[SIMBOL_LCD_WIDTH];
}type_q_lcd_element;


void gui_print_param1(uint8_t dig);


#endif /* GUI_H_ */
