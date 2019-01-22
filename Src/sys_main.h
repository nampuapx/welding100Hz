/*
 * main.h
 *
 *  Created on: 15 ���. 2017 �.
 *      Author: koko
 */

#ifndef SYS_MAIN_H_
#define SYS_MAIN_H_

#include "cmsis_os.h"

//#define GUI_DISABLE

extern osThreadId LedTaskHandle;
extern osThreadId lcd_TaskHandle;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

void Onboard_led_ON(void);
void Onboard_led_OFF(void);
void Onboard_led_TOGG(void);
void led_01_ON(void);
void led_01_OFF(void);
void led_01_TOGG(void);



#endif /* SYS_MAIN_H_ */
