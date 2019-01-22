/*
 * gui.c
 *
 *  Created on: 16 θών. 2017 γ.
 *      Author: koko
 */
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "sys_main.h"
//#include "midi_gen_logic.h"

#include "gui.h"
#include "fonts.h"
//#include "LiquidCrystal_I2C.h"
//#include "PCF8535_dri.h"

#include "ssd1306.h"

xQueueHandle q_lcd = 0;





void gui_print_param1(uint8_t dig){
#ifdef GUI_DISABLE
	return;
#endif
	type_q_lcd_element working_msg;

    working_msg.x = 0;
    working_msg.y = 0;

    	sprintf(working_msg.txt,"param1= %6u",dig);

    if(q_lcd)xQueueSend( q_lcd, (void *) &working_msg, portMAX_DELAY );
//    gui_print_lcd_dev();
}

//void gui_print_lcd_dev(void){
//#ifdef GUI_DISABLE
//    return;
//#endif
//    type_q_lcd_element working_msg;
//
//    working_msg.x = 0;
//    working_msg.y = 1;
//
//    sprintf(working_msg.txt,"OUT BPM %3u  D%u ",bpm/dev_bpm,dev_bpm);
//
//    if(q_lcd)xQueueSend( q_lcd, (void *) &working_msg, portMAX_DELAY );
//
//}

//void gui_print_start_wait(void){
//#ifdef GUI_DISABLE
//	return;
//#endif
//	type_q_lcd_element working_msg;
//
//	working_msg.x = (128-40)/2;
//	working_msg.y = 2;
//    sprintf(working_msg.txt,"WAIT");
//    if(q_lcd)xQueueSend( q_lcd, (void *) &working_msg, portMAX_DELAY );
//}
//
//void gui_print_start_wait_clear(void){
//#ifdef GUI_DISABLE
//	return;
//#endif
//	type_q_lcd_element working_msg;
//	BaseType_t xHigherPriorityTaskWoken;
//
//    working_msg.x = (128-40)/2;
//    working_msg.y = 2;
//    sprintf(working_msg.txt,"    ");
//    //xQueueSend( q_lcd, (void *) &working_msg, portMAX_DELAY );
//    if(q_lcd)xQueueSendFromISR( q_lcd, (void *) &working_msg, &xHigherPriorityTaskWoken);
//}
//




//	if(q_lcd){
//		step_pos = (uint8_t)(MIDI_start_status/12);
//		if(step_pos != pred_step_pos){
//			pred_step_pos = step_pos;
//
//			working_msg.xy = 0x01;
//			sprintf(working_msg.txt,"OOOOOOOOOOOOOOOO");
//
//
//			working_msg.txt[step_pos<<1] = '>';
//			working_msg.txt[(step_pos<<1)+1] = '<';
//
//
//			xQueueSendFromISR( q_lcd, (void *) &working_msg, &xHigherPriorityTaskWoken);
//
//		}
//	}

//
//	if(q_lcd){
//		step_pos = (uint8_t)(MIDI_start_status/12);
//		if(step_pos != pred_step_pos){
//			pred_step_pos = step_pos;
//
//			working_msg.xy = 0x01;
//			sprintf(working_msg.txt,"                ");
//			//sprintf(working_msg.txt,"<><><><><><><><>");
//			//sprintf(&(working_msg.txt[step_pos<<1]),"%c%c",4,5);
//
//			if(step_pos%2){
//				working_msg.txt[step_pos<<1] = '<';
//				working_msg.txt[(step_pos<<1)+1] = '>';
//			}else{
//				working_msg.txt[step_pos<<1] = 4;
//				working_msg.txt[(step_pos<<1)+1] = 5;
//			}
//
//			xQueueSendFromISR( q_lcd, (void *) &working_msg, &xHigherPriorityTaskWoken);
//
//
////			working_msg.xy = (uint8_t)(((step_pos)<<5)|0x01);
////			if(!step_pos){
////				sprintf(working_msg.txt,"%c%c",4,5);
////			}else{
////				sprintf(working_msg.txt,"<>");
////			}
////			xQueueSendFromISR( q_lcd, (void *) &working_msg, &xHigherPriorityTaskWoken);
//		}
//	}

//}








//
//void displayKeyCodes(void) {
//  uint8_t i = 0;
//  while (1) {
//    LCDI2C_clear();
//    //LCDI2C_setCursor(2,2);
//    //LCDI2C_write_String("TEN Electronics");
//    LCDI2C_setCursor(0, 0);
//	char buf[10];
//	itoa(i, buf, 10);
//    LCDI2C_write_String("Cds 0x"); LCDI2C_write_String(buf);
//	itoa(i+19, buf, 10);
//    LCDI2C_write_String("-0x"); LCDI2C_write_String(buf);
//    LCDI2C_setCursor(0, 1);
//    int j;
//    for (j=0; j<20; j++) {
//      LCDI2C_write(i+j);
//    }
//    i+=16;
//    if (i<15) break;
//    osDelay(700);
//  }
//}



//const uint8_t bell[8]  = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
//const uint8_t note[8]  = {0x2,0x3,0x2,0xe,0x1e,0xc,0x0};
//const uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
//const uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};

//const uint8_t b_01[8]  = {0x18,0x18,0x18,0x0,0x0,0x0,0x0};
//const uint8_t b_02[8]  = {0x3,0x3,0x3,0x0,0x0,0x0,0x0};
//const uint8_t b_03[8]  = {0x0,0x0,0x0,0x0,0x3,0x3,0x3};
//const uint8_t b_04[8]  = {0x0,0x0,0x0,0x0,0x18,0x18,0x18};
//
//
//
//const uint8_t ar_left[8]  = {0x1,0x3,0x7,0xf,0x7,0x3,0x1};
//const uint8_t ar_right[8] = {0x8,0xc,0xe,0xf,0xe,0xc,0x8};
//const uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
//const uint8_t retarrow[8] = {0x1,0x1,0x5,0x9,0x1f,0x8,0x4};

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( lcd_TaskHandle,
							( 1UL << 0UL ),
							eSetBits,
							&xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR( lcd_TaskHandle,
							( 1UL << 0UL ),
							eSetBits,
							&xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}






void lcd_Task(void){

	//osDelay(30);
	type_q_lcd_element working_msg;
	q_lcd = xQueueCreate( 8, sizeof( type_q_lcd_element ) );

#ifdef GUI_DISABLE
    while(1){
        osDelay(30);
    }
#endif

#ifdef LIQUIDCRYST
	LCDI2C_init(0x4e,SIMBOL_LCD_WIDTH,SIMBOL_LCD_ROWS);
	LCDI2C_backlight();
	LCDI2C_createChar(0, (uint8_t*)retarrow);
	LCDI2C_createChar(1, (uint8_t*)b_01);
	LCDI2C_createChar(2, (uint8_t*)b_02);
	LCDI2C_createChar(3, (uint8_t*)b_03);
	LCDI2C_createChar(4, (uint8_t*)b_04);
	LCDI2C_createChar(5, (uint8_t*)ar_left);
	LCDI2C_createChar(6, (uint8_t*)ar_right);
	LCDI2C_createChar(7, (uint8_t*)cross);
	LCDI2C_createChar(8, (uint8_t*)retarrow);
	LCDI2C_clear();
	//LCDI2C_setCursor(0,0);
	//LCDI2C_write(4);
	//  Usart1_Send_String("End");
#endif

#ifdef LCD_PCF8535
    LCD_PCF8535_Init();
    Vlcd = 34;
    bias = 4;
    LCD_PCF8535_set_Vlcd(0);
    LCD_PCF8535_set_bias(0);

    LCD_PCF8535_write_txt_8x15(1, 1, "kolio");
#endif

    osDelay(1);
    SSD1306_Init();


//    while(1){
//        osDelay(30);
//    }
	for(;;){


		xQueueReceive( q_lcd, &( working_msg ), portMAX_DELAY );
		//LCDI2C_setCursor(working_msg.xy>>4,working_msg.xy&0x0f);
		//LCDI2C_write_String(working_msg.txt);

		//LCD_PCF8535_write_txt_8x15(working_msg.xy>>4, working_msg.xy&0x0f, working_msg.txt);

		SSD1306_GotoXY((working_msg.x)*Font_7x10.FontWidth, (working_msg.y)*Font_7x10.FontHeight);
		SSD1306_Puts(working_msg.txt, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();

	}

}
