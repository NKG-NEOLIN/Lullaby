/*
 * @date  
 *   2021/03/15
 *
 * @version 
 *   1.0.0
 *
 * @author 
 *   NinoLiu
 *
 * @file iPrint.h 
 * 
 * @Description : 
*/   
#include "stm32f4xx_hal_uart.h"
//-------------------------------------------------------------------------------------------------
#ifndef IPRINT_H
#define IPRINT_H
//-------------------------------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif
//--------------------------------------------------------------------------------------------
/* USER CODE BEGIN Application */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
extern UART_HandleTypeDef huart2;

PUTCHAR_PROTOTYPE 
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
		 return ch;

	}
#ifdef __cplusplus
}
#endif // extern "C" 
//--------------------------------------------------------------------------------------------             
#endif  //end IPRINT_H
