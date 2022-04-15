/*
 * usart_stm32.h
 *
 *  Created on: 31 џэт. 2021 у.
 *      Author: Prog
 */

#ifndef INC_USART_STM32_H_
#define INC_USART_STM32_H_

#include "main.h"


void send_8bit_data_usart(char ch);
void send_str_usart(char * str);

#endif /* INC_USART_STM32_H_ */
