/*
 * usart_stm32.c
 *
 *  Created on: 31 ���. 2021 �.
 *      Author: Prog
 */

#include "usart_stm32.h"
void send_str_usart(char * str)
{
	uint8_t i_usart=0;

	while(str[i_usart])
	{
      send_8bit_data_usart(str[i_usart]);
      i_usart++;
	}
}
void send_8bit_data_usart(char ch)
{
	LL_USART_TransmitData8(USART3, ch);

			    while(LL_USART_IsActiveFlag_TC(USART3)==0)
				  {

				  }

}
