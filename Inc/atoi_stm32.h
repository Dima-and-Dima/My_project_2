/*
 * atoi_stm32.h
 *
 *  Created on: 31 ���. 2021 �.
 *      Author: Prog
 */

#ifndef INC_ATOI_STM32_H_
#define INC_ATOI_STM32_H_

#include "main.h"

void reverse(char str[], int length);
char* itoa(int num, char* buffer, int base);
void swap(char *t1, char *t2);

#endif /* INC_ATOI_STM32_H_ */
