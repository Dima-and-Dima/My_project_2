/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* TIM6 init function */
void MX_TIM6_Init(void)
{

	 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
	//�� ��������� ������� ���� 24 ��� ��� ������������� ������ 8 ���
	TIM6->PSC = 1000000 - 1; //��������� �������� �� 1000 "�����" � �������
	TIM6->ARR =1000; //��������� ���������� ��� � �������
	//TIM6->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������
	//TIM6->CR1 |= TIM_CR1_CEN; //������ �������
	NVIC_EnableIRQ(TIM6_DAC_IRQn); //���������� TIM6_DAC_IRQn ����������
}

void MX_TIM7_Init(void)
{

	 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
	//�� ��������� ������� ���� 24 ��� ��� ������������� ������ 8 ���
	TIM7->PSC = 1000 - 1; //��������� �������� �� 1000 "�����" � �������
	TIM7->ARR =5; //��������� ���������� ��� � �������
	TIM7->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������f
	TIM7->CR1 |= TIM_CR1_CEN; //������ �������
	NVIC_EnableIRQ(TIM7_DAC_IRQn); //���������� TIM6_DAC_IRQn ����������
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
