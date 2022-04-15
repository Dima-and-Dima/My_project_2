/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/

void MX_GPIO_Init(void)
{
  //LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOE);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);


 //RCC->APB2ENR|=(1<<6)|(1<<0); // Clock APB2 enadled, AFIO

//---------Button
  GPIOE->CRL|=(1<<22)|(1<<20);// LED 5
  GPIOE->CRL&=~((1<<21)|(1<<23)); // LED POrt 5
  GPIOA->BSRR|=(1<<21); // LED RESET

  GPIOE->CRL|=(1<<26)|(1<<24);// LED 6
  GPIOE->CRL&=~((1<<25)|(1<<27)); // LED POrt 6
  LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_6);
  //GPIOA->BSRR|=(1<<21); // LED RESET


    /**/
    LL_GPIO_ResetOutputPin(PS_GPIO_Port, PS_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOA, C86_Pin|RD_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOC, A0_Pin|ChipSelect_Pin);

    /**/
    LL_GPIO_ResetOutputPin(GPIOD, D0_Pin|D1_Pin|D2_Pin|D3_Pin
                            |D4_Pin|D5_Pin);

    /**/
    LL_GPIO_SetOutputPin(WR_GPIO_Port, WR_Pin);

    /**/
    LL_GPIO_SetOutputPin(RST_GPIO_Port, RST_Pin);

    /**/
    LL_GPIO_SetOutputPin(GPIOD, SCL_Pin|SDA_Pin);

    /**/
    GPIO_InitStruct.Pin = PS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    //GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(PS_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = C86_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(C86_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = RD_Pin|WR_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    //GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = A0_Pin|RST_Pin|ChipSelect_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                            |D4_Pin|D5_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    //GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = SCL_Pin|SDA_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
   // GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);


    GPIO_InitStruct.Pin = LL_GPIO_PIN_6; // LCD_LED
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_9; // SOLENOID_1
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8; // SOLENOID_2
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_7; // SOLENOID_3
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_14; // SOLENOID_1
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);


   // GPIO_InitStruct.Pin = LL_GPIO_PIN_14; // Oscylator
   // GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
   // GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    //LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /**/
      LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE10);

      /**/
      EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_10;
      EXTI_InitStruct.LineCommand = ENABLE;
      EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
      EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
      LL_EXTI_Init(&EXTI_InitStruct);

      /**/
      LL_GPIO_SetPinPull(ENCODER_BUTTON_GPIO_Port, ENCODER_BUTTON_Pin, LL_GPIO_PULL_UP);

      /**/
      LL_GPIO_SetPinMode(ENCODER_BUTTON_GPIO_Port, ENCODER_BUTTON_Pin, LL_GPIO_MODE_INPUT);

      /* EXTI interrupt init*/
      NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
      NVIC_EnableIRQ(EXTI15_10_IRQn);

//------------------------------I2C





}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
