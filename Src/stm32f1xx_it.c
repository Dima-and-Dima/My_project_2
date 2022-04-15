/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */
extern unixColodar unixTime;
extern volatile uint8_t tmp_hour_var;
extern volatile uint8_t tmp_minute_var;
//extern volatile uint8_t tmp_work_time_var;
extern volatile uint32_t led_lcd_timer;

//uint8_t tim_mas[]={'0',}


 void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
	if(  (GPIOE->IDR&(1<<5))==0  )
		     {
		    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_5);  //led_1
		    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_6);//led_2
		    	 //GPIOE->BSRR|=(1<<3);
		    	 //NewState=1;
		     }
		     else
		     {
		    	 //GPIOE->BRR|=(1<<3);
		    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_5);
		    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_6);
		    	 //NewState=0;
		     }
}

void RTC_IRQHandler(void)
{


  if (RTC->CRL & (1<<1) )
  {                       // check alarm flag
    RTC->CRL &= ~(1<<1);                          // clear alarm flag

   }

} // end RTC_IRQHandler


/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/*
void RTC_IRQHandler(void)
{
	//if( RTC->CRL&(1<<0) )
	//{
		RTC->CRL&=~(1<<0); // Clear flag
		counterToColodar(mRTCGetCounter(), &unixTime);
				  send_str_usart("    Read time, sec ");
				  send_8bit_data_usart(digit_mas[unixTime.sec/10]);
				  send_8bit_data_usart(digit_mas[unixTime.sec%10]);
				  send_str_usart(" , min  ");
				  send_8bit_data_usart(digit_mas[unixTime.min/10]);
				  send_8bit_data_usart(digit_mas[unixTime.min%10]);
				  send_str_usart("  END      ");

	     if(  (GPIOE->IDR&(1<<5))==0  )
	     {
	    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_5);  //led_1
	    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_6);//led_2
	    	 //GPIOE->BSRR|=(1<<3);
	    	 //NewState=1;
	     }
	     else
	     {
	    	 //GPIOE->BRR|=(1<<3);
	    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_5);
	    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_6);
	    	 //NewState=0;
	     }

	//}
}

*/


void EXTI4_IRQHandler(void)
{

  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);

   //__NVIC_ClearPendingIRQ();

  }
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */

void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    /* USER CODE BEGIN LL_EXTI_LINE_8 */

    /* USER CODE END LL_EXTI_LINE_8 */
  }

  __disable_irq ();
 // TIM7->CR1 |= (TIM_CR1_CEN); //������ �������
  EXTI->IMR&=~(1<<8);
  __enable_irq ();
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}


void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	//AFIO->EXTICR[1]&=~(1<<10);// EXTI6 -> Encoder button
    //EXTI->IMR&=~(1<<10);//Button 6 PIN // off interrupt
	__disable_irq ();
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
	  {
	    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
	  }
	//LL_mDelay(200);

	LL_USART_TransmitData8(USART3, 'R');

			   		  	 	 	while(LL_USART_IsActiveFlag_TC(USART3)==0)
			   		  	 	 		{

			   		  	 	 		}


	 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
	  	//�� ��������� ������� ���� 24 ��� ��� ������������� ������ 8 ���
	  	TIM6->PSC = 32000 - 1; //��������� �������� �� 1000 "�����" � �������
	  	TIM6->ARR =25; //��������� ���������� ��� � ....
	  	TIM6->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������
	  	TIM6->CR1 |= TIM_CR1_CEN; //������ �������
	  	NVIC_EnableIRQ(TIM6_DAC_IRQn); //���������� TIM6_DAC_IRQn

	  	//AFIO->EXTICR[1]&=~(1<<10);// EXTI6 -> Encoder button
        EXTI->IMR&=~(1<<10);//Button 6 PIN // off interrupt

        //TIM7->CR1 &= ~(TIM_CR1_CEN);


  /* USER CODE END EXTI9_5_IRQn 0 */

    /* USER CODE BEGIN LL_EXTI_LINE_5 */

    /* USER CODE END LL_EXTI_LINE_5 */




  __enable_irq ();
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
//void TIM6_IRQHandler(void)
void TIM6_DAC_IRQHandler(void)
{

	 TIM6->SR &= ~TIM_SR_UIF; // clear tim6 flag
	// TIM7->CCR1|=(1<<7);

	__disable_irq ();


   if(button_flag==0)
   {
	   if(  (is_pin_button_set) ==0 )
	   {
		           button_flag=1;
		   	   	   TIM6->PSC = 32000 - 1; //��������� �������� �� 1000 "�����" � �������
		   	   	   TIM6->ARR =350; //��������� ���������� ��� � �������
		   	   	   TIM6->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������
		   	   	   TIM6->CR1 |= TIM_CR1_CEN; //������ �������

		   	   	   button_flag=1;
		   	   	   LL_USART_TransmitData8(USART3, 'U');

		   	   		while(LL_USART_IsActiveFlag_TC(USART3)==0)
		   	   			{

		   	   			}
	   }
	   else
	   {
		   TIM6->CR1 &= ~(1<<0);
		   TIM6->DIER &= ~(1<<0);

		   //AFIO->EXTICR[1]|=(1<<10);// EXTI6 -> Encoder button
		   EXTI->IMR|=(1<<10);//Button 6 PIN // off interrupt
		   LL_USART_TransmitData8(USART3, 'K');

		   while(LL_USART_IsActiveFlag_TC(USART3)==0)
		   	{

		   	}
	   }


   }

   else if(button_flag==1)
    {
	   if(  (is_pin_button_set)==0)
	   {
	      TIM6->PSC = 32000 - 1; //��������� �������� �� 1000 "�����" � �������
	      TIM6->ARR =400; //��������� ���������� ��� � �������
	      TIM6->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������
	      TIM6->CR1 |= TIM_CR1_CEN; //������ �������
	      button_flag=2;
	      LL_USART_TransmitData8(USART3, 'D');

	      while(LL_USART_IsActiveFlag_TC(USART3)==0)
	      	{

	      	}
	   }
	   else
	   {
		   //GPIOE->BRR|=(1<<3);

		   LL_USART_TransmitData8(USART3, '1');

		   		  	 	 	while(LL_USART_IsActiveFlag_TC(USART3)==0)
		   		  	 	 		{

		   		  	 	 		}
	 led_lcd_timer=50000;


//----------------------------------- WORK Place -----------------------//
	 upState=0;
	 downState=0;

		   if(eWindow==MAIN_W)
		   {
			   eWindow=ZONA_X;
			   curent_position=0;
			   f_sel_list=0;
		   }
		   else if(eWindow==ZONA_X)
		   {
			   if(curent_position==0)
			     {
				   eWindow=CALENDAR;
				   eUpdate=UPDATE;

			     }
		       else if(curent_position==1)
			   	 {
			   	   eZONA_SETUP=ZONA_1;
			   	eWindow=WEEK_SETUP;
			   	 }
			   else if(curent_position==2)
			     {
                    eZONA_SETUP=ZONA_2;
                    eWindow=WEEK_SETUP;
			     }
			   else if(curent_position==3)
			   {
				   eZONA_SETUP=ZONA_3;
				   eWindow=WEEK_SETUP;
			   }
			   else if(curent_position==4)
			   {
				   eZONA_SETUP=ZONA_4;
				   eWindow=WEEK_SETUP;
			   }

			   //eUpdate=UPDATE;

			   curent_position=0;
			   f_sel_list=0;
		   }
		   else if(eWindow==WEEK_SETUP)
		   {
			   //eWindow=DAY_TIME;
			   eWindow=AMOUNT_SETUP;
			   if(curent_position==0)
			   {
				   eDAY_NOW=PONEDILOK;
			   }
			   else if(curent_position==1)
			   	 {
			   		eDAY_NOW=VIVTOROK;
			   	 }
			   else if(curent_position==2)
			   {
				   eDAY_NOW=SEREDA;
			   }
			   else if(curent_position==3)
			   {
				   eDAY_NOW=CHETVER;
			   }
			   else if(curent_position==4)
			   {
				   eDAY_NOW=PYATNUTSYA;
			   }
			   else if(curent_position==5)
			   {
				   eDAY_NOW=SUBOTA;
			   }
			   else if(curent_position==6)
			   {
				   eDAY_NOW=NEDILYA;
			   }
			   else if(curent_position==7)
			   	{
			   	   eDAY_NOW=PARNI_DNI;
			   	}

			   send_str_usart("                     eDAY_NOW :::::    ");
			   send_8bit_data_usart(digit_mas[eDAY_NOW]);
			   send_str_usart("                         ");
			   curent_position=0;
			   f_sel_list=0;

			   //u8g2_UserInterfaceMessage("Title1", "Title2", "Title3", " Ok \n Cancel ");
		   }
		   else if(eWindow==MESSAGE_X)
		   		   {
		   			   eWindow=DAY_TIME;
		   			  curent_position=0;
		   			   f_sel_list=0;

		   			   //u8g2_UserInterfaceMessage("Title1", "Title2", "Title3", " Ok \n Cancel ");
		   		   }
		   else if(eWindow==AMOUNT_SETUP)
		   {
			   if(curent_position==0)
			   {
				   if(area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day==0)// Add
				   {
				      eWindow=DAY_TIME;
				   }
				   else
				   {
					   //clearData();
					   eWindow=CLEAR_DATA;
				   }

			   }
			   else
			   {
				   eWindow=DAY_TIME;
				   flag_rewrite_data=curent_position;
			   }
			   //else if(curent_position<=area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day)
			   //{

			   //}


		   }
		   else if(eWindow==DAY_TIME)
		   {
			   flag_frame=1;
			   if(eHOUR_MINUTE==HOUR)
			   {
				   eHOUR_MINUTE=MINUTE;
				   flag_frame=1;
				   frame_timer=4000;
				   eUpdate=UPDATE;
			   }
			   else
			   {
				   eHOUR_MINUTE=HOUR;
				   flag_frame=1;
				   frame_timer=4000;
				   eWindow=MINUTE_SEC;




			   }
		   }
		   else if(eWindow==MINUTE_SEC)
		   {
			   eHOUR_MINUTE=HOUR;
			   flag_frame=1;
			  frame_timer=4000;


			  if(f_sel_list==0)
				  send_str_usart("    MINUTE  !!!!    ");
			  else
				  send_str_usart("    SECUNDE  !!!!    ");

			   eWindow=WORK_TIME;
			   eUpdate=UPDATE;
		   }
		   else if(eWindow==WORK_TIME)
		   {
			   eWindow=WEEK_SETUP;
			   curent_position=0;
			   flag_write_flash=1;
			   //f_sel_list=0;
		   }
		   else if(eWindow==CALENDAR)
		   {
			   if(eCalendar==DAY_CALENDAR)
			   {

				   eCalendar=MON_CALENDAR;
				   frame_timer=4000;
				   flag_frame=1;
			   }
			   else if(eCalendar==MON_CALENDAR)
			   {
				   eCalendar=YEAR_CALENDAR;
				   frame_timer=4000;
				   flag_frame=1;
			   }
			   else if(eCalendar==YEAR_CALENDAR)
			   {
				   eCalendar=HOUR_CALENDAR;
				   frame_timer=4000;
				   flag_frame=1;
			   }
			   else if(eCalendar==HOUR_CALENDAR)
			   {
				   eCalendar=MIN_CALENDAR;
				   frame_timer=4000;
				   flag_frame=1;
			   }
			   else if(eCalendar==MIN_CALENDAR)
			   {
				   eCalendar=DAY_CALENDAR;
				   eWindow=ZONA_X;
				   eUpdate=UPDATE;
			   }
			   //eWindow=ZONA_X;
		   }
		   button_flag=0;
		   //------------------------------------END---------------------------------//

		   TIM6->CR1 &= ~(1<<0);
		   TIM6->DIER &= ~(1<<0);

		   EXTI->IMR|=(1<<10);//Button 6 PIN // off interrupt
		  // TIM7->CR1 |= TIM_CR1_CEN;// Encoder timer ON
		   upState=0;
		   downState=0;
	   }

    }


   else if(button_flag==2)
   {


           if(  (is_pin_button_set)==0)
           	   {

        	   LL_USART_TransmitData8(USART3, '2');

        	   	  	 	 	while(LL_USART_IsActiveFlag_TC(USART3)==0)
        	   	  	 	 		{

        	   	  	 	 		}
        	   	  	 	 LL_USART_TransmitData8(USART3, ' ');

        	   	  	 	 	  	 	 	while(LL_USART_IsActiveFlag_TC(USART3)==0)
        	   	  	 	 	  	 	 		{

        	   	  	 	 	  	 	 		}

         led_lcd_timer=50000;

        //------------------------------WORK Place --------------------------//
        	if(eWindow!=MAIN_W)
        	  {

        		if(eWindow==WEEK_SETUP)
        		        	   	   {
        		        	   		   flag_write_flash=1;
        		        	   	   }
        		        	   	   else
        		        	   	   {
        		        	   	     flag_write_flash=0;
        		        	   	   }


        		   if(eWindow==CALENDAR)
        		   {
        			   unixTime.year=tmp_year_calendar+2000;
        			   unixTime.mon=tmp_mon_calendar;
        			   unixTime.mday= tmp_day_calendar;
        			   unixTime.hour=tmp_hour_calendar;
        		       unixTime.min=tmp_min_calendar;
        		       mRTCSetCounter(colodarToCounter(&unixTime) );
        		       eCalendar=DAY_CALENDAR;
        		   }
        		   eHOUR_MINUTE=HOUR;
        		   flag_frame=1;
        		   frame_timer=4000;


        	   	   eWindow=MAIN_W;
        	       eUpdate=UPDATE;
        	       curent_position=0;
        	   	   f_sel_list=0;


        	  }
        	else if(eWindow==MAIN_W)
        	{
        		if(flag_start_stop)
        			flag_start_stop=0;
        		else
        			flag_start_stop=1;
        	}


           	      TIM6->PSC = 32000 - 1; //��������� �������� �� 1000 "�����" � �������
           	      TIM6->ARR =50; //��������� ���������� ��� � �������
           	      TIM6->DIER |= TIM_DIER_UIE; //���������� ���������� �� �������
           	      TIM6->CR1 |= TIM_CR1_CEN; //������ �������
           	      button_flag=3;
           	   }
           else
           {

        	   	EXTI->IMR|=(1<<10);//Button 6 PIN // off interrupt
        	   	TIM6->CR1 &= ~(1<<0);
        	    TIM6->DIER &= ~(1<<0);
        	   	button_flag=0;
        	   	LL_USART_TransmitData8(USART3, 'Q');

        	   	while(LL_USART_IsActiveFlag_TC(USART3)==0)
        	   		{

        	   		}
           }
           //-----------------------------------end---------------------------------//

   }
   else
   {

	           	      // AFIO->EXTICR[1]|=(1<<10);// EXTI6 -> Encoder button
	   //TIM7->CR1 |= TIM_CR1_CEN;// Encoder timer ON
	   upState=0;
	   downState=0;
	           	   	EXTI->IMR|=(1<<10);//Button 6 PIN // off interrupt
	           	   	TIM6->CR1 &= ~(1<<0);
	           	    TIM6->DIER &= ~(1<<0);
	           	   	button_flag=0;
	           	 LL_USART_TransmitData8(USART3, 'P');

	           	 while(LL_USART_IsActiveFlag_TC(USART3)==0)
	           	 	{

	           	 	}
   }

	 		__enable_irq ();





}

void TIM7_IRQHandler(void)
{
	__disable_irq ();
	TIM7->SR &= ~TIM_SR_UIF; // clear tim6 flag
	eUpdate=UPDATE;

	if (RTC->CRL & (1<<0) )
	  {                       // check second flag
		RTC->CRL &= ~(1<<0);                          // clear alarm flag


		if(flag_sec_work==1)
			flag_sec_work=0;
		else
			flag_sec_work=1;


		if(time_mesure_sensor>=0)
			--time_mesure_sensor;


		eUpdate=UPDATE;



	    if(main_Day_Now!=unixTime.mday)
	    {
	    	main_Day_Now=unixTime.mday;
	    	//if(main_Day_Now)
	    	whatDayOfNow();
	    }

	    flag_on_off_zona=1;

		counterToColodar(mRTCGetCounter(), &unixTime);
		//send_str_usart("    Den is ");


	     // вычисление дня недели



/*
		uint8_t a, y, m, dNow=0;

		a = (14-unixTime.mon)/12;
		y = unixTime.year-a;
		m = unixTime.mon + 12*a -2;
		dNow = (unixTime.mday +y + y/4 - y/100 + y/400 + (31*m)/12 )%7;
*/


		//send_8bit_data_usart(digit_mas[doW]);


		send_str_usart("    ");

		  counterToColodar(mRTCGetCounter(), &unixTime);
		                  send_str_usart("  hour ");
		  		  		  send_8bit_data_usart(digit_mas[unixTime.hour/10]);
		  		  		  send_8bit_data_usart(digit_mas[unixTime.hour%10]);
		                  send_str_usart("   min  ");
		  		  		  send_8bit_data_usart(digit_mas[unixTime.min/10]);
		  		  		  send_8bit_data_usart(digit_mas[unixTime.min%10]);
		  				  send_str_usart("  sec ");
		  				  send_8bit_data_usart(digit_mas[unixTime.sec/10]);
		  				  send_8bit_data_usart(digit_mas[unixTime.sec%10]);

		  				  send_str_usart("  --      ");



		  				  /*
		  	     if(  (GPIOE->IDR&(1<<5))==0  )
		  	     {
		  	    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_5);  //led_1
		  	    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_6);//led_2
		  	    	 //GPIOE->BSRR|=(1<<3);
		  	    	 //NewState=1;
		  	     }
		  	     else
		  	     {
		  	    	 //GPIOE->BRR|=(1<<3);
		  	    	 LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_5);
		  	    	 LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_6);
		  	    	 //NewState=0;
		  	     }
		  	     */




	    }



				   NewState=read_gray_code_from_encoder();



			  			 			if(NewState!=OldState)
			  			 			{
			  			 				switch(OldState)
			  			 				{
			  			 					case 2:
			  			 					{
			  			 						if(NewState == 3) upState++;
			  			 						if(NewState == 0) downState++;
			  			 						break;
			  			 					}

			  			 					case 0:
			  			 					{
			  			 						if(NewState == 2) upState++;
			  			 						if(NewState == 1) downState++;
			  			 						break;
			  			 					}
			  			 					case 1:
			  			 					{
			  			 						if(NewState == 0) upState++;
			  			 						if(NewState == 3) downState++;
			  			 						break;
			  			 					}

			  			 					case 3:
			  			 					{
			  			 						if(NewState == 1) upState++;
			  			 						if(NewState == 2) downState++;
			  			 						break;
			  			 					}

			  			 				}
			  			 			 }

			  			 			OldState=NewState;


			  			 	if (upState >= 4)
			  			 		{
			  			 	      led_lcd_timer=500000;
			  			 		  upState=0;
			  			 		//TIM7->CR1 &= ~(TIM_CR1_CEN); //������ �������
			  			 		//EXTI->IMR|=(1<<8);

			  			 		   if(eWindow!=MAIN_W && eWindow!=MESSAGE_X && eWindow!=DAY_TIME && eWindow!=WORK_TIME && eWindow!=CALENDAR)
			  			 		   {
			  			 			  eDirection=UP;
			  			 		   }
			  			 		   else if(eWindow==DAY_TIME)
			  			 		   {
			  			 			switch(eHOUR_MINUTE)
			  			 			{
			  			 			  case HOUR:
			  			 			   if(tmp_hour_var<=0)
			  			 			    {
			  			 			      tmp_hour_var=24;

			  			 			     }
			  			 			     --tmp_hour_var;
			  			 			  eUpdate=UPDATE;
			  			 			   break;

			  			 			  case MINUTE:
			  			 				if(tmp_minute_var<=0)
			  			 				{
			  			 					tmp_minute_var=60;
			  			 				}
			  			 				--tmp_minute_var;
			  			 				eUpdate=UPDATE;
			  			 				break;
			  			 			}




			  			 		   }

			  			 		else if(eWindow==WORK_TIME)
			  			 		{
			  			 			if(tmp_sec_min_work_tim<=0)
			  			 			{
			  			 				tmp_sec_min_work_tim=60;
			  			 			}
			  			 			--tmp_sec_min_work_tim;
			  			 			eUpdate=UPDATE;
			  			 		}
			  			 		else if(eWindow==CALENDAR)
			  			 		{

                                   if(eCalendar==YEAR_CALENDAR)
                                   {
                                	   if(tmp_year_calendar<=1)
                                	   {
                                		   tmp_year_calendar=40;
                                	   }
                                	   else
                                	   {
                                	     --tmp_year_calendar;
                                	   }
                                   }
                                   else if(eCalendar==MON_CALENDAR)
                                   {
                                	   if(tmp_mon_calendar<=1)
                                	   {
                                		   tmp_mon_calendar=12;
                                	   }
                                	   else
                                	   {
                                	     --tmp_mon_calendar;
                                	   }
                                   }
                                   else if(eCalendar==DAY_CALENDAR)
                                   {
                                		if(tmp_mon_calendar==1)
                                			{
                                				if(tmp_day_calendar<=1)
                                				  {
                                				  	tmp_day_calendar=31;
                                				  }
                                				 else
                                				  {
                                				  	--tmp_day_calendar;
                                				  }
                                			}
                                		else if(tmp_mon_calendar==2)
                                			{
                                				if(tmp_day_calendar<=1)
                                				  	{
                                				  		tmp_day_calendar=28;
                                				  	}
                                				else
                                				  	{
                                				  		--tmp_day_calendar;
                                				  	}
                                			}
                                		else if(tmp_mon_calendar==3)
                                			{
                                				if(tmp_day_calendar<=1)
                                				  {
                                				  	tmp_day_calendar=31;
                                				  }
                                				else
                                				 {
                                				  	--tmp_day_calendar;
                                				 }
                                			}
                                		else if(tmp_mon_calendar==4)
                                			{
                                				if(tmp_day_calendar<=1)
                                				  {
                                				  	tmp_day_calendar=30;
                                				  }
                                			   else
                                				  {
                                				  	--tmp_day_calendar;
                                				  }
                                			}
                                				  			 					  			 					else if(tmp_mon_calendar==5)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=31;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==6)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=30;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==7)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=31;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==8)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=31;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==9)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=30;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==10)
                                				  			 					  			 				    {
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=31;
                                				  			 					  			 						    }
                                				  			 					  			 					    else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==11)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=30;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                				  			 					  			 					else if(tmp_mon_calendar==12)
                                				  			 					  			 					{
                                				  			 					  			 						if(tmp_day_calendar<=1)
                                				  			 					  			 							{
                                				  			 					  			 								tmp_day_calendar=31;
                                				  			 					  			 							}
                                				  			 					  			 						else
                                				  			 					  			 							{
                                				  			 					  			 								--tmp_day_calendar;
                                				  			 					  			 							}
                                				  			 					  			 					}
                                   }
                                   else if(eCalendar==HOUR_CALENDAR)
                                   {
                                	   if(tmp_hour_calendar<=0)
                                	   {
                                		   tmp_hour_calendar=23;
                                	   }
                                	   else
                                	   {
                                	     --tmp_hour_calendar;
                                	   }
                                   }
                                   else if(eCalendar==MIN_CALENDAR)
                                   {
                                	   if(tmp_min_calendar<=1)
                                	   {
                                		   tmp_min_calendar=59;
                                	   }
                                	   else
                                	   {
                                	     --tmp_min_calendar;
                                	   }
                                   }


			  			 		}
			  			 		//else if(eCalendar==DAY_CALENDAR)



			  			 			  //LL_USART_TransmitData8(USART3, '+');

			  			 		  		while(LL_USART_IsActiveFlag_TC(USART3)==0)
			  			 		  			  	    		{

			  			 		  			  	    		}

                                }


			  			 		if (downState >=4)
			  			 			{
			  			 			  led_lcd_timer=500000;
			  			 			  downState=0;
			  			 			  //TIM7->CR1 &= ~(TIM_CR1_CEN); //������ �������
			  			 			 // EXTI->IMR|=(1<<8);


			  			 			if(eWindow!=MAIN_W && eWindow!=MESSAGE_X && eWindow!=DAY_TIME && eWindow!=WORK_TIME && eWindow!=CALENDAR)
			  			  			   {
			  			 			     eDirection=DOWN;
			  			  			   }
			  			 			else if(eWindow==DAY_TIME)
			  			 			  {
			  			 				switch(eHOUR_MINUTE)
			  			 				{
			  			 				  case HOUR:
			  			 				    if(tmp_hour_var>=23)
			  			 				    {
			  			 					 tmp_hour_var=0;
			  			 				    }
			  			 				   else
			  			 				    {
			  			 				     ++tmp_hour_var;
			  			 				    }
			  			 				 eUpdate=UPDATE;
			  			 				   break;

			  			 				  case MINUTE:
			  			 					if(tmp_minute_var>=59)
			  			 					{
			  			 						tmp_minute_var=0;
			  			 					}
			  			 					else
			  			 					{
			  			 						++tmp_minute_var;
			  			 					}
			  			 					eUpdate=UPDATE;
			  			 					break;
			  			 				}

			  			 			   }
			  			 			else if(eWindow==WORK_TIME)
			  			 			 {


			  			 					if(tmp_sec_min_work_tim>=59)
			  			 						{
			  			 						  tmp_sec_min_work_tim=0;
			  			 						}
			  			 					else
			  			 						{
			  			 						   ++tmp_sec_min_work_tim;
			  			 						}
			  			 					eUpdate=UPDATE;


			  			 			  }
			  			 			else if(eWindow==CALENDAR)
			  			 			{
			  			 				if(eCalendar==YEAR_CALENDAR)
			  			 				{
			  			 					if(tmp_year_calendar>=40)
			  			 					{
			  			 						tmp_year_calendar=1;
			  			 					}
			  			 					else
			  			 					{
			  			 					  ++tmp_year_calendar;
			  			 					}
			  			 				}
			  			 				else if(eCalendar==MON_CALENDAR)
			  			 				{
			  			 					if(tmp_mon_calendar>=12)
			  			 					{
			  			 						tmp_mon_calendar=1;
			  			 					}
			  			 					else
			  			 					{
			  			 					   ++tmp_mon_calendar;
			  			 					}
			  			 				}
			  			 				else if(eCalendar==DAY_CALENDAR)
			  			 				{
			  			 					if(tmp_mon_calendar==1)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 						{
			  			 							tmp_day_calendar=1;
			  			 						}
			  			 						else
			  			 						{
			  			 					      ++tmp_day_calendar;
			  			 						}
			  			 					}
			  			 					else if(tmp_mon_calendar==2)
			  			 					{
			  			 						if(tmp_day_calendar>=28)
			  			 						{
			  			 							tmp_day_calendar=1;
			  			 						}
			  			 						else
			  			 						{
			  			 							++tmp_day_calendar;
			  			 						}
			  			 					}
			  			 					else if(tmp_mon_calendar==3)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==4)
			  			 					{
			  			 						if(tmp_day_calendar>=30)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==5)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==6)
			  			 					{
			  			 						if(tmp_day_calendar>=30)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==7)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==8)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==9)
			  			 					{
			  			 						if(tmp_day_calendar>=30)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==10)
			  			 				    {
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 						    }
			  			 					    else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==11)
			  			 					{
			  			 						if(tmp_day_calendar>=30)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 					else if(tmp_mon_calendar==12)
			  			 					{
			  			 						if(tmp_day_calendar>=31)
			  			 							{
			  			 								tmp_day_calendar=1;
			  			 							}
			  			 						else
			  			 							{
			  			 								++tmp_day_calendar;
			  			 							}
			  			 					}
			  			 				}
			  			 				else if(eCalendar==HOUR_CALENDAR)
			  			 				{
			  			 					if(tmp_hour_calendar>=23)
			  			 					{
			  			 						tmp_hour_calendar=0;
			  			 					}
			  			 					else
			  			 					{
			  			 					   ++tmp_hour_calendar;
			  			 					}

			  			 				}
			  			 				else if(eCalendar==MIN_CALENDAR)
			  			 				{
			  			 					if(tmp_min_calendar>=59)
			  			 					{
			  			 						tmp_min_calendar=1;
			  			 					}
			  			 					else
			  			 					{
			  			 					  ++tmp_min_calendar;
			  			 					}
			  			 				}

			  			 			}

			  			 			 // LL_USART_TransmitData8(USART3, '-');

			  			 		  	  while(LL_USART_IsActiveFlag_TC(USART3)==0)
			  			 		  			  	    		{

			  			 		  			  	    		}
			  			 			}

		if(led_lcd_timer<=0)
		 {
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6); // OFF LED_LCD
		 }
		else
		{
			--led_lcd_timer;
			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); // On LED_LCD
		}

		if(eWindow==DAY_TIME||eWindow==CALENDAR)
		{
			if(frame_timer<=0)
			{
              if(flag_frame)
            	  flag_frame=0;
              else
				  flag_frame=1;


              frame_timer=4000;
			}


			else
			{

				--frame_timer;
			}
		}




		__enable_irq ();

    //EXTI->IMR|=(1<<6);//Button 6 PIN

}
