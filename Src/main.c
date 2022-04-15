/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "u8g2/u8g2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint16_t  kd=0;

unixColodar unixTime;
u8g2_t u8g2;
const uint8_t digit_mas[10]={'0','1','2','3','4','5','6','7','8','9'};
const uint8_t lcd_hight=32;
const uint8_t lcd_wight=128;
int8_t width;
int8_t hight;


uint8_t read_gray_code_from_encoder(void )
{
	uint8_t val=0;

	if( (is_pin_encoder_set_1)!=0)
	val |= (1<<1);

	if( (is_pin_encoder_set_2 )!=0)
	val |= (1<<0);

	return val;
}

static uint8_t state, cnt0, cnt1;
uint8_t delta, toggle;


uint8_t debounce(uint8_t sample)
{


	delta = sample ^ state;

	cnt1 = cnt1 ^ cnt0;
	cnt0 = ~cnt0;

	cnt0 &= delta;
	cnt1 &= delta;

	toggle = cnt0 & cnt1;
	state ^= toggle;

	return state;
}

uint32_t colodarSetting(int year, char month, char day, char hour, char min, char sec)
 {
  unixTime.year=year;
  unixTime.mon=month;
  unixTime.mday=day;
  unixTime.hour=hour;
  unixTime.min=min;
  unixTime.sec=sec;
  return colodarToCounter(&unixTime);
 }


uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
	switch(msg){
		//Initialize SPI peripheral
		case U8X8_MSG_GPIO_AND_DELAY_INIT:


		break;

		//Function which implements a delay, arg_int contains the amount of ms
		case U8X8_MSG_DELAY_MILLI:
			LL_mDelay(arg_int);

		break;
		//Function which delays 10us
		case U8X8_MSG_DELAY_10MICRO:
		for (uint16_t n = 0; n < 320; n++)
		{
			__NOP();
		}

		break;
		//Function which delays 100ns
		case U8X8_MSG_DELAY_100NANO:
		__NOP();

		break;
		//Function to define the logic level of the clockline
		case U8X8_MSG_GPIO_SPI_CLOCK:
			if (arg_int)  LL_GPIO_SetOutputPin(SCL_GPIO_Port, SCL_Pin);
			else  LL_GPIO_ResetOutputPin(SCL_GPIO_Port, SCL_Pin);

		break;
		//Function to define the logic level of the data line to the display
		case U8X8_MSG_GPIO_SPI_DATA:
			if (arg_int)  LL_GPIO_SetOutputPin(SDA_GPIO_Port, SDA_Pin);
			 else  LL_GPIO_ResetOutputPin(SDA_GPIO_Port, SDA_Pin);

		break;
		// Function to define the logic level of the CS line
		case U8X8_MSG_GPIO_CS:
			if (arg_int)  LL_GPIO_SetOutputPin(ChipSelect_GPIO_Port, ChipSelect_Pin);
			  else  LL_GPIO_ResetOutputPin(ChipSelect_GPIO_Port, ChipSelect_Pin);

		break;
		//Function to define the logic level of the Data/ Command line
		case U8X8_MSG_GPIO_DC:
			if (arg_int)  LL_GPIO_SetOutputPin(A0_GPIO_Port, A0_Pin);
			  else  LL_GPIO_ResetOutputPin(A0_GPIO_Port, A0_Pin);

		break;
		//Function to define the logic level of the RESET line
		case U8X8_MSG_GPIO_RESET:
			if (arg_int)  LL_GPIO_SetOutputPin(RST_GPIO_Port, RST_Pin);
			  else  LL_GPIO_ResetOutputPin(RST_GPIO_Port, RST_Pin);

		break;
		default:
			return 0; //A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}

void SystemClock_Config(void);

int main(void)
{

  AHT10_TmpHum_Cmd = 0xAC;
  tmp_hour_var=0;
  tmp_minute_var=0;
  //tmp_work_time_var=0;
  tmp_sec_min_work_tim=0;
  curent_position=0;
  led_lcd_timer=50000;
  flag_clear_line=0;
  flag_frame=1;
  flag_start_stop=0;   //  Write to FLASH
  tmp_zona_x=0;
  is_free_time_flag=0;
  length_of_time_overlay=0;
  tmp_amount_of_setup_day=0;
  flag_rewrite_data=0;
  frame_timer=4000;
  flag_sec_work=1;
  flag_on_off_zona=1;
  flag_sensor_work=1;
  time_mesure_sensor=5;
  flag_write_flash=0;
  day_now=PONEDILOK;

  eWindow=MAIN_W;
  eDAY_NOW=PONEDILOK;
  eHOUR_MINUTE=HOUR;
  eMINUTE_SECUNDE=MINUTE;
  eCalendar=DAY_CALENDAR;



  //whatDayOfNow();



  SystemClock_Config();

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
   LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

   NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);




  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();

  MX_I2C1_Init();
  //MX_RTC_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();


  //MX_RTC_Init();

  LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_5);//RED led

    /* USER CODE BEGIN 2 */


    //LL_USART_TransmitData8(USART3, 'S');  //USART1
    send_str_usart("                 All init. Start ...                  ");
    NewState=0;
    button_flag=0;
    press_button_flag=0;

  LL_mDelay(10);
  //LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);  //Solenoid
  //LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_14);
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6); // On LED_LCD
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);  //Solenoid
  //LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
  //LL_GPIO_ResetOutputPin(PS_GPIO_Port, PS_Pin);

  u8g2_Setup_st7565_nhd_c12832_f( &u8g2, U8G2_R2, u8x8_byte_4wire_sw_spi, u8g2_gpio_and_delay_stm32 );
        	u8g2_InitDisplay(&u8g2);
        	u8g2_SetPowerSave(&u8g2, 0)	;
        	u8g2_SetContrast(&u8g2, 0);

        	u8g2_ClearDisplay(&u8g2);
        	u8g2_SetFont(&u8g2, u8g2_font_cu12_t_cyrillic);
        	eDirection=NOT_MOVE;



LL_mDelay(500);
send_str_usart("               Start RTC init...                 ");

 mRTCInit();

 send_str_usart("                 Set data and counter                 ");


// mRTCSetCounter(colodarSetting(2021,3,18,10,10,0));
 main_Day_Now=unixTime.mday;

 whatDayOfNow();

// send_str_usart("       END Set data and counter      ");


 send_str_usart("                       Send     Init        AHT10                 ");

 aht10_Begin();


 send_str_usart("                       SIZE     OF ::::                 ");

 uint16_t  size_of_mas=0;
/*
 for(uint i=0; i<4; i++)
 {
   //size_of_mas = size_of_mas + sizeof(area_x[i]);
	 for(uint8_t i=0; i<7; i++)
	 {
		 size_of_mas = size_of_mas + sizeof(amount_of_setup_day);
		 for(uint8_t i=0; i<15; i++)
		 {

		 }
	 }
 }
*/

 uint8_t tmp_val;

 size_of_mas = sizeof(area_x[0]);
 //send_8bit_data_usart(digit_mas[size_of_mas/1000]);
 //tmp_val=size_of_mas%1000;
 send_8bit_data_usart(digit_mas[size_of_mas/100]);
 tmp_val=size_of_mas%1000;
 send_8bit_data_usart(digit_mas[tmp_val/10]);
 send_8bit_data_usart(digit_mas[tmp_val%10]);


 //send_8bit_data_usart(digit_mas[sizeof(area_x[0].week_x[0].time_x[0])]);



 send_str_usart("                                       ");
//AHT10_ADC_Raw = (((uint32_t)AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
//AHT10_Temperature = (float)(AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

//send_str_usart("          T==       ");
//send_8bit_data_usart(digit_mas[ (uint8_t) AHT10_Temperature/10]);
//send_8bit_data_usart(digit_mas[ (uint8_t) AHT10_Temperature%10]);



 send_str_usart(                            "Day now:              ");
   send_8bit_data_usart(digit_mas[what_day_now]);
   send_str_usart("                                     ");




 if( flashReadZona_X(ZONA_1) == FLASH_NEW)
 {
	 send_str_usart("                                     ");
	 send_str_usart("     NEW DATA            ");
	 send_str_usart("                                     ");
 }



 __enable_irq ();
  //counterToColodar(mRTCGetCounter(), &unixTime);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  //LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_5);
	 // LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_6);
	  if(eWindow==MAIN_W)
	  	{
		  if(flag_write_flash==1)
		  	 {
		  		 flasWriteZona_X(ZONA_1);
		  		 flag_write_flash=0;
		  	 }
		  mainWindow();
	    }
	  else if(eWindow==ZONA_X)
	  {

		  zonaX();
	  }
	  else if(eWindow==WEEK_SETUP)
	  {
		  weekSetup();
	  }
	  else if(eWindow==AMOUNT_SETUP)
	  {
		  amountSetupOfDay();
	  }
	  else if(eWindow==MESSAGE_X)
	  {
		  messageX();
	  }
	  else if(eWindow==DAY_TIME)
	   {

	  	  timeStartWork();
	   }
	  else if(eWindow==MINUTE_SEC)
	  {
		  if(flag_rewrite_data!=0 &&flag_rewrite_data<=tmp_amount_of_setup_day)
		  				   	{
		  				   		if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min==MINUTE_TYPE)
		  				   		{
		  				   		    //eMINUTE_SECUNDE=MINUTE_TYPE;
		  				   			f_sel_list=0;
		  				   		}
		  				   				   //send_str_usart("Minuta type  ***");
		  				   		else
		  				   		{
		  				   			//eMINUTE_SECUNDE=SECUNDE_TYPE;
		  				   		    f_sel_list=1;
		  				   		}
		  				   				  // send_str_usart("Secunda type  ***");
		  				   	}

		  				   	 else
		  				   		{
		  				   			if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].flag_select_sec_min==MINUTE_TYPE)
		  				   				 {
		  				   				 	 //eMINUTE_SECUNDE=MINUTE_TYPE;
		  				   			       f_sel_list=0;
		  				   				 				   //send_str_usart("Minuta type  ***");
		  				   				 }


		  				   			else
		  				   				{
		  				   				   //eMINUTE_SECUNDE=SECUNDE_TYPE;
		  				   			        f_sel_list=1;
		  				   				 				  // send_str_usart("Secunda type  ***");
		  				   				}
		  				   		}
		  minSecSelect();
	  }
	  else if(eWindow==WORK_TIME)
	  {
		   timeWork();
	  }
	  else if(eWindow==CLEAR_DATA)
	  {
		  clearData();
	  }
	  else if(eWindow==CALENDAR)
	  {
		 setupCalendar();
	  }


  }
  /* USER CODE END 3 */
}


void SystemClock_Config(void)


{

	//FLASH->KEYR = FLASH_KEY1;
	//FLASH->KEYR = FLASH_KEY2;


  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }


  LL_RCC_HSE_Enable();


  	  while(LL_RCC_HSE_IsReady() != 1)
  	  {

  	  }


  	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
  		  LL_RCC_PLL_Enable();


  		  while(LL_RCC_PLL_IsReady() != 1)
  		  {

  		  }
  		  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  		  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  		  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  		  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);


  		  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }





  //-----------------------------------------------------------------



  //------------------------------------------------------------------



  LL_Init1msTick(32000000-1);
  LL_SetSystemCoreClock(32000000);

}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


void mainWindow(void)
{
	 width=5;
	 hight=9;

	  for(;;)
	   {

		  if(eUpdate==UPDATE)
		  	{
		  		u8g2_ClearBuffer(&u8g2);
		  		eUpdate=NO_UPDATE;
		  	}
		 // u8g2_DrawUTF8(&u8g2,1,12,"01.01.21    12:31");

		  u8g2_DrawGlyph(&u8g2,1,hight+4, digit_mas[unixTime.mday/10] );
		  u8g2_DrawGlyph(&u8g2,1+width+1,hight+4,digit_mas[(unixTime.mday%10)]);

		  u8g2_DrawUTF8(&u8g2,width*2+4,hight+4,".");

		  u8g2_DrawGlyph(&u8g2,1+(width*2)+8,hight+4,digit_mas[( unixTime.mon/10 )]);
		  u8g2_DrawGlyph(&u8g2,1+width*3+10,hight+4,digit_mas[( unixTime.mon%10)]);

		  u8g2_DrawUTF8(&u8g2,width*4+13,hight+4,".");

		  u8g2_DrawGlyph(&u8g2,1+(width*4)+16,hight+4,digit_mas[( (unixTime.year-2000)/10 )]);
		  u8g2_DrawGlyph(&u8g2,1+width*5+18,hight+4,digit_mas[( (unixTime.year-2000)%10)]);

		/*

		  if(what_day_now==0)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Сб");
		  else if(what_day_now==1)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Нд");
		  else if(what_day_now==2)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Пн");
		  else if(what_day_now==3)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Вт");
		  else if(what_day_now==4)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Ср");
		  else if(what_day_now==5)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Чт");
		  else if(what_day_now==6)
			  u8g2_DrawUTF8(&u8g2,3+width*6+22,hight+4,"Пт");

    */

		  if(flag_sensor_work==0)
		  {
			  u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
			  u8g2_DrawGlyph(&u8g2, width*12 - 4 ,  hight+4, digit_mas[tempX/10] );
			  u8g2_DrawGlyph(&u8g2, width*13 - 3,  hight+4, digit_mas[tempX%10]);
			  u8g2_DrawUTF8(&u8g2, 3+width*14-1 ,hight+4,"C");
			  u8g2_SetFont(&u8g2, u8g2_font_cu12_t_cyrillic);

		  }

		  u8g2_DrawGlyph(&u8g2,85,hight+4, digit_mas[unixTime.hour/10] );
		  u8g2_DrawGlyph(&u8g2,85+width+1,hight+4,digit_mas[(unixTime.hour%10)]);

		  if(flag_sec_work==1)
		  {
		    u8g2_DrawUTF8(&u8g2,width*2+87,hight+2,":");
		  }

		  u8g2_DrawGlyph(&u8g2,85+(width*2)+9,hight+4,digit_mas[( unixTime.min/10 )]);
		  u8g2_DrawGlyph(&u8g2,96+width*3,hight+4,digit_mas[( unixTime.min%10)]);


		  if(flag_start_stop)
		  {
			  if(next_work_zona==-1)
			  {
				  u8g2_DrawUTF8(&u8g2,1,28,"     Wet Forest");
			  }
			  else if(next_work_zona==0)
			  {
				  u8g2_DrawUTF8(&u8g2,3,29,"Зона 1");


				  u8g2_DrawGlyph(&u8g2,80,hight*2+10, digit_mas[ tmp_next_hour/10] );
				  u8g2_DrawGlyph(&u8g2,80+width+1,hight*2+10,digit_mas[( tmp_next_hour%10)]);

				  //if(flag_sec_work==1)
				  	//{
				  		u8g2_DrawUTF8(&u8g2,width*2+82,hight*2+8,":");
				  	//}

				  u8g2_DrawGlyph(&u8g2,80+(width*2)+9,hight*2+10,digit_mas[( tmp_next_min/10 )]);
				  u8g2_DrawGlyph(&u8g2,91+width*3,hight*2+10,digit_mas[( tmp_next_min%10)]);

			  }
			  else if(next_work_zona==1)
			  {
			  	u8g2_DrawUTF8(&u8g2,1,29,"Зона 2");


			  	u8g2_DrawGlyph(&u8g2,80,hight*2+10, digit_mas[ tmp_next_hour/10] );
			  	u8g2_DrawGlyph(&u8g2,80+width+1,hight*2+10,digit_mas[( tmp_next_hour%10)]);

			  	//if(flag_sec_work==1)
			  		//{
			  			u8g2_DrawUTF8(&u8g2,width*2+82,hight*2+8,":");
			  		//}

			  	u8g2_DrawGlyph(&u8g2,80+(width*2)+9,hight*2+10,digit_mas[( tmp_next_min/10 )]);
			  	u8g2_DrawGlyph(&u8g2,91+width*3,hight*2+10,digit_mas[( tmp_next_min%10)]);

			   }
			  else if(next_work_zona==2)
			  	{
			  		u8g2_DrawUTF8(&u8g2,1,29,"Зона 3");


			  		u8g2_DrawGlyph(&u8g2,80,hight*2+10, digit_mas[ tmp_next_hour/10] );
			  		u8g2_DrawGlyph(&u8g2,80+width+1,hight*2+10,digit_mas[( tmp_next_hour%10)]);

			  			  	//if(flag_sec_work==1)
			  			  		//{
			  		u8g2_DrawUTF8(&u8g2,width*2+82,hight*2+8,":");
			  			  		//}

			  		u8g2_DrawGlyph(&u8g2,80+(width*2)+9,hight*2+10,digit_mas[( tmp_next_min/10 )]);
			  		u8g2_DrawGlyph(&u8g2,91+width*3,hight*2+10,digit_mas[( tmp_next_min%10)]);

			  	}
			  else if(next_work_zona==3)
			  	{
			  		u8g2_DrawUTF8(&u8g2,1,29,"Зона 4");


			  		u8g2_DrawGlyph(&u8g2,80,hight*2+10, digit_mas[ tmp_next_hour/10] );
			  		u8g2_DrawGlyph(&u8g2,80+width+1,hight*2+10,digit_mas[( tmp_next_hour%10)]);

			  			  	//if(flag_sec_work==1)
			  			  		//{
			  		u8g2_DrawUTF8(&u8g2,width*2+82,hight*2+8,":");
			  			  		//}

			  		u8g2_DrawGlyph(&u8g2,80+(width*2)+9,hight*2+10,digit_mas[( tmp_next_min/10 )]);
			  		u8g2_DrawGlyph(&u8g2,91+width*3,hight*2+10,digit_mas[( tmp_next_min%10)]);

			  	}

		    // u8g2_DrawUTF8(&u8g2,1,27,"Зона 1      16:48");
		     //   LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
		  }
		  else
		  {
			  u8g2_DrawUTF8(&u8g2,1,28,"Полив зупинено");
			 //    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);  //Solenoid
		  }


		  u8g2_SendBuffer(&u8g2);


		  if (eWindow!=MAIN_W)
		  	   {
		  		   return;
		  	   }
		  if(flag_on_off_zona==1)
		  {

			  send_str_usart("                       Read FLASH data      ::::   ");
			 uint16_t dd= FLASH_Read(PAGE_ZONA_1_ADDRESS);

			  send_8bit_data_usart(digit_mas[(uint8_t) dd/10]);
			   send_8bit_data_usart(digit_mas[(uint8_t) dd%10]);

			   send_str_usart("                         ");

			if(time_mesure_sensor==0)
			{
			  flag_sensor_work = aht10_Measure(&tempX, &humX);

			  if(flag_sensor_work==-1)
			  {
				// clearBeasyFlagI2C1();

                 aht10_Reset();
                 aht10_Begin();
			  }
			  time_mesure_sensor=2;
			}

			  flag_on_off_zona=0;
			  onOffZonaX(workZonaNow());
		  }
	   }

}

void zonaX(void)
{


    u8g2_UserInterfaceSelectionList(&u8g2,0,curent_position+1, "Календар\n З о н а 1\n З о н а 2\n З о н а 3\n З о н а 4");

}
void weekSetup(void)
{
	if(what_day_now==SUBOTA)
		curent_position=5;

	else if(what_day_now==NEDILYA)
	    curent_position=6;
	else if(what_day_now==PONEDILOK)
		    curent_position=0;
	else if(what_day_now==VIVTOROK)
		    curent_position=1;
	else if(what_day_now==SEREDA)
		    curent_position=2;
	else if(what_day_now==CHETVER)
		    curent_position=3;
	else if(what_day_now==PYATNUTSYA)
		    curent_position=4;


	u8g2_UserInterfaceSelectionList(&u8g2,0,curent_position+1, " Понеділок\n Вівторок\n Середа\n Четвер\n П'ятниця\n Субота\n Неділя\n Парні дні");
}
void timeStartWork(void)
{

 //u8g2_SetFont(&u8g2,u8g2_font_courR14_tn);
 //u8g2_SetFont(&u8g2, u8g2_font_cu12_t_cyrillic);
	if(flag_on_off_zona==1)
			  {
				  flag_on_off_zona=0;
				  onOffZonaX(workZonaNow());
			  }


 u8g2_SetFont(&u8g2,  u8g2_font_courR12_tn);
 width=u8g2_GetMaxCharWidth(&u8g2);
 hight=u8g2_GetMaxCharHeight(&u8g2);
 writeStartTime(( (lcd_wight-((width*4)+11))/2 ) , ( ( (lcd_hight-hight)/2)+hight) +3);


}
void timeWork(void)
{
	if(flag_on_off_zona==1)
			  {
				  flag_on_off_zona=0;
				  onOffZonaX(workZonaNow());
			  }

  u8g2_SetFont(&u8g2,  u8g2_font_courR12_tn);
  width=u8g2_GetMaxCharWidth(&u8g2);
  hight=u8g2_GetMaxCharHeight(&u8g2);
  writeWorkTime(( (lcd_wight-((width*2)+2))/2 ) , ( ( (lcd_hight-hight)/2)+hight) +3);
}
void messageX(void)
{

	if(flag_on_off_zona==1)
			  {
				  flag_on_off_zona=0;
				  onOffZonaX(workZonaNow());
			  }

	//u8g2_UserInterfaceMessage(&u8g2,"Накладення часу!",NULL,NULL, "12хв \n Ok ");
	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
	u8g2_DrawUTF8(&u8g2,6,7,"  Накладення часу!");
	uint8_t weight=u8g2_GetMaxCharWidth(&u8g2);
	volatile uint8_t hour_time;
	volatile uint8_t minute_time;
    convertMinuteToHour(&hour_time, &minute_time, &is_free_time_flag);
    switch(tmp_zona_x)
    {
     case ZONA_1:
       u8g2_DrawUTF8(&u8g2,3,20,"Зона1");
       u8g2_DrawGlyph(&u8g2,weight*6+4,20,digit_mas[( hour_time/10 )]);
       u8g2_DrawGlyph(&u8g2,weight*7+4,20,digit_mas[( hour_time%10 )]);

       u8g2_DrawUTF8(&u8g2,weight*8+4,19,":");

       u8g2_DrawGlyph(&u8g2,weight*8+8,20,digit_mas[( minute_time/10 )]);
       u8g2_DrawGlyph(&u8g2,weight*9+8,20,digit_mas[( minute_time%10 )]);

       u8g2_DrawGlyph(&u8g2,weight*10+20,20,digit_mas[( length_of_time_overlay/10 )]);
       u8g2_DrawGlyph(&u8g2,weight*11+20,20,digit_mas[( length_of_time_overlay%10 )]);
       u8g2_DrawUTF8(&u8g2,weight*12+20,20,"хв");


       u8g2_SendBuffer(&u8g2);
       break;

     case ZONA_2:
       u8g2_DrawUTF8(&u8g2,3,20,"Зона2");
       u8g2_DrawGlyph(&u8g2,weight*6+4,20,digit_mas[( hour_time/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*7+4,20,digit_mas[( hour_time%10 )]);

              u8g2_DrawUTF8(&u8g2,weight*8+4,19,":");

              u8g2_DrawGlyph(&u8g2,weight*8+8,20,digit_mas[( minute_time/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*9+8,20,digit_mas[( minute_time%10 )]);

              u8g2_DrawGlyph(&u8g2,weight*10+20,20,digit_mas[( length_of_time_overlay/10 )]);
                     u8g2_DrawGlyph(&u8g2,weight*11+20,20,digit_mas[( length_of_time_overlay%10 )]);
                     u8g2_DrawUTF8(&u8g2,weight*12+20,20,"хв");
              u8g2_SendBuffer(&u8g2);
      break;

     case ZONA_3:
       u8g2_DrawUTF8(&u8g2,3,20,"Зона3");
       u8g2_DrawGlyph(&u8g2,weight*6+4,20,digit_mas[( hour_time/10 )]);
       u8g2_DrawGlyph(&u8g2,weight*7+4,20,digit_mas[( hour_time%10 )]);

       u8g2_DrawUTF8(&u8g2,weight*8+4,19,":");

       u8g2_DrawGlyph(&u8g2,weight*8+8,20,digit_mas[( minute_time/10 )]);
       u8g2_DrawGlyph(&u8g2,weight*9+8,20,digit_mas[( minute_time%10 )]);

       u8g2_DrawGlyph(&u8g2,weight*10+20,20,digit_mas[( length_of_time_overlay/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*11+20,20,digit_mas[( length_of_time_overlay%10 )]);
              u8g2_DrawUTF8(&u8g2,weight*12+20,20,"хв");
              u8g2_SendBuffer(&u8g2);
       break;

      case ZONA_4:
       u8g2_DrawUTF8(&u8g2,3,20,"Зона4");
       u8g2_DrawGlyph(&u8g2,weight*6+4,20,digit_mas[( hour_time/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*7+4,20,digit_mas[( hour_time%10 )]);

              u8g2_DrawUTF8(&u8g2,weight*8+4,19," : ");



              u8g2_DrawGlyph(&u8g2,weight*8+8,20,digit_mas[( minute_time/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*9+8,20,digit_mas[( minute_time%10 )]);

              u8g2_DrawGlyph(&u8g2,weight*10+20,20,digit_mas[( length_of_time_overlay/10 )]);
              u8g2_DrawGlyph(&u8g2,weight*11+20,20,digit_mas[( length_of_time_overlay%10 )]);
              u8g2_DrawUTF8(&u8g2,weight*12+20,20,"хв");
              u8g2_SendBuffer(&u8g2);
       break;
    }
	while(eWindow==MESSAGE_X)
	{

		if(flag_on_off_zona==1)
				  {
					  flag_on_off_zona=0;
					  onOffZonaX(workZonaNow());
				  }
	}

}
void writeStartTime( int8_t x, int8_t y)
{
	if(flag_on_off_zona==1)
			  {
				  flag_on_off_zona=0;
				  onOffZonaX(workZonaNow());
			  }

	eUpdate=UPDATE;

	u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    send_str_usart("               ** Write start time. ZONA:        ");
    send_8bit_data_usart(digit_mas[ (eZONA_SETUP)]);
    send_str_usart("                      ");

    send_str_usart("               ** eDAY_NOW        ");
        send_8bit_data_usart(digit_mas[ (eDAY_NOW)]);
        send_str_usart("                      ");

    if(flag_rewrite_data!=0 &&flag_rewrite_data<=tmp_amount_of_setup_day)
    		      {
    		      	tmp_hour_var=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].start_work_hour;
    		      	tmp_minute_var=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].start_work_minute;
    		      }
    else
    {
      tmp_hour_var=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].start_work_hour;
      tmp_minute_var=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].start_work_minute;
    }
	do
	{

	  if(eUpdate==UPDATE)
	   {
		  //ds1307_GetTime();


		   u8g2_ClearBuffer(&u8g2);
		   u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
		   u8g2_DrawUTF8(&u8g2,6,7,"Час початку поливу");


		   u8g2_SetFont(&u8g2,  u8g2_font_courR10_tn);


		   //if(curent_position!=0&&curent_position<=tmp_amount_of_setup_day)
		   //tmp_hour_var=

		   u8g2_DrawGlyph(&u8g2,x,y,digit_mas[( tmp_hour_var/10 )]);
		   u8g2_DrawGlyph(&u8g2,x+width+2,y,digit_mas[( tmp_hour_var%10)]);

		   u8g2_DrawGlyph(&u8g2,x+width*2+11,y,digit_mas[( tmp_minute_var/10 )]);
		   u8g2_DrawGlyph(&u8g2,x+width*3+13,y,digit_mas[( tmp_minute_var%10)]);

		   u8g2_DrawStr(&u8g2,x+width*2+2,y-2,":");
		switch(eHOUR_MINUTE)
		{
		   case HOUR:

			   if(flag_frame)
			    u8g2_DrawFrame(&u8g2, x-1, y-hight+1, width*2+4, hight+2);
		     break;

		   case MINUTE:

			   if(flag_frame)
			    u8g2_DrawFrame(&u8g2, x+width*2+10, y-hight+1, width*2+4, hight+2);

			  break;
		}

		u8g2_SendBuffer(&u8g2);
		//u8g2_UpdateDisplayArea(&u8g2, 12, 2, 3, 2);
		eUpdate=NO_UPDATE;
	  }

	  if(flag_on_off_zona==1)
	  		  {
	  			  flag_on_off_zona=0;
	  			  onOffZonaX(workZonaNow());
	  		  }

	}while(eWindow==DAY_TIME);// end while

	//tmp_hour_var=*hour;
	//tmp_minute_var=*minute;


	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_cu12_t_cyrillic);
		eUpdate=UPDATE;
}
void minSecSelect(void)
{

	if(flag_rewrite_data!=0 &&flag_rewrite_data<=tmp_amount_of_setup_day)
	    	{
	    	   if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min==1)
	    		//if(eMINUTE_SECUNDE==SECUNDE_TYPE)
	    		  curent_position=1;
	    	   else
	    		   curent_position=0;
	    	}
	 else
	    	{
	    		//if(eMINUTE_SECUNDE==SECUNDE_TYPE)
		 if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min==1)
		          curent_position=1;
	    		else
	    		{
	    			curent_position=0;
	    		}
	    	}
	//curent_position=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].time_type;
	//send_str_usart("in Selec function ");
	//if(curent_position)
	 // {
		//send_str_usart("-secunda ");
		//++curent_position;
	 // }
	//else
		//send_str_usart("minuta *** ");

	u8g2_UserInterfaceSelectionList(&u8g2,0,curent_position+1, " Хвилини\n Секунди");
	//area_x[eZONA_SETUP].week_x[eDAY_NOW].time_type=curent_position;
	curent_position=0;
}

void writeWorkTime(int8_t x, int8_t y)
{

	eUpdate=UPDATE;

	u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);

    	if(flag_rewrite_data!=0 &&flag_rewrite_data<=tmp_amount_of_setup_day)
    	{
    	   //if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].time_type==SECUNDE_TYPE)
    		//if(eMINUTE_SECUNDE==SECUNDE_TYPE)
    		if(f_sel_list==1)
    		  tmp_sec_min_work_tim=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].work_time_sec;
    	   else
    		   tmp_sec_min_work_tim=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].work_time_min;
    	}
    	else
    	{
    		//if(eMINUTE_SECUNDE==SECUNDE_TYPE)
    		if(f_sel_list==1)
    	      tmp_sec_min_work_tim=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].work_time_sec;
    		else
    		{
    			tmp_sec_min_work_tim=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].work_time_min;
    		}
    	}


	do
	{

	  if(eUpdate==UPDATE)
	   {
		   u8g2_ClearBuffer(&u8g2);
		   u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
		   u8g2_DrawUTF8(&u8g2,12,7,"Тривалість поливу");


		   u8g2_SetFont(&u8g2,  u8g2_font_courR10_tn);


		   u8g2_DrawGlyph(&u8g2,x,y,digit_mas[( tmp_sec_min_work_tim/10 )]);
		   u8g2_DrawGlyph(&u8g2,x+width+2,y,digit_mas[( tmp_sec_min_work_tim%10)]);


		   if(flag_rewrite_data!=0 &&flag_rewrite_data<=tmp_amount_of_setup_day)
		     {
		        //if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].time_type==SECUNDE_TYPE)
			   if(f_sel_list==1)
		        {
		        	u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
		        	u8g2_DrawUTF8(&u8g2,x+width*2+3,y,"сек");
		        }
		        else
		        {
		        	u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
		        	u8g2_DrawUTF8(&u8g2,x+width*2+3,y,"хв");
		        }
		     }
		   else
		   {
			   //if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].time_type==SECUNDE_TYPE)
			   if(f_sel_list==1)
			   {
				   u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
				   u8g2_DrawUTF8(&u8g2,x+width*2+3,y,"сек");
			   }
			   else
			   {
				   u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
				   u8g2_DrawUTF8(&u8g2,x+width*2+3,y,"хв");
			   }
		   }


		   u8g2_SendBuffer(&u8g2);
		   //u8g2_UpdateDisplayArea(&u8g2, 12, 2, 3, 2);
		   eUpdate=NO_UPDATE;
	    }

	  if(flag_on_off_zona==1)
	  		  {
	  			  flag_on_off_zona=0;
	  			  onOffZonaX(workZonaNow());
	  		  }

	}while(eWindow==WORK_TIME);// end while






	//if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].time_type==SECUNDE_TYPE)
	if(f_sel_list==1)
	{
      is_free_time_flag=isFreeTime(tmp_hour_var, tmp_minute_var, 1);
	}
	else
	{
		is_free_time_flag=isFreeTime(tmp_hour_var,tmp_minute_var, tmp_sec_min_work_tim);
	}



	if(is_free_time_flag!=0)
	{
		eWindow=MESSAGE_X;
		flag_write_flash=0;
		messageX();
	}




   if(is_free_time_flag==0)
   {
	  send_str_usart("     WriteWorkTime. flag_write_data==");
	  send_8bit_data_usart(digit_mas[flag_rewrite_data]);
	  send_str_usart("     tmp_amount_of_setup_day==");
	  send_8bit_data_usart(digit_mas[tmp_amount_of_setup_day]);

	 if(flag_rewrite_data<=tmp_amount_of_setup_day&&flag_rewrite_data!=0)
	 {
		 area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].start_work_hour=tmp_hour_var;
		 area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].start_work_minute=tmp_minute_var;

		 if(f_sel_list==1)
		 {
		       area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].work_time_sec=tmp_sec_min_work_tim;
		       area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min=1;
		       send_str_usart("       Remove old data      ");
		       send_8bit_data_usart(digit_mas[area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min]);
		 }
		 else
		 {
		      area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].work_time_min=tmp_sec_min_work_tim;
		      area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min=0;
		      send_str_usart("       Remove old data      ");
	          send_8bit_data_usart(digit_mas[area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[flag_rewrite_data-1].flag_select_sec_min]);

		 }



		 send_str_usart("                                        Write day is ->>>>>>>      ");
		 send_8bit_data_usart(digit_mas[eDAY_NOW]);
		 send_str_usart("                                  ");
	  }


	 else
	 {
	  area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].start_work_hour=tmp_hour_var;
      area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].start_work_minute=tmp_minute_var;


          		//if(area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].time_type==SECUNDE_TYPE)
    	        if(f_sel_list==1)
    	        {
          	      area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].work_time_sec=tmp_sec_min_work_tim;
          	      area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].flag_select_sec_min=1;

          	    send_str_usart("       New data      ");
          	    send_8bit_data_usart(digit_mas[area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].flag_select_sec_min]);

    	        }
          	      else
          		{
          			area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].work_time_min=tmp_sec_min_work_tim;
          			area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].flag_select_sec_min=0;
          			send_str_usart("       New data      ");
          			send_8bit_data_usart(digit_mas[area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[tmp_amount_of_setup_day].flag_select_sec_min]);
          		}

    	        send_str_usart("                                   Write day is ->>>>>>>      ");
    	        send_8bit_data_usart(digit_mas[eDAY_NOW]);
    	        send_str_usart("                              ");

        tmp_amount_of_setup_day++;
        area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day=tmp_amount_of_setup_day;
        tmp_amount_of_setup_day=0;
	 }






   }

	u8g2_ClearBuffer(&u8g2);
	u8g2_SetFont(&u8g2, u8g2_font_cu12_t_cyrillic);
		eUpdate=UPDATE;
}
uint16_t isFreeTime(uint8_t hour, uint8_t min,  uint8_t work_time)
{

	uint16_t start_time;
	uint16_t stop_time;
	uint16_t hour_min_tmp=(hour*60)+min;
	uint16_t tmp_val;

    for(uint8_t i=0;i<4;i++)
    {
      for(uint8_t i_i=0; i_i<area_x[i].week_x[eDAY_NOW].amount_of_setup_day+1;i_i++)
      {
    	  //send_str_usart("   ** i_i==");
    	  //send_8bit_data_usart(digit_mas[i_i]);
    	  //send_str_usart("  **flag_rewrite_data==");
    	  //send_8bit_data_usart(digit_mas[flag_rewrite_data]);
    	 // send_str_usart("     ");
    	  send_str_usart("            eZONA_SETUP    ");
    	      		  send_8bit_data_usart(digit_mas[eZONA_SETUP]);
    	      		send_str_usart("              ");
    	  if(i_i==flag_rewrite_data-1 && eZONA_SETUP==i)
    	  {
    		  //send_str_usart("            eZONA_SETUP    ");
    		  //send_8bit_data_usart(digit_mas[eZONA_SETUP]);
    		  send_str_usart("            Continue.....        ");
    		  continue;
    	  }

    	start_time=area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_hour*60
    			+area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_minute;

    	send_str_usart("              Start time==  ");
    	send_8bit_data_usart(digit_mas[start_time/1000]);
    	tmp_val=start_time%1000;
    	send_8bit_data_usart(digit_mas[tmp_val/100]);
    	tmp_val=tmp_val%100;
    	send_8bit_data_usart(digit_mas[tmp_val/10]);
    	send_8bit_data_usart(digit_mas[tmp_val%10]);




    	  send_str_usart("                               ** i_i==  ");
    	      	 send_8bit_data_usart(digit_mas[i_i]);
    	      	 send_str_usart("       **i==    ");
    	      	 send_8bit_data_usart(digit_mas[i]);
    	      	 send_str_usart("     ");


    	  if(area_x[i].week_x[eDAY_NOW].time_x[i_i].flag_select_sec_min==MINUTE_TYPE)
    	  {
    		  send_str_usart("                      MINUTE   TYPE                 ");
    		stop_time=area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_hour*60
    				+ area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_minute
					+ area_x[i].week_x[eDAY_NOW].time_x[i_i].work_time_min;
    	  }
    	  else
    	  {
    		  send_str_usart("                      SECOND   TYPE                 ");
    		  stop_time=area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_hour*60
    		      				+ area_x[i].week_x[eDAY_NOW].time_x[i_i].start_work_minute
    		  					+ 1;
    	  }

    	//uint16_t i_for=area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_hour*60 + area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_minute;
    	uint16_t i_for=(hour*60)+min;

    	for( ;i_for< ((hour*60)+min+work_time); i_for++ )
    	{
    		//if(i_for>=area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_hour*60 + area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_minute && i_for<=area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_hour*60  +  area_x[eZONA_SETUP].week_x[eDAY_NOW].start_work_minute + area_x[eZONA_SETUP].week_x[eDAY_NOW].work_time)
    		if(i_for>=start_time && i_for<stop_time)
    		{
    			tmp_zona_x=i;

    			if(hour_min_tmp<start_time)
    			{
    				length_of_time_overlay=(hour_min_tmp+work_time)-start_time; // lenght of
    			}
    			else
    			{
    				length_of_time_overlay=stop_time-hour_min_tmp;
    			}

    			return i_for;
    		}
    	}




      }
    }

    return 0;
}

void amountSetupOfDay(void)
{
	curent_position=0;

	tmp_amount_of_setup_day=area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day;

	convertAmountSetupToStr();
	send_str_usart("**");
	uint8_t cnt=0;
	while(tmp_str[cnt])
	{
	  send_8bit_data_usart(tmp_str[cnt]);
	  cnt++;
	}


    u8g2_UserInterfaceSelectionList(&u8g2,0,curent_position, tmp_str );

}

void convertAmountSetupToStr(void)
{
	for(uint8_t i=0; i<200; i++)
	{
		tmp_str[i]='\0';
	}
  uint8_t hour=0, min=0;
  if(tmp_amount_of_setup_day==0)
  {

	  char *str = "Додавання часу";
	  strcpy(tmp_str,str);
  }

  else
  {
	  uint16_t c;
	  char *str="Скидання часу";
	  uint8_t len = strlen(str);
	  char *str2 = malloc(200); /* one for extra char, one for trailing zero */
	  for(uint8_t i=0; i<200; i++)
	  	{
	  		str2[i]='\0';
	  	}
	  strcpy(str2, str);

    for(uint8_t i=0;i<tmp_amount_of_setup_day; i++)
    {
    	hour=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].start_work_hour;
        min=area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].start_work_minute;
        //len++;
        str2[len] = '\n';

        c=digit_mas[i+1];
        len++;
        str2[len] = c;

        c=' ';
        len++;
        str2[len] = c;

        c='-';
        len++;
        str2[len] = c;

        c=' ';
        len++;
        str2[len] = c;

        c=digit_mas[hour/10];
        len++;
        str2[len] = c;

        c=digit_mas[hour%10];
        len++;
        str2[len] = c;

        len++;
        str2[len] = ':';

        c=digit_mas[min/10];
        len++;
        str2[len] = c;

        c=digit_mas[min%10];
        len++;
        str2[len] = c;

        len++;
    }

    str2[len] = '\n';

    char *str3="Додавання часу";

    strcat(str2 ,str3);
    str2[strlen(str2)] = '\0';
    strcpy(tmp_str,str2);


    free( str2 );


  }



}

void convertMinuteToHour(volatile uint8_t *hour, volatile uint8_t *min, volatile uint16_t *time)
{
  *hour=*time/60;
  *min=*time-(*hour*60);
}

void clearData(void)
{
	uint8_t amount=area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day;
	for(uint8_t i=0; i<amount; i++)
	{
		area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].start_work_hour=0;
		area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].start_work_minute=0;
		//area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].time_type=0;
		area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].work_time_min=0;
		area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].work_time_sec=0;
		area_x[eZONA_SETUP].week_x[eDAY_NOW].time_x[i].flag_select_sec_min=0;
	}
	area_x[eZONA_SETUP].week_x[eDAY_NOW].amount_of_setup_day=0;
	eWindow=AMOUNT_SETUP;
}

void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  //LL_mDelay(100);
  //LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_I2C1);
  //LL_mDelay(100);
  //LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);





  /**I2C1 GPIO Configuration
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);




  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);

  I2C1->CR1 |= I2C_CR1_PE;

}

void I2C_Write(uint8_t addres,  uint8_t reg, uint8_t data)
{
	 uint32_t d;
/*
	 data=LL_I2C_GetPeriphClock (I2C1);
	 send_str_usart("      I2C peripherals closk - >");
	 send_8bit_data_usart(digit_mas[data/100]);
	 data=data%100;
	 send_8bit_data_usart(digit_mas[data/10]);
	 send_8bit_data_usart(digit_mas[data%10]);

	 data=LL_I2C_GetRiseTime (I2C1);
	 send_str_usart("      I2C Rise Time - >");
	 send_8bit_data_usart(digit_mas[data/100]);
	 data=data%100;
	 send_8bit_data_usart(digit_mas[data/10]);
	 send_8bit_data_usart(digit_mas[data%10]);

	 data=LL_I2C_GetClockPeriod (I2C1);
	 send_str_usart("      I2C Closk period - >");
	 send_8bit_data_usart(digit_mas[data/100]);
	 data=data%100;
	 send_8bit_data_usart(digit_mas[data/10]);
	 send_8bit_data_usart(digit_mas[data%10]);


*/
        //стартуем


	 d= LL_I2C_IsEnabled (I2C1);
	 if(d)
	    {
	 	   send_str_usart("      I2C peripherals is enabled    ");
	    }
	    else
	    {
	 	   send_str_usart("    I2C peripherals is disabled     ");
	    }
	send_str_usart("      Start ...    ");
        I2C1->CR1 |= I2C_CR1_START;

        uint16_t i=5000;

	while(!(I2C1->SR1 & I2C_SR1_SB))
	{
		if(i==0)
			return ;

		--i;
	}
	   (void) I2C1->SR1;

	   d=LL_I2C_IsActiveFlag_MSL ( I2C1);
	   if(d)
	     {
	        send_str_usart("      Master mode    ");
	     }
	   else
	     {
	       send_str_usart("    Slave mode    ");
	     }

	send_str_usart("      Start is ok    ");

        //передаем адрес устройства
	I2C1->DR =addres;  /// D0

	i=500;

	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		if(i==0)
			return;
		--i;
	}
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	send_str_usart("      Adres send    ");
        //передаем адрес регистра
	//I2C1->DR = reg;
	//while(!(I2C1->SR1 & I2C_SR1_TXE)){};
	//send_str_usart("      Register send    ");
        //пишем данные


	//             end adress



	//data= 0x33;
	I2C1->DR = 0x33;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		if(i==0)
			return;
		--i;
	}
	send_str_usart("      Byte flag finished    ");



	//data=0x00;
	I2C1->DR = 0x00;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		if(i==0)
			return;
		--i;
	}
	send_str_usart("      Byte flag finished    ");




	I2C1->DR = data;

		i=500;

		while(!(I2C1->SR1 & I2C_SR1_BTF))
		{
			if(i==0)
				return;
			--i;
		}
		send_str_usart("      Byte flag finished    ");

	I2C1->CR1 |= I2C_CR1_STOP;

	//I2C_CR1_SMBUS
}


void I2C_WriteReset(void)
{
	send_str_usart("      Start ...    ");
	        I2C1->CR1 |= I2C_CR1_START;

	        uint16_t i=5000;

		while(!(I2C1->SR1 & I2C_SR1_SB))
		{
			if(i==0)
				return ;

			--i;
		}
		   (void) I2C1->SR1;

		   send_str_usart("      Start is ok    ");

		           //передаем адрес устройства
		   	I2C1->DR = 0x70;  /// D0

		   	i=500;

		   	while(!(I2C1->SR1 & I2C_SR1_ADDR))
		   	{
		   		if(i==0)
		   			return;
		   		--i;
		   	}
		   	(void) I2C1->SR1;
		   	(void) I2C1->SR2;
		   	send_str_usart("      Adres send    ");




		   	I2C1->DR = 0xBA;

		   			i=500;

		   			while(!(I2C1->SR1 & I2C_SR1_BTF))
		   			{
		   				if(i==0)
		   					return;
		   				--i;
		   			}
		   			send_str_usart("      Byte flag finished    ");

		   		I2C1->CR1 |= I2C_CR1_STOP;


}


int8_t I2C_Read(uint8_t adres, uint8_t reg)
{
	uint8_t data=0;
	uint16_t i=5000;
	//стартуем
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB))
	{
	   if(i==0)
		   return -1;
	   --i;
	}
	send_str_usart("      Start is OK   ");
	(void) I2C1->SR1;

	//передаем адрес устройства
	I2C1->DR = adres;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		if(i==0)
			return -1;
		--i;
	}
	send_str_usart("      Adres send finished   ");
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	//передаем адрес регистра
	//I2C1->DR = reg;
	//while(!(I2C1->SR1 & I2C_SR1_TXE)){};
	//send_str_usart("      Register send finished   ");
	//I2C1->CR1 |= I2C_CR1_STOP;

	//рестарт!!!
	//I2C1->CR1 |= I2C_CR1_START;
	//while(!(I2C1->SR1 & I2C_SR1_SB)){};
	//send_str_usart("      Restart is OK   ");
	//(void) I2C1->SR1;

	//передаем адрес устройства, но теперь для чтения
	//I2C1->DR = adres;
	//while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	//send_str_usart("      Adres send OK   ");
	//(void) I2C1->SR1;
	//(void) I2C1->SR2;

	//читаем
	I2C1->CR1 &= ~I2C_CR1_ACK;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_RXNE))
	{
		if(i==0)
			return -1;
		--i;
	}
	//send_str_usart("      Read data finished   ");
	data = I2C1->DR;

	send_str_usart("              Read data finished   ");

	send_8bit_data_usart(digit_mas[ data/10 ]);
	send_8bit_data_usart(digit_mas[ data%10 ]);

	I2C1->CR1 |= I2C_CR1_STOP;
	send_str_usart("      STOP   ");

	return data;
}

void I2C_Read_Mas(uint8_t adres, uint8_t * mas)
{
	send_str_usart("      READ  FUNCTION      ");

	uint16_t id=5000;

	    I2C1->CR1 |= I2C_CR1_START;
		while(!(I2C1->SR1 & I2C_SR1_SB))
		{
			if(id==0)
				return;
			--id;
		}
		send_str_usart("      Start  OK   ");
		(void) I2C1->SR1;

		//передаем адрес устройства
		I2C1->DR = adres;

		id=5000;

		while(!(I2C1->SR1 & I2C_SR1_ADDR))
		{
			if(id==0)
				return;
			--id;
		}
		send_str_usart("      Adres send finished   ");
		(void) I2C1->SR1;
		(void) I2C1->SR2;


		for(uint8_t i=0; i<6; i++)
		{
			id=5000;
			while(!(I2C1->SR1 & I2C_SR1_RXNE))
			{
				if(id==0)
					return;
				--id;
			}
			send_str_usart("      Read data finished   ");
			mas[i] = I2C1->DR;

			send_str_usart("          MASi ==  ");
			send_8bit_data_usart(digit_mas[ i ]);
			send_str_usart("            ");

			send_8bit_data_usart(digit_mas[ mas[i]/10 ]);
			send_8bit_data_usart(digit_mas[ mas[i]%10 ]);

			I2C1->CR1 |= I2C_CR1_ACK; // ASK
		}
		I2C1->CR1 &= ~I2C_CR1_ACK;
		I2C1->CR1 |= I2C_CR1_STOP;
		send_str_usart("                    STOP                       ");

}

uint8_t ds1307_decToBcd(uint8_t val) {
    return ((val / 10 * 16) + (val % 10));
}

uint8_t ds1307_bcdToDec(uint8_t val) {
    return ((val / 16 * 10) + (val % 16));
}

void ds1307_GetTime(void)
{

    second     = ds1307_bcdToDec(I2C_Read(ds1307_read, 0x00) & 0x7f);
    minute     = ds1307_bcdToDec(I2C_Read(ds1307_read, 0x01));
    hour       = ds1307_bcdToDec(I2C_Read(ds1307_read, 0x02) & 0x3f);// Need to change this if 12 hour am/pm
    //dayOfWeek  = bcdToDec(Wire.read());
    //dayOfMonth = bcdToDec(Wire.read());
    //month      = bcdToDec(Wire.read());
    //year       = bcdToDec(Wire.read());

    //ds1307_GetTime();
    		send_str_usart("     Second is: ");
    		send_8bit_data_usart(digit_mas[second/10]);
    		send_8bit_data_usart(digit_mas[second%10]);

    		send_str_usart("     Minute is: ");
    		send_8bit_data_usart(digit_mas[minute/10]);
    		send_8bit_data_usart(digit_mas[minute%10]);

    		send_str_usart("     Hour is: ");
    			send_8bit_data_usart(digit_mas[hour/10]);
    			send_8bit_data_usart(digit_mas[hour%10]);
    			   NewState=read_gray_code_from_encoder();
}

int8_t workZonaNow(void)
{
	uint16_t start_time=0;
	uint16_t stop_time;
	uint16_t time_now=(unixTime.hour*60)+unixTime.min;
	uint16_t tmp_val=0;

	next_work_zona=-1;//



	if(flag_start_stop)
	{
		/*

		send_str_usart("               Day Now     ");
		send_8bit_data_usart(digit_mas[what_day_now]);

		send_str_usart("                 ||   ");
		*/
		for(uint8_t i=0;i<4;i++)
			   {
			      for(uint8_t i_i=0; i_i<area_x[i].week_x[what_day_now].amount_of_setup_day+1;i_i++)
			      {
			    	  next_work_time=(area_x[i].week_x[what_day_now].time_x[i_i].start_work_hour*60)
			    	  	    			+area_x[i].week_x[what_day_now].time_x[i_i].start_work_minute;

			    	  /*
			    	  send_str_usart("         Next_work_time  ");
			    	  send_8bit_data_usart(digit_mas[next_work_time/1000]);
			    	  tmp_val=time_now%1000;
			    	  send_8bit_data_usart(digit_mas[next_work_time/100]);
			    	  tmp_val=tmp_val%100;
			    	  send_8bit_data_usart(digit_mas[next_work_time/10]);
			    	  send_8bit_data_usart(digit_mas[next_work_time%10]);
			    	  */

                      if(next_work_time>time_now )
                      {
                    	  if(start_time<time_now)
                    	  {
                    		   start_time=next_work_time;

/*
                    		   send_str_usart("         Start time==  ");
                    		   send_8bit_data_usart(digit_mas[start_time/1000]);
                    		   tmp_val=time_now%1000;
                    		   send_8bit_data_usart(digit_mas[start_time/100]);
                    		   tmp_val=tmp_val%100;
                    		   send_8bit_data_usart(digit_mas[start_time/10]);
                    		   send_8bit_data_usart(digit_mas[start_time%10]);
*/

                    		   next_work_zona=i;

                    		  // send_str_usart("         Next work zona==  ");
                    		  //send_8bit_data_usart(digit_mas[next_work_zona]);
                    	  }
                    	  else if(start_time>next_work_time)
                    	  {
                    		  start_time=next_work_time;

                    		  /*
                    		  send_str_usart("         Start time==  ");
                    		  send_8bit_data_usart(digit_mas[start_time/1000]);
                    		  tmp_val=time_now%1000;
                    		  send_8bit_data_usart(digit_mas[start_time/100]);
                    		  tmp_val=tmp_val%100;
                    		  send_8bit_data_usart(digit_mas[start_time/10]);
                    		  send_8bit_data_usart(digit_mas[start_time%10]);

                              */

                    		  next_work_zona=i;

                    		  //send_str_usart("         Next work zona==  ");
                    		  //send_8bit_data_usart(digit_mas[next_work_zona]);

                    	  }
                      }



			      }
	            }


		if(next_work_zona==-1)
		{
			tmp_next_hour=0;
			tmp_next_min=0;
			send_str_usart("         No next work zona  ");
		}
		else
		{
			tmp_next_hour=start_time/60;
			tmp_next_min=start_time-(tmp_next_hour*60);


			send_str_usart("         Start time==  ");
			send_8bit_data_usart(digit_mas[start_time/1000]);
			tmp_val=start_time%1000;
			send_8bit_data_usart(digit_mas[tmp_val/100]);
			tmp_val=tmp_val%100;
			send_8bit_data_usart(digit_mas[tmp_val/10]);
			send_8bit_data_usart(digit_mas[tmp_val%10]);

			send_str_usart("         Next work zona==  ");
			send_8bit_data_usart(digit_mas[next_work_zona]);
		}


		/*
		send_str_usart("         ||                   ");
  // what day now //


		send_str_usart("            time now  ");
			    send_8bit_data_usart(digit_mas[time_now/1000]);
			    tmp_val=time_now%1000;
			    send_8bit_data_usart(digit_mas[tmp_val/100]);
			    tmp_val=tmp_val%100;
			    send_8bit_data_usart(digit_mas[tmp_val/10]);
			    send_8bit_data_usart(digit_mas[tmp_val%10]);


*/


	    for(uint8_t i=0;i<4;i++)
	    {
	      for(uint8_t i_i=0; i_i<area_x[i].week_x[what_day_now].amount_of_setup_day+1;i_i++)
	      {

	    	start_time=(area_x[i].week_x[what_day_now].time_x[i_i].start_work_hour*60)
	    			+area_x[i].week_x[what_day_now].time_x[i_i].start_work_minute;

	    	if(area_x[i].week_x[what_day_now].time_x[i_i].flag_select_sec_min==SECUNDE_TYPE)
	    	{
	    		stop_time=area_x[i].week_x[what_day_now].time_x[i_i].start_work_hour*60
	    				+ area_x[i].week_x[what_day_now].time_x[i_i].start_work_minute + 1; // If Sec, +1 to minute.
	    	}
	    	else
	    	{
	    		stop_time=(area_x[i].week_x[what_day_now].time_x[i_i].start_work_hour*60)
	    				+ area_x[i].week_x[what_day_now].time_x[i_i].start_work_minute
						+ area_x[i].week_x[what_day_now].time_x[i_i].work_time_min;
	    	}



/*


	    	send_str_usart("        i ==  ");
	    	send_8bit_data_usart(digit_mas[i]);
	    	send_str_usart("        i_i ==  ");
	    	send_8bit_data_usart(digit_mas[i_i]);

	    	send_str_usart("        start time  ");



	    	send_8bit_data_usart(digit_mas[start_time/1000]);
	    	tmp_val=start_time%1000;
	    	send_8bit_data_usart(digit_mas[tmp_val/100]);
	    	tmp_val=start_time%100;
	    	send_8bit_data_usart(digit_mas[tmp_val/10]);
	    	send_8bit_data_usart(digit_mas[tmp_val%10]);



	    	send_str_usart("        stop time  ");
	    	send_8bit_data_usart(digit_mas[stop_time/1000]);
	    	tmp_val=stop_time%1000;
	    	send_8bit_data_usart(digit_mas[tmp_val/100]);
	    	tmp_val=stop_time%100;
	    	send_8bit_data_usart(digit_mas[tmp_val/10]);
	    	send_8bit_data_usart(digit_mas[tmp_val%10]);


*/
	    	if(time_now>=start_time && time_now<stop_time)
	    	{

	    		if(area_x[i].week_x[what_day_now].time_x[i_i].flag_select_sec_min==SECUNDE_TYPE)
	    		{
	    			if( unixTime.sec >= 1  &&  unixTime.sec <= area_x[i].week_x[what_day_now].time_x[i_i].work_time_sec )
	    				return i;
	    			else
	    				return -1;

	    		}

	    		else
	    		{
	    		 return i;
	    		}

	    	}


	      }


	    }

     return -1;
	}
	else
		return -1;

}

void setupCalendar(void)
{


	tmp_year_calendar=unixTime.year - 2000;
	tmp_mon_calendar=unixTime.mon;
	tmp_day_calendar=unixTime.mday;
	tmp_hour_calendar=unixTime.hour;
	tmp_min_calendar=unixTime.min;

	width=5;
    hight=17;

		  for(;;)
		   {
			  if(flag_on_off_zona==1)
			  		  {
			  			  flag_on_off_zona=0;
			  			  onOffZonaX(workZonaNow());
			  		  }

			  if(eUpdate==UPDATE)
			  	{
			  		u8g2_ClearBuffer(&u8g2);
			  		eUpdate=NO_UPDATE;
			  	}
			 // u8g2_DrawUTF8(&u8g2,1,12,"01.01.21    12:31");
			  if(eCalendar==DAY_CALENDAR)
			  {
				  if(flag_frame)

				    u8g2_DrawFrame(&u8g2, 0, hight-11, width*2+7, hight+2);

			  }
			  else if(eCalendar==MON_CALENDAR)
			  {
				  if(flag_frame)
				  u8g2_DrawFrame(&u8g2, width*2+11, hight-11, width*2+7, hight+2);
			  }
			  else if(eCalendar==YEAR_CALENDAR)
			  {
				  if(flag_frame)
					  u8g2_DrawFrame(&u8g2, width*6+13, hight-11, width*2+8, hight+2);
			  }
			  else if(eCalendar==HOUR_CALENDAR)
			  {
				  if(flag_frame)
					  u8g2_DrawFrame(&u8g2, 84, hight-11, width*2+7, hight+2);
			  }
			  else if(eCalendar==MIN_CALENDAR)
			  {
				  if(flag_frame)
					  u8g2_DrawFrame(&u8g2, 85+(width*2)+11, hight-11, width*2+7, hight+2);
			  }

			  u8g2_DrawGlyph(&u8g2,1,hight+4, digit_mas[tmp_day_calendar/10] );
			  u8g2_DrawGlyph(&u8g2,1+width+2,hight+4,digit_mas[(tmp_day_calendar%10)]);

			  u8g2_DrawUTF8(&u8g2,width*2+7,hight+5,".");

			  u8g2_DrawGlyph(&u8g2,1+(width*2)+12,hight+4,digit_mas[( tmp_mon_calendar/10 )]);
			  u8g2_DrawGlyph(&u8g2,1+width*3+14,hight+4,digit_mas[( tmp_mon_calendar%10)]);

			  u8g2_DrawUTF8(&u8g2,width*4+19,hight+4,".");

			  u8g2_DrawGlyph(&u8g2,(width*4)+25,hight+4,digit_mas[( (tmp_year_calendar)/10 )]);
			  u8g2_DrawGlyph(&u8g2,width*5+28,hight+4,digit_mas[( (tmp_year_calendar)%10)]);






			  u8g2_DrawGlyph(&u8g2,85,hight+4, digit_mas[tmp_hour_calendar/10] );
			  u8g2_DrawGlyph(&u8g2,85+width+3,hight+4,digit_mas[(tmp_hour_calendar%10)]);

			 // if(flag_sec_work==1)


			 // {
			    u8g2_DrawUTF8(&u8g2,width*2+89,hight+2,":");
			 // }

			  u8g2_DrawGlyph(&u8g2,85+(width*2)+12,hight+4,digit_mas[( tmp_min_calendar/10 )]);
			  u8g2_DrawGlyph(&u8g2,96+width*3+3,hight+4,digit_mas[( tmp_min_calendar%10)]);

			  u8g2_SendBuffer(&u8g2);
			  		  if (eWindow!=CALENDAR)
			  		  	   {
			  			      unixTime.year=tmp_year_calendar+2000;
			  			      unixTime.mon=tmp_mon_calendar;
			  			      unixTime.mday= tmp_day_calendar;
			  			      unixTime.hour=tmp_hour_calendar;
			  			      unixTime.min=tmp_min_calendar;
			  			    mRTCSetCounter(colodarToCounter(&unixTime) );
			  		  		   return;
			  		  	   }

		   }
}
void whatDayOfNow(void)
{
	what_day_now = (unixTime.year / 12) + (unixTime.year % 12) + ((unixTime.year % 12) / 4);     // индекс года
				  uint8_t iiM[] = {6, 2, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4}; // индекс месяца в табл
				  what_day_now += iiM[unixTime.mon - 1];                                // индекс месяца выбраный
				  if ((unixTime.year % 44) == 0) {                              // индекс високосного года
				    if ((unixTime.mon == 1) || (unixTime.mon == 2))
				    	what_day_now--;
				  }
				  what_day_now += unixTime.mday;                                         // индекс дня
				  what_day_now = (what_day_now % 7);

}
void onOffZonaX(int8_t i)
{
	                if(i==0)
		    		{
	                	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // On SOLENOID_2
	                	LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7); // On SOLENOID_3
	                	LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_14); // On SOLENOID_4

		    			send_str_usart("        Zona 1 ON      ");
		    			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1




		    		}
		    		else if(i==1)
		    		{
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7); // On SOLENOID_3
		    			LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_14); // On SOLENOID_4

		    			send_str_usart("        Zona 2 ON      ");
		    			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_8); // On SOLENOID_2


		    		}
		    		else if(i==2)
		    		{
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // On SOLENOID_2
		    			LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_14); // On SOLENOID_4

		    			send_str_usart("        Zona 3 ON      ");


		    			LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7); // On SOLENOID_3
		    		}
		    		else if(i==3)
		    		{
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // On SOLENOID_2
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7); // On SOLENOID_3

		    			send_str_usart("        Zona 4 ON      ");

		    			LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_14); // On SOLENOID_4
		    		}
		    		else
		    		{
		    			// All Zona OFF
		    			//send_str_usart("        All OFF      ");
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // Off SOLENOID_1
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8); // Off SOLENOID_2
		    			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7); // On SOLENOID_3
		    			LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_14); // On SOLENOID_4

		    		}
	                //LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); // On SOLENOID_1
}


void aht10_Begin(void)
{

	uint8_t d;
	 d= LL_I2C_IsEnabled (I2C1);
		 if(d)
		    {
		 	   send_str_usart("      I2C peripherals is enabled    ");
		    }
		    else
		    {
		 	   send_str_usart("    I2C peripherals is disabled     ");
		    }
		send_str_usart("            Start     Begin    ");
	    I2C1->CR1 |= I2C_CR1_START;

	        uint16_t i=5000;

		while(!(I2C1->SR1 & I2C_SR1_SB))
		{
			if(i==0)
				return ;

			--i;
		}
		   (void) I2C1->SR1;

		   d=LL_I2C_IsActiveFlag_MSL ( I2C1);
		   if(d)
		     {
		        send_str_usart("      Master mode    ");
		     }
		   else
		     {
		       send_str_usart("    Slave mode    ");
		     }

		send_str_usart("      Start is ok    ");

	        //передаем адрес устройства
		I2C1->DR =0x70;  /// D0

		i=500;

		while(!(I2C1->SR1 & I2C_SR1_ADDR))
		{
			if(i==0)
				return;
			--i;
		}
		(void) I2C1->SR1;
		(void) I2C1->SR2;
		send_str_usart("      Adres send    ");
	        //передаем адрес регистра
		//I2C1->DR = reg;
		//while(!(I2C1->SR1 & I2C_SR1_TXE)){};
		//send_str_usart("      Register send    ");
	        //пишем данные


		//             end adress



		//data= 0x33;
		I2C1->DR = 0xE1;

		i=5000;

		while(!(I2C1->SR1 & I2C_SR1_BTF))
		{
			if(i==0)
				return;
			--i;
		}
		send_str_usart("      Byte flag finished    ");



		//data=0x00;
		I2C1->DR = 0x08;

		i=5000;

		while(!(I2C1->SR1 & I2C_SR1_BTF))
		{
			if(i==0)
				return;
			--i;
		}
		send_str_usart("      Byte flag finished    ");




		I2C1->DR = 0x00;

			i=500;

			while(!(I2C1->SR1 & I2C_SR1_BTF))
			{
				if(i==0)
					return;
				--i;
			}
			send_str_usart("      Byte flag finished    ");

		I2C1->CR1 |= I2C_CR1_STOP;


}

int8_t aht10_Measure(volatile int8_t *temp, volatile int8_t *hum)
{
	volatile uint32_t wait;


	uint8_t data[6];
	uint32_t  d;
	// Send Start and Adress

	send_str_usart("                     Start   Measure                    ");
	I2C1->CR1 |= I2C_CR1_START;

	uint16_t i=5000;

	while(!(I2C1->SR1 & I2C_SR1_SB))
	{
		if(i==0)
		return -1;

		--i;
	}
	(void) I2C1->SR1;

	d=LL_I2C_IsActiveFlag_MSL ( I2C1);
	if(d)
	{
		send_str_usart("          Master mode        ");
	}
	else
	 {
		send_str_usart("         Slave mode        ");
	 }

	send_str_usart("             Start is ok       ");

		        //передаем адрес устройства
	I2C1->DR =0x70;  /// D0

	i=500;

	while(!(I2C1->SR1 & I2C_SR1_ADDR))
	{
		if(i==0)
		  return -1;
		--i;
	}
	(void) I2C1->SR1;
	(void) I2C1->SR2;
	send_str_usart("         Adres send    ");



	I2C1->DR = 0xAC;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_BTF))
	{
		if(i==0)
		  return -1;
		--i;
	}
	send_str_usart("          Byte flag finished    ");



					//data=0x00;
	I2C1->DR = 0x33;

	i=5000;

	while(!(I2C1->SR1 & I2C_SR1_BTF))
		{
			if(i==0)
			return -1;
		  --i;
		}
	send_str_usart("      Byte flag finished    ");


	I2C1->DR = 0x00;

	i=500;

	while(!(I2C1->SR1 & I2C_SR1_BTF))
		{
			if(i==0)
			return -1;
			--i;
		}
	send_str_usart("      Byte flag finished    ");

	I2C1->CR1 |= I2C_CR1_STOP;

  LL_mDelay(150);


  //-----------------------------------READ DATA--------------

  // Enable Acknowledgment
  	I2C1->CR1 |= I2C_CR1_ACK;
  	// Clear POS flag
  	I2C1->CR1 &= ~I2C_CR1_POS; // NACK position current





  	i=5000;

    I2C1->CR1 |= I2C_CR1_START;
  	while(!(I2C1->SR1 & I2C_SR1_SB))
  	{
  	   if(i==0)
  		   return -1;
  	   --i;
  	}
  	send_str_usart("      Start is OK   ");
  	(void) I2C1->SR1;

  	//передаем адрес устройства
  	I2C1->DR = 0x71;

  	i=5000;


  	while(!(I2C1->SR1 & I2C_SR1_ADDR))
  	{
  		if(i==0)
  			return -1;
  		--i;
  	}
  	send_str_usart("      Adres send finished   ");
  	(void) I2C1->SR1;
  	(void) I2C1->SR2;

  	//читаем
  	I2C1->CR1 |= I2C_CR1_ACK; //  ACK  enabled

  	i=5000;

  	uint8_t nbytes = 6;
  	wait = 5000;
  	uint8_t count=0;
  	while (nbytes-- != 3) {
  				// Wait for BTF (cannot guarantee 1 transfer completion time)

  				while ( ! (I2C1->SR1 & I2C_SR1_BTF) && --wait);
  				if (!wait) return -1;
  				data[count] = (uint8_t)I2C1->DR;
  				count++;
  			}
  			// Wait for BTF flag set (byte transfer finished) EV7_2

  	wait = 5000;
  	while ( !( I2C1->SR1 & I2C_SR1_BTF) && --wait)
  		{

  		}
  	  if (!wait) return -1;


  	// Disable the I2C acknowledgment
  			I2C1->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);



  	__disable_irq();
  	data[count] = (uint8_t)I2C1->DR; // Receive byte N-2
  	count++;
  	I2C1->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
  	__enable_irq();



  	data[count] = I2C1->DR; // Receive byte N-1
  			// Wait for last byte received
  	wait = 5000;
  	while (!(I2C1->SR1 & I2C_SR1_RXNE) && --wait);
  	if (!wait) return -1;




  	data[count] = (uint8_t)I2C1->DR; // Receive last byte

  	nbytes = 0;


  	wait = 5000;
  		while ( (I2C1->SR1 &  I2C_SR1_STOPF  ) && --wait);
  		if (!wait) return -1;



  	send_str_usart("                       NOT Busy             ");



   d = ( (uint32_t) (data[3]  & 0x0F)<<16 ) | ((uint32_t) data[4]<<8) | data[5] ;

   *temp = d* 200 / 1048576 - 50;

   d = ( (uint32_t) (data[1]  << 12) ) | ((uint32_t) data[2]<<4) |  (data[3] >> 4) ;

   *hum = (d* 100) / 1048576;


   send_str_usart("               Temp::::         ");
   send_8bit_data_usart(digit_mas[ *temp/10 ]);
   send_8bit_data_usart(digit_mas[ *temp%10 ]);


   return 0;

}

void aht10_Reset(void)
{
	I2C1->CR1 |= I2C_CR1_START;

		        uint16_t i=5000;

			while(!(I2C1->SR1 & I2C_SR1_SB))
			{
				if(i==0)
					return ;

				--i;
			}
			   (void) I2C1->SR1;



			send_str_usart("      Start is ok    ");

		        //передаем адрес устройства
			I2C1->DR =0x70;  /// D0

			i=500;

			while(!(I2C1->SR1 & I2C_SR1_ADDR))
			{
				if(i==0)
					return;
				--i;
			}
			(void) I2C1->SR1;
			(void) I2C1->SR2;
			send_str_usart("      Adres send    ");


			I2C1->DR = 0xBA;

			i=5000;

			while(!(I2C1->SR1 & I2C_SR1_BTF))
			{
				if(i==0)
					return;
				--i;
			}
			send_str_usart("      Reset finished    ");
}

void clearBeasyFlagI2C1(void)
{

	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	I2C1->CR1 &= ~I2C_CR1_PE;
	LL_I2C_Disable(I2C1);
	GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	//GPIO_InitStruct.Pull = LL_;
	//GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6 | LL_GPIO_PIN_7); //I2C1

	while(  (!(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_6))) && (!(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_7))) )
			{


			}
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7 ); //  SDA Low

    while(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_7))
    {

    }


    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7 ); //  SCL Low

        while(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_7))
        {

        }

   // 8 - 9 - 10 - 11
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6 | LL_GPIO_PIN_7); //I2C1

       while(  (!(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_6))) && (!(LL_GPIO_IsOutputPinSet(GPIOB, LL_GPIO_PIN_7))) )
        {


        }


// 12
       /**I2C1 GPIO Configuration
         PB6   ------> I2C1_SCL
         PB7   ------> I2C1_SDA
         */
         GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
         GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
         //GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
         //GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
         LL_GPIO_Init(GPIOB, &GPIO_InitStruct);


         // 13. Set SWRST bit in I2Cx_CR1 register.
         //SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
         I2C1->CR1 |= I2C_CR1_SWRST;

         asm("nop");

         /* 14. Clear SWRST bit in I2Cx_CR1 register. */
         I2C1->CR1 &= ~I2C_CR1_SWRST;
         asm("nop");

         /* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
         I2C1->CR1 |= I2C_CR1_PE;
         asm("nop");

}



