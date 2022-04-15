#include "calendar_timer.h"
//#include ""
/*
 * calendar_timer.c
 *
 *  Created on: 30 ���. 2021 �.
 *      Author: Prog
 */

 void mRTCInit(void)
{


 if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN)        // Перевірка роботи годинника, якщо не увімкнені, то ініціалізувати
   {
	// LL_RTC_InitTypeDef RTC_InitStruct = {0};
	     send_str_usart("       RCC init     ");
	     RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;  // Увімкнути тактування PWR та Backup
	     PWR->CR |= PWR_CR_DBP;                                  // Дозволити доступ до Backup області
	     RCC->BDCR |= RCC_BDCR_BDRST;                            // Скинути Backup область
	     RCC->BDCR &= ~RCC_BDCR_BDRST;
	     RCC->BDCR |= RCC_BDCR_RTCEN | RCC_BDCR_RTCSEL_LSE;      // Обрати LSE джерело (кварц 32768) і подати тактування
	     RCC->BDCR |= RCC_BDCR_LSEON;
	     send_str_usart("       LSEON wait..      ");// Увімкнути LSE
	     while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){}// Дочекатись увімкнення
	     send_str_usart("           LSEON  on      ");
	     //send_str_usart("LSEON  RDY flag wait...  ");
	     //while(   (RCC->BDCR&(1<<1))!=1 )
	     //{

	     //}
	     //send_str_usart("LSEON  RDY flag OK!  ");
	     BKP->RTCCR |= 0x07;                                        // Калібрування RTC значення від 0 до 0x7F
	     while (!(RTC->CRL & RTC_CRL_RTOFF));                    // Перевірити чи закінчились зміни регістрів RTC
	     send_str_usart("           RTOFF is ok      ");

	     RTC->CRL  |=  RTC_CRL_CNF;                              // Дозволити запис до RTC

	     RTC->CRH|=(1<<RTC_CRH_SECIE)|(1<<RTC_CRH_ALRIE);// Interrupt is enubled
	     RTC->PRLL  = 0x7FFF;                                    // Налаштувати дільник на 32768 (32767+1)
	     RTC->CRL  &=  ~RTC_CRL_CNF;                             // Заборонити запис до RTC
	     //while (!(RTC->CRL & RTC_CRL_RTOFF));


	     while (!(RTC->CRL & RTC_CRL_RTOFF));                    // Дочекатись кінця запису
	     send_str_usart("         RTC_CRL_RTOFF is OK        ");
	     RTC->CRL &= (uint16_t)~RTC_CRL_RSF;                     // Синхронизувати RTC
	     while((RTC->CRL & RTC_CRL_RSF) != RTC_CRL_RSF){}        // Дочекатись синхронізації
	     send_str_usart("         Sinhronisation OK        ");
	     PWR->CR &= ~PWR_CR_DBP;



	     mRTCSetCounter(colodarSetting(2021,3,18,10,10,0));

     }
	     //NVIC_SetPriority(RTC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	     //NVIC_EnableIRQ(RTC_IRQn);

	     //NVIC_SetPriority(RTC_Alarm_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
	     //NVIC_EnableIRQ(RTC_Alarm_IRQn);

}
 uint32_t mRTCGetCounter(void)                           // �������� �������� ���������
{
          return  (uint32_t)((RTC->CNTH << 16) | RTC->CNTL);
}
 void mRTCSetCounter(uint32_t count)                     // �������� ���� �������� ���������
{
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;   // �������� ���������� PWR � Backup
  PWR->CR |= PWR_CR_DBP;                                    // ��������� ������ �� Backup ������
  while (!(RTC->CRL & RTC_CRL_RTOFF));                      // ��������� �� ���������� ���� �������� RTC
  RTC->CRL |= RTC_CRL_CNF;                                  // ��������� ����� � ������� RTC
  RTC->CNTH = count>>16;                                    // �������� ���� �������� ������������� ���������
  RTC->CNTL = count;
  RTC->CRL &= ~RTC_CRL_CNF;                                 // ���������� ����� � ������� RTC
  while (!(RTC->CRL & RTC_CRL_RTOFF));                      // ���������� ���� ������
  PWR->CR &= ~PWR_CR_DBP;                                   // ���������� ������ �� Backup ������
}
 void counterToColodar (unsigned long counter, unixColodar * unixTime)
 {
  unsigned long a;
  char b;
  char c;
  char d;
  unsigned long time;

  time = counter%SECOND_A_DAY;
  a = ((counter+43200)/(SECOND_A_DAY>>1)) + (2440587<<1) + 1;
  a>>=1;
  unixTime->wday = a%7;
  a+=32044;
  b=(4*a+3)/146097;
  a=a-(146097*b)/4;
  c=(4*a+3)/1461;
  a=a-(1461*c)/4;
  d=(5*a+2)/153;
  unixTime->mday=a-(153*d+2)/5+1;
  unixTime->mon=d+3-12*(d/10);
  unixTime->year=100*b+c-4800+(d/10);
  unixTime->hour=time/3600;
  unixTime->min=(time%3600)/60;
  unixTime->sec=(time%3600)%60;
 }

 unsigned long colodarToCounter (unixColodar * unixTime)
 {
  char a;
  int y;
  char m;
  unsigned long Uday;
  unsigned long time;

  a=((14-unixTime->mon)/12);
  y=unixTime->year+4800-a;
  m=unixTime->mon+(12*a)-3;
  Uday=(unixTime->mday+((153*m+2)/5)+365*y+(y/4)-(y/100)+(y/400)-32045)-2440588;
  time=Uday*SECOND_A_DAY;
  time+=unixTime->sec+unixTime->min*60+unixTime->hour*3600;
  return time;
 }

