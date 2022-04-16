/**

  * @file           : eeprom.c
  * @brief          : Eeprom emulation driver
  * @author         : Prog
 
  */



/* Includes ------------------------------------------------------------------*/

#include "eeprom.h"

#include "main.h"

/* Declarations and definitions ----------------------------------------------*/

//static uint32_t pageAddress[PAGES_NUM] = {PAGE_0_ADDRESS, PAGE_1_ADDRESS};
//static uint32_t varIdList[VAR_NUM] = {PARAM_1, PARAM_2};

uint32_t FLASH_Read(uint32_t address);
//PageState EEPROM_ReadPageState(PageIdx idx);
//EepromResult EEPROM_SetPageState(PageIdx idx, PageState state);
EepromResult EEPROM_ClearPage(PageIdx idx);
EepromResult EEPROM_Format();
EepromResult EEPROM_GetActivePageIdx(PageIdx *idx);
EepromResult EEPROM_Init();

FlashError flashErasePage(uint32_t addr);
FlashError flashCopyPage(uint32_t src_addr, uint32_t dest_addr, bool erase);
FlashError flashWrite_16(uint32_t addr, uint16_t data) ;
void flashUnlock(void) ;
void flashLock(void);








/******************************************************************************/
uint32_t FLASH_Read(uint32_t address)
{
  return (*(__IO uint32_t*)address);
}




/******************************************************************************/
EepromResult EEPROM_ClearPage(PageIdx idx)
{
  EepromResult res = EEPROM_OK;
  
  if(idx==PAGE_0)
  {
	  flashErasePage(PAGE_ZONA_1_ADDRESS);
  }
  else if(idx==PAGE_1)
  {
	  flashErasePage(PAGE_ZONA_1_ADDRESS);
  }

  
  
  return res;
}



/******************************************************************************/
EepromResult EEPROM_Format()
{
  EepromResult res = EEPROM_OK;
  
  for (uint8_t i = 0; i < PAGES_NUM; i++)
  {
    res = EEPROM_ClearPage((PageIdx)i);
    
    if (res != EEPROM_OK)
    {
      return res;
    }
  }
    
  return res;
}




FlashError flashErasePage(uint32_t addr)
{
	FlashError status;
	FLASH->CR |= FLASH_CR_PER;              // Page erase operation
	FLASH->AR = addr;                  			// Set the address to the page to be written
	FLASH->CR |= FLASH_CR_STRT;							// Start the page erase

	while ((FLASH->SR & FLASH_SR_BSY) != 0);// Wait until page erase is done

	if ((FLASH->SR & FLASH_SR_EOP) != 0){   // If the end of operation bit is set
		FLASH->SR |= FLASH_SR_EOP;      	    // Clear it, the operation was successful
		status = FLASH_OK;
	}
	else if(FLASH->SR & FLASH_SR_PGERR){    // Otherwise there was an error condition
		FLASH->SR |= FLASH_SR_PGERR;    	    // Clear programming error
		status = FLASH_PRG_ERROR;
	}
	else{                                   // Must be Santa! No, write protection error.
		FLASH->SR |= FLASH_SR_WRPRTERR;
		status = FLASH_WRT_ERROR;
	}
	FLASH->CR &= ~FLASH_CR_PER;             //  Get out of page erase mode
	return status;
}


FlashError flashCopyPage(uint32_t src_addr, uint32_t dest_addr, bool erase)
{
	flashUnlock();
	if(erase){
		flashErasePage(dest_addr);
	}

	for(int i=0; i<1024; i+=2){
		flashWrite_16(dest_addr+i, *((uint16_t*) src_addr+i));
	}

	return FLASH_OK;
}

FlashError flashWrite_16(uint32_t addr, uint16_t data)
{
	FlashError status;

	FLASH->CR |= FLASH_CR_PG;               // Programing mode
	*(__IO uint16_t*)(addr) = data;         // Write data

	while ((FLASH->SR & FLASH_SR_BSY) != 0);// Wait until the end of the operation

	if ((FLASH->SR & FLASH_SR_EOP) != 0){   // If the end of operation bit is set
		FLASH->SR |= FLASH_SR_EOP;          // Clear it, the operation was successful
		status = FLASH_OK;
	}
	else if(FLASH->SR & FLASH_SR_PGERR){    // Otherwise there was an error condition
		FLASH->SR |= FLASH_SR_PGERR;        // Clear programming error
		status = FLASH_PRG_ERROR;
	}
	else{                                   // Must be Santa! No, write protection error.
		FLASH->SR |= FLASH_SR_WRPRTERR;
		status = FLASH_WRT_ERROR;
	}
	return status;
}

FlashError flashWrite_32(uint32_t addr, uint32_t data)
{
	FlashError status;

	status = flashWrite_16(addr, (uint16_t) data);
	if(status !=FLASH_OK){
		return status;
	}
	status = flashWrite_16(addr+2, (uint16_t) data >> 16);

	return status;
}

void flashWriteMemBlock(uint32_t addr, uint8_t* data, uint16_t len){
	//writing in 16 bit incriments, advance by 2 chars
	int i;
	uint32_t word_addr = addr;
	for(i=0; i<len-1; i+=2){
		//archatecture is little-endian (i think)
		uint16_t word = data[i] | (data[i+1] << 8);
		flashWrite_16(word_addr, word);
		word_addr+=2;
	}
	//if odd pad last byte with a null character
	if(len & 1){
		uint16_t word = data[i] | (0x00 << 8);
		flashWrite_16(word_addr, word);
	}
	else {
		//add null character
		flashWrite_16(word_addr, 0x0000);
	}
}


void flashUnlock(void)
{
	 LL_RCC_HSI_SetCalibTrimming(16);
	    LL_RCC_HSI_Enable();

	     /* Wait till HSI is ready */
	    while(LL_RCC_HSI_IsReady() != 1)
	    {

	    }




	while ((FLASH->SR & FLASH_SR_BSY) != 0 );     // Wait for the flash memory not to be busy
	if ((FLASH->CR & FLASH_CR_LOCK) != 0 ){       // Check if the controller is unlocked already
		FLASH->KEYR = FLASH_KEY1;            // Write the first key
		FLASH->KEYR = FLASH_KEY2;        // Write the second key
	}
}

void flashLock(void)
{
	FLASH->CR |= FLASH_CR_LOCK;

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
}

FlashError flasWriteZona_X(uint8_t Zona_X)
{
	  flashUnlock();



   if(Zona_X==0)
   {

	   uint32_t addr = PAGE_ZONA_1_ADDRESS;

	 if(FLASH_Read(addr)  == PAGE_CLEARED)
	 {

		 addr = addr + 2;

	   for(uint8_t i_1=0; i_1<7; i_1++)
	   {
		   flashWrite_16(addr, area_x[0].week_x[i_1].amount_of_setup_day);
		   addr = addr+2;
		   for(uint8_t i_2=0; i_2<15; i_2++)
		   {
			   flashWrite_16(addr, area_x[0].week_x[i_1].time_x[i_2].flag_select_sec_min);
			   addr= addr + 2;
			   flashWrite_16(addr, area_x[0].week_x[i_1].time_x[i_2].start_work_hour);
			   addr= addr + 2;
			   flashWrite_16(addr, area_x[0].week_x[i_1].time_x[i_2].start_work_minute);
			   addr= addr + 2;
			   flashWrite_16(addr, (uint16_t)area_x[0].week_x[i_1].time_x[i_2].work_time_min <<8  | (uint16_t)area_x[0].week_x[i_1].time_x[i_2].work_time_sec );
			  // addr= addr + 2;
			  //flashWrite_16(addr, area_x[0].week_x[i_1].time_x[i_2].work_time_sec);
			   addr= addr + 2;

		   }


	   }


	   // data verefication

	   flashWrite_16( PAGE_ZONA_1_ADDRESS , PAGE_RECEIVING_DATA );

	 }

	 else if(FLASH_Read(addr)  == PAGE_RECEIVING_DATA)
	 {

	 }
   }


   return FLASH_OK;
}

FlashError  flashReadZona_X(uint8_t Zona_X)
{
	if(Zona_X==0)
	   {
		   uint32_t addr = PAGE_ZONA_1_ADDRESS;


		   for(uint8_t i_1=0; i_1<7; i_1++)
		   {
			   if(FLASH_Read(addr) == 0xFFFFFFFF)
				   return FLASH_NEW;
			   area_x[0].week_x[i_1].amount_of_setup_day = FLASH_Read(addr);
			   addr = addr+2;
			   for(uint8_t i_2=0; i_2<15; i_2++)
			   {
				   uint16_t tmp=0;
				   area_x[0].week_x[i_1].time_x[i_2].flag_select_sec_min = FLASH_Read(addr);
				   addr= addr + 2;
				   area_x[0].week_x[i_1].time_x[i_2].start_work_hour = FLASH_Read(addr);
				   addr= addr + 2;
				   area_x[0].week_x[i_1].time_x[i_2].start_work_minute = FLASH_Read(addr);
				   addr= addr + 2;

				   tmp = FLASH_Read(addr);


				   area_x[0].week_x[i_1].time_x[i_2].work_time_min = (uint8_t) tmp>>8;

				   area_x[0].week_x[i_1].time_x[i_2].work_time_sec = (uint8_t) tmp;

				   //area_x[0].week_x[i_1].time_x[i_2].work_time_min = FLASH_Read(addr);
				   //addr= addr + 2;
				   //area_x[0].week_x[i_1].time_x[i_2].work_time_sec = FLASH_Read(addr);
				   addr= addr + 2;

			   }
		   }
	   }

	return FLASH_OK;

}

