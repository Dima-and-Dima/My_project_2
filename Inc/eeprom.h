/**
  ******************************************************************************
  * @file           : eeprom.h
  * @brief          : Eeprom emulation driver
  * @author         : Prog
  ******************************************************************************
  */

#ifndef EEPROM_H
#define EEPROM_H



/* Includes ------------------------------------------------------------------*/

//#include "stm32f1xx_hal.h"

#include "main.h"
#include <stdbool.h>

/* Declarations and definitions ----------------------------------------------*/

#define PAGE_DATA_OFFSET                                                8
#define PAGE_DATA_SIZE                                                  8

#define PARAM_1                                                         0x12121212
#define PARAM_2                                                         0x34343434
#define VAR_NUM                                                         2

#define PAGE_ZONA_1_ADDRESS                                             0x0803E000
#define PAGE_ZONA_2_ADDRESS                                             0x0803E800
#define PAGE_ZONA_3_ADDRESS                                             0x0803F000
#define PAGE_ZONA_4_ADDRESS                                             0x0803F800
#define PAGE_SIZE                                                       2048




#define  PAGE_CLEARED  0xFFFFFFFF
#define  PAGE_ACTIVE  0x00000000
#define  PAGE_RECEIVING_DATA  0x55555555


typedef enum {
  PAGE_0 = 0,
  PAGE_1 = 1,
  PAGES_NUM = 2,
} PageIdx;

typedef enum {
  EEPROM_OK = 0,
  EEPROM_ERROR = 1,
} EepromResult;

typedef struct

{
	uint32_t 	TypeErase;
	uint32_t 	Banks;
	uint32_t 	Sector;
	uint32_t    NbSectors;
	uint32_t 	VoltageRange;
}FLASH_EraseInitTypeDef;


typedef enum

{

 HAL_OK       = 0x00U,

 HAL_ERROR    = 0x01U,

 HAL_BUSY     = 0x02U,

 HAL_TIMEOUT  = 0x03U

} HAL_StatusTypeDef;


//extern enumZONA_SETUP;


//----------------------------------------------------------------------------


#define 	HAL_FLASH_ERROR_NONE   0x00000000U
#define 	HAL_FLASH_ERROR_RD   0x00000001U
#define 	HAL_FLASH_ERROR_PGS   0x00000002U
#define 	HAL_FLASH_ERROR_PGP   0x00000004U
#define 	HAL_FLASH_ERROR_PGA   0x00000008U
#define 	HAL_FLASH_ERROR_WRP   0x00000010U
#define 	HAL_FLASH_ERROR_OPERATION   0x00000020U

//----------------------------------------------------------------------------

#define      HAL_MAX_DELAY        50000

#define 	FLASH_TIMEOUT_VALUE   50000U /* 50 s */


typedef enum
 {
   FLASH_PROC_NONE = 0U,
   FLASH_PROC_SECTERASE,
   FLASH_PROC_MASSERASE,
   FLASH_PROC_PROGRAM
 } FLASH_ProcedureTypeDef;

 //---------------------------------------------------------------------------

 typedef enum
 {
     HAL_UNLOCKED = 0x00U,
     HAL_LOCKED   = 0x01U
 } HAL_LockTypeDef;

//----------------------------------------------------------------------------



typedef enum {
	FLASH_OK,        ///< "Good, code with all of your anger"
	FLASH_PRG_ERROR, ///< Programming error, you didn't call flashErasePage() did you?
	FLASH_WRT_ERROR,  ///< Write protection error, this section of flash is write protected.
	FLASH_NEW
} FlashError;

/* Functions -----------------------------------------------------------------*/

extern EepromResult EEPROM_Init();
extern EepromResult EEPROM_Read(uint32_t varId, uint32_t *varValue);
extern EepromResult EEPROM_Write(uint32_t varId, uint32_t varValue);

extern  void FLASH_Program_DoubleWord(uint32_t Address, uint64_t Data);

extern   FlashError flashErasePage(uint32_t addr);
extern   FlashError flashCopyPage(uint32_t src_addr, uint32_t dest_addr, bool erase);
extern   FlashError flashWrite_16(uint32_t addr, uint16_t data);
extern   void flashUnlock(void);
extern   void flashLock(void);
extern   FlashError flashWrite_32(uint32_t addr, uint32_t data);
extern   uint32_t FLASH_Read(uint32_t address);

FlashError flasWriteZona_X(uint8_t Zona_X);
FlashError  flashReadZona_X(uint8_t Zona_X);

#endif // #ifndef EEPROM_H
