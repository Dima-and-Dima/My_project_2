
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_rtc.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"
#include "calendar_timer.h"
#include "atoi_stm32.h"
#include "usart_stm32.h"
#include "eeprom.h"

//#include "u8g2.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */


#define PS_Pin LL_GPIO_PIN_4
#define PS_GPIO_Port GPIOE
#define C86_Pin LL_GPIO_PIN_11
#define C86_GPIO_Port GPIOA
#define RD_Pin LL_GPIO_PIN_14
#define RD_GPIO_Port GPIOA
#define WR_Pin LL_GPIO_PIN_15
#define WR_GPIO_Port GPIOA
#define A0_Pin LL_GPIO_PIN_10
#define A0_GPIO_Port GPIOC
#define RST_Pin LL_GPIO_PIN_11
#define RST_GPIO_Port GPIOC
#define ChipSelect_Pin LL_GPIO_PIN_12
#define ChipSelect_GPIO_Port GPIOC
#define D0_Pin LL_GPIO_PIN_0
#define D0_GPIO_Port GPIOD
#define D1_Pin LL_GPIO_PIN_1
#define D1_GPIO_Port GPIOD
#define D2_Pin LL_GPIO_PIN_2
#define D2_GPIO_Port GPIOD
#define D3_Pin LL_GPIO_PIN_3
#define D3_GPIO_Port GPIOD
#define D4_Pin LL_GPIO_PIN_4
#define D4_GPIO_Port GPIOD
#define D5_Pin LL_GPIO_PIN_5
#define D5_GPIO_Port GPIOD
#define SCL_Pin LL_GPIO_PIN_6
#define SCL_GPIO_Port GPIOD
#define SDA_Pin LL_GPIO_PIN_7
#define SDA_GPIO_Port GPIOD

#define ENODER_UPC_Pin LL_GPIO_PIN_8
#define ENODER_UPC_GPIO_Port GPIOA
#define ENCODER_DOWN_Pin LL_GPIO_PIN_9
#define ENCODER_DOWN_GPIO_Port GPIOA

#define ENCODER_BUTTON_Pin LL_GPIO_PIN_10
#define ENCODER_BUTTON_GPIO_Port GPIOA
#define ENCODER_BUTTON_EXTI_IRQn EXTI15_10_IRQn

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif


#define ds1307_write          0xD0
#define ds1307_read           0xD1

#define MON 1
#define TUE 2
#define WED 3
#define THU 4
#define FRI 5
#define SAT 6
#define SUN 7



/* Variables for AHT10 */
 uint8_t AHT10_RX_Data[6];
volatile uint32_t AHT10_ADC_Raw;
volatile float AHT10_Temperature;
 uint8_t AHT10_TmpHum_Cmd;
#define AHT10_Adress 0x38 << 1



uint8_t ds1307_decToBcd(uint8_t val);
uint8_t ds1307_bcdToDec(uint8_t val);

void ds1307_Begin();
void ds1307_StartClock(void);
void ds1307_StopClock(void);
void ds1307_SetTime(void);
void ds1307_GetTime(void);
void ds1307_FillByHMS(uint8_t _hour, uint8_t _minute, uint8_t _second);
void ds1307_FillByYMD(uint16_t _year, uint8_t _month, uint8_t _day);
void ds1307_FillDayOfWeek(uint8_t _dow);
void ds1307_GetTime(void);
int8_t workZonaNow(void);
void onOffZonaX(int8_t i);

void aht10_Begin(void);
int8_t aht10_Measure( volatile int8_t *temp, volatile int8_t *hum);
void aht10_Reset(void);
void clearBeasyFlagI2C1(void);

uint8_t second;
uint8_t minute;
uint8_t hour;
uint8_t dayOfWeek;// day of week, 1 = Monday
uint8_t dayOfMonth;
uint8_t month;
uint16_t year;

enum enumDirection
{
	UP,
	DOWN,
	NOT_MOVE
	}eDirection;

enum enumWindow
{
  MAIN_W,
  TIME,
  ZONA_X,
  AMOUNT_SETUP,
  WEEK_SETUP,
  MESSAGE_X,
  DAY_TIME,
  MINUTE_SEC,
  WORK_TIME,
  CLEAR_DATA,
  CALENDAR

}eWindow;

enum enumUpdate
{
  NO_UPDATE,
  UPDATE
}eUpdate;

enum enumDAY_NOW
{
	SUBOTA,
	NEDILYA,
	PONEDILOK,
	VIVTOROK,
	SEREDA,
	CHETVER,
	PYATNUTSYA,
	PARNI_DNI
} eDAY_NOW;

enum enumHOUR_MINUTE
{
  HOUR,
  MINUTE
}eHOUR_MINUTE;

enum enumZONA_SETUP
{
  ZONA_1,
  ZONA_2,
  ZONA_3,
  ZONA_4,
}eZONA_SETUP;

enum enumMINUTE_SECUNDE
{
	MINUTE_TYPE,
	SECUNDE_TYPE

}eMINUTE_SECUNDE;


enum enumCalendar
{
  YEAR_CALENDAR,
  MON_CALENDAR,
  DAY_CALENDAR,
  HOUR_CALENDAR,
  MIN_CALENDAR
}eCalendar;

//const uint8_t AREA_X;
//const uint8_t WEEK_X;
//const uint8_t TIME_X;

struct area_x_t   // WHER is 7 days???????????????????????!!!!!!!!
{
	struct week_x_t
	{
	   volatile uint16_t amount_of_setup_day;

	   struct time_x_t
	   {
	     volatile uint16_t work_time_min;
	     volatile uint16_t start_work_hour;
	     volatile uint16_t start_work_minute;
	     volatile uint8_t work_time_sec;
	     volatile uint8_t flag_select_sec_min;
	   }time_x[15];

	}week_x[7];

}area_x[4]; //work zona





extern const uint8_t digit_mas[10];


/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
volatile uint16_t i_cnt;
volatile uint8_t button_flag;
volatile uint8_t press_button_flag;
volatile int8_t f_sel_list;
volatile uint32_t led_lcd_timer;
volatile uint8_t tmp_hour_var;
volatile uint8_t tmp_minute_var;
volatile uint8_t tmp_sec_min_work_tim;
volatile uint8_t tmp_amount_of_setup_day;
volatile uint8_t length_of_time_overlay;
volatile uint8_t curent_position;
volatile uint8_t flag_clear_line;
volatile uint8_t flag_frame;
volatile uint32_t frame_timer;
volatile uint8_t flag_start_stop;
volatile uint8_t tmp_zona_x;
volatile uint16_t is_free_time_flag;
volatile int8_t flag_rewrite_data;
volatile uint8_t flag_on_off_zona;

char tmp_str[200];
volatile uint8_t time_ds;
volatile uint8_t flag_sec_work;
volatile uint8_t day_now;

volatile uint16_t tmp_year_calendar;
volatile uint8_t  tmp_mon_calendar;
volatile uint8_t  tmp_day_calendar;
volatile uint8_t  tmp_hour_calendar;
volatile uint8_t  tmp_min_calendar;

volatile uint8_t  what_day_now;
volatile uint8_t  main_Day_Now;
volatile uint8_t DayNow;


volatile int8_t next_work_zona;
volatile uint16_t next_work_time;

volatile uint8_t tmp_next_hour;
volatile uint8_t tmp_next_min;

volatile uint8_t flag_sensor_work;
volatile uint8_t time_mesure_sensor;



 volatile int8_t tempX;
 volatile int8_t humX;

volatile uint8_t  flag_write_flash;

 uint16_t data_flash;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//#define led_on          GPIOE->BSRR=GPIO_BSRR_BR3
//#define led_off         GPIOE->BSRR=GPIO_BSRR_BS3


#define is_pin_encoder_set_1   ( GPIOA->IDR&(1<<9) )
#define is_pin_encoder_set_2   ( GPIOA->IDR&(1<<8) )
#define is_pin_button_set      ( GPIOA->IDR&(1<<10) )

//

uint8_t read_gray_code_from_encoder(void );
uint8_t debounce(uint8_t sample);

volatile uint8_t NewState,OldState,Vol,upState,downState;

uint32_t colodarSetting(int year, char month, char day, char hour, char min, char sec);
void mainWindow(void);
void zonaX(void);
void weekSetup(void);
void messageX(void);
void timeStartWork(void);
void writeStartTime( int8_t x, int8_t y);
void writeWorkTime(int8_t x, int8_t y);
void minSecSelect(void);
void timeWork(void);
uint16_t isFreeTime(uint8_t hour, uint8_t min,  uint8_t work_time);
void convertMinuteToHour(volatile uint8_t *hour, volatile uint8_t *min, volatile uint16_t *time);
void amountSetupOfDay(void);
void convertAmountSetupToStr(void);
void clearData(void);
void MX_I2C1_Init(void);
void I2C_Write(uint8_t addres,  uint8_t reg, uint8_t data);
int8_t I2C_Read(uint8_t adres, uint8_t reg);
void I2C_Read_Mas(uint8_t adres, uint8_t * mas);
void I2C_WriteReset(void);




void setupCalendar(void);
void whatDayOfNow(void);

void Error_Handler(void);


#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
