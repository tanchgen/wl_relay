// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "my_time.h"
#include "bat.h"
#include "relay.h"
#include "spi.h"
#include "rfm69.h"
#include "process.h"
#include "main.h"

volatile uint32_t mTick;

volatile tDriveData driveData;    // Структура измеряемых датчиком параметров
volatile eState state;          // Состояние машины
volatile tFlags flags;          // Флаги состояний системы
RCC_ClocksTypeDef RCC_Clocks;

tPkt rxPkt;

static void SetSysClock(void);

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;
  // Send a greeting to the trace device (skipped on Release).
//  trace_puts("Hello ARM World!");

  RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
  DBGMCU->APB1FZ |= DBGMCU_RTC_STOP;
  SetSysClock();
  // At this stage the system clock should have already been configured
  // at high speed.
//  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  rfmInit();
  adcInit();      // Инициализация АЦП для измерения напряжения питания и температуры прибора
  ledInit();
  relayInit();

  //  timeInit();
  //  usTimInit();
  ////  errTimInit();
  mesure();
  //  // Пробуем передавать данные серверу?
  //  csmaRun();

  // Для теста передачи выключаем прерывание от DIO0
  EXTI->IMR &= ~(DIO0_PIN);
  EXTI->RTSR &= ~(DIO0_PIN);
  NVIC_DisableIRQ( DIO0_EXTI_IRQn );

  // Infinite loop
  while (1) {
    // !!! Дебажим  регистры !!!
    for( uint8_t i = 1; i < 0x40; i++ ){
      regBuf[i] = rfmRegRead( i );
    }
    sensDataSend();
    while( dioRead(DIO_PAYL_RDY) == RESET )
    {}
    rfmSetMode_s( REG_OPMODE_FS );
    mDelay(5000);
//    // !!! Дебажим  регистры !!!
//    for( uint8_t i = 1; i < 0x40; i++ ){
//      regBuf[i] = rfmRegRead( i );
//    }
  }
  // Infinite loop, never return.
}

/**
  * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
  *         settings.
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
static void SetSysClock(void)
{
//  __IO uint32_t StartUpCounter = 0, HSIStatus = 0;
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/******************************************************************************/

  /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSION);

  /* Wait till HSE is ready and if Time out is reached exit */
  while( (RCC->CR & RCC_CR_HSIRDY) == 0 )
  {}

  /* Enable Prefetch Buffer and set Flash Latency */
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

  /* Select HSI as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  while( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
  {}

  // HCLK = SYSCLK
  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
//  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV2;

  // PCLK = HCLK
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

  // Select SYSCLK -> I2C clock source
  RCC->CFGR3 &= ~RCC_CFGR3_I2C1SW;

  // Enable PLL
  RCC->CR &= ~RCC_CR_PLLON;
  // Wait till PLL is ready
  while((RCC->CR & RCC_CR_PLLRDY) == 1)
  {}
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL);
  RCC->CFGR |= RCC_CFGR_PLLMUL12;

  // Enable PLL
  RCC->CR |= RCC_CR_PLLON;
  // Wait till PLL is ready
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {}

  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
  // Wait till PLL is used as system clock source
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {}

  FLASH->ACR |= FLASH_ACR_LATENCY;

  RCC_GetClocksFreq(&RCC_Clocks);

#define SYSTICK_CLKSOURCE_HCLK         ((uint32_t)0x00000004)
  SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
  SysTick_Config(RCC_Clocks.HCLK_Frequency/1000);

  NVIC_SetPriority(SysTick_IRQn, 0);
  NVIC_EnableIRQ(SysTick_IRQn);

}
