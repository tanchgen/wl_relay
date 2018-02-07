/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

#include "relay.h"
#include "my_time.h"
#include "main.h"
#include "process.h"
#include "rfm69.h"
#include "stm32l0xx_it.h"

uint8_t connect = FALSE;

extern uint8_t wutCount;
extern struct tWutTest {
  eState wutState;
  uint32_t wutVol;
} wutTest[];




/* External variables --------------------------------------------------------*/

extern volatile uint8_t csmaCount;

void extiPdTest( void ){
  if(EXTI->PR != 0){
    uint32_t tmp = EXTI->PR;
    EXTI->PR = tmp;
    if( tmp != 0x00020000 ){
      return;
    }
    RTC->ISR &= ~RTC_ISR_ALRAF;
    NVIC->ICPR[0] = NVIC->ISPR[0];
  }
}

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

void NMI_Handler(void){
}

void HardFault_Handler(void){
  while (1)
  {}
}

void SVC_Handler(void){
}

void PendSV_Handler(void){
}

void SysTick_Handler(void) {
  mTick++;
  // Таймер выключения светодиода RX
  if(ledBlinkGreen > 0){
    if(ledBlinkGreen-- == 1){
      ledGreenOff();
    }
  }
  // Таймер переключения светодиода Реле1
  if(ledBlinkR1 > 0){
    if(ledBlinkR1 == 1){
      ledR1Toggle();
      ledBlinkR1 = LED_BLINK_TIME;
    }
    else {
      ledBlinkR1--;
    }
  }
  // Таймер переключения светодиода Реле2
  if(ledBlinkR2 > 0){
    if(ledBlinkR2 == 1){
//      ledR2Toggle();
      ledBlinkR2 = LED_BLINK_TIME;
    }
    else {
      ledBlinkR2--;
    }
  }
}

void TIM2_IRQHandler( void ){
  usTimStop();
  usTimHandler();
}

/**
* RTC global interrupt through EXTI lines 17, 19 and 20.
*/
void RTC_IRQHandler(void){
// Восстанавливаем настройки портов

  // Отмечаем запуск MCU
#if DEBUG_TIME
	dbgTime.mcuStart = mTick;
#endif // DEBUG_TIME

  if( RTC->ISR & RTC_ISR_ALRAF ){
    while( (RTC->ISR & RTC_ISR_RSF) == 0 )
    {}
//    uint32_t tmp2 = RTC->DR;
//    (void)tmp2;
//    uint32_t tmp = RTC->TR;
//
    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;

    uxTime = getRtcTime();

    wutTest[wutCount++].wutVol = rtc.sec;
    if( wutCount == 20){
      wutCount = 0;
    }
    if( ((rtc.sec % secTout) == 0) ){
      if( ((rtc.min % minTout) == 0) ){
      // Alarm A interrupt
      if(state == STAT_READY){
        // Периодическое измерение - измеряем все
        mesure();
        if( ((rtc.min % minToutRx) == 0) || flags.driveErr ){
          // Настало время передачи или ошибка устройства: Передаем состояние
          csmaRun();
        }
      }
      }
    }
    // Стираем флаг прерывания EXTI
    EXTI->PR |= EXTI_PR_PR17;
  }

  // Отмечаем Останов MCU
#if DEBUG_TIME
	dbgTime.mcuEnd = mTick;
#endif // DEBUG_TIME

}

/**
* Главное прерывание от RFM - DIO0
*/
void EXTI0_1_IRQHandler(void)
{
  tPkt rxPkt;

  // Стираем флаг прерывания EXTI
  EXTI->PR |= DIO0_PIN;
  if( rfm.mode == MODE_RX ){
    // Приняли команду.

    driveData.rssi = rfmRegRead( REG_RSSI_VAL );
    rfmReceive( &rxPkt );
    rfmRecvStop();
    driveData.cmdNum = rxPkt.payLoad.cmdMsg.cmdNum;
    // Включаем - Выключаем реле
    (rxPkt.payLoad.cmdMsg.cmdNum & 0x01)? relay1On(): relay1Off();
    (rxPkt.payLoad.cmdMsg.cmdNum & 0x02)? relay2On(): relay2Off();
    // Обновляем состояние устройства
    mesure();
    // Отправляем отклик
    csmaRun();
    if( connect == FALSE ){
      setAlrmMask( RESET );
      connect = FALSE;
    }
  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили пакет с температурой
  	usTimStop();
  	txEnd();
    flags.driveErr = RESET;
  }
  // Отмечаем останов RFM_TX
#if DEBUG_TIME
	dbgTime.rfmTxEnd = mTick;
#endif // DEBUG_TIME
  // Включаем RFM69 на RX
  rfmSetMode_s( REG_OPMODE_RX );

}

// Прерывание по PA3 - DIO3 RSSI
// Канал кем-то занят
void EXTI2_3_IRQHandler( void ){

  usTimStop();

  // Выключаем прерывание от DIO3 (RSSI)
  EXTI->PR |= DIO3_PIN;
  EXTI->IMR &= ~(DIO3_PIN);

  driveData.rssi = rfmRegRead( REG_RSSI_VAL );
//  rfmSetMode_s( REG_OPMODE_SLEEP );

  // Отмечаем останов RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxEnd = mTick;
#endif // DEBUG_TIME

  if( csmaCount >= CSMA_COUNT_MAX ){
    // Количество попыток и время на попытки отправить данные вышло - все бросаем до следующего раза
  	csmaCount = 0;
    state = STAT_READY;
  }
  else {
  	// Можно еще попытатся - выждем паузу
    csmaPause();
  }

  return;
}

#if 0
/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
*/
void I2C1_IRQHandler(void) {
  // Обработка прерывания I2C: работа с термодатчиком
  thermoIrqHandler();
}
#endif
