/*
 * relay.c
 *
 *  Created on: 30 янв. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32f0xx.h"
#include "main.h"
#include "relay.h"

union {
  uint8_t u8;
  sRelStat relStat;
} relayState;

volatile uint8_t ledBlinkGreen;    // Счетчик мигания светодиода On/RX
volatile uint8_t ledBlinkR1;    // Счетчик мигания светодиода Реле1
volatile uint8_t ledBlinkR2;    // Счетчик мигания светодиода Реле2

void relayInit( void ){
  // ----------- Relay GPIO configration ----------------------
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  /** RELAY GPIO Configuration
  PB6   ------> RELAY1
  PB7   ------> RELAY2
  */
  GPIOB->BRR |= GPIO_Pin_6 | GPIO_Pin_7;
  GPIOB->OSPEEDR |= (0x02L << (6 * 2)) | (0x02L << (7 * 2));
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
                  | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;
  // Выключаем подтяжку
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);

  relayStat();
}

void relayStat( void ){
  uint32_t tmpPin;
  // Состояние реле 1
  tmpPin = GPIOB->IDR & GPIO_Pin_6;
  if( tmpPin ){
    relayState.relStat.st1 = SET;
//    ledR1On();
  }
  else {
    relayState.relStat.st1 = RESET;
//    ledR1Off();
  }
  if((GPIOB->ODR & GPIO_Pin_6) == tmpPin){
//    ledBlinkR1 = 0;
    relayState.relStat.err1 = RESET;
  }
  else {
    // Ошибка - запускаем таймер мигания светодиодов
//    ledBlinkR1 = LED_BLINK_TIME;
    relayState.relStat.err1 = SET;
    flags.driveErr = SET;
  }

  // Состояние реле 2
  tmpPin = GPIOB->IDR & GPIO_Pin_7;
  if( tmpPin ){
    relayState.relStat.st2 = SET;
//    ledR2On();
  }
  else {
    relayState.relStat.st2 = RESET;
//    ledR2Off();
  }
  if((GPIOB->ODR & GPIO_Pin_7) == tmpPin){
//    ledBlinkR2 = 0;
    relayState.relStat.err2 = RESET;
  }
  else {
    // Ошибка - запускаем таймер мигания светодиодов
//    ledBlinkR2 = LED_BLINK_TIME;
    relayState.relStat.err2 = SET;
    flags.driveErr = SET;
  }

  // Управление ОДНИМ светодиодом
  if( relayState.relStat.err1 || relayState.relStat.err2){
    // Ошибка одного из двух реле
    ledBlinkR1 = LED_BLINK_TIME;
  }
  else if( relayState.relStat.st1 || relayState.relStat.st2){
//    ledR1On();
  }
  else{
//    ledR1Off();
  }

  driveData.devState = relayState.u8;
}

void ledInit( void ){
  // ----------- Relay GPIO configration ----------------------
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
#if 0
  // Схема с 3-мя светодиодами
  /** RELAY GPIO Configuration
  PA10   ------> LED_GREEN
  PA11   ------> LED_REL1
  PA12   ------> LED_REL2
  */
  GPIOA->OSPEEDR |= (0x02L << (10 * 2)) | (0x02L << (11 * 2)) | (0x02L << (12 * 2));
  GPIOA->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) \
                  | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0;
  // Выключаем подтяжку
  GPIOA->PUPDR &= ~(GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
  GPIOA->OTYPER |= GPIO_Pin_10;
#else
  // Схема с 2-мя светодиодами
  /** RELAY GPIO Configuration
  PA11   ------> LED_GREEN
  PA12   ------> LED_REL1
  */
  GPIOA->OTYPER |= GPIO_Pin_11;
  GPIOA->OSPEEDR |= (0x02L << (11 * 2)) | (0x02L << (12 * 2));
  GPIOA->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) \
                  | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER12_0;
  // Выключаем подтяжку
  GPIOA->PUPDR &= ~(GPIO_Pin_11 | GPIO_Pin_12);
#endif
  ledGreenOff();
  (relayState.relStat.st1 == 0)? ledR1Off(): ledR1On();
  (relayState.relStat.st2 == 0)? ledR2Off(): ledR2On();
}

