/*
 * relay.h
 *
 *  Created on: 30 янв. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef RELAY_H_
#define RELAY_H_

#include "stm32f0xx.h"


typedef struct {
  unsigned st1: 1;
  unsigned err1: 1;
  unsigned st2: 1;
  unsigned err2: 1;
  unsigned st3: 1;
  unsigned err3: 1;
  unsigned st4: 1;
  unsigned err4: 1;
} sRelStat;

#define LED_BLINK_TIME  250     // Время переключения светодиода

extern volatile uint8_t ledBlinkGreen;    // Счетчик мигания светодиода On/RX
extern volatile uint8_t ledBlinkR1;    // Счетчик мигания светодиода Реле1
extern volatile uint8_t ledBlinkR2;    // Счетчик мигания светодиода Реле2

void relayInit( void );
void relayStat( void );
void ledInit( void );

inline void relay1On( void ){
  GPIOB->BSRR |= GPIO_Pin_6;
}
inline void relay1Off( void ){
  GPIOB->BRR |= GPIO_Pin_6;
}

inline void relay2On( void ){
  GPIOB->BSRR |= GPIO_Pin_7;
}
inline void relay2Off( void ){
  GPIOB->BRR |= GPIO_Pin_7;
}

#if 0
  // Схема с 3-мя светодиодами
  /** RELAY GPIO Configuration
  PA10   ------> LED_GREEN
  PA11   ------> LED_REL1
  PA12   ------> LED_REL2
  */
// LED питания и приема команды
inline void ledGreenOn( void ){
  GPIOA->BRR |= GPIO_Pin_10;
}
inline void ledGreenOff( void ){
  GPIOA->BSRR |= GPIO_Pin_10;
}

// LED Реле1 вкл/выкл
inline void ledR1On( void ){
  GPIOA->BSRR |= GPIO_Pin_11;
}
inline void ledR1Off( void ){
  GPIOA->BRR |= GPIO_Pin_11;
}
inline void ledR1Toggle( void ){
  GPIOA->ODR ^= GPIO_Pin_11;
}

// LED Реле1 вкл/выкл
inline void ledR2On( void ){
  GPIOA->BSRR |= GPIO_Pin_12;
}
inline void ledR2Off( void ){
  GPIOA->BRR |= GPIO_Pin_12;
}
inline void ledR2Toggle( void ){
  GPIOA->ODR ^= GPIO_Pin_12;
}
#else
  // Схема с 2-мя светодиодами
  /** RELAY GPIO Configuration
  PA11   ------> LED_GREEN
  PA12   ------> LED_REL1
  PA12   ------> LED_REL2
  */
// LED питания и приема команды
inline void ledGreenOn( void ){
  GPIOA->BRR |= GPIO_Pin_11;
}
inline void ledGreenOff( void ){
  GPIOA->BSRR |= GPIO_Pin_11;
}

// LED Реле1 вкл/выкл
inline void ledR1On( void ){
  GPIOA->BSRR |= GPIO_Pin_12;
}
inline void ledR1Off( void ){
  GPIOA->BRR |= GPIO_Pin_12;
}
inline void ledR1Toggle( void ){
  GPIOA->ODR ^= GPIO_Pin_12;
}

// LED Реле1 вкл/выкл
inline void ledR2On( void ){
  GPIOA->BSRR |= GPIO_Pin_12;
}
inline void ledR2Off( void ){
  GPIOA->BRR |= GPIO_Pin_12;
}
inline void ledR2Toggle( void ){
  GPIOA->ODR ^= GPIO_Pin_12;
}
#endif

#endif /* RELAY_H_ */
