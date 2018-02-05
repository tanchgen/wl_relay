/*
 * adc.c
 *
 *  Created on: 2 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32f0xx.h"
#include "main.h"
#include "process.h"
#include "bat.h"

// Batary volt sensor init function
void adcInit(void){

  // Вкл тактирование АЦП
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;// | RCC_APB2ENR_SYSCFGEN;

  if( ADC1->CR != 0 ){
    ADC1->CR |= ADC_CR_ADDIS;
  }
  while( ADC1->CR != 0 )
  {}
  // Конфигурация АЦП
  ADC1->CFGR2 = ADC_CFGR2_CKMODE_0;
  // Включаем программный запуск и AUTOFF
  ADC1->CFGR1 = (ADC1->CFGR1 & ~ADC_CFGR1_EXTEN) | ADC_CFGR1_DISCEN;// | ADC_CFGR1_AUTOFF;
  // Только TSEN и VBAT канал
  ADC1->CHSELR = ADC_CHSELR_CHSEL16 | ADC_CHSELR_CHSEL17 | ADC_CHSELR_CHSEL18;
  // Длительность сэмпла = 12.5 ADCCLK (+12.5) = 25 ADCCLK ( 5.96мкс )
  ADC1->SMPR |= ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1;

  ADC->CCR |= ADC_CCR_TSEN | ADC_CCR_VREFEN;

  // Калибровка АЦП
//  if ((ADC1->CR & ADC_CR_ADEN) != 0) {
//    ADC1->CR |= ADC_CR_ADDIS;
//  }
//  while ((ADC1->CR & ADC_CR_ADEN) != 0)
//  {}

  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) != 0)
  {}

	// Выключаем внутренний регулятор напряжения
//  RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
}

void adcStart( void ){
  driveData.bat = 0;

  // Вкл тактирование АЦП
//  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC->CCR |= ADC_CCR_VBATEN;

  // Опять включаем АЦП после калибровки
  ADC1->CR |= ADC_CR_ADEN;

  // Ждем, когда запустится VREFINT
  while( (PWR->CSR & PWR_CSR_VREFINTRDYF) == 0 )
  {}
  if ((ADC1->CFGR1 &  ADC_CFGR1_AUTOFF) == 0) {
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
    {}
  }
  ADC1->CR |= ADC_CR_ADSTART;
}

void adcEnd( void ){
//  int32_t temperature;
  uint32_t vref;
  uint32_t vdd;
  uint32_t vbat;
  uint32_t vrefCal = *((uint16_t *)0x1FFFF7BA);
  uint16_t temp110Cal = *((uint16_t *) 0x1FFFF7C2);
  uint16_t temp30Cal = *((uint16_t *) 0x1FFFF7B8);

	// Ждем окончания преобразования
  while( (ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC )
  {}
  // Меряем температуру
  /* Temperature sensor calibration value address */
  #define TEMP110_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7C2))
  #define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
  #define VDD_CALIB ((uint16_t)(3300))
  int32_t mcuTemp;
  /* will contain the temperature in degrees Celsius */
  mcuTemp = (int32_t)(ADC1->DR);
//
  // Меряем Vdda
  ADC1->CR |= ADC_CR_ADSTART;
  while( (ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC )
  {}
  vdd = (VDD_CALIB * vrefCal)/(ADC1->DR);

  // Меряем VBAT
  ADC1->CR |= ADC_CR_ADSTART;
  while( (ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC )
  {}
  vbat = ADC1->DR;
  // Выключаем внутренний регулятор напряжения
  ADC1->CR |= ADC_CR_ADDIS;
  // Стираем флаги
  ADC1->ISR |= 0xFF; //ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSMP;


  mcuTemp *= vdd;
  mcuTemp /= VDD_CALIB;
  mcuTemp -= (int32_t)temp30Cal;//*TEMP30_CAL_ADDR ;
  mcuTemp *= (int32_t)(110 - 30);
  mcuTemp /= (int32_t)(temp110Cal - temp30Cal);
  driveData.temp = (int16_t)(mcuTemp + 30);
  driveData.bat = (vdd * vbat / (4096 / 2) )/10 - 150;

  if( (driveData.temp > TEMP_TRESH) || driveData.bat < BAT_TRESH ){
    flags.driveErr = SET;
  }
}
