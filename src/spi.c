/*
 * spi.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32f0xx.h"

#include "main.h"
#include "spi.h"

/* SPI2 init function */
void spiInit(void) {

  // ----------- SPI GPIO configration ----------------------
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  /**SPI2 GPIO Configuration
  PB12   ------> SPI2_NSS
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MOSI
  PB15   ------> SPI2_MOSI
  */
	// Software NSS
  GPIOB->BSRR |= GPIO_Pin_12;
  GPIOB->OSPEEDR |= (0x03L << (12 * 2)) | (0x03L << (13 * 2)) | (0x03L << (14 * 2)) | (0x03L << (15 * 2));
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15)) \
                  | GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
  // Выключаем подтяжку
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12 | GPIO_PUPDR_PUPDR13 | GPIO_PUPDR_PUPDR14 | GPIO_PUPDR_PUPDR15);
  // Альтернативная функция AF0
  GPIOB->AFR[1] &= ~((0x0FL<<((13-8) * 4)) | (0x0FL<<((14-8) * 4)) | (0x0FL<<((15-8) * 4)));

  /* Enable the peripheral clock SPI2 */
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  /* Configure SPI2 in master */
  // Master selection, BR = 4: Fpclk/32, CPOL and CPHA (rising first edge), 8-bit data frame
  // Set Master Mode and Software control of slave select

  SPI2->CR1 = (SPI2->CR1 & ~(SPI_CR1_CPOL | SPI_CR1_CPHA )) \
		| SPI_CR1_BR_1 | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  // Slave select output enabled
  SPI2->CR2 |= SPI_CR2_FRXTH; //SPI_CR2_SSOE;
  // SPI включается непосредственно перед пердачей или приемом
//  SPI2->CR1 |= SPI_CR1_SPE;
}




// Передача по SPI в блокирующем режима
int8_t spiTrans_s( uint8_t *buf, uint8_t len ){
  uint32_t tout;

  // 	Таймаут ~10мс или около того...
  tout = 20000;
  // NSS -> 0
  GPIOB->BRR |= GPIO_Pin_12;
  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 |= SPI_CR1_SPE;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( ((SPI2->SR & SPI_SR_TXE) != 0 )){
      *(uint8_t *)&(SPI2->DR) = *buf++;
      len--;
    }
    if( --tout == 0){
      return -1;
    }
  }
  // Ждем окончания передачи
  tout = 20000;
  while( (SPI2->SR & SPI_SR_BSY) != 0 ){
    if( --tout == 0){
      return -1;
    }
  }
  // NSS -> 1
  GPIOB->BSRR |= GPIO_Pin_12;
  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 &= ~SPI_CR1_SPE;

  return 0;
}

// Прием по SPI в блокирующем режима
int8_t spiRecv_s( uint8_t *buf, uint8_t len ){
  uint8_t txCount = len;
  uint32_t tout;

  // На всю операцию отводим не более 10мс
  tout = 200000;
  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 |= SPI_CR1_SPE;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( txCount && ((SPI2->SR & SPI_SR_TXE) != 0) ){
      *(uint8_t *)&(SPI2->DR) = 0xFF;
      txCount--;
    }
    if( (SPI2->SR & SPI_SR_RXNE) != 0 ){
      *buf++ = *(uint8_t *)&(SPI2->DR);
      len--;
    }
    if( --tout == 0){
//      return -1;
    }
  }
  // Ждем окончания приема
  tout = 200000;
  while( (SPI2->SR & SPI_SR_BSY) != 0 ){
    if( --tout == 0){
//      return -1;
    }
  }

  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 &= ~SPI_CR1_SPE;
  return 0;
}

// Передача с одновременным приемом по SPI в блокирующем режима
int8_t spiTransRecv_s( uint8_t *txBuf, uint8_t *rxBuf, uint8_t len ){
  uint8_t txCount = len;
  uint32_t tout;

  // На всю операцию отводим не более 10мс
  tout = 20000;
  // NSS -> 0
  GPIOB->BRR |= GPIO_Pin_12;
  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 |= SPI_CR1_SPE;
  // Отправка из буфера tx в буфер SPI
  while( len ){
    if( txCount && ((SPI2->SR & SPI_SR_TXE) != 0) ){
      *(uint8_t *)&(SPI2->DR) = *txBuf++;
      txCount--;
    }
    if( (SPI2->SR & SPI_SR_RXNE) != 0 ){
      *rxBuf++ = *(uint8_t *)&(SPI2->DR);
      len--;
    }
    if( --tout == 0){
      return -1;
    }
  }
  // Ждем окончания приема
  tout = 20000;
  while( (SPI2->SR & SPI_SR_BSY) != 0 ){
    if( --tout == 0){
      return -1;
    }
  }
  // NSS -> 1
  GPIOB->BSRR |= GPIO_Pin_12;
  // SPI включается непосредственно перед пердачей или приемом
  SPI2->CR1 &= ~SPI_CR1_SPE;

  return 0;
}

