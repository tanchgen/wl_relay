/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32f0xx.h"

#include "main.h"
#include "my_time.h"
#include "relay.h"
#include "bat.h"
#include "rfm69.h"
#include "process.h"

volatile uint8_t csmaCount = 0;
tUxTime sendTryStopTime;
static uint8_t msgNum;      // Порядковый номер отправляемого пакета

void sensDataSend( void );
static uint32_t rngGet( void );

void mesure( void ){
  // Запускаем измерение напряжения батареи
  adcStart();
  relayStat();
  adcEnd();
}

void usTimHandler( void ){

  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_DRIV_MESUR:
      // Отправляем отклик
      csmaRun();
      break;
    case STAT_RF_CSMA_START:
      // Канал свободен - отправляем сообщение
      // Запрещаем прерывание от RSSI
      EXTI->PR |= DIO3_PIN;
      EXTI->IMR &= ~(DIO3_PIN);

      // Отмечаем останов RFM_RX
    #if DEBUG_TIME
    	dbgTime.rfmRxEnd = mTick;
    #endif // DEBUG_TIME

    	rfmSetMode_s( REG_OPMODE_SLEEP );
      // Отправить сообщение
      correctAlrm();
      state = STAT_TX_START;
      sensDataSend();
      break;
    case STAT_TX_START:
    	rfmSetMode_s( REG_OPMODE_SLEEP );
      txEnd();
      break;

    case STAT_RF_CSMA_PAUSE:
      // Пробуем еще раз проверить частоту канала
      csmaRun();
      break;
    default:
      break;
  }
}

// Начинаем слушат эфир на предмет свободности канала
void csmaRun( void ){
  state = STAT_RF_CSMA_START;
  // Включаем прерывание от DIO3 (RSSI)
  EXTI->IMR |= (DIO3_PIN);
  EXTI->PR |= DIO3_PIN;

  // Отмечаем запуск RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxStart = mTick;
#endif // DEBUG_TIME

	csmaCount++;
  // Будем слушать эфир в течение времени передачи одного пакета * 2
  usTimSet( TX_DURAT );
}

// Устанавливааем паузу случайной длительности (30-150 мс) в прослушивании канала на предмет тишины
void csmaPause( void ){
  uint32_t pause;
  // TODO: Генератор случайных чисел !!!

  pause = rngGet();
  // Длительность паузы
  pause = ((pause / (0xFFFFFFFFL/9)  ) + 1) * TX_DURAT * csmaCount;
  state = STAT_RF_CSMA_PAUSE;
  usTimSet( pause );
}

void sensDataSend( void ){
  // ---- Формируем пакет данных -----
	pkt.payDriveType = DRIV_TYPE_REL;
  pkt.paySrcNode = rfm.nodeAddr;
  pkt.payMsgNum = msgNum++;
  pkt.payBat = driveData.bat;
  pkt.payState = driveData.devState;
  pkt.payCmdNum = driveData.cmdNum;
  pkt.payTemp = driveData.temp;

  // Передаем заполненую при измерении запись
  pkt.nodeAddr = BCRT_ADDR;
  // Длина payload = 1(nodeAddr) + 1(msgNum) + 1(bat) + 2(temp)
  pkt.payLen = sizeof(tDriveMsg);

  rfmTransmit( &pkt );
  // Таймаут до окончания передачи
  usTimSet( TX_DURAT*10 );
  // Зажигаем красный светодиод
  ledR1On();
  ledBlinkR1 = LED_BLINK_TIME;
}

void txEnd( void ){
  csmaCount = 0;
  flags.sensCplt = FALSE;
  flags.batCplt = FALSE;
  state = STAT_READY;
}

static uint32_t rngGet( void ){
  uint32_t rand0;
  uint8_t b;
  uint32_t m, k;

  rand0 = (getRtcTime() << 16) + rtc.ss;
  m = 0x3FFFFFFF;                 // 2^31-1 - Модуль
  k = 1220703125;              // Множитель
  b = 7;                          // Прироащение
  rand0 = (( k * rand0 + b ) % m);
  return rand0;
}
