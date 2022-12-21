#ifndef __CAN_SPI_H
#define	__CAN_SPI_H

#include "stm32f0xx_hal.h"
#include "stdbool.h"


typedef union {
  struct {
    uint8_t idType;
    uint32_t id;
    uint8_t dlc;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;
  } frame;
  uint8_t array[14];
} uCAN_MSG;

#define dSTANDARD_CAN_MSG_ID_2_0B 1
#define dEXTENDED_CAN_MSG_ID_2_0B 2

bool CANSPI_Initialize(SPI_HandleTypeDef *SPI_CAN);
void CANSPI_Sleep(SPI_HandleTypeDef *SPI_CAN);
uint8_t CANSPI_Transmit(SPI_HandleTypeDef *SPI_CAN, uCAN_MSG *tempCanMsg);
uint8_t CANSPI_Receive(SPI_HandleTypeDef *SPI_CAN, uCAN_MSG *tempCanMsg);
uint8_t CANSPI_messagesInBuffer(SPI_HandleTypeDef *SPI_CAN);
uint8_t CANSPI_isBussOff(SPI_HandleTypeDef *SPI_CAN);
uint8_t CANSPI_isRxErrorPassive(SPI_HandleTypeDef *SPI_CAN);
uint8_t CANSPI_isTxErrorPassive(SPI_HandleTypeDef *SPI_CAN);

#endif	/* __CAN_SPI_H */
