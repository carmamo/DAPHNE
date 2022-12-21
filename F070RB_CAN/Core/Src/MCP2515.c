#include "main.h"
#include "MCP2515.h"

/* Pin 설정에 맞게 수정필요. Modify below items for your SPI configurations */
extern SPI_HandleTypeDef        hspi1;
#define SPI_CAN1                &hspi1
#define SPI_CAN2                &hspi2
#define SPI_TIMEOUT             10
//#define MCP2515_CS_HIGH()   HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET)
//#define MCP2515_CS_LOW()    HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_RESET)

void MCP2515_CS_HIGH(SPI_HandleTypeDef *SPI_CAN)
{
	if (SPI_CAN == &hspi1)
		HAL_GPIO_WritePin(CAN1_CS_GPIO_Port, CAN1_CS_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_SET);
}
void MCP2515_CS_LOW(SPI_HandleTypeDef *SPI_CAN)
{
	if (SPI_CAN == &hspi1)
		HAL_GPIO_WritePin(CAN1_CS_GPIO_Port, CAN1_CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_RESET);
}
/* Prototypes */
static void SPI_Tx(SPI_HandleTypeDef *SPI_CAN, uint8_t data);
static void SPI_TxBuffer(SPI_HandleTypeDef *SPI_CAN, uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx(SPI_HandleTypeDef *SPI_CAN);
static void SPI_RxBuffer(SPI_HandleTypeDef *SPI_CAN, uint8_t *buffer, uint8_t length);

/* MCP2515 초기화 */
bool MCP2515_Initialize(SPI_HandleTypeDef *SPI_CAN)
{
  MCP2515_CS_HIGH(SPI_CAN);
  
  uint8_t loop = 10;
  
  do {
    /* SPI Ready 확인 */
    if(HAL_SPI_GetState(SPI_CAN) == HAL_SPI_STATE_READY)
      return true;
    
    loop--;
  } while(loop > 0); 
      
  return false;
}

/* MCP2515 를 설정모드로 전환 */
bool MCP2515_SetConfigMode(SPI_HandleTypeDef *SPI_CAN)
{
  /* CANCTRL Register Configuration 모드 설정 */  
  MCP2515_WriteByte(SPI_CAN, MCP2515_CANCTRL, 0x80);
  
  uint8_t loop = 10;
  
  do {    
    /* 모드전환 확인 */    
    if((MCP2515_ReadByte(SPI_CAN, MCP2515_CANSTAT) & 0xE0) == 0x80)
      return true;
    
    loop--;
  } while(loop > 0); 
  
  return false;
}

/* MCP2515 를 Normal모드로 전환 */
bool MCP2515_SetNormalMode(SPI_HandleTypeDef *SPI_CAN)
{
  /* CANCTRL Register Normal 모드 설정 */  
  MCP2515_WriteByte(SPI_CAN, MCP2515_CANCTRL, 0x00);
  
  uint8_t loop = 10;
  
  do {    
    /* 모드전환 확인 */    
    if((MCP2515_ReadByte(SPI_CAN, MCP2515_CANSTAT) & 0xE0) == 0x00)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP2515 를 Sleep 모드로 전환 */
bool MCP2515_SetSleepMode(SPI_HandleTypeDef *SPI_CAN)
{
  /* CANCTRL Register Sleep 모드 설정 */  
  MCP2515_WriteByte(SPI_CAN, MCP2515_CANCTRL, 0x20);
  
  uint8_t loop = 10;
  
  do {    
    /* 모드전환 확인 */    
    if((MCP2515_ReadByte(SPI_CAN, MCP2515_CANSTAT) & 0xE0) == 0x20)
      return true;
    
    loop--;
  } while(loop > 0);
  
  return false;
}

/* MCP2515 SPI-Reset */
void MCP2515_Reset(SPI_HandleTypeDef *SPI_CAN)
{    
  MCP2515_CS_LOW(SPI_CAN);
      
  SPI_Tx(SPI_CAN, MCP2515_RESET);
      
  MCP2515_CS_HIGH(SPI_CAN);
}

/* 1바이트 읽기 */
uint8_t MCP2515_ReadByte (SPI_HandleTypeDef *SPI_CAN, uint8_t address)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_READ);
  SPI_Tx(SPI_CAN, address);
  retVal = SPI_Rx(SPI_CAN);
      
  MCP2515_CS_HIGH(SPI_CAN);
  
  return retVal;
}

/* Sequential Bytes 읽기 */
void MCP2515_ReadRxSequence(SPI_HandleTypeDef *SPI_CAN, uint8_t instruction, uint8_t *data, uint8_t length)
{
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, instruction);
  SPI_RxBuffer(SPI_CAN, data, length);
    
  MCP2515_CS_HIGH(SPI_CAN);
}

/* 1바이트 쓰기 */
void MCP2515_WriteByte(SPI_HandleTypeDef *SPI_CAN, uint8_t address, uint8_t data)
{    
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_WRITE);
  SPI_Tx(SPI_CAN, address);
  SPI_Tx(SPI_CAN, data);
    
  MCP2515_CS_HIGH(SPI_CAN);
}

/* Sequential Bytes 쓰기 */
void MCP2515_WriteByteSequence(SPI_HandleTypeDef *SPI_CAN, uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{    
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_WRITE);
  SPI_Tx(SPI_CAN, startAddress);
  SPI_TxBuffer(SPI_CAN, data, (endAddress - startAddress + 1));
  
  MCP2515_CS_HIGH(SPI_CAN);
}

/* TxBuffer에 Sequential Bytes 쓰기 */
void MCP2515_LoadTxSequence(SPI_HandleTypeDef *SPI_CAN, uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, instruction);
  SPI_TxBuffer(SPI_CAN, idReg, 4);
  SPI_Tx(SPI_CAN, dlc);
  SPI_TxBuffer(SPI_CAN, data, dlc);
       
  MCP2515_CS_HIGH(SPI_CAN);
}

/* TxBuffer에 1 Bytes 쓰기 */
void MCP2515_LoadTxBuffer(SPI_HandleTypeDef *SPI_CAN, uint8_t instruction, uint8_t data)
{
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, instruction);
  SPI_Tx(SPI_CAN, data);
        
  MCP2515_CS_HIGH(SPI_CAN);
}

/* RTS 명령을 통해서 TxBuffer 전송 */
void MCP2515_RequestToSend(SPI_HandleTypeDef *SPI_CAN, uint8_t instruction)
{
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, instruction);
      
  MCP2515_CS_HIGH(SPI_CAN);
}

/* MCP2515 Status 확인 */
uint8_t MCP2515_ReadStatus(SPI_HandleTypeDef *SPI_CAN)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_READ_STATUS);
  retVal = SPI_Rx(SPI_CAN);
        
  MCP2515_CS_HIGH(SPI_CAN);
  
  return retVal;
}

/* MCP2515 RxStatus 레지스터 확인 */
uint8_t MCP2515_GetRxStatus(SPI_HandleTypeDef *SPI_CAN)
{
  uint8_t retVal;
  
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_RX_STATUS);
  retVal = SPI_Rx(SPI_CAN);
        
  MCP2515_CS_HIGH(SPI_CAN);
  
  return retVal;
}

/* 레지스터 값 변경 */
void MCP2515_BitModify(SPI_HandleTypeDef *SPI_CAN, uint8_t address, uint8_t mask, uint8_t data)
{    
  MCP2515_CS_LOW(SPI_CAN);
  
  SPI_Tx(SPI_CAN, MCP2515_BIT_MOD);
  SPI_Tx(SPI_CAN, address);
  SPI_Tx(SPI_CAN, mask);
  SPI_Tx(SPI_CAN, data);
        
  MCP2515_CS_HIGH(SPI_CAN);
}

/* SPI Tx Wrapper 함수 */
static void SPI_Tx(SPI_HandleTypeDef *SPI_CAN, uint8_t data)
{
  HAL_SPI_Transmit(SPI_CAN, &data, 1, SPI_TIMEOUT);    
}

/* SPI Tx Wrapper 함수 */
static void SPI_TxBuffer(SPI_HandleTypeDef *SPI_CAN, uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Transmit(SPI_CAN, buffer, length, SPI_TIMEOUT);    
}

/* SPI Rx Wrapper 함수 */
static uint8_t SPI_Rx(SPI_HandleTypeDef *SPI_CAN)
{
  uint8_t retVal;
  HAL_SPI_Receive(SPI_CAN, &retVal, 1, SPI_TIMEOUT);
  return retVal;
}

/* SPI Rx Wrapper 함수 */
static void SPI_RxBuffer(SPI_HandleTypeDef *SPI_CAN, uint8_t *buffer, uint8_t length)
{
  HAL_SPI_Receive(SPI_CAN, buffer, length, SPI_TIMEOUT);
}
