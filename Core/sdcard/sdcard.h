/* vim: set ai et ts=4 sw=4: */
#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "stm32f1xx_hal.h"

#define SDCARD_SPI_PORT      hspi2
#define SDCARD_CS_Pin        GPIO_PIN_14 // Arduino shield: D4
#define SDCARD_CS_GPIO_Port  GPIOG

extern SPI_HandleTypeDef SDCARD_SPI_PORT;

#define SDCARD_TYPE_ERR     ((uint8_t)0X00)
#define SDCARD_TYPE_MMC     ((uint8_t)0X01)
#define SDCARD_TYPE_V1      ((uint8_t)0X02)
#define SDCARD_TYPE_V2      ((uint8_t)0X04)
#define SDCARD_TYPE_V2HC    ((uint8_t)0X06)

// call before initializing any SPI devices
void SDCARD_Unselect();

// all procedures return 0 on success, < 0 on failure

int SDCARD_Init();
int SDCARD_GetBlocksNumber(uint32_t* num);
int SDCARD_ReadSingleBlock(uint32_t blockNum, uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_WriteSingleBlock(uint32_t blockNum, const uint8_t* buff); // sizeof(buff) == 512!

// Read Multiple Blocks
int SDCARD_ReadBegin(uint32_t blockNum);
int SDCARD_ReadData(uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_ReadEnd();

// Write Multiple Blocks
int SDCARD_WriteBegin(uint32_t blockNum);
int SDCARD_WriteData(const uint8_t* buff); // sizeof(buff) == 512!
int SDCARD_WriteEnd();

// TODO: read lock flag? CMD13, SEND_STATUS

#endif // __SDCARD_H__
