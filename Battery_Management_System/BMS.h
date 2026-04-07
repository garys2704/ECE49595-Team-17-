#ifndef MAX17055_H
#define MAX17055_H

#include "stm32f7xx_hal_conf.h"

#define MAX17055_I2C_ADDR (0x36 << 1)

#define REG_STATUS        0x00
#define REG_REPCAP        0x05
#define REG_REPSOC        0x06
#define REG_SOC           0x0D
#define REG_MIXCAP        0x0F
#define REG_FULLCAPREP    0x10
#define REG_TTE           0x11
#define REG_CYCLES        0x17
#define REG_FULLCAPNOM    0x23
#define REG_RCOMP0        0x38
#define REG_TEMPCO        0x39
#define REG_FSTAT         0x3D
#define REG_DQACC         0x45
#define REG_DPACC         0x46
#define REG_COMMAND       0x60
#define REG_HIBCFG        0xBA

HAL_StatusTypeDef MAX17055_Init(I2C_HandleTypeDef *h12c);

#endif
