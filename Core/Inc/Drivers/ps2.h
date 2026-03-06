#ifndef __PS2_H
#define __PS2_H

#include "main.h"
#include <stdint.h>

/*-----------------------------------------
  手柄数据结构体
------------------------------------------*/
typedef struct
{
    uint8_t mode;
    uint8_t btn1;
    uint8_t btn2;
    uint8_t RJoy_LR;
    uint8_t RJoy_UD;
    uint8_t LJoy_LR;
    uint8_t LJoy_UD;
} PS2_TypeDef;

/* 初始化、扫描 */
void PS2_Init(void);
uint8_t PS2_Scan(PS2_TypeDef *ps2);

#endif