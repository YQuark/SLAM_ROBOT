#ifndef __PS2_H
#define __PS2_H

#include "main.h"
#include <stdint.h>

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

#define PS2_MODE_DIGITAL           0x41u
#define PS2_MODE_ANALOG_RED        0x73u
#define PS2_MODE_ANALOG_PRESSURE   0x79u

/* btn1 bits */
#define PS2_BTN_SELECT   0x01u
#define PS2_BTN_L3       0x02u
#define PS2_BTN_R3       0x04u
#define PS2_BTN_START    0x08u
#define PS2_BTN_UP       0x10u
#define PS2_BTN_RIGHT    0x20u
#define PS2_BTN_DOWN     0x40u
#define PS2_BTN_LEFT     0x80u

/* btn2 bits */
#define PS2_BTN_L2       0x01u
#define PS2_BTN_R2       0x02u
#define PS2_BTN_L1       0x04u
#define PS2_BTN_R1       0x08u
#define PS2_BTN_TRI      0x10u
#define PS2_BTN_CIR      0x20u
#define PS2_BTN_CROSS    0x40u
#define PS2_BTN_SQUARE   0x80u

void PS2_Init(void);
uint8_t PS2_Scan(PS2_TypeDef *ps2);

/* 1 means CMD/DAT was auto-swapped during init; 0 means normal mapping. */
uint8_t PS2_IsCmdDatSwapped(void);

#endif
