//
// Created by yushu on 2025/11/23.
//
#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "stm32f4xx_hal.h"
#include "battery.h"

#ifdef __cplusplus
extern "C" {
#endif

// 0.96" OLED dimensions
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

// I2C handle (defined in i2c.c / main.c)
extern I2C_HandleTypeDef hi2c1;
#define SSD1306_I2C      (&hi2c1)
#define SSD1306_I2C_ADDR (0x3Cu << 1)

// Public APIs
HAL_StatusTypeDef SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);           // 0: black, non-zero: white
void SSD1306_UpdateScreen(void);
void SSD1306_GotoXY(uint8_t x, uint8_t y);  // y must be multiple of 8
void SSD1306_Puts(const char *s);

// Battery-only helper: show voltage and percentage
void SSD1306_ShowBattery(const BatteryStatus_t *batt);

typedef struct {
    uint32_t uptime_s;
    uint8_t mode;
    uint8_t src;
    float battery_v;
    float battery_pct;
    uint8_t uv_limit_active;
    uint8_t uv_cutoff_active;
} SSD1306_DashboardData_t;

// 0.96" OLED single-page dashboard
void SSD1306_ShowDashboard(const SSD1306_DashboardData_t *d);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */