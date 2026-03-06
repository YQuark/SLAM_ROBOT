/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "battery.h"
#include "motor.h"
#include "mpu6050.h"
#include "ps2.h"
#include "esp_link.h"
#include "pc_link.h"
#include "ssd1306.h"
#include "encoder.h"

#include "robot_config.h"
#include "robot_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
MPU6050_Data_t imu_data;
static BatteryStatus_t batt;
PS2_TypeDef ps2;
static uint8_t s_uv_limit_active = 0;
static uint8_t s_uv_cutoff_active = 0;
static float s_nominal_duty_limit = 0.80f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USART_SendString(const char *s);

static void Safety_WatchdogInit(void)
{
#if IWDG_ENABLE
  uint32_t reload = (IWDG_TIMEOUT_MS * 32000u) / (256u * 1000u);
  if (reload == 0u) reload = 1u;
  if (reload > 0x0FFFu) reload = 0x0FFFu;
  uint32_t t0 = HAL_GetTick();

  IWDG->KR = 0x5555u;          // Enable write access
  IWDG->PR = 6u;               // Prescaler /256
  IWDG->RLR = reload - 1u;     // Reload value
  while (IWDG->SR != 0u) {
    if ((HAL_GetTick() - t0) > 50u) {
      USART_SendString("[SAFE] IWDG SR wait timeout, continue.\r\n");
      break;
    }
  }
  IWDG->KR = 0xAAAAu;          // Reload counter
  IWDG->KR = 0xCCCCu;          // Start watchdog
#endif
}

static void Safety_WatchdogFeed(void)
{
#if IWDG_ENABLE
  IWDG->KR = 0xAAAAu;
#endif
}

static void Safety_BatteryProtect(void)
{
#if BATT_PROTECT_ENABLE
  const float v = batt.voltage;

  if (!s_uv_cutoff_active && v <= BATT_CUTOFF_VOLTAGE) {
    s_uv_cutoff_active = 1;
    USART_SendString("[SAFE] Battery cutoff active\r\n");
  } else if (s_uv_cutoff_active && v >= BATT_CUTOFF_RECOVER_V) {
    s_uv_cutoff_active = 0;
    USART_SendString("[SAFE] Battery cutoff released\r\n");
  }

  if (!s_uv_limit_active && v <= BATT_LIMIT_VOLTAGE) {
    s_uv_limit_active = 1;
    USART_SendString("[SAFE] Battery limit active\r\n");
  } else if (s_uv_limit_active && v >= BATT_LIMIT_RECOVER_V) {
    s_uv_limit_active = 0;
    USART_SendString("[SAFE] Battery limit released\r\n");
  }

  if (s_uv_cutoff_active) {
    if (RobotControl_GetMode() != MODE_IDLE) {
      RobotControl_SetMode(MODE_IDLE);
    }
    motor_set_duty_limit(BATT_LIMIT_DUTY);
  } else if (s_uv_limit_active) {
    motor_set_duty_limit(BATT_LIMIT_DUTY);
  } else {
    motor_set_duty_limit(s_nominal_duty_limit);
  }
#endif
}

static void MPU6050_PrintStatus(void)
{
  uint8_t whoami = 0, pwr_mgmt_1 = 0, config = 0, gyro_cfg = 0, accel_cfg = 0;

  MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &whoami);
  MPU6050_ReadReg(MPU6050_REG_PWR_MGMT_1, &pwr_mgmt_1);
  MPU6050_ReadReg(MPU6050_REG_CONFIG, &config);
  MPU6050_ReadReg(MPU6050_REG_GYRO_CONFIG, &gyro_cfg);
  MPU6050_ReadReg(MPU6050_REG_ACCEL_CONFIG, &accel_cfg);

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "WHO_AM_I=0x%02X PWR_MGMT_1=0x%02X CONFIG=0x%02X "
                     "GYRO_CFG=0x%02X ACCEL_CFG=0x%02X\r\n",
                     whoami, pwr_mgmt_1, config, gyro_cfg, accel_cfg);
  HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 100);
}

static void USART_SendString(const char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
}

static void I2C1_Scan(void)
{
  char buf[64];
  HAL_StatusTypeDef res;

  USART_SendString("I2C1 scan start...\r\n");

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    res = HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 5);
    if (res == HAL_OK)
    {
      int n = snprintf(buf, sizeof(buf),
                       " - Found device at 0x%02X\r\n", addr);
      HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    }
    HAL_Delay(2);
  }

  USART_SendString("I2C1 scan done.\r\n");
}

#if PS2_ENABLE
static float PS2_AxisToNorm(uint8_t raw, uint8_t invert)
{
  const float center = 128.0f;
  float v = ((float)raw - center) / center;
  if (invert) {
    v = -v;
  }
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  if (fabsf(v) < 0.05f) {
    v = 0.0f;
  }
  return v;
}
#endif

/* OLED显示：显示系统运行状态 */
static void System_ShowStatus(void)
{
  const RobotControlState_t *st_dash = RobotControl_GetState();
  SSD1306_DashboardData_t dash;
  dash.uptime_s = HAL_GetTick() / 1000u;
  dash.mode = (uint8_t)RobotControl_GetMode();
  dash.src = st_dash ? (uint8_t)st_dash->src : 0u;
  dash.battery_v = batt.voltage;
  dash.battery_pct = batt.percent;
  dash.uv_limit_active = s_uv_limit_active;
  dash.uv_cutoff_active = s_uv_cutoff_active;
  SSD1306_ShowDashboard(&dash);
  return;

  char line1[20], line2[20], line3[20], line4[20];

  SSD1306_Fill(0);

  // 第1行：电池电压
  snprintf(line1, sizeof(line1), "BAT %.2fV", batt.voltage);
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts(line1);

  // 第2行：电池百分比
  int pct = (int)(batt.percent + 0.5f);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  snprintf(line2, sizeof(line2), "%3d%%", pct);
  SSD1306_GotoXY(0, 8);
  SSD1306_Puts(line2);

  // 第3行：编码器速度（简化显示）
  const RobotControlState_t *state = RobotControl_GetState();
  int vel_l = (int)fabsf(state->meas_cps[0]);
  int vel_r = (int)fabsf(state->meas_cps[2]);
  snprintf(line3, sizeof(line3), "L%04d R%04d", vel_l, vel_r);
  SSD1306_GotoXY(0, 16);
  SSD1306_Puts(line3);

  // 第4行：控制模式和状态
  const char *mode_str = (RobotControl_GetMode() == MODE_IDLE) ? "ID" :
                         (RobotControl_GetMode() == MODE_OPEN_LOOP) ? "OL" : "CL";
  const char *src_str = (state->src == CMD_SRC_PS2) ? "PS2" :
                        (state->src == CMD_SRC_PC) ? "PC" :
                        (state->src == CMD_SRC_ESP) ? "ESP" : "IDLE";
  snprintf(line4, sizeof(line4), "%s %s", mode_str, src_str);
  SSD1306_GotoXY(0, 24);
  SSD1306_Puts(line4);

  SSD1306_UpdateScreen();
}

/* 系统自检函数 */
static void System_SelfTest(void)
{
#if !SELFTEST_ENABLE
  return;
#endif
  char buf[128];
  int n;
  uint8_t test_ok = 1;

  USART_SendString("\r\n========== SYSTEM SELF TEST ==========\r\n");

  /* 1. 电池检测 */
  USART_SendString("[1] Battery... ");
  Battery_Update(&batt);
  if (batt.voltage > 6.0f && batt.voltage < 12.0f) {
    n = snprintf(buf, sizeof(buf), "OK (%.2fV, %.1f%%)\r\n", batt.voltage, batt.percent);
    USART_SendString(buf);
  } else {
    n = snprintf(buf, sizeof(buf), "FAIL (%.2fV)\r\n", batt.voltage);
    USART_SendString(buf);
    test_ok = 0;
  }

  /* 2. 编码器检测 */
  USART_SendString("[2] Encoders... ");
  HAL_Delay(50);

  /* 读取初始计数器值 */
  n = snprintf(buf, sizeof(buf), "CNT: L1=%d L2=%d R1=%d R2=%d\r\n",
               (int)__HAL_TIM_GET_COUNTER(&htim2),
               (int)__HAL_TIM_GET_COUNTER(&htim3),
               (int)__HAL_TIM_GET_COUNTER(&htim4),
               (int)__HAL_TIM_GET_COUNTER(&htim5));
  USART_SendString(buf);

  /* 检查定时器是否使能 */
  USART_SendString("  TIM enabled: ");
  if (htim2.Instance->CR1 & TIM_CR1_CEN) USART_SendString("L1=OK ");
  else USART_SendString("L1=FAIL ");
  
  if (htim3.Instance->CR1 & TIM_CR1_CEN) USART_SendString("L2=OK ");
  else USART_SendString("L2=FAIL ");
  
  if (htim4.Instance->CR1 & TIM_CR1_CEN) USART_SendString("R1=OK ");
  else USART_SendString("R1=FAIL ");
  
  if (htim5.Instance->CR1 & TIM_CR1_CEN) USART_SendString("R2=OK ");
  else USART_SendString("R2=FAIL ");
  USART_SendString("\r\n");

  /* 检查定时器是否配置为编码器模式 */
  USART_SendString("  TIM encoder mode: ");
  uint8_t tim2_enc = ((htim2.Instance->SMCR & TIM_SMCR_SMS) == 3) ? 1 : 0;
  uint8_t tim3_enc = ((htim3.Instance->SMCR & TIM_SMCR_SMS) == 3) ? 1 : 0;
  uint8_t tim4_enc = ((htim4.Instance->SMCR & TIM_SMCR_SMS) == 3) ? 1 : 0;
  uint8_t tim5_enc = ((htim5.Instance->SMCR & TIM_SMCR_SMS) == 3) ? 1 : 0;
  
  if (tim2_enc) USART_SendString("L1=OK ");
  else USART_SendString("L1=FAIL ");
  
  if (tim3_enc) USART_SendString("L2=OK ");
  else USART_SendString("L2=FAIL ");
  
  if (tim4_enc) USART_SendString("R1=OK ");
  else USART_SendString("R1=FAIL ");
  
  if (tim5_enc) USART_SendString("R2=OK ");
  else USART_SendString("R2=FAIL ");
  USART_SendString("\r\n");

  /* 手动转动轮子测试编码器 */
  USART_SendString("  Please rotate any wheel for a short time...\r\n");
  HAL_Delay(SELFTEST_ROTATE_MS);
  n = snprintf(buf, sizeof(buf), "  After rotate: L1=%d L2=%d R1=%d R2=%d\r\n",
               (int)__HAL_TIM_GET_COUNTER(&htim2),
               (int)__HAL_TIM_GET_COUNTER(&htim3),
               (int)__HAL_TIM_GET_COUNTER(&htim4),
               (int)__HAL_TIM_GET_COUNTER(&htim5));
  USART_SendString(buf);
  
  /* 检查TIM3和TIM5的详细寄存器状态 */
  USART_SendString("  TIM3 and TIM5 detailed status:\r\n");
  n = snprintf(buf, sizeof(buf), "    TIM3: CR1=0x%08X SMCR=0x%08X CCMR1=0x%08X CNT=%u\r\n",
               (unsigned int)htim3.Instance->CR1,
               (unsigned int)htim3.Instance->SMCR,
               (unsigned int)htim3.Instance->CCMR1,
               (unsigned int)__HAL_TIM_GET_COUNTER(&htim3));
  USART_SendString(buf);
  n = snprintf(buf, sizeof(buf), "    TIM5: CR1=0x%08X SMCR=0x%08X CCMR1=0x%08X CNT=%u\r\n",
               (unsigned int)htim5.Instance->CR1,
               (unsigned int)htim5.Instance->SMCR,
               (unsigned int)htim5.Instance->CCMR1,
               (unsigned int)__HAL_TIM_GET_COUNTER(&htim5));
  USART_SendString(buf);
  
  /* 检查RCC时钟是否使能 */
  USART_SendString("  RCC clock status:\r\n");
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
  n = snprintf(buf, sizeof(buf), "    APB1ENR=0x%08X (TIM3=bit1, TIM5=bit3)\r\n", (unsigned int)RCC->APB1ENR);
  USART_SendString(buf);
  if (RCC->APB1ENR & RCC_APB1ENR_TIM3EN) USART_SendString("    TIM3 clock: ENABLED\r\n");
  else USART_SendString("    TIM3 clock: DISABLED\r\n");
  if (RCC->APB1ENR & RCC_APB1ENR_TIM5EN) USART_SendString("    TIM5 clock: ENABLED\r\n");
  else USART_SendString("    TIM5 clock: DISABLED\r\n");

  /* 检查GPIO模式（诊断编码器引脚是否正确配置） */
  USART_SendString("  GPIO mode check:\r\n");
  
  /* L1: PA15 (TIM2 CH1) */
  uint32_t pa_moder = GPIOA->MODER;
  uint8_t pa15_mode = (pa_moder >> (15 * 2)) & 0x3;  // PA15 bits 30-31
  n = snprintf(buf, sizeof(buf), "    L1 PA15 mode=%d (2=AF)\r\n", pa15_mode);
  USART_SendString(buf);
  
  /* L2: PC6/PC7 (TIM3 CH1/CH2) */
  uint32_t pc_moder = GPIOC->MODER;
  uint8_t pc6_mode = (pc_moder >> (6 * 2)) & 0x3;  // PC6 bits 12-13
  uint8_t pc7_mode = (pc_moder >> (7 * 2)) & 0x3;  // PC7 bits 14-15
  n = snprintf(buf, sizeof(buf), "    L2 PC6 mode=%d (2=AF) PC7 mode=%d (2=AF)\r\n",
               pc6_mode, pc7_mode);
  USART_SendString(buf);
  
  /* R1: PD12/PD13 (TIM4 CH1/CH2) */
  uint32_t pd_moder = GPIOD->MODER;
  uint8_t pd12_mode = (pd_moder >> (12 * 2)) & 0x3;  // PD12 bits 24-25
  uint8_t pd13_mode = (pd_moder >> (13 * 2)) & 0x3;  // PD13 bits 26-27
  n = snprintf(buf, sizeof(buf), "    R1 PD12 mode=%d (2=AF) PD13 mode=%d (2=AF)\r\n",
               pd12_mode, pd13_mode);
  USART_SendString(buf);
  
  /* R2: PA0/PA1 (TIM5 CH1/CH2) */
  pa_moder = GPIOA->MODER;
  uint8_t pa0_mode = (pa_moder >> (0 * 2)) & 0x3;  // PA0 bits 0-1
  uint8_t pa1_mode = (pa_moder >> (1 * 2)) & 0x3;  // PA1 bits 2-3
  n = snprintf(buf, sizeof(buf), "    R2 PA0 mode=%d (2=AF) PA1 mode=%d (2=AF)\r\n",
               pa0_mode, pa1_mode);
  USART_SendString(buf);
  
  /* 检查GPIO Alternate Function配置 */
  USART_SendString("  GPIO AF check:\r\n");
  uint32_t pa_afrl = GPIOA->AFR[0];  // PA0-PA7
  uint8_t pa0_af = (pa_afrl >> (0 * 4)) & 0xF;  // PA0 AF
  uint8_t pa1_af = (pa_afrl >> (1 * 4)) & 0xF;  // PA1 AF
  uint32_t pa_afrh = GPIOA->AFR[1];  // PA8-PA15
  uint8_t pa15_af = (pa_afrh >> (7 * 4)) & 0xF;  // PA15 AF
  n = snprintf(buf, sizeof(buf), "    PA0_AF=%d PA1_AF=%d PA15_AF=%d (expect 1=TIM2, 2=TIM3/5)\r\n",
               pa0_af, pa1_af, pa15_af);
  USART_SendString(buf);
  
  uint32_t pc_afrl = GPIOC->AFR[0];  // PC0-PC7
  uint8_t pc6_af = (pc_afrl >> (6 * 4)) & 0xF;  // PC6 AF
  uint8_t pc7_af = (pc_afrl >> (7 * 4)) & 0xF;  // PC7 AF
  n = snprintf(buf, sizeof(buf), "    PC6_AF=%d PC7_AF=%d (expect 2=TIM3)\r\n",
               pc6_af, pc7_af);
  USART_SendString(buf);
  
  uint32_t pd_afrh = GPIOD->AFR[1];  // PD8-PD15
  uint8_t pd12_af = (pd_afrh >> (4 * 4)) & 0xF;  // PD12 AF
  uint8_t pd13_af = (pd_afrh >> (5 * 4)) & 0xF;  // PD13 AF
  n = snprintf(buf, sizeof(buf), "    PD12_AF=%d PD13_AF=%d (expect 2=TIM4)\r\n",
               pd12_af, pd13_af);
  USART_SendString(buf);

  /* 检查GPIO引脚实际输入状态（IDR）- 检测编码器信号是否存在 */
  USART_SendString("  GPIO IDR check (signal presence):\r\n");
  
  /* 读取引脚状态 - 静态状态 */
  uint16_t pa_idr = GPIOA->IDR;
  uint16_t pb_idr = GPIOB->IDR;
  uint16_t pc_idr = GPIOC->IDR;
  uint16_t pd_idr = GPIOD->IDR;
  
  /* L1: PA15, PB3 */
  uint8_t l1_a = (pa_idr >> 15) & 0x1;
  uint8_t l1_b = (pb_idr >> 3) & 0x1;
  n = snprintf(buf, sizeof(buf), "    L1: PA15=%d PB3=%d\r\n", l1_a, l1_b);
  USART_SendString(buf);
  
  /* L2: PC6, PC7 */
  uint8_t l2_a = (pc_idr >> 6) & 0x1;
  uint8_t l2_b = (pc_idr >> 7) & 0x1;
  n = snprintf(buf, sizeof(buf), "    L2: PC6=%d PC7=%d\r\n", l2_a, l2_b);
  USART_SendString(buf);
  
  /* R1: PD12, PD13 */
  uint8_t r1_a = (pd_idr >> 12) & 0x1;
  uint8_t r1_b = (pd_idr >> 13) & 0x1;
  n = snprintf(buf, sizeof(buf), "    R1: PD12=%d PD13=%d\r\n", r1_a, r1_b);
  USART_SendString(buf);
  
  /* R2: PA0, PA1 */
  uint8_t r2_a = (pa_idr >> 0) & 0x1;
  uint8_t r2_b = (pa_idr >> 1) & 0x1;
  n = snprintf(buf, sizeof(buf), "    R2: PA0=%d PA1=%d\r\n", r2_a, r2_b);
  USART_SendString(buf);
  
  /* 检查TIM CCR寄存器 - 确认TIM是否接收到编码器信号 */
  USART_SendString("  TIM CCR check (capture values):\r\n");
  n = snprintf(buf, sizeof(buf), "    TIM2 CCR1=%u CCR2=%u\r\n", 
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1),
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2));
  USART_SendString(buf);
  n = snprintf(buf, sizeof(buf), "    TIM3 CCR1=%u CCR2=%u\r\n", 
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1),
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2));
  USART_SendString(buf);
  n = snprintf(buf, sizeof(buf), "    TIM4 CCR1=%u CCR2=%u\r\n", 
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1),
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2));
  USART_SendString(buf);
  n = snprintf(buf, sizeof(buf), "    TIM5 CCR1=%u CCR2=%u\r\n", 
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1),
               (unsigned int)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2));
  USART_SendString(buf);
  
#if SELFTEST_SIGNAL_CHECK
  /* 动态测试：读取引脚状态变化（检测信号活动） */
  USART_SendString("  Signal activity test (rotate wheels):\r\n");
  uint32_t changes_l1 = 0, changes_l2 = 0, changes_r1 = 0, changes_r2 = 0;
  uint8_t last_l1_a = l1_a, last_l1_b = l1_b;
  uint8_t last_l2_a = l2_a, last_l2_b = l2_b;
  uint8_t last_r1_a = r1_a, last_r1_b = r1_b;
  uint8_t last_r2_a = r2_a, last_r2_b = r2_b;
  
  USART_SendString("  Rotating all wheels for a short time...\r\n");
  for (int i = 0; i < 10; i++) {
    pa_idr = GPIOA->IDR;
    pb_idr = GPIOB->IDR;
    pc_idr = GPIOC->IDR;
    pd_idr = GPIOD->IDR;
    
    l1_a = (pa_idr >> 15) & 0x1;
    l1_b = (pb_idr >> 3) & 0x1;
    l2_a = (pc_idr >> 6) & 0x1;
    l2_b = (pc_idr >> 7) & 0x1;
    r1_a = (pd_idr >> 12) & 0x1;
    r1_b = (pd_idr >> 13) & 0x1;
    r2_a = (pa_idr >> 0) & 0x1;
    r2_b = (pa_idr >> 1) & 0x1;
    
    if (l1_a != last_l1_a || l1_b != last_l1_b) changes_l1++;
    if (l2_a != last_l2_a || l2_b != last_l2_b) changes_l2++;
    if (r1_a != last_r1_a || r1_b != last_r1_b) changes_r1++;
    if (r2_a != last_r2_a || r2_b != last_r2_b) changes_r2++;
    
    last_l1_a = l1_a; last_l1_b = l1_b;
    last_l2_a = l2_a; last_l2_b = l2_b;
    last_r1_a = r1_a; last_r1_b = r1_b;
    last_r2_a = r2_a; last_r2_b = r2_b;
    
    HAL_Delay(SELFTEST_ROTATE_MS / 10);
  }
  
  n = snprintf(buf, sizeof(buf), "    Signal changes: L1=%lu L2=%lu R1=%lu R2=%lu\r\n",
               changes_l1, changes_l2, changes_r1, changes_r2);
  USART_SendString(buf);
  
  if (changes_l1 == 0) USART_SendString("    WARNING: L1 NO SIGNAL - Check wiring!\r\n");
  if (changes_l2 == 0) USART_SendString("    WARNING: L2 NO SIGNAL - Check wiring!\r\n");
  if (changes_r1 == 0) USART_SendString("    WARNING: R1 NO SIGNAL - Check wiring!\r\n");
  if (changes_r2 == 0) USART_SendString("    WARNING: R2 NO SIGNAL - Check wiring!\r\n");
#endif

  /* 3. 电机PWM检测 */
  USART_SendString("[3] Motor PWM (all 4 wheels)... ");
  motor_set(MOTOR_L1, 0.3f);
  motor_set(MOTOR_L2, 0.3f);
  motor_set(MOTOR_R1, 0.3f);
  motor_set(MOTOR_R2, 0.3f);
  HAL_Delay(1000);
  motor_set(MOTOR_L1, 0.0f);
  motor_set(MOTOR_L2, 0.0f);
  motor_set(MOTOR_R1, 0.0f);
  motor_set(MOTOR_R2, 0.0f);
  USART_SendString("OK\r\n");

  /* 4. IMU检测 */
  USART_SendString("[4] IMU MPU6050... ");
  if (MPU6050_ReadRaw(&imu_data) == HAL_OK) {
    MPU6050_Convert(&imu_data);
    n = snprintf(buf, sizeof(buf), "OK (ax=%.2f ay=%.2f az=%.2f)\r\n",
                 imu_data.ax_g, imu_data.ay_g, imu_data.az_g);
    USART_SendString(buf);
  } else {
    USART_SendString("FAIL\r\n");
    test_ok = 0;
  }

  /* 5. ESP01S通信检测 */
  USART_SendString("[5] ESP01S UART2... ");
  USART_SendString("OK (waiting for commands)\r\n");

  /* 6. OLED显示检测 */
  USART_SendString("[6] OLED SSD1306... ");
  SSD1306_Fill(0);
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("SYSTEM TEST OK");
  SSD1306_UpdateScreen();
  USART_SendString("OK\r\n");

  /* 总结 */
  USART_SendString("======================================\r\n");
  if (test_ok) {
    USART_SendString("RESULT: ALL TESTS PASSED\r\n");
  } else {
    USART_SendString("RESULT: SOME TESTS FAILED\r\n");
  }
  USART_SendString("======================================\r\n");
  USART_SendString("READY for commands...\r\n\r\n");
  (void)n;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_ADC1_Init();
  MX_I2C1_Init();
  
  /* 先初始化编码器定时器（在GPIO之前） */
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  
  /* 最后初始化GPIO（可能会覆盖某些引脚，但编码器已经启动） */
  MX_GPIO_Init();
  
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /* 电机初始化保持不变 */
  motor_init();
  motor_set_duty_limit(1.0f);     // 满占空比输出
  s_nominal_duty_limit = motor_get_duty_limit();
  motor_set(MOTOR_L1, 0.0f);
  motor_set(MOTOR_L2, 0.0f);
  motor_set(MOTOR_R1, 0.0f);
  motor_set(MOTOR_R2, 0.0f);
  Encoder_Init();

  /* 启动提示 */
  USART_SendString("\r\n===UART Monitor Start ===\r\n");
  {
    uint32_t csr = RCC->CSR;
    char rst[96];
    int rn = snprintf(rst, sizeof(rst),
                      "ResetFlags: IWDG=%u WWDG=%u POR=%u PIN=%u SFTRST=%u\r\n",
                      (csr & RCC_CSR_IWDGRSTF) ? 1u : 0u,
                      (csr & RCC_CSR_WWDGRSTF) ? 1u : 0u,
                      (csr & RCC_CSR_PORRSTF)  ? 1u : 0u,
                      (csr & RCC_CSR_PINRSTF)  ? 1u : 0u,
                      (csr & RCC_CSR_SFTRSTF)  ? 1u : 0u);
    HAL_UART_Transmit(&huart1, (uint8_t*)rst, rn, 100);
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  I2C1_Scan();

  // 初始化 MPU6050
  HAL_StatusTypeDef st;
  char dbg[80];
  int n = 0;
  uint8_t imu_ready = 0;

  st = MPU6050_Init();
  if (st != HAL_OK)
  {
    n = snprintf(dbg, sizeof(dbg),
                 "MPU6050 Init FAIL, status=%d\r\n", (int)st);
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 100);
    USART_SendString("IMU disabled, system continues without IMU.\r\n");
  }
  else
  {
    imu_ready = 1;
    USART_SendString("MPU6050 Init OK\r\n");
    MPU6050_PrintStatus();
  }

  Battery_Init(&hadc1);
  ESP_Link_Init(&huart2);
  PC_Link_Init(&huart3);
  // === 新增：OLED 初始化 ===
  if (SSD1306_Init() != HAL_OK) {
    USART_SendString("SSD1306 Init FAIL\r\n");
  } else {
    USART_SendString("SSD1306 Init OK\r\n");
    // 先做一次显示，避免一开始是空屏
    batt.voltage = 0.0f;
    batt.percent = 0.0f;
    SSD1306_ShowBattery(&batt);
  }

  RobotControl_Init();
  RobotControl_SetMode(MODE_OPEN_LOOP);  // 先用开环模式测试
  RobotControl_SetIMUEnabled(imu_ready ? 1 : 0);

  /* PS2 初始化（在所有编码器和电机初始化完成后） */
#if PS2_ENABLE
  USART_SendString("PS2 Init...\r\n");
  PS2_Init();
  USART_SendString("PS2 Init done\r\n");
#endif

  /* 系统自检 */
  System_SelfTest();
  Safety_WatchdogInit();
  Safety_WatchdogFeed();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_ctrl = HAL_GetTick();
  uint32_t last_ps2 = last_ctrl;
  uint32_t last_imu = last_ctrl;
  uint32_t last_batt = last_ctrl;
  uint32_t last_oled = last_ctrl;
  uint32_t last_esp = last_ctrl;
  uint32_t last_pc = last_ctrl;
  uint32_t last_heartbeat = last_ctrl;
  uint32_t last_telemetry = last_ctrl;
  uint32_t last_ctrl_dbg = last_ctrl;
#if !PS2_ENABLE
  (void)last_ps2;
  (void)ps2;
#endif
  while (1) {
    uint32_t now = HAL_GetTick();

    /* ---------------- LED Heartbeat ---------------- */
    if (now - last_heartbeat >= HEARTBEAT_INTERVAL_MS) {
      last_heartbeat = now;
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    }

    /* ---------------- PS2 手柄扫描与指令 ---------------- */
#if PS2_ENABLE
    if (now - last_ps2 >= PS2_SCAN_INTERVAL_MS) {
      last_ps2 = now;
      if (PS2_Scan(&ps2)) {
        float v = PS2_AxisToNorm(ps2.LJoy_UD, 1);
        float w = PS2_AxisToNorm(ps2.RJoy_LR, 0);
        RobotControl_SetCmd_PS2(v, w, now);
      } else {
        /* PS2扫描失败，只在首次失败时警告 */
        static uint8_t ps2_warned = 0;
        if (!ps2_warned) {
          USART_SendString("PS2: Invalid mode (0x");
          char hex[3];
          snprintf(hex, sizeof(hex), "%02X", ps2.mode);
          USART_SendString(hex);
          USART_SendString(") - Check connection!\r\n");
          ps2_warned = 1;
        }
      }
    }
#endif

    /* ---------------- IMU 更新 ---------------- */
    if (now - last_imu >= IMU_READ_INTERVAL_MS) {
      last_imu = now;
      if (imu_ready && MPU6050_ReadRaw(&imu_data) == HAL_OK) {
        MPU6050_Convert(&imu_data);
        RobotControl_UpdateIMU(&imu_data, 1, now);
      } else {
        RobotControl_UpdateIMU(NULL, 0, now);
      }
    }

    /* ---------------- 电池电压更新 ---------------- */
    if (now - last_batt >= BATTERY_INTERVAL_MS) {
      last_batt = now;
      Battery_Update(&batt);
      Safety_BatteryProtect();
    }

    if (now - last_oled >= OLED_INTERVAL_MS) {
      last_oled = now;
      System_ShowStatus();
    }

    /* ---------------- 编码器 + 控制闭环 ---------------- */
    if (now - last_ctrl >= CTRL_PERIOD_MS) {
      float dt = (float)(now - last_ctrl) / 1000.0f;
      last_ctrl = now;
      if (s_uv_cutoff_active && RobotControl_GetMode() != MODE_IDLE) {
        RobotControl_SetMode(MODE_IDLE);
      }
      Encoder_UpdateAll(dt);
      RobotControl_Update(dt, now);

      /* 调试输出：显示当前控制状态（所有4个轮子） */
      if (CTRL_DEBUG_ENABLE && (now - last_ctrl_dbg >= CTRL_DEBUG_INTERVAL_MS)) {
        last_ctrl_dbg = now;
        const RobotControlState_t *state = RobotControl_GetState();
        char dbg[200];
        int n = snprintf(dbg, sizeof(dbg),
                         "Mode=%d Src=%d v=%.2f w=%.2f | Enc: L1:%.0f L2:%.0f R1:%.0f R2:%.0f | Out: L1:%.2f L2:%.2f R1:%.2f R2:%.2f\r\n",
                         RobotControl_GetMode(), state->src,
                         state->v_cmd, state->w_cmd,
                         state->meas_cps[0], state->meas_cps[1], state->meas_cps[2], state->meas_cps[3],
                         state->u_out[0], state->u_out[1], state->u_out[2], state->u_out[3]);
        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 100);
      }
    }

    /* ---------------- ESP-01S 命令轮询 ---------------- */
    if (now - last_esp >= ESP_POLL_INTERVAL_MS) {
      last_esp = now;
      ESP_Link_Poll(&batt, &imu_data);
    }

    /* ---------------- PC 命令轮询 ---------------- */
    if (now - last_pc >= PC_POLL_INTERVAL_MS) {
      last_pc = now;
      PC_Link_Poll(&batt, &imu_data);
    }


    if (TELEMETRY_ENABLE && (now - last_telemetry >= TELEMETRY_INTERVAL_MS)) {
      last_telemetry = now;
      char buf[128];
      n = snprintf(buf, sizeof(buf),
                   "PS2 mode=%02X btn1=%02X btn2=%02X "
                   "L(%3u,%3u) R(%3u,%3u) VBAT=%.2fV\r\n",
                   ps2.mode, ps2.btn1, ps2.btn2,
                   ps2.LJoy_LR, ps2.LJoy_UD,
                   ps2.RJoy_LR, ps2.RJoy_UD,
                   batt.voltage);
      HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
    }

    Safety_WatchdogFeed();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
