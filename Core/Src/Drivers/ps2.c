#include "ps2.h"

/*-----------------------------------------
   微秒延时：使用 DWT 周期计数器
------------------------------------------*/
static void DWT_DelayInit(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t clk = SystemCoreClock / 1000000;
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < us * clk);
}

/*-----------------------------------------
   引脚操作定义
------------------------------------------*/
#define PS2_DI()   (HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin)==GPIO_PIN_SET)

#define PS2_CMD_HIGH() HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET)
#define PS2_CMD_LOW()  HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET)

#define PS2_CS_HIGH()  HAL_GPIO_WritePin(PS2_CS_GPIO_Port,  PS2_CS_Pin, GPIO_PIN_SET)
#define PS2_CS_LOW()   HAL_GPIO_WritePin(PS2_CS_GPIO_Port,  PS2_CS_Pin, GPIO_PIN_RESET)

#define PS2_CLK_HIGH() HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define PS2_CLK_LOW()  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)

/* PS2 指令 */
static const uint8_t PS2_cmd[9] = {0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static uint8_t PS2_buf[9] = {0};

static uint8_t PS2_RW(uint8_t tx);
static void PS2_Send(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    PS2_CS_LOW();
    delay_us(20);

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t val = PS2_RW(tx[i]);
        if (rx)
        {
            rx[i] = val;
        }
        delay_us(16);
    }

    PS2_CS_HIGH();
    delay_us(20);
}

void PS2_ConfigAnalog(void)
{
    static const uint8_t enter_cfg[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    static const uint8_t set_analog[] = {0x01, 0x44, 0x00, 0x01, 0x03};
    static const uint8_t enable_motor[] = {0x01, 0x4D, 0x00, 0x00, 0x01};
    static const uint8_t exit_cfg[] = {0x01, 0x43, 0x00, 0x00, 0x5A};
    
    /* 增加延时，确保PS2手柄有足够时间响应 */
    PS2_Send(enter_cfg, NULL, sizeof(enter_cfg));
    HAL_Delay(10);  /* 从1ms增加到10ms */
    
    PS2_Send(set_analog, NULL, sizeof(set_analog));
    HAL_Delay(10);  /* 从1ms增加到10ms */
    
    PS2_Send(enable_motor, NULL, sizeof(enable_motor));
    HAL_Delay(10);  /* 从1ms增加到10ms */
    
    PS2_Send(exit_cfg, NULL, sizeof(exit_cfg));
    HAL_Delay(10);  /* 从1ms增加到10ms */
}

static uint8_t PS2_ModeValid(uint8_t mode)
{
    return (mode == 0x41u) || (mode == 0x73u) || (mode == 0x79u);
}
/*-----------------------------------------
      初始化（DATA 输入，CMD/CLK/CS 输出）
------------------------------------------*/
void PS2_Init(void)
{
    DWT_DelayInit();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* DATA 输入 */
    GPIO_InitStruct.Pin = PS2_DAT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(PS2_DAT_GPIO_Port, &GPIO_InitStruct);

    /* CMD / CLK / CS 输出 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = PS2_CMD_Pin;
    HAL_GPIO_Init(PS2_CMD_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PS2_CLK_Pin;
    HAL_GPIO_Init(PS2_CLK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PS2_CS_Pin;
    HAL_GPIO_Init(PS2_CS_GPIO_Port, &GPIO_InitStruct);

    /* 默认状态 */
    PS2_CMD_HIGH();
    PS2_CLK_HIGH();
    PS2_CS_HIGH();
    /* 进入模拟模式，避免只读到 0xFF */
    PS2_ConfigAnalog();
}


/*-----------------------------------------
   PS2 单字节读写（时序精确）
------------------------------------------*/
static uint8_t PS2_RW(uint8_t tx)
{
    uint8_t ref;
    uint8_t rx = 0;

    for (ref = 0x01; ref != 0; ref <<= 1)
    {
        // 1. 先准备好要发送的数据 (CMD)
        if (tx & ref) PS2_CMD_HIGH();
        else          PS2_CMD_LOW();

        // 给一点点建立时间（可选，为了稳健）
        // delay_us(1);

        // 2. 拉低时钟：通知接收器“我要开始传输这一位了”
        PS2_CLK_LOW();

        // 3. 延时：等待数据稳定
        // 手册说时钟周期约 30-50us，半周期给 10-15us 足够
        delay_us(10);

        // 4. 【关键修正】在时钟低电平期间读取 DAT
        // 此时数据是最稳定的
        if (PS2_DI())
        {
            rx |= ref;
        }

        // 5. 拉高时钟：结束这一位
        PS2_CLK_HIGH();

        // 6. 延时：保持高电平一段时间
        delay_us(10);
    }

    PS2_CMD_HIGH(); // 释放 CMD
    return rx;
}

/*-----------------------------------------
   主扫描函数（外部直接调用）
------------------------------------------*/
static uint8_t reconfig_count = 0;
static uint8_t first_invalid = 1;

uint8_t PS2_Scan(PS2_TypeDef *ps2)
{
    PS2_Send(PS2_cmd, PS2_buf, sizeof(PS2_cmd));

    ps2->mode     = PS2_buf[1];
    ps2->btn1     = ~PS2_buf[3];
    ps2->btn2     = ~PS2_buf[4];
    ps2->RJoy_LR  = PS2_buf[5];
    ps2->RJoy_UD  = PS2_buf[6];
    ps2->LJoy_LR  = PS2_buf[7];
    ps2->LJoy_UD  = PS2_buf[8];

    if (!PS2_ModeValid(ps2->mode)) {
        /* 只在前3次检测到无效模式时尝试重新配置 */
        if (reconfig_count < 3) {
            PS2_ConfigAnalog();
            reconfig_count++;
        }
        return 0;
    }

    /* 重置配置计数器 */
    reconfig_count = 0;
    first_invalid = 1;
    return 1;
}