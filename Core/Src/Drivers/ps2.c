#include "ps2.h"

#define PS2_FRAME_LEN   9u
#define PS2_RETRY_LIMIT 3u

/* Use DWT cycle counter for stable microsecond delays. */
static void DWT_DelayInit(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    const uint32_t clk_mhz = SystemCoreClock / 1000000u;
    const uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < (us * clk_mhz)) {
    }
}

/* CMD/DAT can be auto-swapped in software for boards with reversed wiring. */
static uint8_t s_swap_cmd_dat = 0u;

static inline GPIO_TypeDef *ps2_cmd_port(void)
{
    return s_swap_cmd_dat ? PS2_DAT_GPIO_Port : PS2_CMD_GPIO_Port;
}

static inline uint16_t ps2_cmd_pin(void)
{
    return s_swap_cmd_dat ? PS2_DAT_Pin : PS2_CMD_Pin;
}

static inline GPIO_TypeDef *ps2_dat_port(void)
{
    return s_swap_cmd_dat ? PS2_CMD_GPIO_Port : PS2_DAT_GPIO_Port;
}

static inline uint16_t ps2_dat_pin(void)
{
    return s_swap_cmd_dat ? PS2_CMD_Pin : PS2_DAT_Pin;
}

static inline uint8_t PS2_DI(void)
{
    return HAL_GPIO_ReadPin(ps2_dat_port(), ps2_dat_pin()) == GPIO_PIN_SET;
}

static inline void PS2_CMD_HIGH(void)
{
    HAL_GPIO_WritePin(ps2_cmd_port(), ps2_cmd_pin(), GPIO_PIN_SET);
}

static inline void PS2_CMD_LOW(void)
{
    HAL_GPIO_WritePin(ps2_cmd_port(), ps2_cmd_pin(), GPIO_PIN_RESET);
}

static inline void PS2_CS_HIGH(void)
{
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET);
}

static inline void PS2_CS_LOW(void)
{
    HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET);
}

static inline void PS2_CLK_HIGH(void)
{
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET);
}

static inline void PS2_CLK_LOW(void)
{
    HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET);
}

static void PS2_ConfigurePins(void)
{
    GPIO_InitTypeDef init = {0};

    init.Pin = ps2_dat_pin();
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ps2_dat_port(), &init);

    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_PULLUP;
    init.Speed = GPIO_SPEED_FREQ_HIGH;
    init.Pin = ps2_cmd_pin();
    HAL_GPIO_Init(ps2_cmd_port(), &init);

    init.Pin = PS2_CLK_Pin;
    HAL_GPIO_Init(PS2_CLK_GPIO_Port, &init);

    init.Pin = PS2_CS_Pin;
    HAL_GPIO_Init(PS2_CS_GPIO_Port, &init);

    PS2_CMD_HIGH();
    PS2_CLK_HIGH();
    PS2_CS_HIGH();
}

static const uint8_t k_poll_frame[PS2_FRAME_LEN] = {
    0x01u, 0x42u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
};
static uint8_t s_rx_buf[PS2_FRAME_LEN] = {0};

static uint8_t PS2_RW(uint8_t tx);

static void PS2_Send(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    PS2_CS_LOW();
    delay_us(20u);

    for (uint8_t i = 0u; i < len; ++i) {
        uint8_t v = PS2_RW(tx[i]);
        if (rx != NULL) {
            rx[i] = v;
        }
        delay_us(16u);
    }

    PS2_CS_HIGH();
    delay_us(20u);
}

static uint8_t PS2_HandshakeOK(const uint8_t *rx)
{
    return (rx[1] != 0xFFu) && (rx[2] == 0x5Au);
}

static uint8_t PS2_ModeIsAnalog(uint8_t mode)
{
    return (mode == 0x73u) || (mode == 0x79u);
}

static uint8_t PS2_ModeValid(uint8_t mode)
{
    return (mode == 0x41u) || PS2_ModeIsAnalog(mode);
}

static void PS2_ShortPoll(void)
{
    static const uint8_t short_poll[5] = {0x01u, 0x42u, 0x00u, 0x00u, 0x00u};
    PS2_Send(short_poll, NULL, (uint8_t)sizeof(short_poll));
}

/* Follow WHEELTEC sequence: short poll x3 -> enter cfg -> analog -> vibration -> exit cfg. */
static void PS2_ConfigAnalog(void)
{
    static const uint8_t enter_cfg[PS2_FRAME_LEN] = {
        0x01u, 0x43u, 0x00u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u
    };
    static const uint8_t set_analog[PS2_FRAME_LEN] = {
        0x01u, 0x44u, 0x00u, 0x01u, 0x03u, 0x00u, 0x00u, 0x00u, 0x00u
    };
    static const uint8_t enable_motor[5] = {0x01u, 0x4Du, 0x00u, 0x00u, 0x01u};
    static const uint8_t exit_cfg[PS2_FRAME_LEN] = {
        0x01u, 0x43u, 0x00u, 0x00u, 0x5Au, 0x5Au, 0x5Au, 0x5Au, 0x5Au
    };

    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    HAL_Delay(2u);

    PS2_Send(enter_cfg, NULL, (uint8_t)sizeof(enter_cfg));
    HAL_Delay(2u);

    PS2_Send(set_analog, NULL, (uint8_t)sizeof(set_analog));
    HAL_Delay(2u);

    PS2_Send(enable_motor, NULL, (uint8_t)sizeof(enable_motor));
    HAL_Delay(2u);

    PS2_Send(exit_cfg, NULL, (uint8_t)sizeof(exit_cfg));
    HAL_Delay(2u);
}

static uint8_t PS2_ProbeLink(uint8_t require_analog)
{
    uint8_t rx[PS2_FRAME_LEN] = {0};
    for (uint8_t i = 0u; i < 6u; ++i) {
        PS2_Send(k_poll_frame, rx, (uint8_t)sizeof(rx));
        if (PS2_HandshakeOK(rx) && (!require_analog || PS2_ModeIsAnalog(rx[1]))) {
            return 1u;
        }
        HAL_Delay(2u);
    }
    return 0u;
}

static uint8_t PS2_InitOneMapping(void)
{
    for (uint8_t i = 0u; i < PS2_RETRY_LIMIT; ++i) {
        PS2_ConfigAnalog();
        if (PS2_ProbeLink(1u)) {
            return 1u;
        }
    }
    return PS2_ProbeLink(0u);
}

void PS2_Init(void)
{
    DWT_DelayInit();

    s_swap_cmd_dat = 0u;
    PS2_ConfigurePins();
    if (PS2_InitOneMapping()) {
        return;
    }

    s_swap_cmd_dat = 1u;
    PS2_ConfigurePins();
    (void)PS2_InitOneMapping();
}

/* Single-byte transfer, matching the timing style in WHEELTEC PSTWO demo. */
static uint8_t PS2_RW(uint8_t tx)
{
    uint8_t rx = 0u;
    for (uint8_t ref = 0x01u; ref != 0u; ref <<= 1u) {
        if ((tx & ref) != 0u) {
            PS2_CMD_HIGH();
        } else {
            PS2_CMD_LOW();
        }

        PS2_CLK_HIGH();
        delay_us(5u);
        PS2_CLK_LOW();
        delay_us(5u);
        PS2_CLK_HIGH();

        if (PS2_DI()) {
            rx |= ref;
        }
    }

    delay_us(16u);
    PS2_CMD_HIGH();
    return rx;
}

static uint8_t s_reconfig_count = 0u;

uint8_t PS2_Scan(PS2_TypeDef *ps2)
{
    if (ps2 == NULL) {
        return 0u;
    }

    PS2_Send(k_poll_frame, s_rx_buf, (uint8_t)sizeof(k_poll_frame));

    ps2->mode = s_rx_buf[1];
    ps2->btn1 = (uint8_t)~s_rx_buf[3];
    ps2->btn2 = (uint8_t)~s_rx_buf[4];
    ps2->RJoy_LR = s_rx_buf[5];
    ps2->RJoy_UD = s_rx_buf[6];
    ps2->LJoy_LR = s_rx_buf[7];
    ps2->LJoy_UD = s_rx_buf[8];

    if (!PS2_HandshakeOK(s_rx_buf)) {
        if (s_reconfig_count < PS2_RETRY_LIMIT) {
            PS2_ConfigAnalog();
            ++s_reconfig_count;
        }
        return 0u;
    }

    if (!PS2_ModeValid(ps2->mode)) {
        if (s_reconfig_count < PS2_RETRY_LIMIT) {
            PS2_ConfigAnalog();
            ++s_reconfig_count;
        }
        return 1u;
    }

    if (!PS2_ModeIsAnalog(ps2->mode)) {
        if (s_reconfig_count < PS2_RETRY_LIMIT) {
            PS2_ConfigAnalog();
            ++s_reconfig_count;
        }
    } else {
        s_reconfig_count = 0u;
    }

    return 1u;
}

uint8_t PS2_IsCmdDatSwapped(void)
{
    return s_swap_cmd_dat;
}
