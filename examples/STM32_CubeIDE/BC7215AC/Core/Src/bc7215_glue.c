#include "bc7215_glue.h"

// 静态变量保存硬件句柄
static UART_HandleTypeDef* g_huart_bc7215;
static GPIO_TypeDef*       g_modPort;
static uint16_t            g_modPin;
static GPIO_TypeDef*       g_busyPort;
static uint16_t            g_busyPin;

// 用于 HAL 中断接收的单字节缓冲
static uint8_t rx_byte;

/* --- 回调函数实现 --- */

// 1. 开启 UART 中断
static void Glue_UartIntEnable(void)
{
    // STM32 HAL 开启接收中断
    HAL_UART_Receive_IT(g_huart_bc7215, &rx_byte, 1);
}

// 2. 关闭 UART 中断 
static void Glue_UartIntDisable(void)
{
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);        // ******************** !!! 此处使用 LPUART1_IRQn，移植时需根据实际情况修改 IRQn
}

// 3. 重新开启 UART 中断
static void Glue_UartIntRestore(void) { HAL_NVIC_EnableIRQ(LPUART1_IRQn); }

// 4. 发送一个字节 (阻塞式)
static void Glue_UartSendByte(uint8_t data) { HAL_UART_Transmit(g_huart_bc7215, &data, 1, 100); }

// 5. 设置 MOD 高
static void Glue_ModSetHigh(void) { HAL_GPIO_WritePin(g_modPort, g_modPin, GPIO_PIN_SET); }

// 6. 设置 MOD 低
static void Glue_ModSetLow(void) { HAL_GPIO_WritePin(g_modPort, g_modPin, GPIO_PIN_RESET); }

// 7. 读取 MOD 状态
static uint8_t Glue_ReadMod(void) { return (HAL_GPIO_ReadPin(g_modPort, g_modPin) == GPIO_PIN_SET) ? 1 : 0; }

// 8. 读取 BUSY 状态
static uint8_t Glue_ReadBusy(void) { return (HAL_GPIO_ReadPin(g_busyPort, g_busyPin) == GPIO_PIN_SET) ? 1 : 0; }

/* --- 初始化与中断处理 --- */

void BC7215_Glue_Init(
    UART_HandleTypeDef* huart, GPIO_TypeDef* modPort, uint16_t modPin, GPIO_TypeDef* busyPort, uint16_t busyPin)
{
    g_huart_bc7215 = huart;
    g_modPort = modPort;
    g_modPin = modPin;
    g_busyPort = busyPort;
    g_busyPin = busyPin;

    // 注册回调函数到 BC7215 库
    bc7215_config_uart_int_en_funct(Glue_UartIntRestore);        // 使用 NVIC 控制更安全
    bc7215_config_uart_int_dis_funct(Glue_UartIntDisable);
    bc7215_config_uart_send_byte(Glue_UartSendByte);
    bc7215_config_mod_set_high_funct(Glue_ModSetHigh);
    bc7215_config_mod_set_low_funct(Glue_ModSetLow);
    bc7215_config_read_mod_funct(Glue_ReadMod);
    bc7215_config_read_busy_funct(Glue_ReadBusy);

    // 启动第一次接收
    Glue_UartIntEnable();
}

// 放在 stm32fxxx_it.c 或 main.c 的 HAL_UART_RxCpltCallback 中
void BC7215_Glue_RxCallback()
{
	// 将接收到的字节送入库处理
	bc7215_process_uart_data(rx_byte);
	// 重新开启中断接收下一个字节
	HAL_UART_Receive_IT(g_huart_bc7215, &rx_byte, 1);
}
