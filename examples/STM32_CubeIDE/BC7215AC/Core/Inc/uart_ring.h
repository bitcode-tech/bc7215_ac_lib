#ifndef UART_RING_H
#define UART_RING_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RING_BUF_SIZE 16

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t buffer[RING_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t rx_byte_tmp; // 用于HAL中断接收单个字节
} UartRing_t;

// 初始化
void UartRing_Init(UART_HandleTypeDef* huart);
// 放入数据 (在中断回调中调用)
void UartRing_Push(void);
// 检查是否有数据
int UartRing_Available(void);
// 读取一个字节
int UartRing_Read(void);
// 清空缓冲区
void UartRing_Clear(void);

#endif
