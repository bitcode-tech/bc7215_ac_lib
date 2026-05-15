#ifndef BC7215_GLUE_H
#define BC7215_GLUE_H

#include "main.h"
#include "bc7215_lib.h"

// 初始化胶水层，传入硬件句柄
void BC7215_Glue_Init(UART_HandleTypeDef* huart_bc7215, 
                      GPIO_TypeDef* modPort, uint16_t modPin,
                      GPIO_TypeDef* busyPort, uint16_t busyPin);

// 需要在 STM32 的 HAL_UART_RxCpltCallback 中调用此函数
void BC7215_Glue_RxCallback(void);

#endif
