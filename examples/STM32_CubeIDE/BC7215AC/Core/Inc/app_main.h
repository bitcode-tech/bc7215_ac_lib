#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "main.h" // 包含 HAL
#include "bc7215_glue.h"
#include "uart_ring.h"

typedef struct {
	UART_HandleTypeDef* consolePort;
	UART_HandleTypeDef* bc7215Port;
	GPIO_TypeDef*		modPort;
	GPIO_TypeDef*		busyPort;
	uint16_t			modPin;
	uint16_t			busyPin;
} appConfig_t;

extern appConfig_t	appConfig;

// 供 main.c 调用
void App_Init(appConfig_t* hwConfig);
void App_Loop(void);

#endif
