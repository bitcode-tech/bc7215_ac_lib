#include "uart_ring.h"
#include <string.h>

UartRing_t          consoleRing;


void UartRing_Init(UART_HandleTypeDef* huart) {
	consoleRing.huart = huart;
	consoleRing.head = 0;
	consoleRing.tail = 0;
    // 启动接收中断
    HAL_UART_Receive_IT(huart, &consoleRing.rx_byte_tmp, 1);
}

void UartRing_Push(void) {
    uint16_t next_head = (consoleRing.head + 1) % RING_BUF_SIZE;
    if (next_head != consoleRing.tail) {
    	consoleRing.buffer[consoleRing.head] = consoleRing.rx_byte_tmp;
        consoleRing.head = next_head;
    }
    // 重新开启中断
    HAL_UART_Receive_IT(consoleRing.huart, &consoleRing.rx_byte_tmp, 1);
}

int UartRing_Available(void) {
    return (RING_BUF_SIZE + consoleRing.head - consoleRing.tail) % RING_BUF_SIZE;
}

int UartRing_Read(void) {
    if (consoleRing.head == consoleRing.tail) return -1;
    uint8_t data = consoleRing.buffer[consoleRing.tail];
    consoleRing.tail = (consoleRing.tail + 1) % RING_BUF_SIZE;
    return data;
}

void UartRing_Clear(void) {
    while(UartRing_Available()) UartRing_Read();
}
