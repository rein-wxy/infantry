#ifndef _BSP_USART_H
#define _BSP_USART_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Remote_Control.h"
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern uint8_t remote_control_lever_buff[100];

void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void uart_rx_idle_callback(UART_HandleTypeDef* huart);
void uart_receive_handler(UART_HandleTypeDef *huart);
void u6_uart_init(void);

#endif
