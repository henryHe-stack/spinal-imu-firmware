#include "neck_uart_rx.h"
#include "main.h"
static uint8_t rx_byte,ring[128];
static volatile uint32_t head,tail,fault;
int neck_uart_rx_start(void)
{
    head=tail=fault=0;
    HAL_NVIC_SetPriority(USART2_IRQn,7,0); HAL_NVIC_EnableIRQ(USART2_IRQn);
    return HAL_UART_Receive_IT(&huart2,&rx_byte,1)==HAL_OK;
}
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint32_t next;
    if(uart!=&huart2) return;
    next=(head+1)&127u;
    if(next==tail) fault=1;
    else { ring[head]=rx_byte; __DMB(); head=next; }
    if(HAL_UART_Receive_IT(&huart2,&rx_byte,1)!=HAL_OK) fault=1;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if(uart!=&huart2) return;
    fault=1;
    (void)HAL_UART_Receive_IT(&huart2,&rx_byte,1);
}
int neck_uart_rx_pop(uint8_t *byte)
{
    uint32_t mask;
    if(!byte) return 0;
    mask=__get_PRIMASK(); __disable_irq();
    if(fault) { tail=head; fault=0; __set_PRIMASK(mask); return -1; }
    if(tail==head) { __set_PRIMASK(mask); return 0; }
    *byte=ring[tail]; tail=(tail+1)&127u; __set_PRIMASK(mask); return 1;
}
