#include "cy_uart.h"


#include "usart.h"
cy_uart_type cy_uart_;











void cy_uart_init(void)
{

	
	cy_circular_buffer_u8_init(&cy_uart_.rx_rb,128);


	HAL_UART_Receive_DMA(&huart1,&cy_uart_.rx_data,1);



}









void cy_uart_polling(void)
{

	







}




uint8_t uart1_read(uint8_t *buffer,uint8_t size)
{
	if (buffer == 0 || size == 0)
		return 0;
	
	if (cy_circular_buffer_u8_empty(&cy_uart_.rx_rb)){
		return 0;
	}
	
	uint8_t size_ = cy_circular_buffer_u8_length(&cy_uart_.rx_rb);
	
	if (size > size_){
		size = size_;
	}
	
	for (int i=0;i<size;++i){
		buffer[i] = cy_circular_buffer_u8_pop_front(&cy_uart_.rx_rb);
	}
	return size;
	
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
		cy_circular_buffer_u8_push_back(&cy_uart_.rx_rb,cy_uart_.rx_data);
	}
}



















