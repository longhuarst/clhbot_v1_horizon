#ifndef __cy_uart_h
#define __cy_uart_h




#include <stdint.h>
#include <stdbool.h>


#include "../buffer/cy_circular_buffer_u8.h"






typedef struct {

	uint8_t rx_data;
	cy_circular_buffer_u8_type rx_rb;
}cy_uart_type;









extern cy_uart_type cy_uart_;











void cy_uart_init(void);
void cy_uart_polling(void);



uint8_t uart1_read(uint8_t *buffer,uint8_t size);




#endif //__cy_uart_h
