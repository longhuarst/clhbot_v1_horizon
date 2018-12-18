#ifndef __cy_circular_buffer_u8_h
#define __cy_circular_buffer_u8_h




#include <stdint.h>
#include <stdbool.h>




typedef struct {
	uint8_t *buffer_;//����
	int32_t wr_idx_;//����ַ
	int32_t rd_idx_;//д��ַ
	uint32_t size_;//���峤��
	uint32_t length_;//��ǰ����
}cy_circular_buffer_u8_type;




extern void cy_circular_buffer_u8_init(cy_circular_buffer_u8_type *pBuffer, uint32_t size);
extern void cy_circular_buffer_u8_delete(cy_circular_buffer_u8_type *pBuffer, uint32_t size);
extern uint32_t cy_circular_buffer_u8_length(cy_circular_buffer_u8_type *pBuffer);
extern bool cy_circular_buffer_u8_empty(cy_circular_buffer_u8_type *pBuffer);
extern bool cy_circular_buffer_u8_full(cy_circular_buffer_u8_type *pBuffer);
extern void cy_circular_buffer_u8_push_back(cy_circular_buffer_u8_type *pBuffer, uint8_t data);
extern uint8_t cy_circular_buffer_u8_pop_front(cy_circular_buffer_u8_type *pBuffer);
extern uint8_t cy_circular_buffer_u8_peek(cy_circular_buffer_u8_type *pBuffer, uint32_t pos);









#endif //__cy_circular_buffer_u8_h
