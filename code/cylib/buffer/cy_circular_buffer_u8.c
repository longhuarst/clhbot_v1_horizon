#include "cy_circular_buffer_u8.h"


#include <stdio.h>

#include <stdlib.h>





//初始化 循环缓冲区
void cy_circular_buffer_u8_init(cy_circular_buffer_u8_type *pBuffer, uint32_t size)
{
	if (pBuffer == NULL){
		return;
	}
	
	pBuffer->buffer_ = (uint8_t *)malloc(sizeof(uint8_t) * size);
	pBuffer->length_ = 0;
	pBuffer->wr_idx_ = 0;
	pBuffer->rd_idx_ = 0;
	pBuffer->size_ = size -1;
}



void cy_circular_buffer_u8_delete(cy_circular_buffer_u8_type *pBuffer, uint32_t size)
{
	if (pBuffer == NULL){
		return;
	}
	
	if (pBuffer->buffer_ == NULL){
		return;
	}
	
	free(pBuffer->buffer_);
}





uint32_t cy_circular_buffer_u8_length(cy_circular_buffer_u8_type *pBuffer)
{	
	if (pBuffer == NULL)
		return 0;
	
	return ((pBuffer->wr_idx_ - pBuffer->rd_idx_) + (pBuffer->size_+1)) % (pBuffer->size_+1);
}


bool cy_circular_buffer_u8_empty(cy_circular_buffer_u8_type *pBuffer)
{
	if (pBuffer == NULL)
		return 0;
	
	return pBuffer->wr_idx_ == pBuffer->rd_idx_;
}


bool cy_circular_buffer_u8_full(cy_circular_buffer_u8_type *pBuffer)
{
	if (pBuffer == NULL)
		return 0;
	
	return cy_circular_buffer_u8_length(pBuffer) == (pBuffer->size_+1);
}





void cy_circular_buffer_u8_push_back(cy_circular_buffer_u8_type *pBuffer, uint8_t data)
{
	
	if (pBuffer == NULL)
		return;
	
	if (cy_circular_buffer_u8_full(pBuffer)){
		pBuffer->rd_idx_++;
		pBuffer->rd_idx_ %= (pBuffer->size_+1);
		pBuffer->length_--;
	}

	pBuffer->buffer_[pBuffer->wr_idx_] = data;
	
	pBuffer->wr_idx_++;
	pBuffer->wr_idx_ %= (pBuffer->size_+1);
	
	pBuffer->length_++;
}





uint8_t cy_circular_buffer_u8_pop_front(cy_circular_buffer_u8_type *pBuffer)
{
	
	
	if (pBuffer == NULL)
		return 0;
	
	if (cy_circular_buffer_u8_empty(pBuffer)){
		return 0;
	}

	uint8_t data = pBuffer->buffer_[pBuffer->rd_idx_];
	
	pBuffer->rd_idx_++;
	pBuffer->rd_idx_ %= (pBuffer->size_+1);
	pBuffer->length_--;

	return data;

}



uint8_t cy_circular_buffer_u8_peek(cy_circular_buffer_u8_type *pBuffer, uint32_t pos)
{
	if (pBuffer == NULL)
		return 0;
	
	return pBuffer->buffer_[(pBuffer->rd_idx_+pos)%(pBuffer->size_+1)];
}

















