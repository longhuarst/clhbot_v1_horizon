#include "cy_can.h"

#include "can.h"

#include <string.h>


cy_can_type cy_can_[CY_CAN_MAX];





static CanRxMsgTypeDef        RxMessage;
static CanTxMsgTypeDef        TRxMessage;

CAN_FilterConfTypeDef  sFilterConfig;




#define mBYTE0(dwTemp)       (*(char *)(&dwTemp))
#define mBYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define mBYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define mBYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))



void cy_can_init(void)
{

	hcan.pRxMsg = &RxMessage;
	hcan.pTxMsg = &TRxMessage;


	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;


	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}


	if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	
	
	memset(&cy_can_[0],0,sizeof(cy_can_[0]));
	memset(&cy_can_[1],0,sizeof(cy_can_[1]));
	memset(&cy_can_[2],0,sizeof(cy_can_[2]));
	memset(&cy_can_[3],0,sizeof(cy_can_[3]));
	memset(&cy_can_[4],0,sizeof(cy_can_[4]));
	memset(&cy_can_[5],0,sizeof(cy_can_[5]));




}






void cy_can_wait_for_sensor_init_ok(bool f1,bool f2,bool f3,bool f4,bool f5,bool f6)
{
	if (f1){
		while(cy_can_[0].updated == false);
	}
	
	if (f2){
		while(cy_can_[1].updated == false);
	}
	
	if (f3){
		while(cy_can_[2].updated == false);
	}
	
	if (f4){
		while(cy_can_[3].updated == false);
	}
	
	if (f5){
		while(cy_can_[4].updated == false);
	}
	
	if (f6){
		while(cy_can_[5].updated == false);
	}
}




void cy_can_polling(void)
{







	if (hcan.State == HAL_CAN_STATE_READY){
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	}
	  
}


void cy_can_resend(void)
{
	if (hcan.State == HAL_CAN_STATE_READY){
		HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	}
}
	









void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle){

	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
		__asm("nop");
	
	if (RxMessage.ExtId == 0x1310 || RxMessage.ExtId == 0x1311 || RxMessage.ExtId == 0x1312 || RxMessage.ExtId == 0x1313){
		uint8_t val = RxMessage.ExtId %10;
		if (val >= 4)
			return;

		
		
		cy_can_[val].updated = true;
		mBYTE0(cy_can_[val].x) = RxMessage.Data[0];
		mBYTE1(cy_can_[val].x) = RxMessage.Data[1];
		mBYTE2(cy_can_[val].x) = RxMessage.Data[2];
		mBYTE3(cy_can_[val].x) = RxMessage.Data[3];
		mBYTE0(cy_can_[val].y) = RxMessage.Data[4];
		mBYTE1(cy_can_[val].y) = RxMessage.Data[5];
		mBYTE2(cy_can_[val].y) = RxMessage.Data[6];
		mBYTE3(cy_can_[val].y) = RxMessage.Data[7];
	}
	
}


