#include "cy_timer.h"

#include "tim.h"

#include "can.h"


cy_timer_type cy_timer_;












void cy_timer_init(void)
{







}









void cy_timer_polling(void)
{









}















__weak void cy_timer_tim1_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim2_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim3_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim4_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim5_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim6_PeriodElapsedCallback(void)
{
	
}


__weak void cy_timer_tim7_PeriodElapsedCallback(void)
{
	
}



__weak void cy_timer_tim8_PeriodElapsedCallback(void)
{
	
}






void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	
	
	
	if (htim->Instance == TIM1){
	
	
	}else if (htim->Instance == TIM2){
	
	
	}else if (htim->Instance == TIM3){
	
	
	}else if (htim->Instance == TIM4){
		
	
	}else if (htim->Instance == TIM5){
	
	
	}else if (htim->Instance == TIM6){
		if (hcan.State == HAL_CAN_STATE_READY){
			HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
		}
	
	}else if (htim->Instance == TIM7){
	
	
	}else if (htim->Instance == TIM8){
	
	
	}
	
	
	
}








