#include "cy_stepper.h"

#include "tim.h"

#include <math.h>


#include <string.h>

#include <stdlib.h>




cy_stepper_type cy_stepper_[Stepper_Max];





void cy_stepper_init(void)
{
	memset(&cy_stepper_[0],0,sizeof(cy_stepper_[0]));
	memset(&cy_stepper_[1],0,sizeof(cy_stepper_[0]));
	memset(&cy_stepper_[2],0,sizeof(cy_stepper_[0]));
	memset(&cy_stepper_[3],0,sizeof(cy_stepper_[0]));
	memset(&cy_stepper_[4],0,sizeof(cy_stepper_[0]));
	memset(&cy_stepper_[5],0,sizeof(cy_stepper_[0]));
	
	
	
	
	HAL_GPIO_WritePin(STEP_CH1_EN_GPIO_Port,STEP_CH1_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH1_M1_GPIO_Port,STEP_CH1_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH1_M2_GPIO_Port,STEP_CH1_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH1_M3_GPIO_Port,STEP_CH1_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH1_RESET_GPIO_Port,STEP_CH1_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH1_ST_GPIO_Port,STEP_CH1_ST_Pin,GPIO_PIN_SET);

	
	HAL_GPIO_WritePin(STEP_CH2_EN_GPIO_Port,STEP_CH2_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH2_M1_GPIO_Port,STEP_CH2_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH2_M2_GPIO_Port,STEP_CH2_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH2_M3_GPIO_Port,STEP_CH2_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH2_RESET_GPIO_Port,STEP_CH2_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH2_ST_GPIO_Port,STEP_CH2_ST_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(STEP_CH3_EN_GPIO_Port,STEP_CH3_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH3_M1_GPIO_Port,STEP_CH3_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH3_M2_GPIO_Port,STEP_CH3_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH3_M3_GPIO_Port,STEP_CH3_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH3_RESET_GPIO_Port,STEP_CH3_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH3_ST_GPIO_Port,STEP_CH3_ST_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(STEP_CH4_EN_GPIO_Port,STEP_CH4_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH4_M1_GPIO_Port,STEP_CH4_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH4_M2_GPIO_Port,STEP_CH4_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH4_M3_GPIO_Port,STEP_CH4_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH4_RESET_GPIO_Port,STEP_CH4_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH4_ST_GPIO_Port,STEP_CH4_ST_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(STEP_CH5_EN_GPIO_Port,STEP_CH5_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH5_M1_GPIO_Port,STEP_CH5_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH5_M2_GPIO_Port,STEP_CH5_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH5_M3_GPIO_Port,STEP_CH5_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH5_RESET_GPIO_Port,STEP_CH5_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH5_ST_GPIO_Port,STEP_CH5_ST_Pin,GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(STEP_CH6_EN_GPIO_Port,STEP_CH6_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH6_M1_GPIO_Port,STEP_CH6_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH6_M2_GPIO_Port,STEP_CH6_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH6_M3_GPIO_Port,STEP_CH6_M3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH6_RESET_GPIO_Port,STEP_CH6_RESET_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_CH6_ST_GPIO_Port,STEP_CH6_ST_Pin,GPIO_PIN_SET);

}



void cy_stepper_polling(void)
{

}



void cy_stepper_run_x(uint8_t x, int32_t step)
{
	switch(x){
		case 0:
			
			if (step >=0 ){
				HAL_GPIO_WritePin(STEP_CH1_DIR_GPIO_Port,STEP_CH1_DIR_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(STEP_CH1_DIR_GPIO_Port,STEP_CH1_DIR_Pin,GPIO_PIN_RESET);
			}
			cy_stepper_[0].step_ = abs(step);
			if(cy_stepper_[0].step_ != 0) {cy_stepper_[0].done_ = false;Stepper_CH1_Start();}
		break;
		case 1:
			
			if (step >=0 ){
				HAL_GPIO_WritePin(STEP_CH2_DIR_GPIO_Port,STEP_CH2_DIR_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(STEP_CH2_DIR_GPIO_Port,STEP_CH2_DIR_Pin,GPIO_PIN_RESET);
			}
			cy_stepper_[1].step_ = abs(step);
			if(cy_stepper_[1].step_ != 0) {cy_stepper_[1].done_ = false;Stepper_CH2_Start();}
		break;
		case 2:
			
			if (step >=0 ){
					HAL_GPIO_WritePin(STEP_CH3_DIR_GPIO_Port,STEP_CH3_DIR_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(STEP_CH3_DIR_GPIO_Port,STEP_CH3_DIR_Pin,GPIO_PIN_RESET);
			}
			cy_stepper_[2].step_ = abs(step);
			if(cy_stepper_[2].step_ != 0) {cy_stepper_[2].done_ = false;Stepper_CH3_Start();}	
		break;
		case 3:
			
			if (step >=0 ){
				HAL_GPIO_WritePin(STEP_CH4_DIR_GPIO_Port,STEP_CH4_DIR_Pin,GPIO_PIN_RESET);
			}else{
				HAL_GPIO_WritePin(STEP_CH4_DIR_GPIO_Port,STEP_CH4_DIR_Pin,GPIO_PIN_SET);
			}
			cy_stepper_[3].step_ = abs(step);
			if(cy_stepper_[3].step_ != 0) {cy_stepper_[3].done_ = false;Stepper_CH4_Start();}
		break;
		case 4:
			
			if (step >=0 ){
				HAL_GPIO_WritePin(STEP_CH5_DIR_GPIO_Port,STEP_CH5_DIR_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(STEP_CH5_DIR_GPIO_Port,STEP_CH5_DIR_Pin,GPIO_PIN_RESET);
			}
			cy_stepper_[4].step_ = abs(step);
			if(cy_stepper_[4].step_ != 0) {cy_stepper_[4].done_ = false;Stepper_CH5_Start();}
		break;
		case 5:
			
			if (step >=0 ){
				HAL_GPIO_WritePin(STEP_CH6_DIR_GPIO_Port,STEP_CH6_DIR_Pin,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(STEP_CH6_DIR_GPIO_Port,STEP_CH6_DIR_Pin,GPIO_PIN_RESET);
			}
			cy_stepper_[5].step_ = abs(step);
			if(cy_stepper_[5].step_ != 0) {cy_stepper_[5].done_ = false;Stepper_CH6_Start();}
		break;
		default:
			break;
	}
}





void cy_stepper_stop_x(uint8_t x)
{
	switch(x){
		case 0:
			cy_stepper_[0].step_ = 0;
			cy_stepper_[0].done_ = true;
			Stepper_CH1_Stop();
		break;
		case 1:
			cy_stepper_[1].step_ = 0;
			cy_stepper_[1].done_ = true;
			Stepper_CH2_Stop();
		break;
		case 2:
			cy_stepper_[2].step_ = 0;
			cy_stepper_[2].done_ = true;
			Stepper_CH3_Stop();
		break;
		case 3:
			cy_stepper_[3].step_ = 0;
			cy_stepper_[3].done_ = true;
			Stepper_CH4_Stop();
		break;
		case 4:
			cy_stepper_[4].step_ = 0;
			cy_stepper_[4].done_ = true;
			Stepper_CH5_Stop();
		break;
		case 5:
			cy_stepper_[5].step_ = 0;
			cy_stepper_[5].done_ = true;
			Stepper_CH6_Stop();
		break;
		default:
			break;
	}
}




void cy_stepper_wait_x(uint8_t x)
{
	if (x >= 6)
		return;
	while(cy_stepper_[x].done_ == false ){
		if (cy_stepper_[x].step_ == 0){
			break;
		}
	} 
}

void cy_stepper_run(int32_t step1, int32_t step2, int32_t step3, int32_t step4, int32_t step5, int32_t step6)
{
	cy_stepper_[0].done_ = false;
	cy_stepper_[1].done_ = false;
	cy_stepper_[2].done_ = false;
	cy_stepper_[3].done_ = false;
	cy_stepper_[4].done_ = false;
	cy_stepper_[5].done_ = false;
	
	if (step1 >=0 ){
		HAL_GPIO_WritePin(STEP_CH1_DIR_GPIO_Port,STEP_CH1_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH1_DIR_GPIO_Port,STEP_CH1_DIR_Pin,GPIO_PIN_RESET);
	}

	
	if (step2 >=0 ){
		HAL_GPIO_WritePin(STEP_CH2_DIR_GPIO_Port,STEP_CH2_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH2_DIR_GPIO_Port,STEP_CH2_DIR_Pin,GPIO_PIN_RESET);
	}
	
	
	if (step3 >=0 ){
		HAL_GPIO_WritePin(STEP_CH3_DIR_GPIO_Port,STEP_CH3_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH3_DIR_GPIO_Port,STEP_CH3_DIR_Pin,GPIO_PIN_RESET);
	}
	
	
	if (step4 >=0 ){
		HAL_GPIO_WritePin(STEP_CH4_DIR_GPIO_Port,STEP_CH4_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH4_DIR_GPIO_Port,STEP_CH4_DIR_Pin,GPIO_PIN_RESET);
	}
	
	if (step5 >=0 ){
		HAL_GPIO_WritePin(STEP_CH5_DIR_GPIO_Port,STEP_CH5_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH5_DIR_GPIO_Port,STEP_CH5_DIR_Pin,GPIO_PIN_RESET);
	}
	
	if (step6 >=0 ){
		HAL_GPIO_WritePin(STEP_CH6_DIR_GPIO_Port,STEP_CH6_DIR_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(STEP_CH6_DIR_GPIO_Port,STEP_CH6_DIR_Pin,GPIO_PIN_RESET);
	}



	
	
	cy_stepper_[0].step_ = abs(step1);
	cy_stepper_[1].step_ = abs(step2);
	cy_stepper_[2].step_ = abs(step3);
	cy_stepper_[3].step_ = abs(step4);
	cy_stepper_[4].step_ = abs(step5);
	cy_stepper_[5].step_ = abs(step6);

	if(cy_stepper_[0].step_ != 0) {Stepper_CH1_Start();}
	if(cy_stepper_[1].step_ != 0) {Stepper_CH2_Start();}
	if(cy_stepper_[2].step_ != 0) {Stepper_CH3_Start();}
	if(cy_stepper_[3].step_ != 0) {Stepper_CH4_Start();}
	if(cy_stepper_[4].step_ != 0) {Stepper_CH5_Start();}
	if(cy_stepper_[5].step_ != 0) {Stepper_CH6_Start();}
}






//步进电机测试代码
void cy_stepper_test(void)
{
	
	HAL_Delay(1000);
	cy_stepper_run(11,12,13,14,15,16);
	
	

	
}





















#define mini_freq (1*1000)
#define max_freq (100*1000)
#define times (1000)

//uint16_t calc_(uint32_t time)
//{
//	uint32_t b = mini_freq;
//	uint32_t k = (max_freq - mini_freq) / times;
//	uint32_t freq = time * k + b;
//	
//	return 1100 - freq / 10;
//}






void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

	static uint32_t tim2_counter = 0;
	static uint32_t tim4_counter = 0;
	
  if (htim->Instance == TIM2){
	  tim2_counter++;
	  if (tim2_counter >= times){
		tim2_counter = times;
		  
		  
	  }else{
		TIM2->ARR = 20;//calc_(tim2_counter);
	  }
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		//电机 1 
		
		TIM2->CCR2 = TIM2->ARR / 2;
		cy_stepper_[0].step_--;  // 9 8 7 6 5 4 3 2 1 
		if (cy_stepper_[0].step_<=0){
			//Stepper_CH1_Duty(0);//关闭PWM
			cy_stepper_[0].done_ = true;
			Stepper_CH1_Stop();
			tim2_counter = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		//电机 6 
		TIM2->ARR = 10;
		TIM2->CCR1 = TIM2->ARR / 2;
		cy_stepper_[5].step_--;
		if (cy_stepper_[5].step_ <= 0){
			//Stepper_CH6_Duty(0);//关闭PWM
			cy_stepper_[5].done_ = true;
			Stepper_CH6_Stop();
			tim2_counter = 0;
		}
	}
  }else if (htim->Instance == TIM4){
	tim4_counter++;
	if (tim4_counter >= times){
		tim4_counter = times;
	}else{
		TIM4->ARR = 20;//calc_(tim2_counter);
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		//电机 5 
		TIM4->CCR1 = TIM4->ARR / 2;
		cy_stepper_[4].step_--;
		if (cy_stepper_[4].step_ <= 0){
			//Stepper_CH5_Duty(0);//关闭PWM
			cy_stepper_[4].done_ = true;
			Stepper_CH5_Stop();
			tim4_counter = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		//电机 4 
		TIM4->CCR2 = TIM4->ARR / 2;
		cy_stepper_[3].step_--;
		if (cy_stepper_[3].step_ <= 0){
			//Stepper_CH4_Duty(0);//关闭PWM
			cy_stepper_[3].done_ = true;
			Stepper_CH4_Stop();
			tim4_counter = 0;

		}

	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
		//电机 2
		TIM4->CCR3 = TIM4->ARR / 2;
		cy_stepper_[1].step_--;
		if (cy_stepper_[1].step_ <= 0){
			//Stepper_CH2_Duty(0);//关闭PWM
			cy_stepper_[1].done_ = true;
			Stepper_CH2_Stop();
			tim4_counter = 0;
		}
	}else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
		//电机 3 
		TIM4->CCR4 = TIM4->ARR / 2;
		cy_stepper_[2].step_--;
		if (cy_stepper_[2].step_ <= 0){
			//Stepper_CH3_Duty(0);//关闭PWM
			cy_stepper_[2].done_ = true;
			Stepper_CH3_Stop();
			tim4_counter = 0;
		}
	
	}
	
	
  }
}









