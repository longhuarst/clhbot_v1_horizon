#ifndef __cy_stepper_h
#define __cy_stepper_h


#include <stdint.h>
#include <stdbool.h>





#define Stepper_CH1_Duty(x) do{TIM2->CCR2 = x;}while(0)
#define Stepper_CH2_Duty(x) do{TIM4->CCR3 = x;}while(0)
#define Stepper_CH3_Duty(x) do{TIM4->CCR4 = x;}while(0)
#define Stepper_CH4_Duty(x) do{TIM4->CCR2 = x;}while(0)
#define Stepper_CH5_Duty(x) do{TIM4->CCR1 = x;}while(0)
#define Stepper_CH6_Duty(x) do{TIM2->CCR1 = x;}while(0)


#define Stepper_CH1_Stop() do{HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_2);}while(0)
#define Stepper_CH2_Stop() do{HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_3);}while(0)
#define Stepper_CH3_Stop() do{HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_4);}while(0)
#define Stepper_CH4_Stop() do{HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_2);}while(0)
#define Stepper_CH5_Stop() do{HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_1);}while(0)
#define Stepper_CH6_Stop() do{HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_1);}while(0)


#define Stepper_CH1_Start() do{HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);TIM2->CCR2 = TIM2->ARR/2;}while(0)
#define Stepper_CH2_Start() do{HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_3);TIM4->CCR3 = TIM4->ARR/2;}while(0)
#define Stepper_CH3_Start() do{HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_4);TIM4->CCR4 = TIM4->ARR/2;}while(0)
#define Stepper_CH4_Start() do{HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_2);TIM4->CCR2 = TIM4->ARR/2;}while(0)
#define Stepper_CH5_Start() do{HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);TIM4->CCR1 = TIM4->ARR/2;}while(0)
#define Stepper_CH6_Start() do{HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);TIM2->CCR1 = TIM2->ARR/2;}while(0)



			
	

#define Stepper_Max (6) //最大步进电机数量

typedef struct {
	int32_t step_;//需要走的步数
	bool done_;//完成运动标志
}cy_stepper_type;




void cy_stepper_run_x(uint8_t x, int32_t step);

void cy_stepper_stop_x(uint8_t x);
void cy_stepper_wait_x(uint8_t x);
extern cy_stepper_type cy_stepper_[Stepper_Max];











void cy_stepper_init(void);
void cy_stepper_polling(void);
extern void cy_stepper_test(void);







#endif //__cy_stepper_h
