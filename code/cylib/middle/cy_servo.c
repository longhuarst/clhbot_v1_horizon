#include "cy_servo.h"


#include "tim.h"
#include <stdlib.h>
#include <math.h>

cy_servo_type cy_servo_;

#include "../bsp/cy_can.h"





void servo_inner(float dg)
{
	if (fabs(dg) <= 0.5){
		TIM3->CCR2 = 1880;
	}
	else if (dg > 0){
		TIM3->CCR2 = 6*dg+1880;
	}else if (dg < 0){
		TIM3->CCR2 = 1880 - 7.1*dg;
	}
}


void servo_outter(float dg)
{
	if (fabs(dg) <= 0.5){
		TIM3->CCR3 = 1600;
	}
	else if (dg > 0){
		TIM3->CCR3 = 10*dg+1600;
	}else if (dg < 0){
		TIM3->CCR3 = 1600 - 11.1*dg;
	}
}



void keep_outter_level(void)
{
	
	if (cy_can_[0].updated == false){
		cy_can_resend();//重发CAN
	}
	while(cy_can_[0].updated == false);	//等待传感器更新
	
	servo_outter(-cy_can_[1].y);
	
}

void cy_servo_init(void)
{







}









void cy_servo_polling(void)
{









}





