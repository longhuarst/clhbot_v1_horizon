#include "cy_b.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../middle/cy_stepper.h"


cy_b_type cy_b_;






#define r1 (120)
#define r2 (128.5)

#define C_PI       3.14159265358979323846
#define Rad2Deg(x) (180 / C_PI * x)


void cy_b_move_abs(float x_,float y_,float z_) {
	float x;
	float y;
	float z;
	float R = sqrt(x*x+y*y);
	float dg2;
	float dgx = 0;
	float dg1 =0;
	
	
	x = cy_b_.x = x_;
	y = cy_b_.y = y_;
	z = cy_b_.z = z_;
	

	if (R < r1 + r2) {
	
		dg2 = 180 - Rad2Deg((acos((r1*r1 + r2 * r2 - R * R) / (2 * r1*r2))));

		dgx = Rad2Deg(acos((r1*r1+R*R-r2*r2)/(2*r1*R)));

		if (x == 0) {
			if (y > 0) {
				dg1 = 360 - dgx;
			}
			else if (y < 0){
				dg1 = 180 - dgx;
			}
			else if (y == 0) {
				printf("坐标（x,y） = (0,0) 不可达！\r\n");
				return;
			}
		}
		else if (x > 0) {
			if (y == 0) {
				dg1 = 90 - dgx;
			}
			else if (y > 0) {
				dg1 = 90 - dgx - Rad2Deg(atan(fabs(y / x)));
			}
			else if (y < 0) {
				dg1 = 180 - dgx - Rad2Deg(atan(fabs(x / y)));
				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(x / y))));
			}
		}
		else if (x < 0) {
			if (y == 0) {
				dg1 = 270 - dgx;
			}
			else if (y > 0) {
				dg1 = 360 - dgx - Rad2Deg(atan(fabs(x/y)));
				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(x / y))));
			}
			else if (y < 0) {
				dg1 = 270 - dgx - Rad2Deg(atan(fabs(y / x)));
				//printf("%f %f\r\n", dgx, Rad2Deg(atan(fabs(y / x))));
			}
		}
	}
	else if (R == r1 + r2) {
		if (x == 0) {
			if (y > 0) {
				dg1 = 0;
				dg2 = 0;
			}
			else if (y < 0) {
				dg1 = 180;
				dg2 = 0;
			}
			else {
				printf("坐标（x,y） = (0,0) 不可达！\r\n");
				return;
			}
		}
		else if (x > 0) {
			if (y > 0) {
				dg1 =  90 - Rad2Deg(atan(fabs(y/x)));
				dg2 = 0;
			}
			else if (y < 0) {
				dg1 = 90 + Rad2Deg(atan(fabs(y / x)));
				dg2 = 0;
			}
			else {
				dg1 = 90;
				dg2 = 0;
			}

		}
		else if (x < 0) {
			if (y > 0) {
				dg1 = 270 + Rad2Deg(atan(fabs(y / x)));
				dg2 = 0;
			}
			else if (y < 0) {
				dg1 = 270 - Rad2Deg(atan(fabs(y / x)));
				dg2 = 0;
			}
			else {
				dg1 = 270;
				dg2 = 0;
			}
		}
	}
	else if (R > r1 + r2) {
		printf("输入超过计算范围!\r\n");
	}



	printf("(%f,%f) --> (%f,%f)\r\n",x,y,dg1,dgx);
	
	
	#define yaw_angle_to_step(x) ((x)/0.00140625f)//(x / (1.8/10/128))
	#define MAX_UP_LOW_MM (100)
	#define MAX_STEP (MAX_UP_LOW_MM/8.0f*360/(1.8/128))
	#define MM_TO_STEP(X) (((X)/8.0f*360/(1.8/128)) > MAX_STEP ? MAX_STEP : (X/8.0f*360/(1.8/128)))
	
	
	
	
	int32_t step = yaw_angle_to_step(dg1-cy_b_.dg1_current);
	cy_stepper_run_x(3,step);//等待步进电机调整完成
	cy_stepper_wait_x(3);//等待运动完成
	cy_b_.dg1_current = dg1;
	
	step = yaw_angle_to_step(dgx-cy_b_.dg2_current);
	cy_stepper_run_x(4,step);//等待步进电机调整完成
	cy_stepper_wait_x(4);//等待运动完成
	cy_b_.dg2_current = dg2;
	
	step = MM_TO_STEP(-z);
	cy_stepper_run_x(5,step);//等待步进电机调整完成
	cy_stepper_wait_x(5);//等待运动完成
	cy_b_.z_current = z;
	
}








void cy_b_init(void)
{

	memset(&cy_b_,0,sizeof(cy_b_));
	
	
	cy_b_.x_current = 0;
	
	cy_b_.y_current = r1 + r2;
	
	cy_b_.z_current = 50;


	cy_b_.dg1_current = 0;
	cy_b_.dg2_current = 0;


}









void cy_b_polling(void)
{









}





