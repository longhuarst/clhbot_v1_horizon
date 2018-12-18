#include "cy_abb.h"

#include <math.h>

#include "../middle/cy_stepper.h"
#include "../bsp/cy_can.h"


cy_abb_type cy_abb_;





#define C_PI       3.14159265358979323846
#define Rad2Deg(x) (180 / C_PI * x)

#define L (160)
#define L3 (59)
#define H (131)
#define H2 (0)

#define C_Motor_Step_Deg (1.8/128/10) //每个脉冲的角度
#define C2_Motor_Step_Deg (1.8/128/30) //每个脉冲的角度



float d1,d2,d3;

float d1_cur = 45,d2_cur = -45,d3_cur = 0;//当前角度


void abb_move(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//无解
	}else if (x == 0){
		if (y > 0){
			d3 = 0;
		}else if (y < 0){
			d3 = 180;
		}
	}else if (y == 0){
		if (x > 0){
			d3 = 90;
		}else if (x < 0){
			d3 = -90;
		}
	}else{
		
		if (x > 0 && y > 0){
			//第一象限
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//第四象限
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//第二象限
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//第三象限
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//计算无效
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 需要取负角
	
	
	
	
	//YAW 电机
	//逆时针转    motor_3  给 正
	//顺时针转    motor_3  给 负
	float d3_diff = (d3 - d3_cur);//目标值 - 当前值
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//内关节
	//向上        motor_1  给 负
	//向下        motor_1  给 正
//	float d1_diff = d1 - d1_cur;
//	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
//	cy_stepper_run_x(3,d1_run_step);
	
	for (int i=0;i<50;++i){
		//计算误差角
		//内壁电机 45度
		cy_can_[0].updated = false;//清除CAN 标记
		float error = d1 - cy_can_[0].y;//如果采样值小于 45度 表示 需要向上转 既 正转
		#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
		
		int32_t err_step = pitch_inner_angle_to_step(error);
		cy_stepper_run_x(3,err_step);//内电机 CH4
		cy_stepper_wait_x(3);//等待运动完成
		
		if (cy_can_[0].updated == false){
			cy_can_resend();//重发CAN
		}
		while(cy_can_[0].updated == false);	//等待传感器更新
	}
	
	
	d1_cur = d1;
	//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//外关节
	//向上        motor_2  给 正
	//向下        motor_2  给 负
//	float d2_diff = -(d2 - d2_cur);
//	int32_t d2_run_step = d2_diff / C_Motor_Step_Deg;
//	cy_stepper_run_x(4,d2_run_step);
	for (int i=0;i<50;++i){
		//计算误差角
		//外臂电机 -45度
		cy_can_[1].updated = false;//清除CAN 标记
		float error = - (d2 - cy_can_[1].y);//如果采样值小于 -45度 表示 需要向上转 既 反转
		#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
		
		int32_t err_step = pitch_outter_angle_to_step(error);
		cy_stepper_run_x(4,err_step);//内电机 CH4
		cy_stepper_wait_x(4);//等待运动完成
		
		if (cy_can_[1].updated == false){
			cy_can_resend();//重发CAN
		}
		while(cy_can_[1].updated == false);	//等待传感器更新
	}
	//motor[motor_2].step_ = d2_diff / C2_Motor_Step_Deg;
	
	d2_cur = d2;

}






void abb_move_fast(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//无解
	}else if (x == 0){
		if (y > 0){
			d3 = 0;
		}else if (y < 0){
			d3 = 180;
		}
	}else if (y == 0){
		if (x > 0){
			d3 = 90;
		}else if (x < 0){
			d3 = -90;
		}
	}else{
		
		if (x > 0 && y > 0){
			//第一象限
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//第四象限
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//第二象限
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//第三象限
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//计算无效
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 需要取负角
	
	
	
	
	//YAW 电机
	//逆时针转    motor_3  给 正
	//顺时针转    motor_3  给 负
	float d3_diff = (d3 - d3_cur);//目标值 - 当前值
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//内关节
	//向上        motor_1  给 负
	//向下        motor_1  给 正
//	float d1_diff = d1 - d1_cur;
//	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
//	cy_stepper_run_x(3,d1_run_step);
		//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//外关节
	//向上        motor_2  给 正
	//向下        motor_2  给 负
//	float d2_diff = -(d2 - d2_cur);
//	int32_t d2_run_step = d2_diff / C_Motor_Step_Deg;
//	cy_stepper_run_x(4,d2_run_step);











	
	
	for (int i=0;i<5;++i){
		
		{
			//计算误差角
			//内壁电机 45度
			cy_can_[0].updated = false;//清除CAN 标记
			float error = d1 - cy_can_[0].y;//如果采样值小于 45度 表示 需要向上转 既 正转
			#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_inner_angle_to_step(error);
			cy_stepper_run_x(3,err_step);//内电机 CH4
			cy_stepper_wait_x(3);//等待运动完成
			
			if (cy_can_[0].updated == false){
				cy_can_resend();//重发CAN
			}
		}
			
		{
			//计算误差角
			//外臂电机 -45度
			cy_can_[1].updated = false;//清除CAN 标记
			float error = - (d2 - cy_can_[1].y);//如果采样值小于 -45度 表示 需要向上转 既 反转
			#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_outter_angle_to_step(error);
			cy_stepper_run_x(4,err_step);//内电机 CH4
			cy_stepper_wait_x(4);//等待运动完成
			
			if (cy_can_[1].updated == false){
				cy_can_resend();//重发CAN
			}
			//while(cy_can_[1].updated == false);	//等待传感器更新
			
		}
		
		
		while(cy_can_[0].updated == false);	//等待传感器更新
		
		
		
	}
	//motor[motor_2].step_ = d2_diff / C2_Motor_Step_Deg;
	d1_cur = d1;

	d2_cur = d2;

}





void abb_move2(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//无解
	}else if (x == 0){
		if (y > 0){
			d3 = 0;
		}else if (y < 0){
			d3 = 180;
		}
	}else if (y == 0){
		if (x > 0){
			d3 = 90;
		}else if (x < 0){
			d3 = -90;
		}
	}else{
		
		if (x > 0 && y > 0){
			//第一象限
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//第四象限
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//第二象限
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//第三象限
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//计算无效
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 需要取负角
	
	
	
	
	//YAW 电机
	//逆时针转    motor_3  给 正
	//顺时针转    motor_3  给 负
	float d3_diff = (d3 - d3_cur);//目标值 - 当前值
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//内关节
	//向上        motor_1  给 负
	//向下        motor_1  给 正
	float d1_diff = d1 - d1_cur;
	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
	cy_stepper_run_x(3,d1_run_step);
	
	
	d1_cur = d1;
	//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//外关节
	//向上        motor_2  给 正
	//向下        motor_2  给 负
	float d2_diff = -(d2 - d2_cur);
	int32_t d2_run_step = d2_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(4,d2_run_step);
	
	//motor[motor_2].step_ = d2_diff / C2_Motor_Step_Deg;
	
	d2_cur = d2;

}







void cy_abb_init(void)
{







}









void cy_abb_polling(void)
{









}





