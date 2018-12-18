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

#define C_Motor_Step_Deg (1.8/128/10) //ÿ������ĽǶ�
#define C2_Motor_Step_Deg (1.8/128/30) //ÿ������ĽǶ�



float d1,d2,d3;

float d1_cur = 45,d2_cur = -45,d3_cur = 0;//��ǰ�Ƕ�


void abb_move(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//�޽�
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
			//��һ����
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//��������
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//�ڶ�����
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//��������
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//������Ч
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 ��Ҫȡ����
	
	
	
	
	//YAW ���
	//��ʱ��ת    motor_3  �� ��
	//˳ʱ��ת    motor_3  �� ��
	float d3_diff = (d3 - d3_cur);//Ŀ��ֵ - ��ǰֵ
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//�ڹؽ�
	//����        motor_1  �� ��
	//����        motor_1  �� ��
//	float d1_diff = d1 - d1_cur;
//	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
//	cy_stepper_run_x(3,d1_run_step);
	
	for (int i=0;i<50;++i){
		//��������
		//�ڱڵ�� 45��
		cy_can_[0].updated = false;//���CAN ���
		float error = d1 - cy_can_[0].y;//�������ֵС�� 45�� ��ʾ ��Ҫ����ת �� ��ת
		#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
		
		int32_t err_step = pitch_inner_angle_to_step(error);
		cy_stepper_run_x(3,err_step);//�ڵ�� CH4
		cy_stepper_wait_x(3);//�ȴ��˶����
		
		if (cy_can_[0].updated == false){
			cy_can_resend();//�ط�CAN
		}
		while(cy_can_[0].updated == false);	//�ȴ�����������
	}
	
	
	d1_cur = d1;
	//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//��ؽ�
	//����        motor_2  �� ��
	//����        motor_2  �� ��
//	float d2_diff = -(d2 - d2_cur);
//	int32_t d2_run_step = d2_diff / C_Motor_Step_Deg;
//	cy_stepper_run_x(4,d2_run_step);
	for (int i=0;i<50;++i){
		//��������
		//��۵�� -45��
		cy_can_[1].updated = false;//���CAN ���
		float error = - (d2 - cy_can_[1].y);//�������ֵС�� -45�� ��ʾ ��Ҫ����ת �� ��ת
		#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
		
		int32_t err_step = pitch_outter_angle_to_step(error);
		cy_stepper_run_x(4,err_step);//�ڵ�� CH4
		cy_stepper_wait_x(4);//�ȴ��˶����
		
		if (cy_can_[1].updated == false){
			cy_can_resend();//�ط�CAN
		}
		while(cy_can_[1].updated == false);	//�ȴ�����������
	}
	//motor[motor_2].step_ = d2_diff / C2_Motor_Step_Deg;
	
	d2_cur = d2;

}






void abb_move_fast(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//�޽�
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
			//��һ����
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//��������
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//�ڶ�����
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//��������
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//������Ч
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 ��Ҫȡ����
	
	
	
	
	//YAW ���
	//��ʱ��ת    motor_3  �� ��
	//˳ʱ��ת    motor_3  �� ��
	float d3_diff = (d3 - d3_cur);//Ŀ��ֵ - ��ǰֵ
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//�ڹؽ�
	//����        motor_1  �� ��
	//����        motor_1  �� ��
//	float d1_diff = d1 - d1_cur;
//	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
//	cy_stepper_run_x(3,d1_run_step);
		//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//��ؽ�
	//����        motor_2  �� ��
	//����        motor_2  �� ��
//	float d2_diff = -(d2 - d2_cur);
//	int32_t d2_run_step = d2_diff / C_Motor_Step_Deg;
//	cy_stepper_run_x(4,d2_run_step);











	
	
	for (int i=0;i<5;++i){
		
		{
			//��������
			//�ڱڵ�� 45��
			cy_can_[0].updated = false;//���CAN ���
			float error = d1 - cy_can_[0].y;//�������ֵС�� 45�� ��ʾ ��Ҫ����ת �� ��ת
			#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_inner_angle_to_step(error);
			cy_stepper_run_x(3,err_step);//�ڵ�� CH4
			cy_stepper_wait_x(3);//�ȴ��˶����
			
			if (cy_can_[0].updated == false){
				cy_can_resend();//�ط�CAN
			}
		}
			
		{
			//��������
			//��۵�� -45��
			cy_can_[1].updated = false;//���CAN ���
			float error = - (d2 - cy_can_[1].y);//�������ֵС�� -45�� ��ʾ ��Ҫ����ת �� ��ת
			#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_outter_angle_to_step(error);
			cy_stepper_run_x(4,err_step);//�ڵ�� CH4
			cy_stepper_wait_x(4);//�ȴ��˶����
			
			if (cy_can_[1].updated == false){
				cy_can_resend();//�ط�CAN
			}
			//while(cy_can_[1].updated == false);	//�ȴ�����������
			
		}
		
		
		while(cy_can_[0].updated == false);	//�ȴ�����������
		
		
		
	}
	//motor[motor_2].step_ = d2_diff / C2_Motor_Step_Deg;
	d1_cur = d1;

	d2_cur = d2;

}





void abb_move2(float x, float y, float z)
{
	
	float R = sqrt(x*x+y*y);
	
	
	
	if (x == 0 && y == 0){
		//�޽�
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
			//��һ����
			d3 = atan(x/y);
		}else if (x > 0 && y < 0){
			//��������
			d3 = atan(fabs(y/x)) + 90;
		}else if (x < 0 && y > 0){
			//�ڶ�����
			d3 = -atan(fabs(x/y));
		}else if (x < 0 && y < 0){
			//��������
			d3 = -atan(fabs(y/x)) - 90;
		}else{
			//������Ч
		}
		d3 = Rad2Deg(d3);
	}
	
	
	
	
	
	float a = (R - L3) / L;
	
	float b = (z - H - H2) / L;
	
	
	d1 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) - 4*b ) / (a*a + 2*a + b*b) );
	
	
	d2 = -2 * atan( ( 2*b - sqrt(-(a*a + b*b)*(a*a + b*b -4)) ) / (a*a + 2*a + b*b));
	
	
	d1 = Rad2Deg(d1);
	
	d2 = - Rad2Deg(d2);//d2 ��Ҫȡ����
	
	
	
	
	//YAW ���
	//��ʱ��ת    motor_3  �� ��
	//˳ʱ��ת    motor_3  �� ��
	float d3_diff = (d3 - d3_cur);//Ŀ��ֵ - ��ǰֵ
	
	int32_t d3_run_step = d3_diff / C_Motor_Step_Deg;
	cy_stepper_run_x(5,d3_run_step);
		
	d3_cur = d3;
	//motor[motor_3].step_ = ;
	
	//�ڹؽ�
	//����        motor_1  �� ��
	//����        motor_1  �� ��
	float d1_diff = d1 - d1_cur;
	int32_t d1_run_step = d1_diff / C2_Motor_Step_Deg;
	cy_stepper_run_x(3,d1_run_step);
	
	
	d1_cur = d1;
	//motor[motor_1].step_ = -d1_diff / C2_Motor_Step_Deg;
	
	//��ؽ�
	//����        motor_2  �� ��
	//����        motor_2  �� ��
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





