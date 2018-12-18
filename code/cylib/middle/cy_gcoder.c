#include "cy_gcoder.h"
#include "../bsp/cy_uart.h"
#include "cy_string.h"
#include "cy_controller.h"

#include "gpio.h"
#include <stdio.h>


cy_gcoder_type cy_gcoder_;




void cy_gcoder_decoder(void);







void cy_gcoder_init(void)
{







}






char cy_gcoder_buffer[128];
uint8_t cy_gcoder_index = 0;

void cy_gcoder_polling(void)
{


	
	if (uart1_read((uint8_t *)&cy_gcoder_buffer[cy_gcoder_index],1) > 0){
		
		
		if (cy_gcoder_buffer[cy_gcoder_index] == '\r' || cy_gcoder_buffer[cy_gcoder_index] == '\n'){
			
			
			if (cy_gcoder_index + 1 >= sizeof(cy_gcoder_buffer) - 1){
				cy_gcoder_buffer[sizeof(cy_gcoder_buffer) - 1] = 0;
			}else{
				cy_gcoder_buffer[cy_gcoder_index+1] = 0;
			}
			cy_gcoder_decoder();//����
			
			
			cy_gcoder_index = 0;
			
			
			
			
		}else
			cy_gcoder_index++;
	}
	





}


void cy_gcoder_g_(void);
void cy_gcoder_m_(void);

void cy_gcoder_decoder(void)
{
	
	
	switch(cy_gcoder_buffer[0]){
		
		case 'G':
			//��G����
			cy_gcoder_g_();
		break;
		case 'M':
			cy_gcoder_m_();
		break;
	}

}





//G���a����
void cy_gcoder_g_(void)
{
	
	uint8_t code = (cy_gcoder_buffer[1]-'0') * 10 +
					(cy_gcoder_buffer[2] - '0');
	float x,y,z,f,p;
	int ret;
	
	cy_string_low2up(cy_gcoder_buffer);//Сдת��д  ͨ�ô���
	
	switch(code){
		case 0:
			//���ٶ�λ
			ret = sscanf(cy_gcoder_buffer,"G00 X%f Y%f Z%f",&x,&y,&z);
			if (ret == 3){
				printf("G00 X%f Y%f Z%f\r\n",x,y,z);
				
				cy_controller_move(x,y,z);
			}else{
				printf("G00 Error\r\n");
			}
			break;
		case 1:
			//ֱ����������
			ret = sscanf(cy_gcoder_buffer,"G01 X%f Y%f Z%f F%f",&x,&y,&z,&f);
			if (ret == 4){
				printf("G01 X%f Y%f Z%f F%f\r\n",x,y,z,f);
			}else{
				printf("G01 Error\r\n");
			}
			break;
		case 4:
			//��ͣ
			ret = sscanf(cy_gcoder_buffer,"G04 P%f",&p);
			if (ret == 1){
				printf("G04 P%f\r\n",p);
			}else{
				printf("G04 Error\r\n");
			}
			break;
		case 90:
			//�л�Ϊ��������
			printf("G90\r\n");
			break;
		case 91:
			//�л�Ϊ�������
			printf("G91\r\n");
			break;
		default:
			printf("G?? Error\r\n");
			break;
		
	}
}


//M���a����
void cy_gcoder_m_(void)
{
	//������ת M03
	//���ᷴת M04
	//����ͣת M05
	
	uint8_t code = (cy_gcoder_buffer[1]-'0') * 10 +
					(cy_gcoder_buffer[2] - '0');
	
	switch(code){
		case 2:
			//������� M02
			printf("M02\r\n");
			while(1);
		case 3:
			//������ת M03
			HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_SET);
			printf("M03\r\n");
			break;
		case 4:
			//���ᷴת M04
			//�ð汾û�иù���
			break;
		case 5:
			//����ͣת M05
			HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_RESET);
			printf("M05\r\n");
			break;
		default:
			printf("M?? Error\r\n");
			break;
		
	}
	
	
	
}





