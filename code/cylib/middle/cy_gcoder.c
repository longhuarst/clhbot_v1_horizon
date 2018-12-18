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
			cy_gcoder_decoder();//解码
			
			
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
			//是G代码
			cy_gcoder_g_();
		break;
		case 'M':
			cy_gcoder_m_();
		break;
	}

}





//G代碼解析
void cy_gcoder_g_(void)
{
	
	uint8_t code = (cy_gcoder_buffer[1]-'0') * 10 +
					(cy_gcoder_buffer[2] - '0');
	float x,y,z,f,p;
	int ret;
	
	cy_string_low2up(cy_gcoder_buffer);//小写转大写  通用处理
	
	switch(code){
		case 0:
			//快速定位
			ret = sscanf(cy_gcoder_buffer,"G00 X%f Y%f Z%f",&x,&y,&z);
			if (ret == 3){
				printf("G00 X%f Y%f Z%f\r\n",x,y,z);
				
				cy_controller_move(x,y,z);
			}else{
				printf("G00 Error\r\n");
			}
			break;
		case 1:
			//直线切削给进
			ret = sscanf(cy_gcoder_buffer,"G01 X%f Y%f Z%f F%f",&x,&y,&z,&f);
			if (ret == 4){
				printf("G01 X%f Y%f Z%f F%f\r\n",x,y,z,f);
			}else{
				printf("G01 Error\r\n");
			}
			break;
		case 4:
			//暂停
			ret = sscanf(cy_gcoder_buffer,"G04 P%f",&p);
			if (ret == 1){
				printf("G04 P%f\r\n",p);
			}else{
				printf("G04 Error\r\n");
			}
			break;
		case 90:
			//切换为绝对坐标
			printf("G90\r\n");
			break;
		case 91:
			//切换为相对坐标
			printf("G91\r\n");
			break;
		default:
			printf("G?? Error\r\n");
			break;
		
	}
}


//M代碼解析
void cy_gcoder_m_(void)
{
	//主轴正转 M03
	//主轴反转 M04
	//主轴停转 M05
	
	uint8_t code = (cy_gcoder_buffer[1]-'0') * 10 +
					(cy_gcoder_buffer[2] - '0');
	
	switch(code){
		case 2:
			//程序结束 M02
			printf("M02\r\n");
			while(1);
		case 3:
			//主轴正转 M03
			HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_SET);
			printf("M03\r\n");
			break;
		case 4:
			//主轴反转 M04
			//该版本没有该功能
			break;
		case 5:
			//主轴停转 M05
			HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_RESET);
			printf("M05\r\n");
			break;
		default:
			printf("M?? Error\r\n");
			break;
		
	}
	
	
	
}





