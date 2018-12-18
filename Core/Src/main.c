
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "../../code/cylib/middle/cy_servo.h"
#include "../../code/cylib/middle/cy_stepper.h"
#include "../../code/cylib/middle/cy_gcoder.h"
#include "../../code/cylib/middle/cy_controller.h"
#include "../../code/cylib/bsp/cy_can.h"
#include "../../code/cylib/bsp/cy_uart.h"
#include "../../code/cylib/controller/cy_abb.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
bool flag_servo1 =false;
bool flag_servo2 = false;

bool flag_motor1 = false;
bool flag_motor2 = false;
bool flag_motor3 = false;


int servo1_value = 0;
int servo2_value = 0;
float motor1_value = 0;
float motor2_value = 0;
float motor3_value = 0;


bool move_flag = false;

float move_x = 0;
float move_y = 0;
float move_z = 0;


bool keep_outter_level_flag = false;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
 
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  
  
 
  #define yaw_angle_to_step(x) (x / (1.8/10/128))
  
			

//初始化电机

		cy_stepper_init();

		
		
		
		#define MAX_UP_LOW_MM (100)
		#define MAX_STEP (MAX_UP_LOW_MM/8.0f*360/(1.8/128))
		#define MM_TO_STEP(X) ((X/8.0f*360/(1.8/128)) > MAX_STEP ? MAX_STEP : (X/8.0f*360/(1.8/128)))
		
		
	
		
		TIM2->ARR = 10;
		//校准上下方向电机 对应限位开关1
		if (HAL_GPIO_ReadPin(Limit_SW_CH1_GPIO_Port,Limit_SW_CH1_Pin) == GPIO_PIN_RESET)
		{
			//int32_t step = MM_TO_STEP(200);
			//cy_stepper_run_x(5,step);//等待步进电机调整完成
			
			TIM2->CCR1 = TIM2->ARR / 2;
			HAL_GPIO_WritePin(STEP_CH6_DIR_GPIO_Port,STEP_CH6_DIR_Pin,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);

			//调整 10 秒
			while(1){
				//寻找接近开关
				if (HAL_GPIO_ReadPin(Limit_SW_CH1_GPIO_Port,Limit_SW_CH1_Pin) == GPIO_PIN_SET){
					//检测到了接近开关
					//设定0点
					HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
					//cy_stepper_stop_x(5);//电机停转
					break;
				}
			}
			
		
		}
		TIM2->ARR = 10;
		
		
		
		//回零
		{
			int32_t step = -1 * MM_TO_STEP(0);//不往下动
			cy_stepper_run_x(5,step);//等待步进电机调整完成
		}
		
		
		
		cy_stepper_wait_x(5);//等待运动完成
		
		/////////////////////////////////////////////////////////////////////////
		
		
		int32_t step = yaw_angle_to_step(180);
		cy_stepper_run_x(3,step);//等待步进电机调整完成
		
		//调整 10 秒
		while(1){
			
			//计算误差角
			
			
			
			
			//寻找接近开关
			if (HAL_GPIO_ReadPin(Proximity_SW_CH4_GPIO_Port,Proximity_SW_CH4_Pin) == GPIO_PIN_SET){
				//检测到了接近开关
				//设定0点
				cy_stepper_stop_x(3);//电机停转
				break;
			}
			
			
			if (cy_stepper_[3].done_ == true){
				//完成了 还没有找到
				 
				//反转
				int32_t step = yaw_angle_to_step(-360);
				cy_stepper_run_x(3,step);//等待步进电机调整完成
			}
			
			
			//cy_stepper_wait_x(5);//等待步进电机调整完成
			
			//HAL_Delay(1);//
		}
		
		
		
		
		//校准中间方向电机 对应限位开关2
		{
			int32_t step = yaw_angle_to_step(270);
			cy_stepper_run_x(4,step);//等待步进电机调整完成
			//调整 10 秒
			while(1){
				//寻找接近开关
				if (HAL_GPIO_ReadPin(Limit_SW_CH2_GPIO_Port,Limit_SW_CH2_Pin) == GPIO_PIN_SET){
					//检测到了接近开关
					//设定0点
					cy_stepper_stop_x(4);//电机停转
					break;
				}
			}
			
		
		}
		
		
		//回零
		{
			int32_t step = yaw_angle_to_step(-107);
			cy_stepper_run_x(4,step);//等待步进电机调整完成
		}
		
		
		
		cy_stepper_wait_x(4);//等待运动完成
		
		
		//初始化执行器
		cy_controller_init();
		
		//初始化G代码串口
		cy_uart_init();
		//初始化G代码解码器
		cy_gcoder_init();
		
		while(1)
		{
			
			cy_gcoder_polling();
			
			
		
		}
		
		
		//正转 45 + 45 度  放
		
		
		
		{
			int32_t step = yaw_angle_to_step(20);
			cy_stepper_run_x(3,step);//等待步进电机调整完成
			cy_stepper_wait_x(3);//等待运动完成
		}
		
//		{
//			int32_t step = yaw_angle_to_step(45);
//			cy_stepper_run_x(4,step);//等待步进电机调整完成
//			cy_stepper_wait_x(4);//等待运动完成
//		}
		
		{
			int32_t step = -1 * MM_TO_STEP(52);
			cy_stepper_run_x(5,step);//等待步进电机调整完成
			cy_stepper_wait_x(5);//等待运动完成
		}
			////////////
		
		HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_SET);
		HAL_Delay(1000);
		
		{
			int32_t step = 1 * MM_TO_STEP(52);
			cy_stepper_run_x(5,step);//等待步进电机调整完成
			cy_stepper_wait_x(5);//等待运动完成
		}
		
//		{
//			int32_t step = -1 * yaw_angle_to_step(45);
//			cy_stepper_run_x(4,step);//等待步进电机调整完成
//			cy_stepper_wait_x(4);//等待运动完成
//		}
		
		{
			int32_t step = -1 * yaw_angle_to_step(20);
			cy_stepper_run_x(3,step);//等待步进电机调整完成
			cy_stepper_wait_x(3);//等待运动完成
		}
		
		
		
		
		
		
		
		
		
		
		{
			int32_t step = -1 * yaw_angle_to_step(20);
			cy_stepper_run_x(3,step);//等待步进电机调整完成
			cy_stepper_wait_x(3);//等待运动完成
		}
		
//		{
//			int32_t step = -1 * yaw_angle_to_step(45);
//			cy_stepper_run_x(4,step);//等待步进电机调整完成
//			cy_stepper_wait_x(4);//等待运动完成
//		}
		
		{
			int32_t step = -1 * MM_TO_STEP(40);
			cy_stepper_run_x(5,step);//等待步进电机调整完成
			cy_stepper_wait_x(5);//等待运动完成
		}
			////////////
		
		HAL_GPIO_WritePin(OUTPUT_CH0_GPIO_Port,OUTPUT_CH0_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
		
		{
			int32_t step = 1 * MM_TO_STEP(40);
			cy_stepper_run_x(5,step);//等待步进电机调整完成
			cy_stepper_wait_x(5);//等待运动完成
		}
		
//		{
//			int32_t step = 1 * yaw_angle_to_step(45);
//			cy_stepper_run_x(4,step);//等待步进电机调整完成
//			cy_stepper_wait_x(4);//等待运动完成
//		}
		
		{
			int32_t step = 1 * yaw_angle_to_step(20);
			cy_stepper_run_x(3,step);//等待步进电机调整完成
			cy_stepper_wait_x(3);//等待运动完成
		}
		
		
		
		
		NVIC_SystemReset();
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		while(1);
		
		
		
		for (int i=0;i<50;++i){
			//计算误差角
			//内壁电机 45度
			cy_can_[0].updated = false;//清除CAN 标记
			float error = 75.5 - cy_can_[0].y;//如果采样值小于 45度 表示 需要向上转 既 正转
			#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_inner_angle_to_step(error);
			cy_stepper_run_x(3,err_step);//内电机 CH4
			cy_stepper_wait_x(3);//等待运动完成
			
			if (cy_can_[0].updated == false){
				cy_can_resend();//重发CAN
			}
			while(cy_can_[0].updated == false);	//等待传感器更新
		}
		
		
	
		
		
		
		
		for (int i=0;i<50;++i){
			//计算误差角
			//外臂电机 -45度
			cy_can_[1].updated = false;//清除CAN 标记
			float error = - (8.37 - cy_can_[1].y);//如果采样值小于 -45度 表示 需要向上转 既 反转
			#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_outter_angle_to_step(error);
			cy_stepper_run_x(4,err_step);//内电机 CH4
			cy_stepper_wait_x(4);//等待运动完成
			
			if (cy_can_[1].updated == false){
				cy_can_resend();//重发CAN
			}
			while(cy_can_[1].updated == false);	//等待传感器更新
		}
		
		
		
		
		//旋转 5 度
		//反转
		{
		int32_t step = yaw_angle_to_step(3.5);
		cy_stepper_run_x(5,step);//等待步进电机调整完成
			cy_stepper_wait_x(5);//等待运动完成
		}
		
		
		TIM3->CCR3 = 1600;//放下
		
		HAL_Delay(1000);
		
		//往下运动
		
		for (int i=0;i<50;++i){
			//计算误差角
			//内壁电机 45度
			cy_can_[0].updated = false;//清除CAN 标记
			float error = 75.73 - cy_can_[0].y;//如果采样值小于 45度 表示 需要向上转 既 正转
			#define pitch_inner_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_inner_angle_to_step(error);
			cy_stepper_run_x(3,err_step);//内电机 CH4
			cy_stepper_wait_x(3);//等待运动完成
			
			if (cy_can_[0].updated == false){
				cy_can_resend();//重发CAN
			}
			while(cy_can_[0].updated == false);	//等待传感器更新
		}
		for (int i=0;i<50;++i){
			//计算误差角
			//外臂电机 -45度
			cy_can_[1].updated = false;//清除CAN 标记
			float error = - (3.27 - cy_can_[1].y);//如果采样值小于 -45度 表示 需要向上转 既 反转
			#define pitch_outter_angle_to_step(x) (x / (1.8/30/128))
			
			int32_t err_step = pitch_outter_angle_to_step(error);
			cy_stepper_run_x(4,err_step);//内电机 CH4
			cy_stepper_wait_x(4);//等待运动完成
			
			if (cy_can_[1].updated == false){
				cy_can_resend();//重发CAN
			}
			while(cy_can_[1].updated == false);	//等待传感器更新
		}
		
//		
//		TIM3->CCR2 = 740;
//		//HAL_Delay(1000);
//		
//		TIM3->CCR3 = 1680;
//		//HAL_Delay(1000);
//		
//		if (flag_motor3){
//			flag_motor3 = false;
//			//旋转 5 度
//			//反转
//			{
//			int32_t step = yaw_angle_to_step(1);
//			cy_stepper_run_x(5,step);//等待步进电机调整完成
//				cy_stepper_wait_x(5);//等待运动完成
//			}
//		}
//		HAL_Delay(2000);
//		
//		
//		//TIM3->CCR2 = 740;
//		//HAL_Delay(1000);
//		
//		TIM3->CCR3 = 1700;
//		//HAL_Delay(1000);
// 		HAL_Delay(2000);
//			//旋转 5 度
//			//反转
//		{
//		int32_t step = yaw_angle_to_step(1);
//		cy_stepper_run_x(5,step);//等待步进电机调整完成
//			cy_stepper_wait_x(5);//等待运动完成
//		}
//		
//		
//		HAL_Delay(2000);
//		
//		TIM3->CCR2 = 700;
//		//HAL_Delay(1000);
//		
//		//TIM3->CCR3 = 1700;
//		//HAL_Delay(1000);
//			//旋转 5 度
//			//反转
//			{
//			int32_t step = yaw_angle_to_step(0.5);
//			cy_stepper_run_x(5,step);//等待步进电机调整完成
//				cy_stepper_wait_x(5);//等待运动完成
//			}
//		
//			
//			HAL_Delay(2000);
//			
//			
//			TIM3->CCR2 = 660;
//		
//			TIM3->CCR3 = 1740;
//			//旋转 5 度
//			//反转
//			{
//			int32_t step = yaw_angle_to_step(0.5);
//			cy_stepper_run_x(5,step);//等待步进电机调整完成
//				cy_stepper_wait_x(5);//等待运动完成
//			}
//		
//			
			
			
			
			
		
		
		
		
		
		
		
		
		
		//运动
		
		
		typedef struct{
			int x1;
			int y1;
			int x2;
			int y2;
		}point;
		
		
		point pt[9] = {
//			{10,15,2,29},
//			{6,24,7,44},
//			{17,15,13,25},
//			{16,20,31,21},
//			{31,21,29,25},
//			{23,24,22,44},
//			{22,44,15,44},
//			{16,30,12,38},
//			{27,30,33,39},
			{4,7,0,13},
			{2,11,3,19},
			{7,7,4,12},
			{5,9,12,9},
			{12,9,12,12},
			{8,11,8,19},
			{8,19,6,17},
			{6,13,4,15},
			{10,13,13,16},
		};
		
		
		
//		for (int i=0;i<9;++i){
//			pt[i].x1 -= 205;
//			pt[i].x2 -= 205;
//			pt[i].y1 -= 1475;
//			pt[i].y2 -= 1475;
//		}
		
		for (int i=0;i<9;++i){
			pt[i].y1 += 300;
			pt[i].y2 += 300;
		}
		
		
		
//		for(int i=0;i<9;++i){
//			
//			int start_pointx = pt[i].x1; int stop_pointx = pt[i].x2;
//			int start_pointy = pt[i].y1; int stop_pointy = pt[i].y2;
//			
//			abb_move_fast(start_pointx,start_pointy,100);
//			cy_stepper_wait_x(4);//等待运动完成
//			cy_stepper_wait_x(5);//等待运动完成
//			cy_stepper_wait_x(6);//等待运动完成
//			for (int i=0;i<10;++i){
//				abb_move_fast(start_pointx + i*(stop_pointx - start_pointx)/10.0f,
//				start_pointy + i*(stop_pointy - start_pointy)/10.0f,100);
//				cy_stepper_wait_x(4);//等待运动完成
//				cy_stepper_wait_x(5);//等待运动完成
//				cy_stepper_wait_x(6);//等待运动完成
//			}
//			abb_move_fast(stop_pointx,stop_pointy,100);
//			cy_stepper_wait_x(4);//等待运动完成
//			cy_stepper_wait_x(5);//等待运动完成
//			cy_stepper_wait_x(6);//等待运动完成
//			
//		}
		
		
		
		
//		
//		abb_move(-150,300,60);
//		
//		cy_stepper_wait_x(4);//等待运动完成
//		cy_stepper_wait_x(5);//等待运动完成
//		cy_stepper_wait_x(6);//等待运动完成
//		
//		
//		abb_move(-50,300,60);
//		
//		cy_stepper_wait_x(4);//等待运动完成
//		cy_stepper_wait_x(5);//等待运动完成
//		cy_stepper_wait_x(6);//等待运动完成
//		
//		abb_move(50,300,60);
//		
//		cy_stepper_wait_x(4);//等待运动完成
//		cy_stepper_wait_x(5);//等待运动完成
//		cy_stepper_wait_x(6);//等待运动完成
//		
//		abb_move(150,300,60);
//		
//		cy_stepper_wait_x(4);//等待运动完成
//		cy_stepper_wait_x(5);//等待运动完成
//		cy_stepper_wait_x(6);//等待运动完成
		
//		
		
		
		
		
		
		
	

	//cy_stepper_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  
	  cy_can_polling();
	  
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
#include <stdio.h>
int fputc(int ch, FILE* stream)
{
    //USART_SendData(USART1, (unsigned char) ch);
    //while (!(USART1->SR & USART_FLAG_TXE));
    
	//USART_SendChar(USART1, (uint8_t)ch);
    
	while(huart1.gState != HAL_UART_STATE_READY);
	HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&ch,1);
	
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
