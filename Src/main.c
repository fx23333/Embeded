/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_hal.h"


#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "tim.h"

/* USER CODE BEGIN Includes */
#include "zlg7290.h"
#include "stdio.h"
#include "stdio.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint32_t adcx[4]={0};
__IO uint16_t adcx[4]={0};
float temp0,temp1,temp2,temp3;
float delta = 3.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define ZLG_READ_ADDRESS1         0x01
#define ZLG_READ_ADDRESS2         0x10

#define ZLG_WRITE_ADDRESS1        0x10
#define ZLG_WRITE_ADDRESS2        0x11
#define BUFFER_SIZE1              (countof(Tx1_Buffer))
#define BUFFER_SIZE2              (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a)))
	
#define NOTE_SILENT 0    // ??
#define NOTE_C4  261
#define NOTE_D4  294
#define NOTE_E4  329
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  493
#define NOTE_C5  523

// ?????????(??:ms)
#define DURATION_QUARTER  250  // ????
#define DURATION_DOTTED_QUARTER  375  // ?????? (???????1.5?)
#define DURATION_HALF     500  // ????

typedef struct {
    int frequency;  // ?????
    int duration;   // ???????
} Note;

// ?????? "066336" ????6?????
Note melody[] = {
    {NOTE_C4, DURATION_QUARTER},           // 0
    {NOTE_A4, DURATION_DOTTED_QUARTER},    // 6 (????)
    {NOTE_A4, DURATION_DOTTED_QUARTER},    // 6 (????)
    {NOTE_E4, DURATION_QUARTER},           // 3
    {NOTE_E4, DURATION_QUARTER},           // 3
    {NOTE_A4, DURATION_DOTTED_QUARTER},    // 6 (????)
    {NOTE_A4, DURATION_DOTTED_QUARTER}     // 6 (????,??)
};
unsigned char seg7code[10]={ 0xFC,0x0C,0xDA,0xF2,0x66,0xB6,0xBE,0xE0,0xFE,0xE6}; //数码管字根
uint8_t Tx1_Buffer[8]={0};
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void alert(int count)
{
    int length = sizeof(melody) / sizeof(melody[0]);
    while (count--) {
        for (int i = 0; i < length; i++) {
            int frequency = melody[i].frequency;
            int duration = melody[i].duration;

            if (frequency == NOTE_SILENT) {
                HAL_Delay(duration); // ?????,????
                continue;
            }

            int period = 1000 / frequency;  // ??(??:ms)
            int half_period = period / 2;   // ???

            int cycles = (frequency * duration) / 1000;  // ?????????????

            for (int j = 0; j < cycles; j++) {
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); // ??
                HAL_Delay(half_period); // ?????
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // ??
                HAL_Delay(half_period); // ?????
            }

            HAL_Delay(50); // ?????????
        }
    }
}

void Show(double data) //显示函数
{
/*****************数据转换*****************************/ 

    for (int i = 0; i < 8; i++)
    {
        int idx = (int)data %10;
        Tx1_Buffer[i] = seg7code[idx];
        data *=10;
    }
    
    Tx1_Buffer[0] |= 1;
  I2C_ZLG7290_Write(&hi2c1,0x70,ZLG_WRITE_ADDRESS1,Tx1_Buffer,8);
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adcx,4);//开启ADC转换
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	temp0 = (float)adcx[0]*(3.3/4096);
	temp1 = (float)adcx[1]*(3.3/4096); 
	temp2 = (float)adcx[2]*(3.3/4096); 
	temp3 = (float)adcx[3]*(3.3/4096); 	
	printf("\r\n 酒精传感器 =%f \n\r",temp0);
	// 显示temp0
	if(1)alert(1);
	
	Show(temp0);	
	HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
/* USER CODE END 4 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
