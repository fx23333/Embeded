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
#include "i2c.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#define ZLG_READ_ADDRESS1 0x01
#define ZLG_READ_ADDRESS2 0x10

#define ZLG_WRITE_ADDRESS1 0x10
#define ZLG_WRITE_ADDRESS2 0x11
#define BUFFER_SIZE1 (countof(Tx1_Buffer))
#define BUFFER_SIZE2 (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a)))

#define NOTE_SILENT 0 // ??
#define NOTE_C4 261
#define NOTE_D4 294
#define NOTE_E4 329
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 493
#define NOTE_C5 523

// ?????????(??:ms)
#define DURATION_QUARTER 250        // ????
#define DURATION_DOTTED_QUARTER 375 // ?????? (???????1.5?)
#define DURATION_HALF 500           // ????
#define max(a, b) a > b ? a : b
#define min(a, b) a < b ? a : b

typedef struct
{
    int frequency; // ?????
    int duration;  // ???????
} Note;

// ?????? "066336" ????6?????
Note melody[] = {
    {NOTE_C4, DURATION_QUARTER},        // 0
    {NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (????)
		
    {NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (????)
    {NOTE_E4, DURATION_QUARTER},        // 3
    {NOTE_E4, DURATION_QUARTER},        // 3
    {NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (????)
    {NOTE_A4, DURATION_DOTTED_QUARTER}  // 6 (????,??)
};
// unsigned char seg7code[10] = {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6} ; // 数码管字根


__IO uint16_t adcx[4] = {0};
unsigned char Empty[8] = {0};
uint8_t Tx1_Buffer[8] = {0};
IWDG_HandleTypeDef hiwdg;

// 凡是不会被初始化的变量都是重要变量！需要备份！
unsigned int remainingLoopToSleep __attribute__( (section("NO_INIT"), zero_init) );
unsigned int isSleeping __attribute__( (section("NO_INIT"), zero_init) );
double alcoholLastTime __attribute__( (section("NO_INIT"), zero_init) );
unsigned int isWarmStarting __attribute__( (section("NO_INIT"), zero_init) );
unsigned int remainingHardWareWaitingLoop __attribute__( (section("NO_INIT"), zero_init) );
unsigned int state __attribute__( (section("NO_INIT"), zero_init) );
float currentAlcohol __attribute__( (section("NO_INIT"), zero_init) );
#define SLEEPTHRESHOLD 5
#define ALERTTHRESHOLD 0.9
#define MAXDIFFRENCE 0.1
#define SAMPLINGCOUNT 10
#define WARMSTARTFLAG 114514


enum States {
		TESTSTART, ALERT, SHOW, WAITINGFORHARDWARE, READ, WAKEUP, SHOULDSLEEP, DELAY, GOTOSLEEP, START, FEEDDOG
};

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void ErrorState()
{
	  state = START;
	  printf("\r\n系统出现异常，开始热启动\n\r");
}

// {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6}
int seg7code(int x) 
{
	  if (x < 0 || x >= 10)
		{
			  printf("\r\n无效输入\n\r");
				return 0;
		}
		
		switch (x)
	  {
			  case 0: return  0xFC;
			  case 1: return 0x0C;
				case 2: return 0xDA;
				case 3: return 0xF2;
				case 4: return 0x66;
				case 5: return 0xB6;
				case 6: return 0xBE;
				case 7: return 0xE0;
				case 8: return 0xFE;
				case 9: return 0xE6;
		}
		
}



void alert(int count)
{
    int length = sizeof(melody) / sizeof(melody[0]);
    while (count--)
    {
        for (int i = 0; i < length; i++)
        {
            int frequency = melody[i].frequency;
            int duration = melody[i].duration;

            if (frequency == NOTE_SILENT)
            {
                HAL_Delay(duration); // ?????,????
                continue;
            }

            int period = 1000 / frequency; // ??(??:ms)
            int half_period = period / 2;  // ???

            int cycles = (frequency * duration) / 1000; // ?????????????

            for (int j = 0; j < cycles; j++)
            {
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // ??
                HAL_Delay(half_period);                               // ?????
                HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // ??
                HAL_Delay(half_period);                               // ?????
            }

            HAL_Delay(50); // ?????????
        }
    }
}

// @brief: 显示浮点数
void Show(double data) 
{
    /*****************数据转换*****************************/

		
		
		
    for (int i = 0; i < 8; i++)
    {
        int idx = (int)data % 10;
        Tx1_Buffer[i] = seg7code(idx);
        data *= 10;
    }

    Tx1_Buffer[0] |= 1;
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
		
		
		
		
}

void ShowSafe(double data)
{
	  if (state != SHOW)
		{
			  ErrorState();
			  return;
		}
		Show(data);
		state = SHOULDSLEEP;
}

// @brief: 显示整数
void ShowInteger(unsigned data) 
{
    /*****************数据转换*****************************/

		if (data > 99999999) 
		{
			  data = 99999999;
			  printf("\r\n数码管溢出！\n\r");
		}
		
    for (int i = 0; i < 8; i++)
    {
        int idx = data % 10;
        Tx1_Buffer[7 - i] = seg7code(idx);
			  data /= 10;
    }

    Tx1_Buffer[0] |= 1;
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
}

float ReadAlcohol(void)
{
    double maxAlcohol = 0;
    double minAlcohol = 10;
    double uncommonLarge = 100;
    double uncommonSmall = 1e-3;
    double sum = 0;
    for (int i = 0; i < SAMPLINGCOUNT; i++)
    {
        double temp = (float)adcx[0] * (3.3 / 4096);
        maxAlcohol = max(temp, maxAlcohol);
        minAlcohol = min(temp, minAlcohol);
        sum += temp;
    }
    int total = SAMPLINGCOUNT;
    if (SAMPLINGCOUNT)
    {
        sum -= maxAlcohol;
        sum -= minAlcohol;
        total -= 2;
    }

    if (minAlcohol < uncommonSmall || maxAlcohol > uncommonLarge)
    {
        printf("这里应该重启并且热启动！\n");
    }

    return sum / total;
}

void ShouldSleep(float temp)
{

    // 这里我们首先判断当前浓度和上一次浓度是否差距过大，若是，则remainingLoopToSleep自减
    // remainingLoopToSleep减到0则去睡觉
		if (state != SHOULDSLEEP) 
		{
			  ErrorState();
			  return;
		}
	  printf("\r\n这里应该对数据完整性进行一个检查！\n\r");
    if (temp < 0.4 && 
			(temp < alcoholLastTime && alcoholLastTime - temp < MAXDIFFRENCE ||
        temp >= alcoholLastTime && temp - alcoholLastTime < MAXDIFFRENCE))
    {
        remainingLoopToSleep--;
    }
    else
    {
        // 若浓度出现了较大差距，我们则更新remainingLoopToSleep
        remainingLoopToSleep = SLEEPTHRESHOLD;
    }

    // 更新上次浓度
    alcoholLastTime = temp;

    if (remainingLoopToSleep == 0)
    {
        remainingLoopToSleep = SLEEPTHRESHOLD;
			  state = GOTOSLEEP;
    }
    else
        state = FEEDDOG;
}

void GoToSleep(void)
{
	  if (state != GOTOSLEEP)
		{
			  ErrorState();
			  return;
		}
    printf("\r\n准备睡觉\n\r");
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Empty, 8);
    isSleeping = 1;
    // HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		state = FEEDDOG;
}

void WakeUp(void)
{
		if (state != WAKEUP)
		{
			  ErrorState();
			  return;
		}
	  // 这里我们只读一次，也不滤波，因为我们只是需要判断当前是否需要醒来
	  float temp = (float)adcx[0] * (3.3 / 4096);
	  remainingLoopToSleep = SLEEPTHRESHOLD;
	  printf("\r\n准备醒来\n\r");
	  if (!(temp > alcoholLastTime && temp - alcoholLastTime < MAXDIFFRENCE ||
			temp <= alcoholLastTime && alcoholLastTime - temp < MAXDIFFRENCE))
		{
			  // 发现传感器数值差距大了，正式醒来
			  isSleeping = 0;
			  // HAL_PWR_DisableSleepOnExit();
			  // HAL_ResumeTick();
			  Show(0);
		}
		
		
		state = FEEDDOG;
		
}

void InitAll(void) 
{    
	
	  if (state != START)
		{
			  ErrorState();
			  HAL_Delay(1000);
			  return;
		}
	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();
	  /* Initialize all configured peripherals */

    // 这一部分6条似乎老师说初始化顺序无关，可以进行随机化执行
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_ADC3_Init();
    MX_USART1_UART_Init();
	  
	
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcx, 4); // 开启ADC转换
		
		
		// 初始化看门狗
		hiwdg.Instance = IWDG;
		hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
		hiwdg.Init.Reload = 0x01D3;
		HAL_IWDG_Init(&hiwdg);
		HAL_IWDG_Start(&hiwdg);
	  printf("\r\n硬件，端口和看门狗初始化完毕！\n\r");
		state = TESTSTART;
}

void WaitingForHardWare()
{
	  if (state != WAITINGFORHARDWARE)
		{
			  ErrorState();
			  return;
		}
	  while (remainingHardWareWaitingLoop > 0) 
		{
			  // 这里两个代码块之间可以随机化
				{
						remainingHardWareWaitingLoop--;
						ShowInteger(remainingHardWareWaitingLoop);
				}
				
				{
						HAL_Delay(1000);
				}
		}
		if (!isSleeping)
				state = READ;
		else
			  state = WAKEUP;
}

void TestStart(void)
{
		if (state != TESTSTART)
		{
			  ErrorState();
			  return;
		}
	  if (isWarmStarting == WARMSTARTFLAG /*&& 校验通过*/)
		{
			  printf("\r\n热启动，注意这里还需要补充校验！！！！！\n\r");
		}
		else
		{
			  // 这里也可以随机化
			  printf("\r\n冷启动\n\r");
		    remainingLoopToSleep = SLEEPTHRESHOLD;
        isSleeping = 1;
		    isWarmStarting = WARMSTARTFLAG;
			  remainingHardWareWaitingLoop = 6;
		}
		state = WAITINGFORHARDWARE;
}

void ReadSafe()
{
	  if (state != READ)
		{
			  ErrorState();
			  return;
		}
		currentAlcohol = ReadAlcohol();
		printf("\r\n 酒精传感器 =%f \n\r", currentAlcohol);
		state = ALERT;
}

void AlertSafe()
{
		if (state != ALERT)
		{
			  ErrorState();
			  return;
		}
		// printf("\r\n处于Alert函数中\n\r");
	  if (currentAlcohol > ALERTTHRESHOLD)
			  alert(1);
		state = SHOW;
}

void FeedDog()
{
	  if (state != FEEDDOG)
		{
				ErrorState();
			  return;
		}
		HAL_IWDG_Refresh(&hiwdg);
		
		state = DELAY;
		
}

void Delay()
{
	  if (state != DELAY)
		{
			  ErrorState();
			  return;
		}
	  HAL_Delay(1000);
		
		if (isSleeping)
		    state = WAKEUP;
		else
			  state = READ;
}



int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/
		state = START;
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
			  switch (state) 
				{
					case START: 
						InitAll();
					  break;
					case TESTSTART:
						TestStart();
					  break;
					case WAITINGFORHARDWARE:
						WaitingForHardWare();
					  break;
					case READ:
						ReadSafe();
					  break;
					case SHOULDSLEEP:
						ShouldSleep(currentAlcohol);
					  break;
					case GOTOSLEEP:
						GoToSleep();
					  break;
					case WAKEUP:
						WakeUp();
					  break;
					case DELAY:
					  Delay();
						break;
					case ALERT:
						AlertSafe();
						break;
					case SHOW:
						ShowSafe(currentAlcohol);
					  break;
					case FEEDDOG:
						FeedDog();
					  break;
					default:
						printf("\r\n存在未识别状态！！！\n\r");
						ErrorState();
				}
			
			
			
//				if (!isSleeping) 
//				{
//					// Step 1 读取酒精传感器
//						ReadSafe();

//						// Step 2 判断阈值并报警；2和3可颠倒进行随机化执行
//						AlertSafe();

//						// Step 3 显示数字；2和3可颠倒
//						Show(currentAlcohol);

//						// step 4 睡眠躲避干扰：判断是否应该睡眠
//						ShouldSleep(currentAlcohol);

//						// Step5 如果应该睡眠则进行睡眠
//						if (shouldSleep)
//								GoToSleep();
//			  }
//        // Step6 检查是否从睡眠状态苏醒
//        if (isSleeping)
//        {
//					  WakeUp();
//        }
//        HAL_Delay(1000);
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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0)
        ; // 循环发送,直到发送完毕
    USART1->DR = (uint8_t)ch;
    return ch;
}
/* USER CODE END 4 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
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
