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
#define ZLG_READ_ADDRESS1 0x01 // 读键值，也就是某个设备的寄存器的地址
#define ZLG_READ_ADDRESS2 0x10

#define ZLG_WRITE_ADDRESS1 0x10 // 写入到数码管当中的一个地址
#define ZLG_WRITE_ADDRESS2 0x11
#define BUFFER_SIZE1 (countof(Tx1_Buffer))
#define BUFFER_SIZE2 (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a))) // 求某个数组的元素个数

#define NOTE_SILENT 0 // 静音
#define NOTE_C4 261	  // 音符的频率
#define NOTE_D4 294
#define NOTE_E4 329
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 493
#define NOTE_C5 523

// 定义音符的持续时间（单位：ms）
#define DURATION_QUARTER 250		// 四分音符
#define DURATION_DOTTED_QUARTER 375 // 附点四分音符（四分音符时值的1.5倍）
#define DURATION_HALF 500			// 二分音符
#define max(a, b) a > b ? a : b
#define min(a, b) a < b ? a : b

typedef struct
{
	int frequency; // 音符的频率
	int duration;  // 音符的持续时间
} Note;

// 实例音符序列 "066336" 其中音符6是附点音符，包含一些音符和对应的时长
Note melody[] = {
	{NOTE_C4, DURATION_QUARTER},		// 0
	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (附点音符)

	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (附点音符)
	{NOTE_E4, DURATION_QUARTER},		// 3
	{NOTE_E4, DURATION_QUARTER},		// 3
	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (附点音符)
	{NOTE_A4, DURATION_DOTTED_QUARTER}	// 6 (附点音符)
};
// unsigned char seg7code[10] = {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6} ; // 数码管字根

__IO uint16_t adcx[4] = {0};  // 酒精传感器读取的值
unsigned char Empty[8] = {0}; // 用于睡觉时，熄灭数码管的功能
uint8_t Tx1_Buffer[8] = {0};  // 存储用于写到数码管当中的数组
IWDG_HandleTypeDef hiwdg;	  // 看门狗实例
bool st[5];					  // 存的某个状态的信息
int count = 0;				  // 发生警报的次数

enum SEQ1
{
	ALERT_SEQ1,
	SHOW_SEQ1
};

#define SLEEPTHRESHOLD 5	 // 某种睡眠阈值，判断5次浓度是否接近
#define ALERTTHRESHOLD 0.9	 // 超出浓度的阈值警报
#define MAXDIFFRENCE 0.1	 // 两个读取值之间的最大差异
#define SAMPLINGCOUNT 10	 // 采样次数
#define WARMSTARTFLAG 114514 // 热启动的标志

enum States
{
	TESTSTART,
	ALERT,
	SHOW,
	WAITINGFORHARDWARE,
	READ,
	WAKEUP,
	SHOULDSLEEP,
	DELAY,
	GOTOSLEEP,
	START,
	FEEDDOG
};

static uint32_t crc_table[256];
void (*function_array[5])() = {function1, function2, function3, function4, function5};

/*--------------------------安全部分-备份----------------------------*/
unsigned int remainingLoopToSleep __attribute__((section("NO_INIT"), zero_init));		  // 连续多少轮酒精值是不变的
unsigned int isSleeping __attribute__((section("NO_INIT"), zero_init));					  // 标记当前系统是否在睡觉
double alcoholLastTime __attribute__((section("NO_INIT"), zero_init));					  // 存储上一次的酒精值
unsigned int isWarmStarting __attribute__((section("NO_INIT"), zero_init));				  // 判断是否时热启动
unsigned int remainingHardWareWaitingLoop __attribute__((section("NO_INIT"), zero_init)); // 启动时等待一段时间
unsigned int state __attribute__((section("NO_INIT"), zero_init));						  // 当前处在的状态是哪个
float currentAlcohol __attribute__((section("NO_INIT"), zero_init));					  // 读取当前酒精的值
unsigned int st1[2] __attribute__((section("NO_INIT"), zero_init));						  // 用于存储某个状态的信息

unsigned int V_BackUP1 __attribute__((section("NO_INIT"), zero_init)); // 校验值1
unsigned int V_BackUP2 __attribute__((section("NO_INIT"), zero_init)); // 校验值2
unsigned int V_BackUP3 __attribute__((section("NO_INIT"), zero_init)); // 校验值3

// 备份结构体
typedef struct
{
	unsigned int remainingLoopToSleep;
	unsigned int isSleeping;
	double alcoholLastTime;
	unsigned int isWarmStarting;
	unsigned int remainingHardWareWaitingLoop;
	unsigned int state;
	float currentAlcohol;
	unsigned int st1[2];
	uint8_t Tx1_Buffer[8];

	unsigned int none[200]; // 用于在物理位置上面隔离不同的备份数据
} BackUP;

// 备份
// 凡是不会被初始化的变量都是很重要的变量！需要备份！ Need To DO
BackUP __attribute__((section("NO_INIT"), zero_init)) bk1;
BackUP __attribute__((section("NO_INIT"), zero_init)) bk2;
BackUP __attribute__((section("NO_INIT"), zero_init)) bk3;
void crc32(BackUP p);

void shuffle(int *arr, int len)
{
	for (int i = len - 1; i > 0; i--)
	{
		int j = rand() % (i + 1);
		int temp = arr[i];
		arr[i] = arr[j];
		arr[j] = temp;
	}
}

void set_zero(unsigned int buf[], int len)
{
	int indexs[len];
	for (int i = 0; i < len; i++)
	{
		indexs[i] = i;
	}
	shuffle(indexs, len);
	for (int i = 0; i < len; i++)
	{
		buf[indexs[i]] = 0;
	}
}

void set_zero_uint8(uint8_t buf[], int len)
{
	int indexs[len];
	for (int i = 0; i < len; i++)
	{
		indexs[i] = i;
	}
	shuffle(indexs, len);
	for (int i = 0; i < len; i++)
	{
		buf[indexs[i]] = 0;
	}
}

void buf_move(unsigned int buf1[], unsigned int buf2[], int len)
{
	int indexs[len];
	for (int i = 0; i < len; i++)
	{
		indexs[i] = i;
	}
	shuffle(indexs, len);
	for (int i = 0; i < len; i++)
	{
		buf2[indexs[i]] = buf1[indexs[i]];
	}
}

void buf_move_uint8(uint8_t buf1[], uint8_t buf2[], int len)
{
	int indexs[len];
	for (int i = 0; i < len; i++)
	{
		indexs[i] = i;
	}
	shuffle(indexs, len);
	for (int i = 0; i < len; i++)
	{
		buf2[indexs[i]] = buf1[indexs[i]];
	}
}

// 初始化数据备份
void initBackUP()
{ // 初始值全部赋为0
	bk1.remainingLoopToSleep = 0;
	bk1.isSleeping = 0;
	bk1.alcoholLastTime = 0;
	bk1.isWarmStarting = 0;
	bk1.remainingHardWareWaitingLoop = 0;
	bk1.state = 0;
	bk1.currentAlcohol = 0;
	set_zero(bk1.st1, 2);
	set_zero_uint8(bk1.Tx1_Buffer, 8);
	V_BackUP1 = 0;

	bk2.remainingLoopToSleep = 0;
	bk2.isSleeping = 0;
	bk2.alcoholLastTime = 0;
	bk2.isWarmStarting = 0;
	bk2.remainingHardWareWaitingLoop = 0;
	bk2.state = 0;
	bk2.currentAlcohol = 0;
	set_zero(bk2.st1, 2);
	set_zero_uint8(bk2.Tx1_Buffer, 8);
	V_BackUP2 = 0;

	bk3.remainingLoopToSleep = 0;
	bk3.isSleeping = 0;
	bk3.alcoholLastTime = 0;
	bk3.isWarmStarting = 0;
	bk3.remainingHardWareWaitingLoop = 0;
	bk3.state = 0;
	bk3.currentAlcohol = 0;
	set_zero(bk3.st1, 2);
	set_zero_uint8(bk3.Tx1_Buffer, 8);
	V_BackUP3 = 0;
}

// 更新一个备份
void update_bk(BackUP *bk, int flag)
{
	bk->remainingLoopToSleep = remainingHardWareWaitingLoop;
	bk->isSleeping = isSleeping;
	bk->alcoholLastTime = alcoholLastTime;
	bk->isWarmStarting = isWarmStarting;
	bk->remainingHardWareWaitingLoop = remainingHardWareWaitingLoop;
	bk->state = state;
	bk->currentAlcohol = currentAlcohol;
	buf_move(st1, bk->st1, 2);
	buf_move_uint8(Tx1_Buffer, bk->Tx1_Buffer, 8);

	int mode = rand() % 3;
	switch (mode)
	{
	case 1:
		if (flag == 1)
			V_BackUP1 = crc32(bk);
		if (flag == 2)
			V_BackUP2 = crc32(bk);
		if (flag == 3)
			V_BackUP3 = crc32(bk);
	case 2:
		if (flag == 2)
			V_BackUP2 = crc32(bk);
		if (flag == 1)
			V_BackUP1 = crc32(bk);
		if (flag == 3)
			V_BackUP3 = crc32(bk);
		break;
	case 3:
		if (flag == 3)
			V_BackUP3 = crc32(bk);
		if (flag == 1)
			V_BackUP1 = crc32(bk);
		if (flag == 2)
			V_BackUP2 = crc32(bk);
		break;
	}
}

// 更新所有备份
void update()
{
	update_bk(&bk1, 1);
	update_bk(&bk2, 2);
	update_bk(&bk3, 3);
	// isWarmStarting = WARMSTARTFLAG; /*问题2*/
}

/*
直接取整个数据
*/
void get_BackUP()
{
	int i = rand() % 3;
	int verify_error = 0;
	switch (i)
	{
	case 1:
		if (V_BackUP1 != crc32(bk1))
			verify_error += 1;
		if (V_BackUP2 != crc32(bk2))
			verify_error += 1;
		if (V_BackUP3 != crc32(bk3))
			verify_error += 1;
		break;
	case 2:
		if (V_BackUP2 != crc32(bk2))
			verify_error += 1;
		if (V_BackUP1 != crc32(bk1))
			verify_error += 1;
		if (V_BackUP3 != crc32(bk3))
			verify_error += 1;
		break;
	case 3:
		if (V_BackUP3 != crc32(bk3))
			verify_error += 1;
		if (V_BackUP1 != crc32(bk1))
			verify_error += 1;
		if (V_BackUP2 != crc32(bk2))
			verify_error += 1;
		break;
	}
	if (verify_error == 0)
	{
		remainingLoopToSleep = bk1.remainingHardWareWaitingLoop;
		isSleeping = bk1.isSleeping;
		alcoholLastTime = bk1.alcoholLastTime;
		isWarmStarting = bk1.isWarmStarting;
		remainingHardWareWaitingLoop = bk1.remainingHardWareWaitingLoop;
		state = bk1.state;
		currentAlcohol = bk1.currentAlcohol;
		buf_move(bk1.st1, st1, 2);
		buf_move_uint8(bk1.Tx1_Buffer, Tx1_Buffer, 8);
	}
	else
	{
		if (V_BackUP1 == crc32(bk1))
		{ // bk1校验通过
			remainingLoopToSleep = bk1.remainingHardWareWaitingLoop;
			isSleeping = bk1.isSleeping;
			alcoholLastTime = bk1.alcoholLastTime;
			isWarmStarting = bk1.isWarmStarting;
			remainingHardWareWaitingLoop = bk1.remainingHardWareWaitingLoop;
			state = bk1.state;
			currentAlcohol = bk1.currentAlcohol;
			buf_move(bk1.st1, st1, 2);
			buf_move_uint8(bk1.Tx1_Buffer, Tx1_Buffer, 8);

			bk2.remainingLoopToSleep = bk1.remainingHardWareWaitingLoop;
			bk2.isSleeping = bk1.isSleeping;
			bk2.alcoholLastTime = bk1.alcoholLastTime;
			bk2.isWarmStarting = bk1.isWarmStarting;
			bk2.remainingHardWareWaitingLoop = bk1.remainingHardWareWaitingLoop;
			bk2.state = bk1.state;
			bk2.currentAlcohol = bk1.currentAlcohol;
			buf_move(bk1.st1, bk2.st1, 2);
			buf_move_uint8(bk1.Tx1_Buffer, bk2.Tx1_Buffer, 8);
			V_BackUP2 = crc32(bk2);

			bk3.remainingLoopToSleep = bk1.remainingHardWareWaitingLoop;
			bk3.isSleeping = bk1.isSleeping;
			bk3.alcoholLastTime = bk1.alcoholLastTime;
			bk3.isWarmStarting = bk1.isWarmStarting;
			bk3.remainingHardWareWaitingLoop = bk1.remainingHardWareWaitingLoop;
			bk3.state = bk1.state;
			bk3.currentAlcohol = bk1.currentAlcohol;
			buf_move(bk1.st1, bk3.st1, 2);
			buf_move_uint8(bk1.Tx1_Buffer, bk3.Tx1_Buffer, 8);
			V_BackUP3 = crc32(bk3);
		}
		else if (V_BackUP2 == crc32(bk2))
		{ // bk2校验通过
			remainingLoopToSleep = bk2.remainingHardWareWaitingLoop;
			isSleeping = bk2.isSleeping;
			alcoholLastTime = bk2.alcoholLastTime;
			isWarmStarting = bk2.isWarmStarting;
			remainingHardWareWaitingLoop = bk2.remainingHardWareWaitingLoop;
			state = bk2.state;
			currentAlcohol = bk2.currentAlcohol;
			buf_move(bk2.st1, st1, 2);
			buf_move_uint8(bk2.Tx1_Buffer, Tx1_Buffer, 8);

			bk1.remainingLoopToSleep = bk2.remainingHardWareWaitingLoop;
			bk1.isSleeping = bk2.isSleeping;
			bk1.alcoholLastTime = bk2.alcoholLastTime;
			bk1.isWarmStarting = bk2.isWarmStarting;
			bk1.remainingHardWareWaitingLoop = bk2.remainingHardWareWaitingLoop;
			bk1.state = bk2.state;
			bk1.currentAlcohol = bk2.currentAlcohol;
			buf_move(bk2.st1, bk1.st1, 2);
			buf_move_uint8(bk2.Tx1_Buffer, bk1.bkTx1_Buffer, 8);
			V_BackUP1 = crc32(bk1);

			bk3.remainingLoopToSleep = bk2.remainingHardWareWaitingLoop;
			bk3.isSleeping = bk2.isSleeping;
			bk3.alcoholLastTime = bk2.alcoholLastTime;
			bk3.isWarmStarting = bk2.isWarmStarting;
			bk3.remainingHardWareWaitingLoop = bk2.remainingHardWareWaitingLoop;
			bk3.state = bk2.state;
			bk3.currentAlcohol = bk2.currentAlcohol;
			buf_move(bk2.st1, bk3.st1, 2);
			buf_move_uint8(bk2.Tx1_Buffer, bk3.Tx1_Buffer, 8);
			V_BackUP3 = crc32(bk3);
		}
		else if (V_BackUP3 == crc32(bk3))
		{ // bk3校验通过
			remainingLoopToSleep = bk3.remainingHardWareWaitingLoop;
			isSleeping = bk3.isSleeping;
			alcoholLastTime = bk3.alcoholLastTime;
			isWarmStarting = bk3.isWarmStarting;
			remainingHardWareWaitingLoop = bk3.remainingHardWareWaitingLoop;
			state = bk3.state;
			currentAlcohol = bk3.currentAlcohol;
			buf_move(bk3.st1, st1, 2);
			buf_move_uint8(bk3.Tx1_Buffer, Tx1_Buffer, 8);

			bk1.remainingLoopToSleep = bk3.remainingHardWareWaitingLoop;
			bk1.isSleeping = bk3.isSleeping;
			bk1.alcoholLastTime = bk3.alcoholLastTime;
			bk1.isWarmStarting = bk3.isWarmStarting;
			bk1.remainingHardWareWaitingLoop = bk3.remainingHardWareWaitingLoop;
			bk1.state = bk3.state;
			bk1.currentAlcohol = bk3.currentAlcohol;
			buf_move(bk3.st1, bk1.st1, 2);
			buf_move_uint8(bk3.Tx1_Buffer, bk1.Tx1_Buffer, 8);
			V_BackUP1 = crc32(bk1);

			bk2.remainingLoopToSleep = bk3.remainingHardWareWaitingLoop;
			bk2.isSleeping = bk3.isSleeping;
			bk2.alcoholLastTime = bk3.alcoholLastTime;
			bk2.isWarmStarting = bk3.isWarmStarting;
			bk2.remainingHardWareWaitingLoop = bk3.remainingHardWareWaitingLoop;
			bk2.state = bk3.state;
			bk2.currentAlcohol = bk3.currentAlcohol;
			buf_move(bk3.st1, bk2.st1, 2);
			buf_move_uint8(bk3.Tx1_Buffer, bk2.Tx1_Buffer, 8);
			V_BackUP2 = crc32(bk2);
		}
		else
			Error_Handler();
	}
}

/*--------------------------安全部分-备份----------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void ErrorState()
{
	state = START;
	update();
	printf("\r\n发生错误，进行热启动\n\r"); // 将当前状态转到START，打印报错信息，系统出现异常，开始热启动
}

// {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6}
int seg7code(int x) // 将输入数字x（0-9）映射到数码管当中显示
{
	if (x < 0 || x >= 10)
	{
		printf("\r\n无效输入\n\r"); // 无效输入
		return 0;
	}

	switch (x)
	{
	case 0:
		return 0xFC;
	case 1:
		return 0x0C;
	case 2:
		return 0xDA;
	case 3:
		return 0xF2;
	case 4:
		return 0x66;
	case 5:
		return 0xB6;
	case 6:
		return 0xBE;
	case 7:
		return 0xE0;
	case 8:
		return 0xFE;
	case 9:
		return 0xE6;
	default:
		return 0; // 默认返回，ADD
	}
}

void alert(int count) // 用于警报，count为警报次数
{
	int length = sizeof(melody) / sizeof(melody[0]);
	while (count--) // 循环播放 melody
	{
		for (int i = 0; i < length; i++)
		{
			int frequency = melody[i].frequency;
			int duration = melody[i].duration;

			if (frequency == NOTE_SILENT)
			{
				HAL_Delay(duration); // 如果是静音，延迟相应的时长
				continue;
			}

			int period = 1000 / frequency; // 周期（单位：ms）
			int half_period = period / 2;  // 半周期

			int cycles = (frequency * duration) / 1000; // 在给定持续时间内的总周期数

			for (int j = 0; j < cycles; j++)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);	  // 打开
				HAL_Delay(half_period);								  // 等待半周期
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // 关闭
				HAL_Delay(half_period);								  // 等待半周期
			}

			HAL_Delay(50); // 音符之间的短暂延迟
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

	Tx1_Buffer[0] |= 1; // 添加小数点的功能
	update();
	I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8); // 0x70是目标设备的地址，ZLG—W是设备的特定寄存器，Tx1是要写入的缓冲区，8是指8个字节
}

// 用于安全地显示浮点数
void ShowSafe(double data) // To Do
{
	get_BackUP();
	if (state != SHOW)
	{
		ErrorState();
		return;
	}
	Show(data);
	st1[SHOW_SEQ1] = 1;
	if (st1[ALERT_SEQ1])
		state = SHOULDSLEEP;
	else
		state = ALERT;
	update();
}

// @brief: 显示整数
void ShowInteger(unsigned data)
{
	/*****************数据转换*****************************/

	if (data > 99999999)
	{
		data = 99999999;
		printf("\r\n数码管溢出！\n\r"); // 打印数码管溢出
	}

	for (int i = 0; i < 8; i++)
	{
		int idx = data % 10;
		Tx1_Buffer[7 - i] = seg7code(idx);
		data /= 10;
	}

	Tx1_Buffer[0] |= 1;
	update();
	get_BackUP();
	I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
}

// 去掉了最大最小值，还有异常检测，还有多重采样的功能
float ReadAlcohol(void)
{
	double maxAlcohol = 0;	// 存储读取值中的最大值
	double minAlcohol = 10; // 存储读取值中的最小值
	double uncommonLarge = 100;
	double uncommonSmall = 1e-3;			// 用于检测异常值的阈值
	double sum = 0;							// 用于累加读取值的总和
	for (int i = 0; i < SAMPLINGCOUNT; i++) // 读取10次酒精的值
	{
		double temp = (float)adcx[0] * (3.3 / 4096); // adcx这个应该是读取传感器的值
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
		printf("\r\n传感器疑似异常！\r\n"); // 这里应该重启，并且热启动
		return -1;
	}

	return sum / total;
}

// 判断去睡觉还是喂狗，也就是看当前状态酒精的值是否持续保持不变，持续不变则去睡觉，否则喂狗
void ShouldSleep(float temp)
{

	// 这里我们首先判断当前浓度和上一次浓度是否差距过大。若是，则remainingLoopToSleep自减
	// remainingLoopToSleep减到0则去睡觉
	get_BackUP();
	if (state != SHOULDSLEEP)
	{
		ErrorState();
		return;
	}
	printf("\r\n这里应该检查完整性\n\r"); // Need To Do，打印“这里应该对数据完整性进行一个检查”
	get_BackUP();
	if (temp < 0.4 &&
		(temp < alcoholLastTime && alcoholLastTime - temp < MAXDIFFRENCE ||
		 temp >= alcoholLastTime && temp - alcoholLastTime < MAXDIFFRENCE))
	{
		remainingLoopToSleep--;
		update();
	}
	else
	{
		// 若浓度出现了较大差距，我们则更新remainingLoopToSleep
		remainingLoopToSleep = SLEEPTHRESHOLD;
		update();
	}

	// 更新上次浓度
	alcoholLastTime = temp;
	update();

	get_BackUP();
	if (remainingLoopToSleep == 0)
	{
		remainingLoopToSleep = SLEEPTHRESHOLD;
		state = GOTOSLEEP;
	}
	else
		state = FEEDDOG;
	update();
}

void GoToSleep(void)
{
	get_BackUP();
	if (state != GOTOSLEEP)
	{
		ErrorState();
		return;
	}
	printf("\r\n准备睡觉\n\r"); // 准备睡觉
	I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Empty, 8);
	isSleeping = 1;
	update();
	// HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	state = FEEDDOG;
	update();
}

void WakeUp(void)
{
	get_BackUP();
	if (state != WAKEUP)
	{
		ErrorState();
		return;
	}
	// 这里我们只读一次，也不滤波，因为我们只是需要判断当前是否需要醒来
	float temp = (float)adcx[0] * (3.3 / 4096);
	remainingLoopToSleep = SLEEPTHRESHOLD;
	update();
	printf("\r\n准备醒来\n\r"); // 准备醒来
	get_BackUP();
	if (!(temp > alcoholLastTime && temp - alcoholLastTime < MAXDIFFRENCE ||
		  temp <= alcoholLastTime && alcoholLastTime - temp < MAXDIFFRENCE))
	{
		// 发现传感器数值差距大了，正式醒来
		isSleeping = 0;
		update();
		// HAL_PWR_DisableSleepOnExit();
		// HAL_ResumeTick();
		Show(temp);
	}
	else
	{
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}

	state = FEEDDOG; // 对应状态机当中，走到下一个状态的步骤
	update();
}

// 硬件，端口和看门狗初始化
void InitAll(void)
{
	get_BackUP();
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

	// 这一部分6条老师说初始化顺序无关，可以进行随机初始化
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_ADC3_Init();
	MX_USART1_UART_Init();
	init_crc_table();								// 初始化 CRC 查找表
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcx, 4); // 开启ADC转换

	// 初始化开门狗
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	hiwdg.Init.Reload = 0xBFFF; // 疑问：这里时钟频率是多少，分频这里采用了128？
	// 注意，看门狗频率改了，我按照实验报告的公式，将时间改成了1s
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Start(&hiwdg);
	printf("\r\n硬件、端口和看门狗初始化完毕！\n\r"); // 硬件，端口和看门狗初始化完成
	state = TESTSTART;
	update();
}

// 启动时延时，导停一段时间
void WaitingForHardWare()
{
	get_BackUP();
	if (state != WAITINGFORHARDWARE)
	{
		ErrorState();
		return;
	}
	get_BackUP();
	while (remainingHardWareWaitingLoop > 0)
	{

		HAL_Delay(1000);
		remainingHardWareWaitingLoop--;
		update();
		ShowInteger(remainingHardWareWaitingLoop);
	}
	get_BackUP();
	if (!isSleeping)
		state = READ;
	else
		state = WAKEUP;
	update();
}

// 判断是热启动还是冷启动
void TestStart(void)
{
	get_BackUP();
	if (state != TESTSTART)
	{
		ErrorState();
		return;
	}
	get_BackUP();
	if (isWarmStarting == WARMSTARTFLAG /*&& У?????*/)
	{
		printf("\r\n热启动！\n\r"); // 打印，热启动，注意这里还要补充校验
		get_BackUP();				// 取备份数据（取的过程当中，就包含了检验的部分）
	}
	else
	{
		// 这里也可以随机化
		printf("\r\n冷启动！\n\r"); // 冷启动
		remainingLoopToSleep = SLEEPTHRESHOLD;
		isSleeping = 1;
		isWarmStarting = WARMSTARTFLAG;
		remainingHardWareWaitingLoop = 20;
		initBackUP();
		update();
	}
	state = WAITINGFORHARDWARE;
	update();
}

void ReadSafe()
{
	get_BackUP();
	if (state != READ)
	{
		ErrorState();
		return;
	}
	currentAlcohol = ReadAlcohol();
	update();
	if (currentAlcohol < 0)
	{
		ErrorState();
		return;
	}
	update();											// currentAlcohol发送改变，进行备份！
	printf("\r\n 酒精传感器 =%f \n\r", currentAlcohol); // 酒精传感器
	for (int i = 0; i < 2; i++)							// To Do，这里的st的作用是标记SHOW和ALERT这两个状态是否走过的感觉
	{
		st1[i] = 0;
	}

	int temp = rand() % 2; // 先发生警报还是打印数字都是可以的
	if (temp == 0)
	{
		state = SHOW;
	}
	else
	{
		state = ALERT;
	}
	update();
}

void AlertSafe()
{
	get_BackUP();
	if (state != ALERT)
	{
		ErrorState();
		return;
	}
	// printf("\r\n????Alert??????\n\r"); // 处于Alert函数当中
	get_BackUP(); // 使用currentAlcohol数据之前，便需要进行校验！
	if (currentAlcohol > ALERTTHRESHOLD)
		alert(1);
	st1[ALERT_SEQ1] = 1;
	if (st1[SHOW_SEQ1])
	{
		state = SHOULDSLEEP;
	}
	else
	{
		state = SHOW;
	}
	update();
}

void FeedDog()
{
	get_BackUP();
	if (state != FEEDDOG)
	{
		ErrorState();
		return;
	}
	HAL_IWDG_Refresh(&hiwdg);

	state = DELAY;
	update();
}

void function1()
{
	get_BackUP();
	if (state != DELAY)
	{
		ErrorState();
		return;
	}
	HAL_Delay(1000);

	get_BackUP();
	if (isSleeping)
		state = WAKEUP;
	else
		state = READ;
	update();
}

void function2()
{
	get_BackUP();
	if (state != DELAY)
	{
		ErrorState();
		return;
	}
	HAL_Delay(1005);

	get_BackUP();
	if (isSleeping)
		state = WAKEUP;
	else
		state = READ;
	update();
}

void function3()
{
	get_BackUP();
	if (state != DELAY)
	{
		ErrorState();
		return;
	}
	HAL_Delay(1010);

	get_BackUP();
	if (isSleeping)
		state = WAKEUP;
	else
		state = READ;
	update();
}

void function4()
{
	get_BackUP();
	if (state != DELAY)
	{
		ErrorState();
		return;
	}
	HAL_Delay(1015);

	get_BackUP();
	if (isSleeping)
		state = WAKEUP;
	else
		state = READ;
	update();
}

void function5()
{
	get_BackUP();
	if (state != DELAY)
	{
		ErrorState();
		return;
	}
	HAL_Delay(1020);

	get_BackUP();
	if (isSleeping)
		state = WAKEUP;
	else
		state = READ;
	update();
}

void Delay()
{
	int random_number = rand() % 5;
	function_array[random_number]();
}

// 初始化 CRC-32 查找表
void init_crc_table()
{
	uint32_t crc;
	for (int i = 0; i < 256; i++)
	{
		crc = i;
		for (int j = 8; j > 0; j--)
		{
			if (crc & 1)
			{
				crc = (crc >> 1) ^ 0xEDB88320;
			}
			else
			{
				crc >>= 1;
			}
		}
		crc_table[i] = crc;
	}
}

uint32_t crc32(BackUP p)
{
	size_t length = sizeof p;
	unsigned char *data = (unsigned char *)&p;
	uint32_t crc = 0xFFFFFFFF;
	for (size_t i = 0; i < length; i++)
	{
		crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0xFF];
	}
	return crc ^ 0xFFFFFFFF;
}

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/
	initBackUP();
	state = START;
	update();
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		get_BackUP();
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
			printf("\r\n存在未识别状态！\n\r"); // 存在未识别的状态
			ErrorState();
		}
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
		; // 循环发送，直到发送完毕
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
