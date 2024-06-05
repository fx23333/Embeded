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
#define ZLG_READ_ADDRESS1 0x01 // ����ֵ��Ҳ����ĳ���豸�ļĴ����ĵ�ַ
#define ZLG_READ_ADDRESS2 0x10

#define ZLG_WRITE_ADDRESS1 0x10 // д�뵽����ܵ��е�һ����ַ
#define ZLG_WRITE_ADDRESS2 0x11
#define BUFFER_SIZE1 (countof(Tx1_Buffer))
#define BUFFER_SIZE2 (countof(Rx2_Buffer))
#define countof(a) (sizeof(a) / sizeof(*(a))) // ��ĳ�������Ԫ�ظ���

#define NOTE_SILENT 0 // ����
#define NOTE_C4 261	  // ������Ƶ��
#define NOTE_D4 294
#define NOTE_E4 329
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 493
#define NOTE_C5 523

// ���������ĳ���ʱ�䣨��λ��ms��
#define DURATION_QUARTER 250		// �ķ�����
#define DURATION_DOTTED_QUARTER 375 // �����ķ��������ķ�����ʱֵ��1.5����
#define DURATION_HALF 500			// ��������
#define max(a, b) a > b ? a : b
#define min(a, b) a < b ? a : b

typedef struct
{
	int frequency; // ������Ƶ��
	int duration;  // �����ĳ���ʱ��
} Note;

// ʵ���������� "066336" ��������6�Ǹ�������������һЩ�����Ͷ�Ӧ��ʱ��
Note melody[] = {
	{NOTE_C4, DURATION_QUARTER},		// 0
	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (��������)

	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (��������)
	{NOTE_E4, DURATION_QUARTER},		// 3
	{NOTE_E4, DURATION_QUARTER},		// 3
	{NOTE_A4, DURATION_DOTTED_QUARTER}, // 6 (��������)
	{NOTE_A4, DURATION_DOTTED_QUARTER}	// 6 (��������)
};
// unsigned char seg7code[10] = {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6} ; // ������ָ�

__IO uint16_t adcx[4] = {0};  // �ƾ���������ȡ��ֵ
unsigned char Empty[8] = {0}; // ����˯��ʱ��Ϩ������ܵĹ���
uint8_t Tx1_Buffer[8] = {0};  // �洢����д������ܵ��е�����
IWDG_HandleTypeDef hiwdg;	  // ���Ź�ʵ��
bool st[5];					  // ���ĳ��״̬����Ϣ
int count = 0;				  // ���������Ĵ���

enum SEQ1
{
	ALERT_SEQ1,
	SHOW_SEQ1
};

#define SLEEPTHRESHOLD 5	 // ĳ��˯����ֵ���ж�5��Ũ���Ƿ�ӽ�
#define ALERTTHRESHOLD 0.9	 // ����Ũ�ȵ���ֵ����
#define MAXDIFFRENCE 0.1	 // ������ȡֵ֮���������
#define SAMPLINGCOUNT 10	 // ��������
#define WARMSTARTFLAG 114514 // �������ı�־

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

/*--------------------------��ȫ����-����----------------------------*/
unsigned int remainingLoopToSleep __attribute__((section("NO_INIT"), zero_init));		  // ���������־ƾ�ֵ�ǲ����
unsigned int isSleeping __attribute__((section("NO_INIT"), zero_init));					  // ��ǵ�ǰϵͳ�Ƿ���˯��
double alcoholLastTime __attribute__((section("NO_INIT"), zero_init));					  // �洢��һ�εľƾ�ֵ
unsigned int isWarmStarting __attribute__((section("NO_INIT"), zero_init));				  // �ж��Ƿ�ʱ������
unsigned int remainingHardWareWaitingLoop __attribute__((section("NO_INIT"), zero_init)); // ����ʱ�ȴ�һ��ʱ��
unsigned int state __attribute__((section("NO_INIT"), zero_init));						  // ��ǰ���ڵ�״̬���ĸ�
float currentAlcohol __attribute__((section("NO_INIT"), zero_init));					  // ��ȡ��ǰ�ƾ���ֵ
unsigned int st1[2] __attribute__((section("NO_INIT"), zero_init));						  // ���ڴ洢ĳ��״̬����Ϣ

unsigned int V_BackUP1 __attribute__((section("NO_INIT"), zero_init)); // У��ֵ1
unsigned int V_BackUP2 __attribute__((section("NO_INIT"), zero_init)); // У��ֵ2
unsigned int V_BackUP3 __attribute__((section("NO_INIT"), zero_init)); // У��ֵ3

// ���ݽṹ��
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

	unsigned int none[200]; // ����������λ��������벻ͬ�ı�������
} BackUP;

// ����
// ���ǲ��ᱻ��ʼ���ı������Ǻ���Ҫ�ı�������Ҫ���ݣ� Need To DO
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

// ��ʼ�����ݱ���
void initBackUP()
{ // ��ʼֵȫ����Ϊ0
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

// ����һ������
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

// �������б���
void update()
{
	update_bk(&bk1, 1);
	update_bk(&bk2, 2);
	update_bk(&bk3, 3);
	// isWarmStarting = WARMSTARTFLAG; /*����2*/
}

/*
ֱ��ȡ��������
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
		{ // bk1У��ͨ��
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
		{ // bk2У��ͨ��
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
		{ // bk3У��ͨ��
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

/*--------------------------��ȫ����-����----------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void ErrorState()
{
	state = START;
	update();
	printf("\r\n�������󣬽���������\n\r"); // ����ǰ״̬ת��START����ӡ������Ϣ��ϵͳ�����쳣����ʼ������
}

// {0xFC, 0x0C, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xE6}
int seg7code(int x) // ����������x��0-9��ӳ�䵽����ܵ�����ʾ
{
	if (x < 0 || x >= 10)
	{
		printf("\r\n��Ч����\n\r"); // ��Ч����
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
		return 0; // Ĭ�Ϸ��أ�ADD
	}
}

void alert(int count) // ���ھ�����countΪ��������
{
	int length = sizeof(melody) / sizeof(melody[0]);
	while (count--) // ѭ������ melody
	{
		for (int i = 0; i < length; i++)
		{
			int frequency = melody[i].frequency;
			int duration = melody[i].duration;

			if (frequency == NOTE_SILENT)
			{
				HAL_Delay(duration); // ����Ǿ������ӳ���Ӧ��ʱ��
				continue;
			}

			int period = 1000 / frequency; // ���ڣ���λ��ms��
			int half_period = period / 2;  // ������

			int cycles = (frequency * duration) / 1000; // �ڸ�������ʱ���ڵ���������

			for (int j = 0; j < cycles; j++)
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);	  // ��
				HAL_Delay(half_period);								  // �ȴ�������
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // �ر�
				HAL_Delay(half_period);								  // �ȴ�������
			}

			HAL_Delay(50); // ����֮��Ķ����ӳ�
		}
	}
}

// @brief: ��ʾ������
void Show(double data)
{
	/*****************����ת��*****************************/

	for (int i = 0; i < 8; i++)
	{
		int idx = (int)data % 10;
		Tx1_Buffer[i] = seg7code(idx);
		data *= 10;
	}

	Tx1_Buffer[0] |= 1; // ���С����Ĺ���
	update();
	I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8); // 0x70��Ŀ���豸�ĵ�ַ��ZLG��W���豸���ض��Ĵ�����Tx1��Ҫд��Ļ�������8��ָ8���ֽ�
}

// ���ڰ�ȫ����ʾ������
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

// @brief: ��ʾ����
void ShowInteger(unsigned data)
{
	/*****************����ת��*****************************/

	if (data > 99999999)
	{
		data = 99999999;
		printf("\r\n����������\n\r"); // ��ӡ��������
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

// ȥ���������Сֵ�������쳣��⣬���ж��ز����Ĺ���
float ReadAlcohol(void)
{
	double maxAlcohol = 0;	// �洢��ȡֵ�е����ֵ
	double minAlcohol = 10; // �洢��ȡֵ�е���Сֵ
	double uncommonLarge = 100;
	double uncommonSmall = 1e-3;			// ���ڼ���쳣ֵ����ֵ
	double sum = 0;							// �����ۼӶ�ȡֵ���ܺ�
	for (int i = 0; i < SAMPLINGCOUNT; i++) // ��ȡ10�ξƾ���ֵ
	{
		double temp = (float)adcx[0] * (3.3 / 4096); // adcx���Ӧ���Ƕ�ȡ��������ֵ
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
		printf("\r\n�����������쳣��\r\n"); // ����Ӧ������������������
		return -1;
	}

	return sum / total;
}

// �ж�ȥ˯������ι����Ҳ���ǿ���ǰ״̬�ƾ���ֵ�Ƿ�������ֲ��䣬����������ȥ˯��������ι��
void ShouldSleep(float temp)
{

	// �������������жϵ�ǰŨ�Ⱥ���һ��Ũ���Ƿ���������ǣ���remainingLoopToSleep�Լ�
	// remainingLoopToSleep����0��ȥ˯��
	get_BackUP();
	if (state != SHOULDSLEEP)
	{
		ErrorState();
		return;
	}
	printf("\r\n����Ӧ�ü��������\n\r"); // Need To Do����ӡ������Ӧ�ö����������Խ���һ����顱
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
		// ��Ũ�ȳ����˽ϴ��࣬���������remainingLoopToSleep
		remainingLoopToSleep = SLEEPTHRESHOLD;
		update();
	}

	// �����ϴ�Ũ��
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
	printf("\r\n׼��˯��\n\r"); // ׼��˯��
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
	// ��������ֻ��һ�Σ�Ҳ���˲�����Ϊ����ֻ����Ҫ�жϵ�ǰ�Ƿ���Ҫ����
	float temp = (float)adcx[0] * (3.3 / 4096);
	remainingLoopToSleep = SLEEPTHRESHOLD;
	update();
	printf("\r\n׼������\n\r"); // ׼������
	get_BackUP();
	if (!(temp > alcoholLastTime && temp - alcoholLastTime < MAXDIFFRENCE ||
		  temp <= alcoholLastTime && alcoholLastTime - temp < MAXDIFFRENCE))
	{
		// ���ִ�������ֵ�����ˣ���ʽ����
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

	state = FEEDDOG; // ��Ӧ״̬�����У��ߵ���һ��״̬�Ĳ���
	update();
}

// Ӳ�����˿ںͿ��Ź���ʼ��
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

	// ��һ����6����ʦ˵��ʼ��˳���޹أ����Խ��������ʼ��
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	MX_ADC3_Init();
	MX_USART1_UART_Init();
	init_crc_table();								// ��ʼ�� CRC ���ұ�
	HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adcx, 4); // ����ADCת��

	// ��ʼ�����Ź�
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
	hiwdg.Init.Reload = 0xBFFF; // ���ʣ�����ʱ��Ƶ���Ƕ��٣���Ƶ���������128��
	// ע�⣬���Ź�Ƶ�ʸ��ˣ��Ұ���ʵ�鱨��Ĺ�ʽ����ʱ��ĳ���1s
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Start(&hiwdg);
	printf("\r\nӲ�����˿ںͿ��Ź���ʼ����ϣ�\n\r"); // Ӳ�����˿ںͿ��Ź���ʼ�����
	state = TESTSTART;
	update();
}

// ����ʱ��ʱ����ͣһ��ʱ��
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

// �ж�������������������
void TestStart(void)
{
	get_BackUP();
	if (state != TESTSTART)
	{
		ErrorState();
		return;
	}
	get_BackUP();
	if (isWarmStarting == WARMSTARTFLAG /*&& ��?????*/)
	{
		printf("\r\n��������\n\r"); // ��ӡ����������ע�����ﻹҪ����У��
		get_BackUP();				// ȡ�������ݣ�ȡ�Ĺ��̵��У��Ͱ����˼���Ĳ��֣�
	}
	else
	{
		// ����Ҳ���������
		printf("\r\n��������\n\r"); // ������
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
	update();											// currentAlcohol���͸ı䣬���б��ݣ�
	printf("\r\n �ƾ������� =%f \n\r", currentAlcohol); // �ƾ�������
	for (int i = 0; i < 2; i++)							// To Do�������st�������Ǳ��SHOW��ALERT������״̬�Ƿ��߹��ĸо�
	{
		st1[i] = 0;
	}

	int temp = rand() % 2; // �ȷ����������Ǵ�ӡ���ֶ��ǿ��Ե�
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
	// printf("\r\n????Alert??????\n\r"); // ����Alert��������
	get_BackUP(); // ʹ��currentAlcohol����֮ǰ������Ҫ����У�飡
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

// ��ʼ�� CRC-32 ���ұ�
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
			printf("\r\n����δʶ��״̬��\n\r"); // ����δʶ���״̬
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
		; // ѭ�����ͣ�ֱ���������
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
