/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	volatile uint16_t CANID;
	volatile uint8_t motorID;
	volatile int16_t actVel;
	volatile int16_t p_actVel;
	volatile int16_t actangle;
	volatile int16_t p_actangle;
	volatile int16_t actCurrent;
	volatile int16_t cu;
	volatile float motor_pos_ref;
	volatile float motor_spd;
	volatile float motor_spd_ref;
	volatile float pos_err;
	volatile float last_pos_err;
	volatile float sum_pos_err;
	volatile float sum_err;
	volatile float output_val;
	volatile uint32_t last_update_time;
	volatile float diff_pro;
	volatile float motor_pos;
	volatile int16_t trgVel;
	volatile float hensa;
	volatile float ind;
}motor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R 0.086602540378//m
#define r 66.2//mm

#define true 1;
#define false 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader_motor;
FDCAN_RxHeaderTypeDef RxHeader_motor;
FDCAN_FilterTypeDef sFilterConfig;

uint8_t TxData[8] = {};
uint8_t RxData[8] = {};
uint8_t TxData_motor[8] = {};
uint8_t RxData_motor[8] = {};

motor robomas[3] = {
		{0x201, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x202, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0x203, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

const float a[3] = {M_PI/6, 5*M_PI/6, 3*M_PI/2};

volatile float vx = 0, vy = 0, omega = 0;

volatile uint8_t is_Right = 0, is_Left = 0, is_Up = 0, is_Down = 0, is_Circle = 0, is_Square = 0, is_Triangle = 0;
volatile uint8_t is_Cross = 0, is_UpRight = 0, is_DownRight = 0, is_UpLeft = 0, is_DownLeft = 0, is_R1 = 0, is_L1 = 0;
volatile uint8_t is_Share = 0, is_Options = 0, is_R3 = 0, is_L3 = 0, is_PsButton = 0, is_Touchpad = 0;

volatile int16_t vel_x = 0, vel_y = 0, omega_c = 0;

volatile float k_p = 7, k_i = 0.5, k_d = 0.0001;

float kp = 100, ki = 4, kd = -20;
float max_sum_pos_err = 10000;
float max_output_val = 10000;

float swervedrive_vel[3][2] = {
    {0, 0},
    {0, 0},
    {0, 0}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

	        /* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &RxHeader_motor, RxData_motor) != HAL_OK) {
			printf("fdcan_getrxmessage_motor is error\r\n");
			Error_Handler();
		}
		/*receive robomas's status*/
		for (int i=0; i<=7; i++){
			if (RxHeader_motor.Identifier == (robomas[i].CANID)) {
				robomas[i].actangle = (int16_t)((RxData_motor[0] << 8) | RxData_motor[1]);
				robomas[i].actVel = (int16_t)((RxData_motor[2] << 8) | RxData_motor[3]);
				robomas[i].actCurrent = (int16_t)((RxData_motor[4] << 8) | RxData_motor[5]);
			}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if (RESET != (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)) {

	        /* Retrieve Rx messages from RX FIFO0 */

		if (HAL_OK != HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData)) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}

		if (0x300 == RxHeader.Identifier) {
			vel_x = (int8_t)RxData[0];
			vel_y = (int8_t)RxData[1];
			omega_c = (uint8_t)RxData[5] - (uint8_t)RxData[4];
			if ((RxData[6] & 0x40) == 0x40){
				is_Right = true;
			}
			else {
				is_Right = false;
			}
			if ((RxData[6] & 0x20) == 0x20){
				is_Left = true;
			}
			else {
				is_Left = false;
			}
			if ((RxData[6] & 0x10) == 0x10){
				is_Up = true;
			}
			else {
				is_Up = false;
			}
			if ((RxData[6] & 0x8) == 0x8){
				is_Down = true;
			}
			else {
				is_Down = false;
			}
		}
		if (RxHeader.Identifier == 0x301) {
			if ((int8_t)RxData[1] == 1) {
				vel_x = 0;
				vel_y = 0;
				is_Right = false;
				is_Left = false;
				is_Up = false;
				is_Down = false;
			}
		}
	}
}

void cal_swerve_drive(float Theta, float Vx, float Vy, float Omega, float *V1, float *Theta1, float *V2, float *Theta2, float *V3, float *Theta3) {
    float velocity[3][2] = {};
    for (int i = 0; i < 3; i++){
        velocity[i][0] = Vx - Omega*R*sin(Theta + a[i]);
        velocity[i][1] = Vy + Omega*R*cos(Theta + a[i]);
    }
    /*
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            if (0 == velocity[i][j]) {
                velocity[i][j] = 0;
                return 1;
            }
        }
    }*/

    *V1 = sqrt(pow(velocity[0][0], 2.0) + pow(velocity[0][1], 2.0));
    *V2 = sqrt(pow(velocity[1][0], 2.0) + pow(velocity[1][1], 2.0));
    *V3 = sqrt(pow(velocity[2][0], 2.0) + pow(velocity[2][1], 2.0));


    *Theta1 = atan2(velocity[0][1], velocity[0][0]);
    *Theta2 = atan2(velocity[1][1], velocity[1][0]);
    *Theta3 = atan2(velocity[2][1], velocity[2][0]);
}

/*Initializing fdcan1*/
void FDCAN_RxTxSettings(void){
	FDCAN_FilterTypeDef FDCAN_Filter_settings;
	FDCAN_Filter_settings.IdType = FDCAN_STANDARD_ID;
	FDCAN_Filter_settings.FilterIndex = 0;
	FDCAN_Filter_settings.FilterType = FDCAN_FILTER_RANGE;
	FDCAN_Filter_settings.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	FDCAN_Filter_settings.FilterID1 = 0;
	FDCAN_Filter_settings.FilterID2 = 0x7FF;

	TxHeader.Identifier = 0x000;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;


	if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings)){
		printf("fdcan_configfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)){
		printf("fdcan_configglobalfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_Start(&hfdcan1)) {
		printf("fdcan_start is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0)){
		printf("fdcan_activatenotification is error\r\n");
		Error_Handler();
	}
}

/*Initializing fdcan3*/
void FDCAN_motor_RxTxSettings(void) {
	FDCAN_FilterTypeDef FDCAN_Filter_settings_motor;
	FDCAN_Filter_settings_motor.IdType = FDCAN_STANDARD_ID;
	FDCAN_Filter_settings_motor.FilterIndex = 0;
	FDCAN_Filter_settings_motor.FilterType = FDCAN_FILTER_RANGE;
	FDCAN_Filter_settings_motor.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN_Filter_settings_motor.FilterID1 = 0x200;
	FDCAN_Filter_settings_motor.FilterID2 = 0x210;

	TxHeader_motor.Identifier = 0x200;
	TxHeader_motor.IdType = FDCAN_STANDARD_ID;
	TxHeader_motor.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_motor.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_motor.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_motor.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_motor.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_motor.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_motor.MessageMarker = 0;

	if (HAL_OK != HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_Filter_settings_motor)){
		printf("fdcan_configfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE)){
		printf("fdcan_configglobalfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_Start(&hfdcan3)) {
		printf("fdcan_start is error\r\n");
		Error_Handler();
	}

	if (HAL_OK != HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)){
		printf("fdcan_activatenotification is error\r\n");
		Error_Handler();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (&htim6 == htim) {
		for (int i = 0; i < 3; i++){
			robomas[i].hensa = robomas[i].trgVel - robomas[i].actVel;
			if (robomas[i].hensa >= 1000) robomas[i].hensa = 1000;
			else if (robomas[i].hensa <= -1000) robomas[i].hensa = -1000;
			float d = (robomas[i].actVel - robomas[i].p_actVel) / 0.001;
			robomas[i].ind += robomas[i].hensa*0.1;
			if (d >= 30000) d = 30000;
			else if (d <= -30000) d = -30000;
			if (robomas[i].ind >= 10000) robomas[i].ind = 10000;
			else if (robomas[i].ind <= -10000) robomas[i].ind = -10000;


			float t = k_p*robomas[i].hensa;
			if (t>=10000) t = 10000;
			else if (t<=-10000) t = -10000;
			robomas[i].cu = (int16_t)(t+k_i*robomas[i].ind+k_d*d);
			if (robomas[i].cu <= -10000) robomas[i].cu = -10000;
			else if (robomas[i].cu >= 10000) robomas[i].cu = 10000;


			TxData_motor[i*2] = (robomas[i].cu) >> 8;
			TxData_motor[i*2+1] = (uint8_t)((robomas[i].cu) & 0xff);
			robomas[i].p_actVel = robomas[i].actVel;
		}
		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader_motor, TxData_motor) != HAL_OK){
			printf("addmassage is error\r\n");
			Error_Handler();
		}
	}

	if (&htim7 == htim) {
		for (int i = 0; i < 3; i++) {
			robomas[i].motor_spd = (float)robomas[i].actVel / 60.0;

			robomas[i].diff_pro=robomas[i].actangle - robomas[i].p_actangle;

			float diff;
			if(robomas[i].actVel >=10000){
				diff = (robomas[i].diff_pro < 0 ? robomas[i].diff_pro + 8192 : robomas[i].diff_pro);
			}else if(robomas[i].actVel <= -10000){
				diff = (robomas[i].diff_pro > 0 ? robomas[i].diff_pro - 8192 : robomas[i].diff_pro);
			}else if( robomas[i].diff_pro < 5000 && robomas[i].diff_pro > -5000){
				diff = robomas[i].diff_pro;
			}else if (robomas[i].diff_pro > 0){
				diff = robomas[i].diff_pro - 8192;
			}else
				diff = robomas[i].diff_pro + 8192;

			robomas[i].p_actangle = robomas[i].actangle;

			float dt = 0.001;
			robomas[i].motor_pos += diff/8192.0;
			robomas[i].motor_spd = diff / 8192.0 / dt;


			robomas[i].motor_pos_ref = -1*swervedrive_vel[i][1]/M_PI/2*117;

			float p_gain = (robomas[i].motor_pos_ref - robomas[i].motor_pos) * kp;
			robomas[i].pos_err = robomas[i].motor_pos_ref - robomas[i].motor_pos;
			if(robomas[i].pos_err>10||robomas[i].pos_err<-10){
				robomas[i].pos_err = 0;
			}
			robomas[i].sum_pos_err += robomas[i].pos_err * dt;
			robomas[i].last_pos_err = robomas[i].pos_err;
			if(robomas[i].sum_pos_err >= max_sum_pos_err){
				robomas[i].sum_pos_err = max_sum_pos_err;
					}
					else if(robomas[i].sum_pos_err <= -max_sum_pos_err){
						robomas[i].sum_pos_err = -max_sum_pos_err;
					}
			float i_gain = robomas[i].sum_pos_err * ki;

			float d_gain = robomas[i].motor_spd * kd;

			robomas[i].trgVel = p_gain + i_gain + d_gain;
/*
			for (int i = 0; i < 3; i++) {
				if (robomas[i].cu > 1000) robomas[i].cu = 1000;
				if (robomas[i].cu < -1000) robomas[i].cu = -1000;
			}*/
		}
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_FDCAN3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  printf("start\r\n");
  FDCAN_motor_RxTxSettings();//Initialize fdcan3
  printf("can_motor_start\r\n");
  FDCAN_RxTxSettings();//Initialize fdcan1
  printf("can start\r\n");
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int8_t state = 1;
  while (1)
  {
	  if (1 == state) {

		  if (vel_x < 10 && vel_x > -10) {
			  vel_x = 0;
		  }
		  if (vel_y < 10 && vel_y > -10) {
			  vel_y = 0;
		  }
		  vx = vel_x*0.01;
		  vy = vel_y*0.01;
		  omega = omega_c/100;

		  if (is_Right){
			  vx += 0.1;
		  }
		  if (is_Left){
			  vx -= 0.1;
		  }
		  if (is_Up){
			  vy += 0.1;
		  }
		  if (is_Down){
			  vy -= 0.1;
		  }

		  float p_swervedrive_vel[3][2] = {};
		  for (int i = 0; i < 3; i++) {
			  for (int j = 0; j < 2; j++) {
				  p_swervedrive_vel[i][j] = swervedrive_vel[i][j];
			  }
		  }

		  cal_swerve_drive(0, vx, vy, omega, &swervedrive_vel[0][0], &swervedrive_vel[0][1], &swervedrive_vel[1][0], &swervedrive_vel[1][1], &swervedrive_vel[2][0], &swervedrive_vel[2][1]);
		  //seigyo hennkan
		  /*tyokuzen_hozon*/
		  for (int i = 0; i < 3; i++) {
			  if (0 == swervedrive_vel[i][0]){
				  swervedrive_vel[i][1] = p_swervedrive_vel[i][1];
			  }
		  }/*tyokuzen_hozon end*/
		  /*gyaku_seigyo*/
		  for (int i = 0; i < 3; i++) {
			  if (swervedrive_vel[i][1] - p_swervedrive_vel[i][1] >= M_PI) {
				  swervedrive_vel[i][0] = -swervedrive_vel[i][0];
				  swervedrive_vel[i][1] = swervedrive_vel[i][1] - M_PI;
			  }
			  if (swervedrive_vel[i][1] - p_swervedrive_vel[i][1] <= -M_PI) {
				  swervedrive_vel[i][0] = -swervedrive_vel[i][0];
				  swervedrive_vel[i][1] = swervedrive_vel[i][1] + M_PI;
			  }
		  }/*gyaku_seigyo_end*/
	  }

	  printf("%f, %f, %f, ", swervedrive_vel[0][1], swervedrive_vel[1][1], swervedrive_vel[2][1]);
	  printf("%f, %f, %f\r\n", robomas[0].motor_pos*2*M_PI/117*(-1), robomas[1].motor_pos*2*M_PI/117*(-1), robomas[2].motor_pos*2*M_PI/117*(-1));
	  HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 2;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 15;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_GPIO_Port, Board_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC3_Pin */
  GPIO_InitStruct.Pin = ADC3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC1_Pin ADC2_Pin */
  GPIO_InitStruct.Pin = ADC1_Pin|ADC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Board_LED_Pin */
  GPIO_InitStruct.Pin = Board_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
