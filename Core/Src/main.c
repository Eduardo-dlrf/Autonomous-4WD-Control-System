/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "Directions.h"
#include "PIDpwm.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */



uint8_t IMU_ADDRESS = 0b11010000; //0xD0

float accel_x;
float accel_y;
float accel_z;
float gyro_x;
float gyro_y;
float gyro_z;

float GyroMean[50] = {};

float GyroValuesx = 0;
float GyroValuesy = 0;
float GyroValuesz = 0;

float Gyro_X_calibration = 0;
float Gyro_Y_calibration = 0;
float Gyro_Z_calibration = 0;

uint8_t accel_raw_data[6];

float yawangle = 0;
float yawangle_new;
float prev_yaw = 0;
float yaw_reference = -90;
float filteredYaw;

uint8_t state = 0;


volatile uint8_t pulses_rf;
volatile uint8_t pulses_lf;
volatile uint8_t pulses_rb;
volatile uint8_t pulses_lb;
uint8_t pulsesperturn = 20;
uint16_t rpm_rf;
uint16_t  rpm_lf;
uint16_t  rpm_rb;
uint16_t rpm_lb;

uint8_t current_tick_rf;
uint8_t current_tick_lf;
uint8_t current_tick_rb;
uint8_t current_tick_lb;

uint8_t rf_flag;
uint8_t lf_flag;
uint8_t rb_flag;
uint8_t lb_flag;

float pid_output_lf = 0;

PIDpwm_Controller pidMotor_rf,pidMotor_lf,pidMotor_rb,pidMotor_lb;


char Commands[3];


uint32_t TiempoInterrupt1;
uint32_t TiempoInterrupt2 = 0;
uint32_t Intervalo;
uint32_t TiempoInterrupt3;
uint32_t TiempoInterrupt4 = 0;
uint32_t Intervalo2;
uint32_t TiempoInterrupt5;
uint32_t TiempoInterrupt6 = 0;
uint32_t Intervalo3;
uint32_t TiempoInterrupt7;
uint32_t TiempoInterrupt8 = 0;
uint32_t Intervalo4;

uint8_t flag =0;



//Variables Magnetometro
uint8_t array[2]={0x01, 0x1D};
uint8_t array2[6];
int16_t MagX,MagY,MagZ;
float YawMag, angulo2;
float x_center = 2107.8;/* Valor de x_center de MATLAB */;
float y_center = 1137.2/* Valor de y_center de MATLAB */;
float z_center = 407.95/* Valor de z_center de MATLAB */;
float radio_promedio = 153.39167/* Valor del radio promedio calculado en MATLAB */;
float x_norm, y_norm, z_norm;
float distancia;
// Variables para almacenar los máximos y mínimos de la calibración
char datos[20];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MPU_init(void){
	uint8_t check, data;

	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x75, 1, &check, 1, 100);

	if(check == 104){
		data = 0;
		HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, 0x6B, 1, &data, 1, 100);

		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, 0x19, 1, &data, 1, 100);


		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x1C, 1, &data, 1, 100);

		uint8_t precission_selector = 0;
		/*
		 * 0 for +- 2
		 * 1 for +- 4
		 * 2 for +- 8
		 * 3 for +- 16
		 */
		data = (data & ~(0b11<<3));
		data = (data | (precission_selector << 3));

		HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, 0x1C, 1, &data, 1, 100);


		HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x1B, 1, &data, 1, 100);

		uint8_t precission_selector_gyro = 0;
		/*
		 * 0 for +- 250
		 * 1 for +- 4
		 * 2 for +- 8
		 * 3 for +- 16
		 */
		data = (data & ~(0b11<<3));
		data = (data | (precission_selector_gyro << 3));

		HAL_I2C_Mem_Write(&hi2c1, IMU_ADDRESS, 0x1B, 1,&data, 1, 100);

	}

}

void MPU_Read_accel(void){



	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x3B, 1, accel_raw_data, 6, 1000);

	 float g_precission = 2;

	 accel_x = (float)(((int16_t)(accel_raw_data[0] << 8 | accel_raw_data[1])) * g_precission/32768);
	 accel_y = (float)(((int16_t)(accel_raw_data[2] << 8 | accel_raw_data[3])) * g_precission/32768);
	 accel_z = (float)(((int16_t)(accel_raw_data[4] << 8 | accel_raw_data[5])) * g_precission/32768); //g_precission = 2,4,8,16


	 accel_x = accel_x * 9.81;
	 accel_y = accel_y * 9.81;
	 accel_z = accel_z * 9.81;
	}

void MPU_Read_gyro(void){



	uint8_t gyro_raw_data[6];

	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDRESS, 0x43, 1, gyro_raw_data, 6, 1000);

	float gyro_Precission = 250;

	 gyro_x = (float)(((int16_t)(gyro_raw_data[0] << 8 | gyro_raw_data[1])) * gyro_Precission/32768);
	 gyro_y = (float)(((int16_t)(gyro_raw_data[2] << 8 | gyro_raw_data[3])) * gyro_Precission/32768);
	 gyro_z = (float)(((int16_t)(gyro_raw_data[4] << 8 | gyro_raw_data[5])) * gyro_Precission/32768);

	 gyro_x += Gyro_X_calibration;
	 gyro_y += Gyro_Y_calibration;
	 gyro_z += Gyro_Z_calibration;


}


void Gyro_calibration(void){

	uint16_t counter = 0;
	uint32_t StartTime_cal = HAL_GetTick();
	uint32_t ElapsedTime_cal = 0;

	while(ElapsedTime_cal < 9000){

		MPU_Read_gyro();

		GyroValuesx += gyro_x;
		GyroValuesy += gyro_y;
		GyroValuesz += gyro_z;

		counter++;

		uint32_t CurrentTime_cal = HAL_GetTick();
		ElapsedTime_cal = CurrentTime_cal - StartTime_cal;
	}

	Gyro_X_calibration = -1 * GyroValuesx/counter;
	Gyro_Y_calibration = -1 * GyroValuesy/counter;
	Gyro_Z_calibration = -1 * GyroValuesz/counter;

}

void MPU_GetYaw(uint8_t elapsed){


/*
	yawangle_new = prev_yaw + gyro_z * elapsed/1000;

	yawangle = alfa * prev_yaw + (1-alfa) * yawangle_new;

	prev_yaw = yawangle_new;
*/
	uint8_t alfa = 0.99;
	filteredYaw = alfa*(filteredYaw + gyro_z * elapsed/1000) + (1-alfa)*YawMag;
}

void Mag_init(void){

	 HAL_I2C_Mem_Write(&hi2c1, 0x1A, 0x08,1, &array[0], 1, 100);
	 HAL_I2C_Mem_Write(&hi2c1, 0x1A, 0x09,1, &array[1], 1, 100);
}

void Mag_read(void){
	HAL_I2C_Mem_Read(&hi2c1, 0x1A, 0x06,1, array2, 1, 100);
	  if(array2[0]&0x01){
		  HAL_I2C_Mem_Read(&hi2c1, 0x1A, 0x00,1, array2, 6, 100);
		  MagX = array2[1]<<8 | array2[0];
		  MagY = array2[3]<<8 | array2[2];
		  MagY= array2[5]<<8 | array2[4];
	  }
	  float x_cent = (float)MagX - x_center;
	  float y_cent = (float)MagY - y_center;
	  float z_cent = (float)MagZ - z_center;

	  distancia = sqrt(x_cent * x_cent + y_cent * y_cent + z_cent * z_cent);
	  if (distancia != 0) { // Evitar división por cero
		  x_norm = (x_cent / distancia); //* radio_promedio;
		  y_norm = (y_cent / distancia); //* radio_promedio;
		  z_norm = (z_cent / distancia); //* radio_promedio;
	  }
	  distancia = sqrt(x_norm * x_norm + y_norm * y_norm + z_norm * z_norm);
	  YawMag = atan2(y_norm,x_norm)*180/M_PI;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

char rpmdata[100] = {0};
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, IMU_ADDRESS, 10, 100);
  if(ret == HAL_OK){
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	  char transmission_buffer[17];
	  uint8_t transmission_size = sprintf(transmission_buffer,"Device connected \n");

	  HAL_UART_Transmit(&huart1,&transmission_buffer,transmission_size, 500);
  }
  else{
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
  }

  HAL_StatusTypeDef ready = HAL_I2C_IsDeviceReady(&hi2c1, 0x1A, 10, 1000);

  Mag_init();
  MPU_init();

  Gyro_calibration();

 // HAL_TIM_Base_Start_IT(&htim4);
  //HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  uint32_t start_time, elapsed_time, current_time;
  uint32_t start_time2,elapsed_time2, current_time2;
  uint32_t Time = HAL_GetTick();
  uint16_t pwm_rf;
  uint16_t pwm_lf;
  uint16_t pwm_rb;
  uint16_t pwm_lb;



 char Dats[50];

 PIDpwm_Init(&pidMotor_lf, 1.83012063f, 7.53971571f, 0.0f, 80.0f, 400.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_UART_Receive(&huart1, &Commands, 3, 100);

	  //MPU_Read_gyro();
	 // MPU_Read_accel();


/*
 *
	  if(elapsed_time > 100){

		 // MPU_GetYaw(elapsed_time);
		 // start_time = HAL_GetTick();
		 // elapsed_time = 0;
	  }
*/
	  //uint32_t CurrentTime = HAL_GetTick();
	  //ElapsedTime = CurrentTime - StartTime;

	  switch(state){

		  case 1:
			  //Forward path
			  pwm_lf = 3200;
			  pwm_lb = 3200;
			  pwm_rb = 3200;
			  pwm_rf = 3200;


			  //Poner en libreria
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_rf);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_lf);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_lb);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_rb);
			 Forward();
			  break;

		  case 2:
			  //Right turn
			  pwm_lb = 100;
			  pwm_rb = 100;
			  pwm_rf = 100;
			  pwm_lf = 100;
			  //Poner en libreria
			 RightTurn();
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_rf);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_lf);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_lb);
			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_rb);


			  break;

		  default:
			  break;
	  }


/*
	  current_time = HAL_GetTick();
	  elapsed_time = current_time - start_time;

	  if(elapsed_time2 > 100){

		  uint32_t millis = HAL_GetTick();
		  //sprintf(rpmdata, "%.u,%.u,%.u,%.u,%.u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", start_time,pwm_lf,pwm_rf,pwm_lb,pwm_rb,rpm_lf,rpm_rf,rpm_lb,rpm_rb,1.00,yawangle);
		 uint8_t num_chars =  sprintf(rpmdata,"%lu,%.2f,%u,%u,%u,%u,%u,%u,%u,%.2f\n",elapsed_time2 ,pid_output_lf,pwm_rf,pwm_lb,pwm_rb,rpm_lf,rpm_rf,rpm_lb,rpm_rb,yawangle);

		 start_time2 = HAL_GetTick();
		 elapsed_time2 = 0;

		 //char data2[1];
		 //uint8_t num_chars2 =  sprintf(rpmdata, "\n");
		 //uint8_t num_chars =  sprintf(rpmdata, "%.2f , %.2f , %.2f, , %.2f , %.2f", start_time, rpm_rf , rpm_lf, rpm_rb, rpm_lb);

		  HAL_UART_Transmit(&huart1, &rpmdata, num_chars, 100);
		  //HAL_UART_Transmit(&huart1, &data2, num_chars2, 100);
	  }




		  if(state == 0){
			  if(Commands[0] == 'S'){
				  start_time = HAL_GetTick();
				  elapsed_time = 0;
				  start_time2 = HAL_GetTick();
				  elapsed_time2 = 0;
				  state++;
			  }
		  }
		  else if(state == 1){
			  if(elapsed_time > 5000){
				  start_time = HAL_GetTick();
				  elapsed_time = 0;
				  start_time2 = HAL_GetTick();
				  elapsed_time2 = 0;
				  state++;
			  }
		  }
		  else if(state == 2){
			  if(yawangle <= yaw_reference){
				  yaw_reference += -90 + (-yawangle + yaw_reference);
				  	//yaw_reference += -90;
					Stop();
					Gyro_calibration();
					start_time = HAL_GetTick();
					elapsed_time = 0;
				  	state = 1;
			  }
		  }

		  if(Commands[0] == 'E'){
			  state = 0;

			  pwm_rf = 0;
			  pwm_lf = 0;
			  pwm_rb = 0;
			  pwm_lb = 0;
			  Stop();
		  }



		  current_time2 = HAL_GetTick();
		  elapsed_time2 = current_time2 - start_time2;



  }
  */
	  if(state == 0){
		  if(Commands[0] == 'S'){
			  start_time = HAL_GetTick();
			  elapsed_time = 0;
			  start_time2 = HAL_GetTick();
			  elapsed_time2 = 0;
			  state++;
		  }
	  }
	  else if(state == 1){
		  if(elapsed_time > 5000){
			  start_time = HAL_GetTick();
			  elapsed_time = 0;
			  start_time2 = HAL_GetTick();
			  elapsed_time2 = 0;
			  state++;
		  }
	  }
	  else if(state == 2){
		  if(yawangle <= yaw_reference){
			  yaw_reference += -90 + (-yawangle + yaw_reference);
			  	//yaw_reference += -90;
				Stop();
				Gyro_calibration();
				start_time = HAL_GetTick();
				elapsed_time = 0;
			  	state = 1;
		  }
	  }

	  if(Commands[0] == 'E'){
		  state = 0;

		  pwm_rf = 0;
		  pwm_lf = 0;
		  pwm_rb = 0;
		  pwm_lb = 0;
		  Stop();
	  }


	  if(flag == 1){
		  TiempoInterrupt1 = HAL_GetTick();
		  Intervalo = TiempoInterrupt1 - TiempoInterrupt2;
		  TiempoInterrupt2 = TiempoInterrupt1;
		  flag = 0;
	  }

	  if(Intervalo > 9){
		  //rpm_rf = 1000 * 60 / (20*Intervalo);
		 // rpm_lf = 1000 * 60 / (20*Intervalo);
		  //rpm_rb = 1000 * 60 / (20*Intervalo);
		  //rpm_lb = 1000 * 60 / (20*Intervalo);


	  }

	  rpm_lf = 1000 * 60 / (20*Intervalo);
	  uint16_t Datas = sprintf(Dats,"%u,%lu\n",Intervalo, pulses_lf);

	  HAL_UART_Transmit(&huart1, &Dats, Datas, 10);



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 150 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4096 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */



  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */


  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


	if(htim -> Instance  == TIM2){

/*
		if(Intervalo > 12){
			rpm_lf = (1/20) * 1000 * 60 / Intervalo;
		}
		if(Intervalo > 12){
			rpm_rf = (1/20) * 1000 * 60 / Intervalo;
		}
		if(Intervalo > 12){
			rpm_lb = (1/20) * 1000 * 60 / Intervalo;
		}
		if(Inervalo > 12){
			rpm_rb = (1/20) * 1000 * 60 / Intervalo;
		}

		pid_output_lf = PIDpwm_Compute(&pidMotor_lf, rpm_lf);
		pid_output_rf = PIDpwm_Compute(&pidMotor_rf, rpm_rf);
		pid_output_lb = PIDpwm_Compute(&pidMotor_lb, rpm_lb);
		pid_output_rb = PIDpwm_Compute(&pidMotor_rb, rpm_rb);
/*




		rpm_rf =  (current_tick_rf * 1000 * 60) / (20 * 10);
	    rpm_lf =  (current_tick_lf * 1000 * 60) / (16 * 10);
	    rpm_rb =  (current_tick_rb * 1000 * 60) / (20 * 10);
	    rpm_lb =  (current_tick_lb * 1000 * 60) / (20 * 10);

		  current_tick_rf = 0;
		  current_tick_lf = 0;
		  current_tick_rb = 0;
		  current_tick_lb = 0;
		  */

	}


	if(htim -> Instance == TIM4){

		MPU_GetYaw(10);
		MPU_Read_gyro();
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  if(GPIO_Pin == GPIO_PIN_14){
	  //Right front wheel

	  flag = 1;
	  pulses_lf++;
	  current_tick_rf++;
	  TiempoInterrupt1 = HAL_GetTick();
	  Intervalo = TiempoInterrupt1 - TiempoInterrupt2;
	  TiempoInterrupt2 = TiempoInterrupt1;


  }
  if(GPIO_Pin == GPIO_PIN_15){
	  //Left front wheel

	  pulses_lf++;
	  current_tick_rf++;
	  TiempoInterrupt3 = HAL_GetTick();
	 // Intervalo2 = TiempoInterrupt3 - TiempoInterrupt4;
	  Intervalo2 = 11;
	  TiempoInterrupt4 = TiempoInterrupt3;


  }
  if(GPIO_Pin == GPIO_PIN_13){
	  //Right back wheel
	  pulses_rb++;
	  current_tick_rb++;
	  TiempoInterrupt5 = HAL_GetTick();
	  //Intervalo3 = TiempoInterrupt5 - TiempoInterrupt6;
	  Intervalo3 = 12;
	  TiempoInterrupt6 = TiempoInterrupt5;


  }
  if(GPIO_Pin == GPIO_PIN_12){
	  //Left back wheel
	  pulses_lb++;
	  current_tick_lb++;
	  TiempoInterrupt7 = HAL_GetTick();
	  //Intervalo4 = TiempoInterrupt7 - TiempoInterrupt8;
	  Intervalo4 = 13;
	  TiempoInterrupt8 = TiempoInterrupt7;


  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

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
