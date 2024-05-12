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
#include "sc_types.h"
#include "Statechart.h"
#include "Statechart_required.h"
#include "math.h"
#include "TJ_MPU6050.h"
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>  // funkcijai sprintf
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Statechart sc_handle; // Statechart pointer

// ---------------- ADC VARIABLES -----------------
#define NUM_ADC_CHANNELS 1
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t) 32)
#define BUFFERLEN_ADC 500
#define RMS_BUFFERLEN_ADC 10
#define DISPLAY_BUFFERLEN_ADC 2

static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];
uint16_t SOUND_SAMPLE[NUM_ADC_CHANNELS];
uint16_t DataBufferADC[BUFFERLEN_ADC][NUM_ADC_CHANNELS];
float SOUND_V[BUFFERLEN_ADC][NUM_ADC_CHANNELS];
float tarpinis_RMS[NUM_ADC_CHANNELS];
float save_tarpinis_RMS[RMS_BUFFERLEN_ADC][NUM_ADC_CHANNELS];
float vidutinis_RMS_PC[NUM_ADC_CHANNELS];
float save_vidutinis_RMS_PC[DISPLAY_BUFFERLEN_ADC][NUM_ADC_CHANNELS];
float vidutinis_RMS_DISP[NUM_ADC_CHANNELS];

// ---------------- I2C VARIABLES -----------------
#define axis_no 3
#define BUFFERLEN_I2C 80
#define DISPLAY_BUFFERLEN_I2C 30

uint8_t Usart_I2CDATA_TxBuffer[20];
int16_t ACC_SAMPLE[axis_no];
int16_t DataBufferI2C[BUFFERLEN_I2C][axis_no];
float VIBRATION[BUFFERLEN_I2C][axis_no];
float tarpinis_VIBRATION_RMS[axis_no];
float save_tarpinis_VIBRATION_RMS[DISPLAY_BUFFERLEN_I2C][axis_no];
float VIBRATION_RMS_DISP[axis_no];

RawData_Def myAccelRaw;

// ---------------- UART VARIABLES -----------------
uint8_t UART_TxBuffer_ADC[40];
uint8_t UART_TxBuffer_I2C[60];
typedef enum {UART_READY, UART_BUSY} UART_Status;
UART_Status Status_uart=UART_READY;

/* for handling I2C errors */
void MyErrorHandlerI2C()
{  
}
/* for handling ADC errors */
void MyErrorHandlerADC()
{  
}

void HAL_USART_TxCpltCallbck(UART_HandleTypeDef *huart2)
{
	if(Status_uart==UART_BUSY)
		Status_uart=UART_READY;
}
	
void HandleUsartError()
{
	uint32_t usart_error;
	usart_error = HAL_UART_GetError(&huart2);
}


typedef enum {STATUS_READY, STATUS_WAIT} TStatus;
TStatus Status=STATUS_WAIT;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);
	if (GPIO_Pin==B1_UserButton_Pin)
	{
		if (Status==STATUS_WAIT){
			Status=STATUS_READY;
			statechart_raise_buttonPush(&sc_handle);
		}
		else{
			Status=STATUS_WAIT;
			statechart_raise_buttonPush(&sc_handle);
		}
		
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (Status == STATUS_WAIT && htim==&htim6)
	{
		statechart_raise_ev_GetSample(&sc_handle);	
	} 
	else if (Status == STATUS_READY && htim==&htim7)
	{
		statechart_raise_ev_GetSample_i2c(&sc_handle);	
	}	
}

// ----------------------------------- ADC --------------------------------------

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	statechart_raise_ev_ADCSampleReady(&sc_handle); 
}

void statechart_startConvADC( Statechart* handle)
{
	if (HAL_ADC_Start_DMA(&hadc,(uint32_t *)aADCxConvertedData, NUM_ADC_CHANNELS) != HAL_OK)
		{
			MyErrorHandlerADC();
		}
}
void statechart_readADCSample( Statechart* handle)
{
	SOUND_SAMPLE[0] = aADCxConvertedData[0];
}

sc_integer statechart_saveADCSample( Statechart* handle, const sc_integer sample_no)
{
	for (int i=0;i<NUM_ADC_CHANNELS;i++)
		DataBufferADC[sample_no][i]=SOUND_SAMPLE[i];
	return 1;
}

void statechart_tarpinis_RMS( Statechart* handle)
{
	float sum[NUM_ADC_CHANNELS];
	
	for (int i=0; i<BUFFERLEN_ADC; i++)
		for (int j=0; j<NUM_ADC_CHANNELS; j++)
			SOUND_V[i][j] = DataBufferADC[i][j]*3.3/4096;
	
	for (int i=0; i<NUM_ADC_CHANNELS; i++){
		sum[i]=0;
		for (int j=0; j<BUFFERLEN_ADC; j++){
			sum[i] += SOUND_V[j][i]*SOUND_V[j][i];
		}
		tarpinis_RMS[i] = sqrt(sum[i]/BUFFERLEN_ADC);
	}
}

sc_integer statechart_save_tarpinis_RMS( Statechart* handle, const sc_integer sample1_no)
{
	
	for (int i = 0; i < NUM_ADC_CHANNELS; i++) 
		save_tarpinis_RMS[sample1_no][i] = tarpinis_RMS[i];
	return 1;
}

void statechart_processDataADC( Statechart* handle)
{
	float sum[NUM_ADC_CHANNELS];
	for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
		sum[i] = 0;
		for (int j = 0; j < RMS_BUFFERLEN_ADC; j++) {
				sum[i] += save_tarpinis_RMS[j][i];
		}
		vidutinis_RMS_PC[i] = sum[i] / RMS_BUFFERLEN_ADC;
  }
}

void statechart_usartTransmit_ADC( Statechart* handle)
{
	if(Status_uart==UART_READY)
	{
		sprintf((char *)UART_TxBuffer_ADC, "Garso lygio patalpoje RMS: %4.3f V\n", vidutinis_RMS_PC[0]);
		if (HAL_UART_Transmit_IT(&huart2, UART_TxBuffer_ADC, strlen(UART_TxBuffer_ADC))!=HAL_OK)
		{
		HandleUsartError();
		}
	else
	Status_uart=UART_BUSY;
	
	Status_uart=UART_READY;
	}
}

sc_integer statechart_save_UsartTransmit_ADC( Statechart* handle, const sc_integer N_ADC)
{ 
	for (int i = 0; i < NUM_ADC_CHANNELS; i++) 
		save_vidutinis_RMS_PC[N_ADC][i] = vidutinis_RMS_PC[i];
	return 1;
}

void statechart_displayInfo_ADC( Statechart* handle)
{
	for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        float sum = 0;
        for (int j = 0; j < DISPLAY_BUFFERLEN_ADC; j++) {
            sum += save_vidutinis_RMS_PC[j][i];
        }
        vidutinis_RMS_DISP[i] = sum / DISPLAY_BUFFERLEN_ADC;
  }
	//atvaizdavimas:
	static char string_display[30];
	ssd1306_Fill(Black); // visas ekranas uzpildomas juodai, kad pries tai buves tekstas pasinaikintu
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(" Garsas RMS ",Font_11x18, White);
	ssd1306_SetCursor(0, 22);
	sprintf(string_display,"  %.3f V   ", vidutinis_RMS_DISP[0]);
	ssd1306_WriteString(string_display, Font_11x18, White);
  ssd1306_UpdateScreen(&hi2c3); // atnaujinamas ekranas su naujuoju tekstu
	
}

// ----------------------------------- I2C --------------------------------------

void statechart_readI2CSensor( Statechart* handle)
{
	MPU6050_Get_Accel_RawData(&myAccelRaw);
	ACC_SAMPLE[0] = myAccelRaw.x;
	ACC_SAMPLE[1] = myAccelRaw.y;
	ACC_SAMPLE[2] = myAccelRaw.z;
}
sc_integer statechart_saveI2CSample( Statechart* handle, const sc_integer sample_i2c)
{
	for(int i=0; i<axis_no; i++)
	{
		DataBufferI2C[sample_i2c][i] = ACC_SAMPLE[i];
	}
	return 1;
}
void statechart_processDataI2C( Statechart* handle)
{
	float sum[axis_no];
	
	for (int i=0; i<BUFFERLEN_I2C; i++)
		for (int j=0; j<axis_no; j++)
			VIBRATION[i][j] = (DataBufferI2C[i][j]+0.0f)/(8192.0f);
	
	for (int i=0; i<axis_no; i++){
		sum[i]=0;
		for (int j=0; j<BUFFERLEN_I2C; j++){
			sum[i] += VIBRATION[j][i]*VIBRATION[j][i];
		}
		tarpinis_VIBRATION_RMS[i] = sqrt(sum[i]/BUFFERLEN_I2C);
	}
}
void statechart_usartTransmit_I2C( Statechart* handle)
{
	if(Status_uart==UART_READY)
	{
		sprintf((char *)UART_TxBuffer_I2C, "Vibraciju lygio RMS: x: %.3f g, y: %.3f g, z: %.3f g \n", tarpinis_VIBRATION_RMS[0], tarpinis_VIBRATION_RMS[1], tarpinis_VIBRATION_RMS[2]);
		if (HAL_UART_Transmit_IT(&huart2, UART_TxBuffer_I2C, strlen(UART_TxBuffer_I2C))!=HAL_OK)
		{
		HandleUsartError();
		}
	else
	Status_uart=UART_BUSY;
	
	Status_uart=UART_READY;
	}
}
sc_integer statechart_save_UsartTransmit_I2C( Statechart* handle, const sc_integer N_I2C)
{
	for (int i = 0; i < axis_no; i++) 
		save_tarpinis_VIBRATION_RMS[N_I2C][i] = tarpinis_VIBRATION_RMS[i];
	return 1;
}
void statechart_displayInfo_I2C( Statechart* handle)
{
	float sum[axis_no];
		for (int i = 0; i < axis_no; i++) {
		sum[i]=0;
		for (int j = 0; j < DISPLAY_BUFFERLEN_I2C; j++) {
			sum[i] += save_tarpinis_VIBRATION_RMS[j][i];
		}
		VIBRATION_RMS_DISP[i] = sum[i] / DISPLAY_BUFFERLEN_I2C;
  }
	
	//atvaizdavimas:
	static char string_display[30];
	ssd1306_Fill(Black); // visas ekranas uzpildomas juodai, kad pries tai buves tekstas pasinaikintu
	ssd1306_SetCursor(0, 0);
	sprintf(string_display,"x=%.3f g    ",VIBRATION_RMS_DISP[0]);
	ssd1306_WriteString(string_display,Font_11x18, White);
	ssd1306_SetCursor(0, 22);
	sprintf(string_display,"y=%.3f g    ",VIBRATION_RMS_DISP[1]);
	ssd1306_WriteString(string_display,Font_11x18, White);
  ssd1306_SetCursor(0, 44);
  sprintf(string_display,"z=%.3f g    ",VIBRATION_RMS_DISP[2]);
  ssd1306_WriteString(string_display,Font_11x18, White);
  ssd1306_UpdateScreen(&hi2c3); // atnaujinamas ekranas su naujuoju tekstu
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	MPU_ConfigTypeDef myMpuConfig;
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	
	//MPU6050 accelerometer:
	MPU6050_Init(&hi2c1);
	myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
	myMpuConfig.ClockSource = Internal_8MHz;
	myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
	myMpuConfig.Gyro_Full_Scale = FS_SEL_500;
	myMpuConfig.Sleep_Mode_Bit = 0;
	MPU6050_Config(&myMpuConfig);
	
	//OLED:
	HAL_GPIO_WritePin(OLED_VCC_GPIO_Port,OLED_VCC_Pin,GPIO_PIN_SET);	 // apply Vcc to OLED display
  ssd1306_Init(&hi2c3);
	ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Starting...",Font_11x18, White);
  ssd1306_SetCursor(0, 12);
  ssd1306_UpdateScreen(&hi2c3);
	 
	if(HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) //run TIM6 timer
	{
	//ErrorHandler();
	 }
	if(HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) //run TIM7 timer
	{
	//ErrorHandler();
	}

	statechart_init(&sc_handle);
	statechart_enter(&sc_handle);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
	statechart_exit(&sc_handle);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00506682;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  htim6.Init.Prescaler = 2400;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim7.Init.Prescaler = 1200;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 25;
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_VCC_GPIO_Port, OLED_VCC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_UserButton_Pin */
  GPIO_InitStruct.Pin = B1_UserButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_UserButton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_VCC_Pin */
  GPIO_InitStruct.Pin = OLED_VCC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_VCC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
