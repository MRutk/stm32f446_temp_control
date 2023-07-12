/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"			//oled ssd1306 taken from https://controllerstech.com/oled-display-using-i2c-stm32/
#include "test.h"
#include "DS18B20.h"
#include <stdio.h>
#include <stdbool.h>

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

TIM_HandleTypeDef htim6;

osThreadId TempTask01Handle;
osThreadId ScreenTask02Handle;
osThreadId ControllTask03Handle;
osMutexId HeatMutex01Handle;
/* USER CODE BEGIN PV */

uint8_t T_byte1 = 0;	// temporary bytes for calculating temperature
uint8_t T_byte2 = 0;
uint16_t TEMP;

float Temperature = 0;
//float Temperature_log[256] = {0};
//int16_t Critical_temp = 120; 	//temp sensors critical temperature max. operational temperature is 125C, moved to .h file as define

float error = 0;				//pid variables  //changing these var to float should result in quicker system response
float integral = 0;
float pwm = 0;

//int16_t error = 0;				//pid variables
//int16_t integral = 0;
//int16_t pwm = 0;
int16_t kp = 10;				//pid controller values,  wspolczynniki
int16_t ki = 1;
uint8_t selection = 0;			//helper variable for target, ki, kp value manipulation


uint8_t Presence = 0;		//helper variable for temp sensor communication
uint16_t Target = 0;
uint8_t i = 0;				//helper variable to check screen value updates

bool RedLEDFlag = false;	//logic status flags
bool BlueLEDFlag = false;
bool TempFlag = false;
bool ManualFlag = false;
char HeatStatus[5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void Toggle_Red_Led()		//red led is used to indicate manual operation of heater and error state
{
	if(RedLEDFlag){
		HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_RESET);
		RedLEDFlag = false;
	}
	else{
		HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
		RedLEDFlag = true;
	}
}

// function separated into 2 functions, was going to delete but I'm using it for manually controlling heating with interupt
void Toggle_heat()
{
	if(TempFlag){
		HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
		TempFlag = false;
		strcpy(HeatStatus, "OFF");
	}
	else{
		HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
		TempFlag = true;
		strcpy(HeatStatus, "ON");
	}
}

void Heat_start()
{
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_SET);
	TempFlag = true;
	strcpy(HeatStatus, "ON");
}

void Heat_stop()
{
	HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
	TempFlag = false;
	strcpy(HeatStatus, "OFF");
}

// function for manual control of heating, function is accessed through interrupt
// it needs to use xTaskResumeFromISR
void Heat_manual()
{
	if(ManualFlag){
		xTaskResumeFromISR(ControllTask03Handle);
		ManualFlag = false;
		Toggle_heat();
		Toggle_Red_Led();
	}
	else{
		ManualFlag = true;
		Toggle_heat();
		Toggle_Red_Led();
	}
}

//functions used in interupts to manipulate target,ki,kp values begin here
void Display_target(float Temp)
{

	char str[20] = {0};
	SSD1306_GotoXY (10,10);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	sprintf (str, "Target: %.2f", Temp);
	SSD1306_Puts (str, &Font_11x18, 1);
	SSD1306_UpdateScreen();

}

void Display_kp(float k)
{

	char str[20] = {0};
	SSD1306_GotoXY (10,10);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	sprintf (str, "Kp: %.2f", k);
	SSD1306_Puts (str, &Font_11x18, 1);
	SSD1306_UpdateScreen();

}

void Display_ki(float k)
{

	char str[20] = {0};
	SSD1306_GotoXY (10,10);
	SSD1306_Fill(SSD1306_COLOR_BLACK);
	sprintf (str, "Ki: %.2f", k);
	SSD1306_Puts (str, &Font_11x18, 1);
	SSD1306_UpdateScreen();

}

void Increase_value()
{

	if(selection == 2){
		ki++;
		Display_ki(ki);
	}
	else if(selection == 1){
		kp++;
		Display_kp(kp);
	}
	else{
		Target+=5;
		Display_target(Target);
	}
}

void Decrease_value()
{

	if(selection == 2){
		ki--;
		Display_ki(ki);

	}
	else if(selection == 1){
		kp--;
		Display_kp(kp);
	}
	else{
		Target-=5;
		Display_target(Target);
	}
}

void Change_selection()
{
	selection++;
	if(selection>2){
			selection = 0;
		}
	if(selection == 2){
			Display_ki(ki);
		}
		else if(selection == 1){
			Display_kp(kp);
		}
		else{
			Display_target(Target);
		}
}
//functions used in interupts to manipulate target,ki,kp values end here

// Code for communicating and setting up temperature sensor borrowed from https://controllerstech.com/ds18b20-and-stm32/

void delay(uint16_t us)		//delay in us, seems to be acurate but setting delay as in specification does not give proper results
{
	__HAL_TIM_SET_COUNTER(&htim6,0);
	while((__HAL_TIM_GET_COUNTER(&htim6))<us);
}


void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) //onewire functions
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL; //try changing to nopull if temperature values are incorrect, NOPULL seems to be the answer
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_5

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay (60);    // delay, according to datasheet supposed to be 60us

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (420);

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (60);  // wait for 60 us, if values seem wrong might need to decresse
		}

		else  // if the bit is low
		{
			// writeTreshold = 30; 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (60);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the data pin LOW
		delay (2);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}


float Temperature_pull(void)
{
		float value=0;
		Presence = DS18B20_Start ();
		delay (1);
		DS18B20_Write (0xCC);  // skip ROM
		DS18B20_Write (0x44);  // convert t
		delay (800);

		Presence = DS18B20_Start ();
	    delay(1);
	    DS18B20_Write (0xCC);  // skip ROM
	    DS18B20_Write (0xBE);  // Read Scratch-pad

	    T_byte1 = DS18B20_Read();
		T_byte2 = DS18B20_Read();
		TEMP = (T_byte2<<8)|T_byte1;	//combining bytes into temperature readout
		value = (float)TEMP/16;

		return value;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim6);
  SSD1306_Init ();

  // displaying starting message and making sure heating is turned of
  SSD1306_GotoXY (10,10); 						// goto 10, 10
  SSD1306_Puts ("STARTING", &Font_11x18, 1); 	// print starting
  SSD1306_UpdateScreen(); 						// update screen

  Heat_stop();			//make sure heat is off

  Temperature_pull(); 	//pulling temperature to establish communication with sensor
  osDelay(1000);		//wait 1 second for the looks of things

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of HeatMutex01 */
  osMutexDef(HeatMutex01);
  HeatMutex01Handle = osMutexCreate(osMutex(HeatMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TempTask01 */
  osThreadDef(TempTask01, StartDefaultTask, osPriorityNormal, 0, 256);
  TempTask01Handle = osThreadCreate(osThread(TempTask01), NULL);

  /* definition and creation of ScreenTask02 */
  osThreadDef(ScreenTask02, StartTask02, osPriorityNormal, 0, 256);
  ScreenTask02Handle = osThreadCreate(osThread(ScreenTask02), NULL);

  /* definition and creation of ControllTask03 */
  osThreadDef(ControllTask03, StartTask03, osPriorityNormal, 0, 512);
  ControllTask03Handle = osThreadCreate(osThread(ControllTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim6.Init.Prescaler = 50-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TimerCheck_GPIO_Port, TimerCheck_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SENSOR_Pin|RedLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TimerCheck_Pin */
  GPIO_InitStruct.Pin = TimerCheck_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TimerCheck_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT0_Pin BT1_Pin BT2_Pin */
  GPIO_InitStruct.Pin = BT0_Pin|BT1_Pin|BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_Pin RedLED_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin|RedLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT3_Pin */
  GPIO_InitStruct.Pin = BT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//int counter = 0;
	osDelay(1000); //added to avoid conflict with initialization
  /* Infinite loop */
  for(;;)
  {

	 //Task for getting temperature values from sensor

	  if(HeatMutex01Handle != NULL){

		  //checking if semaphore is available and taking it, if not available wait 10
		  if(xSemaphoreTake(HeatMutex01Handle, (TickType_t) 10) == pdTRUE){
			  //semaphore obtained, work starts here

			  Temperature = Temperature_pull();
			  //Temperature_log[counter] = Temperature; //using stmcubemonitor for logging, leaving this as comment
			  //counter++;

			 // checking if temperature is above critical and shuting down heating and controll task
			  if(Temperature > Critical_temp){
			  		vTaskSuspend(ControllTask03Handle);
			  		HAL_GPIO_WritePin(GPIOB, IN1_Pin, GPIO_PIN_RESET);
			  		TempFlag = false;
			  		strcpy(HeatStatus, "ERR");
				  	HAL_GPIO_WritePin(RedLED_GPIO_Port, RedLED_Pin, GPIO_PIN_SET);
				  	RedLEDFlag = true;
			  	  }

			  // work ends giving back semaphore
			  xSemaphoreGive(HeatMutex01Handle);

		  }
	  }
	  else{
		  //could not obtain semaphore dont know what exactly to put here
	  }


	  osDelay(10000);  //wait 10 seconds

	//if(counter >= 256){
	//	  counter = 0;
	// }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

	//SCREEN TASK

	char str2[20] = {0};			//helper strings for displaying values on screen
	char str3[20] = {0};

  /* Infinite loop */
  for(;;)
  {
	  //counter for checking if screen and device is operational
	  if(i==250){
		  i=0;
	  }
	  i++;

	  SSD1306_Clear();
	  osDelay(10);

	  //testing new way
	  SSD1306_GotoXY (10,10); 						// goto 10, 10
	  SSD1306_Fill(SSD1306_COLOR_BLACK);
	  sprintf (str2, "Temp: %.2f", Temperature);
	  SSD1306_Puts (str2, &Font_11x18, 1); 			// print temperature

 	  //print counter on screen TODO might change counter to a visual kind with 4 steps (-,--,---,----)
	  SSD1306_GotoXY (80,40);
	  sprintf (str3, "%u", i);
	  SSD1306_Puts (str3, &Font_11x18, 1);	//print additional info

	  //print heating status
	  SSD1306_GotoXY (10,40);
	  SSD1306_Puts (HeatStatus, &Font_11x18, 1);	//print on/off/err
	  
	  SSD1306_UpdateScreen();

	  osDelay(2000); 	//wait for 2 seconds

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */
  for(;;)
  {
	  // PI controller task

	  if(ManualFlag){
	  	 	  	  	vTaskSuspend(ControllTask03Handle);
	  	 	  }

	  if(HeatMutex01Handle != NULL){

	  		  //checking if semaphore is available and taking it, if not available wait 10
	  		  if(xSemaphoreTake(HeatMutex01Handle, (TickType_t) 10) == pdTRUE){
	  			  //semaphore obtained, work starts here

	  			  // PI controller code
	  			  error = Target - Temperature;
	  			  integral = integral + error;

	  			  //pwm = kp * error;
	  			  pwm = (kp * error) + (ki * integral);

	  			  // restricting pwm and integral values

	  			  if (integral > 256){
	  				  integral = 256;
	  			  }
	  			  else if (integral < -256){
	  				  integral = -256;
	  			  }

	  			  if (pwm > 256){
	  				  pwm = 256;
	  			  }
	  			  else if (pwm < -256){
	  				  pwm = -256;
	  			  }

	  			  //controlling heat
	  			  // TODO look and pwm implementation in stm32 nucleo boards
	  			  if(pwm > 0){
	  				  Heat_start();
	  			  }
	  			  else if (pwm < 0){
	  				  Heat_stop();
	  			  }
	  			  else {
	  				  Heat_stop();
	  			  }

	  			  // work ends giving back mutex
	  			  xSemaphoreGive(HeatMutex01Handle);
	  		  }
	  	  }
	  	  else{
	  		  //could not obtain semaphore dont know what exactly to put here
	  	  }

	  osDelay(10000);  //wait 10 seconds

  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
