
#include "main.h"
#include<math.h>
#include"usart.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* Private variables */
ADC_HandleTypeDef hadc1;


TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;


/* Functions declarations */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
uint16_t getADC(void);

/* Functions for converting float to string */
void reverse(char* str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char* res, int afterpoint);

/* Functions for logic in the code */
char* nextQuestion();
void yesAnswer();
int checkYes();
int checkNo();
void outputQuestions(void);
int movementDetected();

int state = 0;
int counter;

char* questions[4]={"Da li ste u posljednjih 14 dana imali povisenu temperaturu?",
		  "Da li imate  problema s disanjem?",
  	  	  "Da li ste putovali negdje izvan granica BiH u posljednjih 14 dana?",
  	  	  "Da li ste bili u kontaktu s nekim ko je bio pozitivan na Covid-19?"};
char* warning = "Molimo Vas da posjetite Covid ambulantu.";


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
 // SystemClock_Config();
  /* Initialize all configured peripherals */
  initUSART2(921600);
  MX_GPIO_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();


  HAL_TIM_Base_Start(&htim12); //Initialize stm32 timer for DC motor

  // HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);  //PB0 Start pwm  motor 100% duty cycle
   //__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 40);

  uint16_t raw;
  while(1){
	  while(!movementDetected())
	  HAL_Delay(1000);
	  printUSART2("Dobrodosli.\n");
	  printUSART2("Molimo Vas dodirnite senzor za mjerenje temperature i sacekajte da se Vasa temperatura izmjeri.\n");
	  HAL_Delay(3000);
        raw = getADC();
 	  	double v = raw * 3.3 / 4096;
 	  	double Rt = 10 * v / ( 3.3 - v );
 	  	double temp = 1 / (log(Rt / 10) / 3950 + 1 / (273.15 + 25));
 	  	double tempc = temp - 273.15 - 2;
	  	 // check if user has touched the thermistor
	  	 // no room temperature is over 30 and person's temperature is under 30
	  	 if(tempc > 30){
	  	 	 printUSART2("Mjerenje ...\n");
	  	 	 HAL_Delay(5000); // wait aprox. 5sec for sensor to measure the temperature
	  	 }
         char res[20];
	  	 ftoa(tempc, res, 1);
	  	 printUSART2("Vasa temperatura iznosi %s\n", res);
	  	 if(tempc > 36.9) { yesAnswer(); return 0;}

	  	 outputQuestions();
  }

}


void outputQuestions(void){
	while(1){
		HAL_Delay(100);
		counter = 0;
	// first question
	printUSART2("%s\n", nextQuestion());

	// check for YES button press
	if(checkYes()) { yesAnswer(); }

	// check for NO button press
	else if(checkNo()) {
		// second question
		printUSART2("%s\n", nextQuestion());

		// check for YES button press
		if(checkYes()) { yesAnswer(); }
		else if(checkNo()) {
			// 3rd question
			printUSART2("%s\n", nextQuestion());

		  	if(checkYes()) { yesAnswer(); }
		  	else if(checkNo()) {
		  		// 4th question
		  		printUSART2("%s\n", nextQuestion());

		  		if(checkYes()) { yesAnswer(); }
		  	 	else if(checkNo()) {
		  	 		// turn on green LED
		  	 	 	 HAL_GPIO_WritePin(GPIOA,green_Pin, GPIO_PIN_SET);
		  	 	 	 printUSART2("Mozete uci u prostoriju.");
		  	 	 	 HAL_GPIO_WritePin(GPIOA,DC_in1_Pin,GPIO_PIN_SET);   // Start motor clockwise rotation
		  	 	 	 HAL_GPIO_WritePin(GPIOA,DC_in2_Pin,GPIO_PIN_RESET);
		  	 	 	 HAL_Delay(3000);

		  	 	 }
		  	}
		 }
	}
	HAL_Delay(100);
	}
}


int movementDetected(){
	if(HAL_GPIO_ReadPin(IR_sensor_GPIO_Port, IR_sensor_Pin) == GPIO_PIN_RESET){
		return 1;
	}
	else return 0;
}

char* nextQuestion(){
	return questions[counter++];
}

void yesAnswer(){
	// turn on red LED
	HAL_GPIO_WritePin(GPIOA,red_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA,DC_in1_Pin,GPIO_PIN_SET);   // Start motor clockwise rotation
    HAL_GPIO_WritePin(GPIOA,DC_in2_Pin,GPIO_PIN_RESET);
	printUSART2("%s", warning);
}

int checkYes(){
	 state = HAL_GPIO_ReadPin(GPIOA,yes_button_Pin);
	 HAL_Delay(10);
	 if(state){
		 // turn on red LED
		 HAL_GPIO_TogglePin(GPIOD,red_Pin);
		 return 1;
	 }
	 else return 0;
}

int checkNo(){
	  // check for NO button press
	  state = HAL_GPIO_ReadPin(GPIOD, no_button_Pin);
	  HAL_Delay(10);
	  if(state){ return 1;}
	  else return 0;
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}


uint16_t getADC(void)
{
	HAL_ADC_Start (&hadc1);
	while(HAL_ADC_PollForConversion(&hadc1,10000) != HAL_OK);
	// HAL_ADC_PollForConversion(&hadc1,10000);

	return HAL_ADC_GetValue(&hadc1);
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DC_in1_Pin|DC_in2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, green_Pin|red_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : yes_button_Pin */
  GPIO_InitStruct.Pin = yes_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(yes_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_in1_Pin DC_in2_Pin */
  GPIO_InitStruct.Pin = DC_in1_Pin|DC_in2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : no_button_Pin */
  GPIO_InitStruct.Pin = no_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(no_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : green_Pin red_Pin */
  GPIO_InitStruct.Pin = green_Pin|red_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_sensor_Pin */
  GPIO_InitStruct.Pin = IR_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IR_sensor_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
