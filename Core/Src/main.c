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
void outputQuestions(void);
int movementDetected();
float mjerenje();

int stateYes = 0;
int stateNo = 0;
int counter;

float R1 = 1000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

char* questions[4]={"Da li ste u posljednjih 14 dana imali povisenu temperaturu?",
		  "Da li imate poteskoca s disanjem, kasalj, gubitak cula mirisa i/ili okusa?",
  	  	  "Da li ste Vi ili neko iz vaseg domacinstva boravili izvan BiH u posljednjih 14 dana?",
  	  	  "Da li ste bili u kontaktu s nekim ko je bio pozitivan na COVID-19?"};
char* warning = "Molimo Vas da se javite na najblize COVID odjeljenje.";


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
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
  float cels;
  float raw;
  while(1){
	  /* run this while loop until IR sensor detects object
	  - this stops the program from running until an object is detected */
	  while(!movementDetected());

	  HAL_Delay(500);

	  printUSART2("Dobrodosli.\n");
	  printUSART2("Molimo Vas dodirnite senzor za mjerenje temperature i sacekajte da se Vasa temperatura izmjeri.\n");
	  while(mjerenje() < 32); // run while loop until temp > 32, that indicates user has touched the sensor
	  printUSART2("Mjerenje ...\n"); // output to user that temperature is being measured
	  HAL_Delay(3000); // 3 seconds for thermistor to take person's body temperature
	  cels = mjerenje(); // measure the temperature and return result to cels



	  // output temperature
	  char res[4];
	  ftoa(cels, res, 1);
	  printUSART2("Vasa temperatura iznosi %s stepeni Celzijusa.\n", res);

	 // if a person's temperature is higher than normal, stop the program and output the warning message
	 if(cels > 36.9) yesAnswer();
	 else  outputQuestions();
	 HAL_Delay(4000);
	 // turn off the LEDs and motor before new program run
	 HAL_GPIO_WritePin(GPIOD,red_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOD,green_Pin, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA,DC_in1_Pin,GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA,DC_in2_Pin,GPIO_PIN_RESET);

  }

}

float mjerenje(){
	float raw = getADC();
	R2 = R1 * (1023.0 / raw - 1.0);
	logR2 = log(R2);
	T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
	return (T - 273.15);
}

void outputQuestions(void){

	HAL_Delay(100);
	counter = 0; // counter for questions[] array
		// should be initialized to 0 every time the program goes trough loop again
	int odgovor;
	// first question
	printUSART2("%s ", nextQuestion());

	// check for YES button press
	if((odgovor = checkYes()) > 0) { yesAnswer(); return; }
	else if (odgovor == 0){
		// second question
		printUSART2("%s ", nextQuestion());
		if((odgovor = checkYes()) > 0) { yesAnswer(); return; }
		else if(odgovor == 0){
			// 3rd question
			printUSART2("%s ", nextQuestion());
			if((odgovor = checkYes()) > 0) { yesAnswer(); return; }
				else if(odgovor == 0){
					// 4th question
					printUSART2("%s ", nextQuestion());
					if((odgovor = checkYes()) > 0) { yesAnswer(); return; }
					else if(odgovor == 0){
						 HAL_GPIO_WritePin(GPIOD,green_Pin, GPIO_PIN_SET); // turn on green LED
						 printUSART2("Mozete uci u prostoriju.\n\n");
						 HAL_Delay(3000);
						 HAL_GPIO_WritePin(GPIOA,DC_in1_Pin,GPIO_PIN_SET);   // Start motor clockwise rotation
						 HAL_GPIO_WritePin(GPIOA,DC_in2_Pin,GPIO_PIN_RESET);
						 return;
					}
				}
		}
	}

	HAL_Delay(1000);

}


int movementDetected(){
	// IR sensor returns LOW (0) when an object is detected
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
	HAL_GPIO_WritePin(GPIOD,red_Pin,GPIO_PIN_SET);
	printUSART2("%s\n\n", warning);
}

int checkYes(){
	while(!(stateYes = HAL_GPIO_ReadPin(GPIOA,yes_button_Pin)) && !(stateNo = HAL_GPIO_ReadPin(GPIOD, no_button_Pin)));

	while(HAL_GPIO_ReadPin(GPIOA,yes_button_Pin) || HAL_GPIO_ReadPin(GPIOD, no_button_Pin));

	if(stateYes) {
		stateYes = stateNo = 0;
		printUSART2("DA\n");

		return 1; // return 1 for yes
	}
	if(stateNo){
		stateYes = stateNo = 0;
		printUSART2("NE\n");
		return 0; // return 0 for no
	}
	return -1;
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

        /* Get the value of fraction part up to given no.
         of points after dot. The third parameter
         is needed to handle cases like 233.007*/
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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


static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

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

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

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
  HAL_TIM_MspPostInit(&htim12);

}


static void MX_USART2_UART_Init(void)
{

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

}


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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

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
