#include "main.h"
#include "stm32l4xx_hal.h"

ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);


// Implicit Functions Declared
int UART_Print_String(UART_HandleTypeDef *huart, char *cArrayData, int size);
int flag = 0;

int main(void)
{
	
	//char ch[5] = {'j','o','b','s','\n'};
	char ch[6] = {'h','e','l','l','o','\n'};
	char y[1] = {'Y'};
	char buffer[1];
	
	char temperature[19] = {'T','e','m','p','e','r','a','t','u','r','e',' ','=',' ','0','0',' ', 'C','\n'};
	char reading[30];
	int readCounter = 0;
		
	uint32_t value;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();

  /* Infinite loop */
  while (1)
  {
		
		/*
		HAL_Delay(100); // delay 100 ms
	  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)'X', 1);
		if (buffer[0] == 'X') {
					UART_Print_String(&huart1,y, 1);
		}
		*/
		
		if(flag == 1) {
			flag = 0; // Reset Flag
			HAL_ADC_Start(&hadc1); // Start HAL
			HAL_ADC_PollForConversion(&hadc1, 5000);
			uint32_t value = HAL_ADC_GetValue(&hadc1);
			value = __HAL_ADC_CALC_TEMPERATURE(3360, value, ADC_RESOLUTION_12B); // 3360 is the reference voltage (mV)
			
			temperature[14] =  value/10 + '0'; 
			temperature[15] = value%10 + '0';
			UART_Print_String(&huart1,temperature, 19);
		}
	
		
		
		/* PRINT NO FLAG
		flag = 0; // set to 0
		HAL_Delay(100);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 5000);
		
		value = HAL_ADC_GetValue(&hadc1);  // Get temperature value
		value = __HAL_ADC_CALC_TEMPERATURE(3360, value, ADC_RESOLUTION_12B); // 3360 is the reference voltage (mV)
		
		temperature[14] =  value/10 + '0'; 
		temperature[15] = value%10 + '0';
		UART_Print_String(&huart1, temperature, 19);
		*/
		
		
		
		/*
		// Accumulate Temperature Values
		if(readCounter < 30){
						// ASCII code to convert temperature to Celcius
						reading[readCounter] = value/10 + '0'; 
						reading[++readCounter] = value%10 + '0';
						reading[++readCounter] = '\n';
						readCounter++;
		}
		// Print Values Obtained
		else{
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)reading, 30);
			readCounter = 0; // Reset Arraypointer
			//UART_Print_String(&huart1,temperature, 19);
			UART_Print_String(&huart1, (uint8_t *)reading, 30);
		}
		*/
	
		
  }
}

static void MX_ADC_Init(void) {
	__HAL_RCC_ADC_CLK_ENABLE();
	
	hadc1.Instance = ADC1;
	// Synchronous clock from AHB clock prescaler division by 1
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
	// Set resolution to 12 bits
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE; 	// Conversion done in single mode, scan one input
//	hadc1.Init.ContinuousConvMode = ENABLE; // Convert continously
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //End of unitary conversion flag 
	
	if(HAL_ADC_Init(&hadc1) != HAL_OK){
		Error_Handler();
	}
	
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	sConfig.Offset = 0;
	
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

int UART_Print_String(UART_HandleTypeDef *huart, char *cArrayData, int size) {
	// 1 SUCCESS,  0 FAILURE
	return !HAL_UART_Transmit(huart, (uint8_t *)cArrayData, size, 30000);

}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 32;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/10); // Changed to 10

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
}


void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
}
#endif /* USE_FULL_ASSERT */
