/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static GPIO_InitTypeDef  GPIO_InitStruct;
uint8_t i2cBuf[2];
uint8_t  xgyrol,xgyroh,ygyrol,ygyroh,zgyrol,zgyroh;
int16_t xgyro,ygyro,zgyro;
uint16_t msb;unsigned int dacout;
float x,y,z,xgyrofinal,ygyrofinal,zgyrofinal,xangle_1,yangle_1,zangle_1,xangle_2,yangle_2,zangle_2,time;

int flag = 0;
int xangle_f,yangle_f,zangle_f;
uint8_t i;

uint16_t temp;


int pinStatus = 0;


////C6/E6 		
//uint8_t relayON[20]= {0x7E,0x00,0x10,0x17,0x01,0x00,0x13,0xA2,0x00,0x40,0xA9,0xCF,0xEA,0xFF,0xFE,0x02,0x64,0x34,0x05,0x14};
//uint8_t relayOFF[20]={0x7E,0x00,0x10,0x17,0x01,0x00,0x13,0xA2,0x00,0x40,0xA9,0xCF,0xEA,0xFF,0xFE,0x02,0x64,0x34,0x04,0x15};
	
	
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* -2- Configure IO in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C4_Init();
  MX_USART6_UART_Init();
  MX_DAC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_I2C_Init(&hi2c4);
  HAL_I2C_MspInit(&hi2c4);
 
 
  //if (( HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,2,10))==HAL_OK)
  //{
  //	
  //	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
  //  HAL_Delay(5000);
  //	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);
  //	
  //}


  //* Transmit via I2C*/

  // Power management 1 register
	i2cBuf[0]=0x6B;
	i2cBuf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,2,10);

	// Gyroscope configuration
	i2cBuf[0]=0x1B;
	i2cBuf[1]=0x00;
	HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,2,10);

  xangle_1=0;
  yangle_1=0;
  zangle_1=0;
  time=1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Offset in Gyroscope

    // Gyro xout [15:8] 
    i2cBuf[0]=0x43;
    HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
    HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
    xgyroh=i2cBuf[1];
		
		// Gyro xout [7:0]	
		i2cBuf[0]=0x44;
	  HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
	  HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
		xgyrol=i2cBuf[1];
		
    // Gyro yout [15:8]
    i2cBuf[0]=0x45;
    HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
    HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
		ygyroh=i2cBuf[1];
		
		// Gyro yout [7:0]
  	i2cBuf[0]=0x46;
    HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
    HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
    ygyrol=i2cBuf[1];

    ////Gyro zout [15:0]
    //	i2cBuf[0]=0x47;
    //	HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
    //	HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
    //	zgyroh=i2cBuf[1];

    //// Gyro zout [7:0]
    // 	i2cBuf[0]=0x48;
    //	HAL_I2C_Master_Transmit(&hi2c4,0x68<<1,i2cBuf,1,10);
    //	HAL_I2C_Master_Receive(&hi2c4,0x68<<1,&i2cBuf[1],1,10);
    //	zgyrol=i2cBuf[1];
		
    xgyro=(xgyroh << 8) | (xgyrol & 0xff);
    ygyro=(ygyroh << 8) | (ygyrol & 0xff);
    zgyro=(zgyroh << 8) | (zgyrol & 0xff);
	
	
    msb=1 << 15;	//Checking the MSB
    
    //Computing the 2's Complement for x data
    if(xgyro & msb)
    {
      xgyrofinal=-(65536-(uint16_t)xgyro);
    }
    else
      xgyrofinal=(int16_t)xgyro;
    //Computing the 2's Complement for y data
    if(ygyro & msb)
    {
      ygyrofinal=-(65536-(uint16_t)ygyro);
    }
    else
      ygyrofinal=(int16_t)ygyro;
      
    //Computing the 2's Complement for x data	 
    //	 if(zgyro & msb)
    //	 {
    //		 zgyrofinal=-(65536-(uint16_t)zgyro);	 
    //	 }
    //	 else
    //		  zgyrofinal=(int16_t)zgyro;
    //	 
	 
    //Scaling the Sensor Measurement according to the Gyro's Range
    xgyrofinal=xgyrofinal/131;
    ygyrofinal=ygyrofinal/131;
    zgyrofinal=zgyrofinal/131;
    
    // Offset
    xgyrofinal=xgyrofinal+3.3;
    ygyrofinal=ygyrofinal-0.5;
    zgyrofinal=zgyrofinal+1.1;
    
    // Angle Calculation
    xangle_2=xangle_1+(((time)*xgyrofinal)/8000);
    xangle_1=xangle_2;
    yangle_2=yangle_1+(((time)*ygyrofinal)/8000);
    yangle_1=yangle_2;
    zangle_2=zangle_1+(((time)*zgyrofinal)/8000);
    zangle_1=zangle_2;

    //Radian to Degree
    xangle_f=(180/3.14)*xangle_2;
    yangle_f=(180/3.14)*yangle_2;
    zangle_f=(180/3.14)*zangle_2;
    
    
    if(yangle_f>80)
    {
      yangle_f=512;
    }
    else if(yangle_f<-80)
    {
      yangle_f=0;
    }
    else
    {
      yangle_f=256;
    }
    if(xangle_f>80)
    {
      xangle_f=512;  
    }
    else if(xangle_f<-80)
    {
      xangle_f=0;
    }
    else
    {
      xangle_f=256;
    }
    
    //DAC to transmit through xbee
    HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,yangle_f);
    
    HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
    HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,xangle_f);
  }
}		

  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C4 init function */
static void MX_I2C4_Init(void)
{

  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x00303D5B;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_7B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
