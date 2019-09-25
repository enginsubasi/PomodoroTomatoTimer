/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void PTT_State_Machine ( void );
void LED_Driver ( uint8_t LED_5_Bit );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_IWDG_Refresh ( &hiwdg );
      PTT_State_Machine ( );
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POW_CTRL_GPIO_Port, POW_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED5_Pin|LED4_Pin|LED3_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : POW_CTRL_Pin LED5_Pin LED4_Pin LED3_Pin 
                           LED1_Pin */
  GPIO_InitStruct.Pin = POW_CTRL_Pin|LED5_Pin|LED4_Pin|LED3_Pin 
                          |LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
enum State {
    INITIAL = 0,
    WAIT_1_PUSH = 1,
    DOWN_CNT = 2,
    WAIT_2_PUSH = 3,
    DOWN_CNT_2 = 4,
    KILL_MYSELF = 5,
};

uint8_t System_State = INITIAL;

/*
 * @about: Main system state machine.
 */
void PTT_State_Machine ( void )
{
    const uint32_t Global_Timeout = 1 * 60 * 60 * 1000; // 1 hour


    const uint32_t SM_Period = 100 - 1;
    static uint32_t SM_TimeStamp = 0;

    static uint32_t SM_DowncounterTimeStamp = 0;
    static uint32_t SM_DowncounterTime = 0;

    static uint8_t SM_Anm_Cntr = 0;
    static uint8_t SM_LEDAnm1 = 0;
    static uint8_t SM_LEDAnm2 = 0;

    if ( ( HAL_GetTick ( ) - SM_TimeStamp ) > SM_Period )
    {
        SM_TimeStamp = HAL_GetTick ( );

        switch ( System_State )
        {
            case INITIAL:

                ++SM_Anm_Cntr;

                if ( SM_Anm_Cntr < 3 )
                {
                    LED_Driver ( 0x01 );
                }
                else if ( SM_Anm_Cntr < 6 )
                {
                    LED_Driver ( 0x03 );
                }
                else if ( SM_Anm_Cntr < 9 )
                {
                    LED_Driver ( 0x07 );
                }
                else if ( SM_Anm_Cntr < 12 )
                {
                    LED_Driver ( 0x0F );
                }
                else if ( SM_Anm_Cntr < 15 )
                {
                    LED_Driver ( 0x1F );
                }
                else
                {
                    SM_Anm_Cntr = 0;
                    System_State = WAIT_1_PUSH;
                }

            break;

            case WAIT_1_PUSH:

                ++SM_Anm_Cntr;

                if ( SM_Anm_Cntr == 4 )
                {
                    LED_Driver ( 0x00 );
                }
                else if ( SM_Anm_Cntr >= 9 )
                {
                    LED_Driver ( 0x1F );
                    SM_Anm_Cntr = 0;
                }

                if ( HAL_GPIO_ReadPin ( BUTTON_GPIO_Port, BUTTON_Pin ) )
                {
                    System_State = DOWN_CNT;
                    SM_DowncounterTimeStamp = 10 * 60 * 25; /* 25 minutes */
                    SM_DowncounterTime = SM_DowncounterTimeStamp;
                }

            break;

            case DOWN_CNT:

                --SM_DowncounterTime;

                if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 4 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x1E;
                    SM_LEDAnm2 = 0x1F;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 3 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x1C;
                    SM_LEDAnm2 = 0x1E;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 2 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x18;
                    SM_LEDAnm2 = 0x1C;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 1 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x10;
                    SM_LEDAnm2 = 0x18;
                }
                else if ( ( SM_DowncounterTime < ( ( SM_DowncounterTimeStamp * 1 ) / 5 ) ) && SM_DowncounterTime != 0 )
                {
                    SM_LEDAnm1 = 0x00;
                    SM_LEDAnm2 = 0x10;
                }
                else if ( SM_DowncounterTime == 0 )
                {
                    System_State = WAIT_2_PUSH;
                }

                ++SM_Anm_Cntr;

                if ( SM_Anm_Cntr == 4 )
                {
                    LED_Driver ( SM_LEDAnm1 );
                }
                else if ( SM_Anm_Cntr >= 9 )
                {
                    LED_Driver ( SM_LEDAnm2 );
                    SM_Anm_Cntr = 0;
                }

            break;

            case WAIT_2_PUSH:

                ++SM_Anm_Cntr;

                if ( SM_Anm_Cntr == 2 )
                {
                    LED_Driver ( 0x00 );
                }
                else if ( SM_Anm_Cntr >= 9 )
                {
                    LED_Driver ( 0x1F );
                    SM_Anm_Cntr = 0;
                }

                if ( HAL_GPIO_ReadPin ( BUTTON_GPIO_Port, BUTTON_Pin ) )
                {
                    System_State = DOWN_CNT_2;
                    SM_DowncounterTimeStamp = 10 * 60 * 5; /* 5 minutes */
                    SM_DowncounterTime = SM_DowncounterTimeStamp;
                }

            break;

            case DOWN_CNT_2:

                --SM_DowncounterTime;

                if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 4 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x1E;
                    SM_LEDAnm2 = 0x1F;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 3 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x1C;
                    SM_LEDAnm2 = 0x1E;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 2 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x18;
                    SM_LEDAnm2 = 0x1C;
                }
                else if ( SM_DowncounterTime > ( ( SM_DowncounterTimeStamp * 1 ) / 5 ) )
                {
                    SM_LEDAnm1 = 0x10;
                    SM_LEDAnm2 = 0x18;
                }
                else if ( ( SM_DowncounterTime < ( ( SM_DowncounterTimeStamp * 1 ) / 5 ) ) && SM_DowncounterTime != 0 )
                {
                    SM_LEDAnm1 = 0x00;
                    SM_LEDAnm2 = 0x10;
                }
                else if ( SM_DowncounterTime == 0 )
                {
                    System_State = KILL_MYSELF;
                }

                ++SM_Anm_Cntr;

                if ( SM_Anm_Cntr == 2 )
                {
                    LED_Driver ( SM_LEDAnm1 );
                }
                else if ( SM_Anm_Cntr >= 9 )
                {
                    LED_Driver ( SM_LEDAnm2 );
                    SM_Anm_Cntr = 0;
                }

            break;

            case KILL_MYSELF:
                HAL_GPIO_WritePin(POW_CTRL_GPIO_Port, POW_CTRL_Pin, GPIO_PIN_RESET);
            break;

            default:

            break;

        }
    }

    /* Battery saving. To avoid unnecessary use. */
    if ( HAL_GetTick ( ) > Global_Timeout )
    {
        HAL_GPIO_WritePin(POW_CTRL_GPIO_Port, POW_CTRL_Pin, GPIO_PIN_RESET );
    }
}

/*
 * @about: Drives LEDs on PTT.
 */
void LED_Driver ( uint8_t LED_5_Bit )
{
    uint8_t ldloop_i = 0;

    for ( ; ldloop_i < 5 ; ++ldloop_i )
    {
        switch ( ldloop_i )
        {
            case 0:
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, ( LED_5_Bit & 0x01 ) );
            break;

            case 1:
                HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, ( ( LED_5_Bit >> 1 ) & 0x01 ) );
            break;

            case 2:
                HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, ( ( LED_5_Bit >> 2 ) & 0x01 ) );
            break;

            case 3:
                HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, ( ( LED_5_Bit >> 3 ) & 0x01 ) );
            break;

            case 4:
                HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, ( ( LED_5_Bit >> 4 ) & 0x01 ) );
            break;

            default:

            break;
        }
    }
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
