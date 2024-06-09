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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "OpenDrone_FC_HwIntf.h"
#include "OpenDrone_FC.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FREQ_HZ_TO_TIME_US(x)       (1000000.0f/(x))
#define TIME_US_TO_FREQ_HZ(x)       (1000000.0f/(x))

#define IDX_TASK_500_HZ             0
#define IDX_TASK_100_HZ             1
#define IDX_TASK_50_HZ              2
#define IDX_TASK_5_HZ               3
#define NUM_OF_TASK                 4

#define FREQ_500_HZ_TIME_US         FREQ_HZ_TO_TIME_US(500)
#define FREQ_100_HZ_TIME_US         FREQ_HZ_TO_TIME_US(100)
#define FREQ_50_HZ_TIME_US          FREQ_HZ_TO_TIME_US(50)
#define FREQ_5_HZ_TIME_US           FREQ_HZ_TO_TIME_US(5)
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define PRINT_ANGLE
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#ifdef USE_SERIAL_DEBUG
uint8_t log_buf[50];
uint16_t task_freq[NUM_OF_TASK];
#endif

uint32_t last_time_us[NUM_OF_TASK] = {0};
OpenDrone_TxProto_Msg_t OpenDrone_TxProto_Msg = {0};
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
    MX_DMA_Init();
    MX_SPI2_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_I2C2_Init();
    /* USER CODE BEGIN 2 */
    PeriphIMU_Init();
#ifdef USE_SERIAL_DEBUG
    sprintf((char *)log_buf, "\r\nInit peripheral IMU complete");
    hw_intf_uart_debug_send(log_buf, 30);
#endif

    PeriphRadio_Init();
#ifdef USE_SERIAL_DEBUG
    sprintf((char *)log_buf, "\r\nInit peripheral RADIO complete");
    hw_intf_uart_debug_send(log_buf, 32);
#endif

    PeriphEsc_Init();
#ifdef USE_SERIAL_DEBUG
    sprintf((char *)log_buf, "\r\nInit peripheral ESC complete");
    hw_intf_uart_debug_send(log_buf, 30);
#endif

    PeriphController_Init();
#ifdef USE_SERIAL_DEBUG
    sprintf((char *)log_buf, "\r\nInit peripheral CONTROLLER complete");
    hw_intf_uart_debug_send(log_buf, 37);
#endif
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t current_time = hw_intf_get_time_us();

        /* Task 500 Hz */
        if ((current_time - last_time_us[IDX_TASK_500_HZ]) > FREQ_500_HZ_TIME_US)
        {
        	PeriphIMU_UpdateAccel();
        	PeriphIMU_UpdateGyro();
        	PeriphIMU_UpdateFilter();

#ifdef USE_SERIAL_DEBUG
            task_freq[IDX_TASK_500_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_500_HZ]);
#endif

            last_time_us[IDX_TASK_500_HZ] = current_time;
        }

        /* Task 100 Hz */
        if ((current_time - last_time_us[IDX_TASK_100_HZ]) > FREQ_100_HZ_TIME_US)
        {
            last_time_us[IDX_TASK_100_HZ] = current_time;
        }

        /* Task 50 Hz */
        if ((current_time - last_time_us[IDX_TASK_50_HZ]) > FREQ_50_HZ_TIME_US)
        {
        	PeriphRadio_Receive((uint8_t *)&OpenDrone_TxProto_Msg);

#ifdef USE_SERIAL_DEBUG
            task_freq[IDX_TASK_50_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_50_HZ]);
#endif

            last_time_us[IDX_TASK_50_HZ] = current_time;
        }

        /* Task 5 Hz */
        if ((current_time - last_time_us[IDX_TASK_5_HZ]) > FREQ_5_HZ_TIME_US)
        {
#ifdef USE_SERIAL_DEBUG

#ifdef PRINT_ANGLE
            float debug_roll, debug_pitch, debug_yaw;
            periph_imu_get_angel(&debug_roll, &debug_pitch, &debug_yaw);

            task_freq[IDX_TASK_5_HZ] = TIME_US_TO_FREQ_HZ(current_time - last_time_us[IDX_TASK_5_HZ]);

            sprintf((char *)log_buf, "\r\nroll: %7.4f\t\tpitch: %7.4f\t\tyaw: %7.4f\t", debug_roll, debug_pitch, debug_yaw);
            hw_intf_uart_debug_send(log_buf, 50);

            sprintf((char *)log_buf, "\r\nTask 500 Hz actual frequency: %d Hz", task_freq[IDX_TASK_500_HZ]);
            hw_intf_uart_debug_send(log_buf, 45);

            sprintf((char *)log_buf, "\r\nTask 50 Hz actual frequency: %d Hz", task_freq[IDX_TASK_50_HZ]);
            hw_intf_uart_debug_send(log_buf, 45);

            sprintf((char *)log_buf, "\r\nTask 5 Hz actual frequency: %d Hz", task_freq[IDX_TASK_5_HZ]);

            hw_intf_uart_debug_send(log_buf, 45);
#endif

            sprintf((char *)log_buf, "\r\nthrottle: %03d \troll: %03d \tpitch: %03d \tyaw: %03d",
                    OpenDrone_TxProto_Msg.Payload.StabilizerCtrl.throttle,
                    OpenDrone_TxProto_Msg.Payload.StabilizerCtrl.roll,
                    OpenDrone_TxProto_Msg.Payload.StabilizerCtrl.pitch,
                    OpenDrone_TxProto_Msg.Payload.StabilizerCtrl.yaw);

            hw_intf_uart_debug_send(log_buf, 50);
#endif
            last_time_us[IDX_TASK_5_HZ] = current_time;
        }
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
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
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
