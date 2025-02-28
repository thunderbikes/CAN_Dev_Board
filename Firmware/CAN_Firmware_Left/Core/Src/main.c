/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> 
#include <stdlib.h>
#include "cli.h"
#include "stm32f1xx_hal_gpio.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static cli_status_t set_bms(int argc, char **argv);
void user_uart_println(char *string);
void CANSend_BMS(int target);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

cmd_t cmd_tbl[] = {
  {.cmd = "set", .func = set_bms},
};
cli_t cli;
uint8_t rx_data[2];

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader; //CAN Bus Transmit Header
uint8_t               TxData[8];
uint32_t              TxMailbox;
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

uint8_t canRX[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};  //CAN Bus Receive Buffer
uint8_t buffer[1];
int bms_value = 0; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  // configuring the HAL uart interupt
  HAL_UART_Receive_IT(&huart1, rx_data, 1);

  cli.println = user_uart_println;
  cli.cmd_tbl = cmd_tbl;
  cli.cmd_cnt = sizeof(cmd_tbl) / sizeof(cmd_t);
  cli.println("VCU (v1.0) Test Suite v0.1\r\n");
  cli_init(&cli);


  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  TxHeader.IDE = CAN_ID_STD; // Standard ID. CAN_ID_EXT is extended ID. 
  TxHeader.StdId = 0x111;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;          // Data frame size. Max 8. 

  TxData[0] = 0x00;  
  TxData[1] = 0x00; 
  TxData[2] = 0x00; 
  TxData[3] = 0x00; 
  TxData[4] = 0x00; 
  TxData[5] = 0x00; 
  TxData[6] = 0x00; 
  TxData[7] = 0x00; 

  HAL_CAN_ConfigFilter(&hcan,&canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  int toggle = 0; 




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    cli_process(&cli);


    // if (HAL_UART_Receive(&huart1, buffer, 1, 0xFFFF) == HAL_OK) {
    //         // If data is received, trigger action

    //         HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    //         receive_transmitCAN(buffer[0]);
    //         //HAL_UART_Transmit(&huart1, buffer, 1, 0xFFFF);
    //         if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    //           {
    //             Error_Handler ();
    //           }
            
    //     }

    // toggle_transmitCAN(toggle);
    // toggle ^= 1; 
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_UART_Receive_IT(&huart1, rx_data, 1);
  cli_put(&cli, rx_data[0]);
  user_uart_println((char *)rx_data);
}

// PUTCHAR_PROTOTYPE
// {
//   /* Place your implementation of fputc here */
//   /* e.g. write a character to the USART1 and Loop until the end of transmission */
//   HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

//   return ch;
// }

// void send_uart_message(char *message) {
//     HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
// }

void user_uart_println(char *string) {
  HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
}



// void printRPM(uint8_t LSB, uint8_t MSB){
//   int RPM; 
//   RPM = MSB * 256 + LSB; 
//   printf("RPM: %d", RPM); 
// }

// This function assumes the 16 bit votlage is a uint16 in volts

void CANSend_BMS(int target){

  if(target > 0)
  {
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x6B0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0x9; 
    TxData[1] = 0x10; 
    TxData[2] = target & 0xFF; 
    TxData[3] = (target >> 8) & 0xFF; 
    TxData[4] = 0x05; 
    TxData[5] = 0x06; 
    TxData[6] = 0x07; 
    TxData[7] = 0x11; 
  } else {
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x6B0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0x9; 
    TxData[1] = 0x10; 
    TxData[2] = 0x00; 
    TxData[3] = 0x00; 
    TxData[4] = 0x05; 
    TxData[5] = 0x06; 
    TxData[6] = 0x07; 
    TxData[7] = 0x11; 
  } 

}



cli_status_t set_bms(int argc, char **argv) {
  if (argc == 2) {
    bms_value = atoi(argv[1]);  
    cli.println("BMS VALUE: ");
    cli.println(argv[1]);
    CANSend_BMS(bms_value);
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    
  } else {
    cli.println("ERROR");
    return CLI_E_INVALID_ARGS;
  }
  return CLI_OK;
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
    //printf("bruh");
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    HAL_Delay(200);
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
