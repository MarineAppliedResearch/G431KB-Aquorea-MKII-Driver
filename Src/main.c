/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include "serial.h"
#include "subc_mkii.h"
#include <stdio.h>
//#include "wizchip_driver.h"
//#include "wizchip_conf.h"
#include "ethernet.h"
#include "ethernet_udp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

const char motd[] =
"========================================================================\n"
"  ███╗   ███╗  █████╗  ██████╗ ███████╗\n"
"  ████╗ ████║ ██╔══██╗ ██╔══██╗██╔════╝\n"
"  ██╔████╔██║ ███████║ ██████╔╝█████╗\n"
"  ██║╚██╔╝██║ ██╔══██║ ██╔══██╗██╔══╝\n"
"  ██║ ╚═╝ ██║ ██║  ██║ ██║  ██║███████╗\n"
"  ╚═╝     ╚═╝ ╚═╝  ╚═╝ ╚═╝  ╚═╝╚══════╝\n"
"\n"
"  Marine Applied Research & Exploration\n"
"------------------------------------------------------------------------\n"
"  Device   : SubC Aquorea MkII Light Controller\n"
"  FW       : v0.1\n"
"  Light FW : MkII v1.20\n"
"  Build    : " __DATE__ " " __TIME__ "\n"
"========================================================================\n";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Create our Serial Port meant for USB debugging
SerialPort SerialUSB;
SerialPort SerialLIGHT;
SubcMkII light_driver;

// UDP echo test instance
static EthernetUDP udp;

// Simple RX buffer for echo testing
static uint8_t udp_rx_buf[256];

// Create our Ethernet Driver
////WizchipDriver eth_driver;

// Track if Ethernet has been initialized
////bool eth_initialized = false;


static const WizchipNetConfig eth_cfg =
{
    .mac     = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x01 },
    .ip      = { 192, 168, 50, 3 },
    .subnet  = { 255, 255, 255, 0 },
    .gateway = { 192, 168, 50, 2 },
    .dns     = { 8, 8, 8, 8 },
    .mode    = WIZCHIP_NET_STATIC
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Simple logger adapter for WIZnet drivers.
// Routes driver log messages to the USB serial port.
static void wiznet_log(const char *msg)
{
    Serial_print(&SerialUSB, (char *)msg);
    Serial_print(&SerialUSB, "\r\n");
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  // Start the Drivers
  Serial_begin(&SerialUSB, &huart2, 115200);
  Serial_begin(&SerialLIGHT, &huart1, 9600);
  subc_mkii_init(&light_driver, &SerialLIGHT);

  Serial_print(&SerialUSB, motd);




  // Bring up Ethernet using the new abstraction
  if (!Ethernet_begin(&eth_cfg))
  {
      Serial_print(&SerialUSB, "Ethernet init FAILED\r\n");
  }
  else
  {
      Serial_print(&SerialUSB, "Ethernet init OK\r\n");

      // Start UDP listener on port 5000
      if (EthernetUDP_begin(&udp, 5000))
      {
          Serial_print(&SerialUSB, "UDP echo listening on port 5000\r\n");
      }
      else
      {
          Serial_print(&SerialUSB, "UDP begin FAILED\r\n");
      }
  }


  EthernetUDP_set_logger(wiznet_log);



 // SPI DEBUG CODE, same as getVERSION
  /*
  uint8_t tx[4]  = {0x00, 0x39, 0x01, 0x00};
  uint8_t rx1[4] = {0};
  uint8_t rx2[4] = {0};

uint8_t tx[4]  = {0x00, 0x39, 0x01, 0x00};
uint8_t rx1[4] = {0};
uint8_t rx2[4] = {0};

// Hard reset W5500
HAL_GPIO_WritePin(SPI3_RESET_GPIO_Port, SPI3_RESET_Pin, GPIO_PIN_RESET);
HAL_Delay(10);
HAL_GPIO_WritePin(SPI3_RESET_GPIO_Port, SPI3_RESET_Pin, GPIO_PIN_SET);
HAL_Delay(200);

// CS LOW — keep it low
HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

// First read
HAL_SPI_TransmitReceive(&hspi3, tx, rx1, 4, 100);

// Small gap, CS still low
HAL_Delay(1);

// Second read
HAL_SPI_TransmitReceive(&hspi3, tx, rx2, 4, 100);

// CS HIGH
HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);

while (1) {}
*/





  // Attach logger callbacks for Ethernet bring-up diagnostics
  /*
  wizchip_port_set_logger(wiznet_log);
  wizchip_driver_set_logger(wiznet_log);

  // Initialize Ethernet driver (SPI + reset + basic checks)
  if (wizchip_driver_init(&eth_driver, &eth_cfg))
  {
      Serial_print(&SerialUSB, "W5500 init OK\r\n");
      eth_initialized = true;
  }
  else
  {
      Serial_print(&SerialUSB, "W5500 init FAILED\r\n");
  }
  */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
    {
	   // Run the Light driver
	   subc_mkii_poll(&light_driver, HAL_GetTick());



	   // UDP echo test using accessors only
	   int packet_size = EthernetUDP_parsePacket(&udp);
	   if (packet_size > 0)
	   {
	       int len = EthernetUDP_read(&udp,
	                                  udp_rx_buf,
	                                  sizeof(udp_rx_buf));

	       if (len > 0)
	       {
	           uint8_t  remote_ip[4];
	           uint16_t remote_port;

	           // Retrieve sender metadata via accessors
	           if (EthernetUDP_remoteIP(&udp, remote_ip) &&
	               EthernetUDP_remotePort(&udp, &remote_port))
	           {
	               // Begin Arduino-style packet back to sender
	               if (EthernetUDP_beginPacket(&udp,
	                                           remote_ip,
	                                           remote_port))
	               {
	                   // Write payload into TX buffer
	                   EthernetUDP_write(&udp,
	                                     udp_rx_buf,
	                                     (size_t)len);

	                   // Transmit echoed packet
	                   EthernetUDP_endPacket(&udp);
	               }
	           }
	       }
	   }

/*
	   if (eth_initialized)
	   {
		   // Run the ethernet driver
	       wizchip_driver_poll(&eth_driver, HAL_GetTick());
	   }
*/


	   if (Serial_available(&SerialUSB) > 0)
	   {
	       int c = Serial_read(&SerialUSB);

	       if (c == '1')
	       {
	           subc_mkii_set_brightness(&light_driver, 100);
	           Serial_print(&SerialUSB, "Brightness set to 100\r\n");
	       }
	       else if (c == '0')
	       {
	           subc_mkii_set_brightness(&light_driver, 0);
	           Serial_print(&SerialUSB, "Brightness set to 0\r\n");
	       }
	       else if (c == 't')
	       {
	           uint32_t t_cur;
	           uint32_t t_min;
	           uint32_t t_max;
	           uint32_t t_avg;

	           char buf[128];

	           if (subc_mkii_get_temperature_stats(&light_driver,
	                                               &t_min,
	                                               &t_max,
	                                               &t_avg,
	                                               &t_cur))
	           {
	               // Convert centi-degrees to human-readable form
	               snprintf(buf, sizeof(buf),
	                        "Temp C:%ld.%02ld  Min:%ld.%02ld  Max:%ld.%02ld  Avg:%ld.%02ld\r\n",
	                        t_cur / 100,  abs(t_cur % 100),
	                        t_min / 100,  abs(t_min % 100),
	                        t_max / 100,  abs(t_max % 100),
	                        t_avg / 100,  abs(t_avg % 100));

	               Serial_print(&SerialUSB, buf);
	           }
	           else
	           {
	               Serial_print(&SerialUSB, "Temp: no data\r\n");
	           }
	       }else if (c == 's'){

	       }
	       else if( c == 'm'){
	    	   Serial_print(&SerialUSB, motd);
	       }
	       else if (c == 'u')
	       {
	           uint32_t uptime_ms;
	           char buf[64];

	           if (subc_mkii_get_uptime(&light_driver, &uptime_ms))
	           {
	               uint32_t seconds = uptime_ms / 1000;
	               uint32_t minutes = seconds / 60;
	               uint32_t hours   = minutes / 60;

	               snprintf(buf, sizeof(buf),
	                        "Uptime: %lu:%02lu:%02lu\r\n",
	                        hours,
	                        minutes % 60,
	                        seconds % 60);

	               Serial_print(&SerialUSB, buf);
	           }
	       }
	       else if (c == 'e')
	       {
	           char buf[128];

	           if (Ethernet_status(buf, sizeof(buf)))
	               Serial_print(&SerialUSB, buf);
	           else
	               Serial_print(&SerialUSB, "Ethernet status unavailable\r\n");
	       }
	       else if (c == 'l')
	       {
	           EthernetLinkStatus st = Ethernet_linkStatus();

	           if (st == ETHERNET_LINK_UP)
	               Serial_print(&SerialUSB, "Ethernet Link: UP\r\n");
	           else if (st == ETHERNET_LINK_DOWN)
	               Serial_print(&SerialUSB, "Ethernet Link: DOWN\r\n");
	           else
	               Serial_print(&SerialUSB, "Ethernet Link: UNKNOWN\r\n");
	       }
	       else if (c == 'i')
	       {
	    	   // print ethernet info
	           uint8_t ip[4];
	           uint8_t gw[4];
	           uint8_t mask[4];
	           uint8_t mac[6];
	           char buf[160];

	           if (Ethernet_localIP(ip) &&
	               Ethernet_gatewayIP(gw) &&
	               Ethernet_subnetMask(mask) &&
	               Ethernet_macAddress(mac))
	           {
	               snprintf(buf, sizeof(buf),
	                        "IP %d.%d.%d.%d  "
	                        "GW %d.%d.%d.%d  "
	                        "MASK %d.%d.%d.%d  "
	                        "MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
	                        ip[0], ip[1], ip[2], ip[3],
	                        gw[0], gw[1], gw[2], gw[3],
	                        mask[0], mask[1], mask[2], mask[3],
	                        mac[0], mac[1], mac[2],
	                        mac[3], mac[4], mac[5]);

	               Serial_print(&SerialUSB, buf);
	           }
	           else
	           {
	               Serial_print(&SerialUSB, "NetInfo unavailable\r\n");
	           }
	       }
	       else if (c == 'd')
	       {
	    	   /*
	           wizchip_driver_request_dhcp(&eth_driver);
	           Serial_print(&SerialUSB, "DHCP requested\r\n");
	           */
	       }
	   }

	   /*
	   uint8_t resp[128];
	   uint16_t resp_len;

	   if (subc_mkii_response_available(&light_driver))
	   {
	       subc_mkii_read_response(&light_driver, resp, &resp_len);

	       // Forward raw response to host //
	       for (uint16_t i = 0; i < resp_len; i++)
	       {
	           char out[2] = { (char)resp[i], 0 };
	           Serial_print(&SerialUSB, out);

	       }
	   }

   	   */




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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_RESET_GPIO_Port, SPI3_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_RESET_Pin */
  GPIO_InitStruct.Pin = SPI3_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI3_RESET_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
