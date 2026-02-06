/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Marine Applied Research & Exploration
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
"  FW       : v0.2\n"
"  Light FW : MkII v1.20\n"
"  Build    : " __DATE__ " " __TIME__ "\n"
"========================================================================\n";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Create our Serial Port meant for USB debugging
SerialPort SerialUSB;

// Create our Serial Port meant for sending commands to our light
SerialPort SerialLIGHT;

// Create our driver for our light
SubcMkII light_driver;

// UDP echo test instance
static EthernetUDP udp;

// Simple RX buffer for echo testing
static uint8_t udp_rx_buf[256];

static const WizchipNetConfig eth_cfg =
{
    .mac     = { 0xDE, 0xAD, 0xDA, 0xDD, 0xEA, 0x01 },
    .ip      = { 10, 1, 10, 3 },
    .subnet  = { 255, 255, 255, 0 },
    .gateway = { 10, 1, 10, 1 },
    .dns     = { 8, 8, 8, 8 },
    .mode    = WIZCHIP_NET_STATIC
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */


// Reset and bring up the ethernet network
static bool bringup_network(void);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start the serial devices
  Serial_begin(&SerialUSB, &huart1, 115200);    // Our debug serial is hooked up as huart1
  Serial_begin(&SerialLIGHT, &huart3, 9600);    // Our light serial is hooked up as huart3

  // Initialize the light driver, with the given serial connection
  subc_mkii_init(&light_driver, &SerialLIGHT);

  // Print out the motd on startup
  Serial_print(&SerialUSB, motd);

  // Bring up Ethernet using the new abstraction
  if (!bringup_network())
  {
      Serial_print(&SerialUSB, "Network bringup FAILED\r\n");
  }


  EthernetUDP_set_logger(wiznet_log);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // this is how we toggle led light on the board
	  /*
	     HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
	     HAL_Delay(2);

	     // RED off, GREEN on
	     HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	     HAL_Delay(2);

	     // GREEN off, BLUE on
	     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
	     HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
	     HAL_Delay(2);

	     // BLUE off
	     HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
	     HAL_Delay(2);
	     */

	    // Run the Light driver
	    subc_mkii_poll(&light_driver, HAL_GetTick());

	   // ---------------- UDP command handling ----------------

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

			   if (EthernetUDP_remoteIP(&udp, remote_ip) &&
				   EthernetUDP_remotePort(&udp, &remote_port))
			   {
				   char c = (char)udp_rx_buf[0];   // SAME command model as Serial
				   char buf[512];
				   int  outlen = 0;

				   // ---------- Brightness ----------
				   if (c == '1')
				   {
					   subc_mkii_set_brightness(&light_driver, 100);
					   outlen = snprintf(buf, sizeof(buf),
										 "Sent ON Command\r\n");
				   }
				   else if (c == '0')
				   {
					   subc_mkii_set_brightness(&light_driver, 0);
					   outlen = snprintf(buf, sizeof(buf),
										 "Send OFF Command\r\n");
				   }

				   // ---------- Temperature ----------
				   else if (c == 't')
				   {
					   uint32_t t_cur, t_min, t_max, t_avg;

					   if (subc_mkii_get_temperature_stats(&light_driver,
														   &t_min,
														   &t_max,
														   &t_avg,
														   &t_cur))
					   {
						   outlen = snprintf(buf, sizeof(buf),
							   "Temp C:%ld.%02ld  Min:%ld.%02ld  Max:%ld.%02ld  Avg:%ld.%02ld\r\n",
							   t_cur / 100,  abs(t_cur % 100),
							   t_min / 100,  abs(t_min % 100),
							   t_max / 100,  abs(t_max % 100),
							   t_avg / 100,  abs(t_avg % 100));
					   }
					   else
					   {
						   outlen = snprintf(buf, sizeof(buf),
											 "Temp: no data\r\n");
					   }
				   }

				   // ---------- MOTD ----------
				   else if (c == 'm')
				   {
					   outlen = snprintf(buf, sizeof(buf), "%s", motd);
				   }

				   // ---------- Uptime ----------
				   else if (c == 'u')
				   {
					   uint32_t uptime_ms;

					   if (subc_mkii_get_uptime(&light_driver, &uptime_ms))
					   {
						   uint32_t seconds = uptime_ms / 1000;
						   uint32_t minutes = seconds / 60;
						   uint32_t hours   = minutes / 60;

						   outlen = snprintf(buf, sizeof(buf),
											 "Uptime: %lu:%02lu:%02lu\r\n",
											 hours,
											 minutes % 60,
											 seconds % 60);
					   }
				   }

				   // ---------- Ethernet status ----------
				   else if (c == 'e')
				   {
					   if (Ethernet_status(buf, sizeof(buf)))
						   outlen = strlen(buf);
					   else
						   outlen = snprintf(buf, sizeof(buf),
											 "Ethernet status unavailable\r\n");
				   }

				   // ---------- Link status ----------
				   else if (c == 'l')
				   {
					   EthernetLinkStatus st = Ethernet_linkStatus();

					   if (st == ETHERNET_LINK_UP)
						   outlen = snprintf(buf, sizeof(buf),
											 "Ethernet Link: UP\r\n");
					   else if (st == ETHERNET_LINK_DOWN)
						   outlen = snprintf(buf, sizeof(buf),
											 "Ethernet Link: DOWN\r\n");
					   else
						   outlen = snprintf(buf, sizeof(buf),
											 "Ethernet Link: UNKNOWN\r\n");
				   }

				   // ---------- Network info ----------
				   else if (c == 'i')
				   {
					   uint8_t ip[4], gw[4], mask[4], mac[6];

					   if (Ethernet_localIP(ip) &&
						   Ethernet_gatewayIP(gw) &&
						   Ethernet_subnetMask(mask) &&
						   Ethernet_macAddress(mac))
					   {
						   outlen = snprintf(buf, sizeof(buf),
							   "IP %d.%d.%d.%d  "
							   "GW %d.%d.%d.%d  "
							   "MASK %d.%d.%d.%d  "
							   "MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
							   ip[0], ip[1], ip[2], ip[3],
							   gw[0], gw[1], gw[2], gw[3],
							   mask[0], mask[1], mask[2], mask[3],
							   mac[0], mac[1], mac[2],
							   mac[3], mac[4], mac[5]);
					   }
					   else
					   {
						   outlen = snprintf(buf, sizeof(buf),
											 "NetInfo unavailable\r\n");
					   }
				   }

				   // ---------- STATS (SPECIAL CASE) ----------
				   else if (c == 's')
				   {
					   if (EthernetUDP_beginPacket(&udp, remote_ip, remote_port))
					   {
						   // ---------- Ethernet global stats ----------
						   const Ethernet_Stats *eth = Ethernet_getStats();

						   int len2 = snprintf(buf, sizeof(buf),
							   "\r\nEthernet stats:\r\n"
							   "  Init OK:        %lu\r\n"
							   "  Init failures: %lu\r\n"
							   "  Link UP events: %lu\r\n"
							   "  Link DOWN events: %lu\r\n"
							   "  RX packets:    %lu\r\n"
							   "  TX packets:    %lu\r\n"
							   "  RX bytes:      %lu\r\n"
							   "  TX bytes:      %lu\r\n",
							   eth->init_count,
							   eth->init_failures,
							   eth->link_up_events,
							   eth->link_down_events,
							   eth->rx_packets_total,
							   eth->tx_packets_total,
							   eth->rx_bytes_total,
							   eth->tx_bytes_total);

						   EthernetUDP_write(&udp, buf, (size_t)len2);

						   // ---------- UDP instance stats ----------
						   len2 = snprintf(buf, sizeof(buf),
							   "\r\nUDP socket %u stats:\n"
							   "  RX packets: %lu\n"
							   "  RX bytes:   %lu\n"
							   "  RX errors:  %lu\n"
							   "  TX packets: %lu\n"
							   "  TX bytes:   %lu\n"
							   "  TX errors:  %lu\n",
							   udp.socket,
							   udp.stats.rx_packets,
							   udp.stats.rx_bytes,
							   udp.stats.rx_errors,
							   udp.stats.tx_packets,
							   udp.stats.tx_bytes,
							   udp.stats.tx_errors);

						   EthernetUDP_write(&udp, buf, (size_t)len2);
						   EthernetUDP_endPacket(&udp);
					   }

					   continue;   // DO NOT fall through to generic send
				   }


				   // ---------- Unknown command ----------
				   else
				   {
					   outlen = snprintf(buf, sizeof(buf),
										 "Unknown cmd '%c'\r\n", udp_rx_buf);
				   }

				   // ---------- Generic UDP reply ----------
				   if (outlen > 0)
				   {
					   if (EthernetUDP_beginPacket(&udp, remote_ip, remote_port))
					   {
						   EthernetUDP_write(&udp, buf, (size_t)outlen);
						   EthernetUDP_endPacket(&udp);
					   }
				   }
			   }
		   }
		}


		// Now check and parse any serial commands
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
			   }else if (c == 'p'){
				   Serial_print(&SerialLIGHT, "$St");
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
			   else if (c == 'r')
			   {
				   if (!Ethernet_reset())
				   {
					   Serial_print(&SerialUSB, "Ethernet reset FAILED\r\n");
				   }
				   else if (!bringup_network())
				   {
					   Serial_print(&SerialUSB, "Network rebind FAILED\r\n");
				   }
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
			   else if (c == 's')
			   {
				   char buf[512];

				   // ---------- Ethernet global stats ----------
				   const Ethernet_Stats *eth = Ethernet_getStats();

				   snprintf(buf, sizeof(buf),
							"\r\nEthernet stats:\r\n"
							"  Init OK:        %lu\r\n"
							"  Init failures: %lu\r\n"
							"  Link UP events: %lu\r\n"
							"  Link DOWN events: %lu\r\n"
							"  RX packets:    %lu\r\n"
							"  TX packets:    %lu\r\n"
							"  RX bytes:      %lu\r\n"
							"  TX bytes:      %lu\r\n",
							eth->init_count,
							eth->init_failures,
							eth->link_up_events,
							eth->link_down_events,
							eth->rx_packets_total,
							eth->tx_packets_total,
							eth->rx_bytes_total,
							eth->tx_bytes_total);

				   Serial_print(&SerialUSB, buf);

				   // ---------- UDP instance stats ----------
				   snprintf(buf, sizeof(buf),
							"\r\nUDP socket %u stats:\r\n"
							"  RX packets: %lu\r\n"
							"  RX bytes:   %lu\r\n"
							"  RX errors:  %lu\r\n"
							"  TX packets: %lu\r\n"
							"  TX bytes:   %lu\r\n"
							"  TX errors:  %lu\r\n",
							udp.socket,
							udp.stats.rx_packets,
							udp.stats.rx_bytes,
							udp.stats.rx_errors,
							udp.stats.tx_packets,
							udp.stats.tx_bytes,
							udp.stats.tx_errors);

				   Serial_print(&SerialUSB, buf);
			   }
			   else if (c == 'd')
				   {
				   char buf[512];
					   if (!subc_mkii_get_stats_formatted(&light_driver,
																	   buf,
																	   sizeof(buf)))
					   {
						   snprintf(buf, sizeof(buf),
									"Environment: no data\r\n");
					   }

					   Serial_print(&SerialUSB, buf);
				   }
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin|WIZ_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WIZ_RST_Pin|WIZ_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : WIZ_CS_Pin */
  GPIO_InitStruct.Pin = WIZ_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WIZ_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WIZ_RST_Pin WIZ_INT_Pin */
  GPIO_InitStruct.Pin = WIZ_RST_Pin|WIZ_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static bool bringup_network(void)
{
    if (!Ethernet_begin(&eth_cfg))
        return false;

    if (!EthernetUDP_begin(&udp, 5000))
        return false;

    return true;
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
