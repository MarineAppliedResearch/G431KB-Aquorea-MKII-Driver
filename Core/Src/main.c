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

#define JSMN_HEADER // Needed by all files that use jsmn.h for json parsing
#include "jsmn.h"   // Json Parsing Library

#include <string.h>
#include "serial.h"
#include "subc_mkii.h"
#include <stdio.h>
#include "ethernet.h"
#include "ethernet_udp.h"
#include <stdint.h>
#include <stdbool.h>
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

TIM_HandleTypeDef htim3;

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

// UDP socket used only for the raw serial console bridge.
static EthernetUDP udp_console;

// RX buffer for the main application control socket.
static uint8_t udp_rx_buf[256];

// RX buffer for the serial console socket.
static uint8_t udp_console_rx_buf[256];

static const WizchipNetConfig eth_cfg =
{
    .mac     = { 0xDE, 0xAD, 0xDA, 0xDD, 0xEA, 0x01 },
    .ip      = { 10, 1, 10, 3 },
    .subnet  = { 255, 255, 255, 0 },
    .gateway = { 10, 1, 10, 1 },
    .dns     = { 8, 8, 8, 8 },
    .mode    = WIZCHIP_NET_STATIC
};


// -----------------------------------------------------------------------------
// Flash timing state
//
// These variables track whether the flash is currently active and when it
// should be turned off.
// -----------------------------------------------------------------------------
static volatile bool g_flashActive = false;
static uint32_t g_flashEndTimeMs = 0;

// Used to set the default OFF brightness of the LED
static uint8_t g_flashAlwaysOnBrightness = 0;


// Sequence counter for outbound serial_rx JSON packets sent over the console
// UDP socket. This is independent from any incoming seq values.
static uint32_t g_console_tx_seq = 1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

// Flash function prototypes.
void flash_set_duty(float dutyCycle);
void flash_start(float dutyCycle, uint8_t defaultDutyCycle, uint32_t durationMs);
void flash_stop(void);
void flash_update(void);
void flash_pin_force_off_gpio(void);
void flash_pin_enable_pwm_mode(void);



// Reset and bring up the ethernet network
static bool bringup_network(void);

// Poll the console UDP socket, parse one JSON command packet if present,
/// and forward the requested text to the selected serial device.
static void poll_udp_console(EthernetUDP *udp_console);

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

// -----------------------------------------------------------------------------
// jsoneq
//
// Check whether one jsmn token matches a specific JSON object key or string
// value in the received packet text.
//
// Wider context:
//   jsmn does not build C strings or objects for us. It only returns tokens
//   that point to start/end positions inside the original JSON buffer.
//   This helper lets higher-level parsing code compare one of those tokens
//   against an expected string such as "type", "device", "data", or
//   "serial_tx" while walking through the parsed token list.
//
// Returns:
//   0  if the token text exactly matches the provided string
//  -1  otherwise
// -----------------------------------------------------------------------------
static int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
    // Require the token to be a JSON string and require its length
    // to exactly match the comparison string length.
    if (tok->type == JSMN_STRING &&
        (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0)
    {
        return 0;
    }

    return -1;
}


// -----------------------------------------------------------------------------
// poll_udp_console
//
// Poll the dedicated console UDP socket and, if a JSON packet is present,
// parse the requested serial device and text payload, then forward that
// text to the matching serial port.
//
// Expected JSON format:
//   {"type":"serial_tx","device":"SerialLIGHT","data":"$St"}
// -----------------------------------------------------------------------------
static void poll_udp_console(EthernetUDP *udp_console)
{
    // Hold the size of the next waiting UDP packet.
    int packet_size = 0;

    // Hold the number of bytes actually read from the UDP packet.
    int len = 0;

    // Hold the sender metadata for this console packet.
    uint8_t remote_ip[4];
    uint16_t remote_port;

    // Hold the jsmn parser instance used to tokenize this JSON packet.
    jsmn_parser parser;

    // Hold the parsed token array for this small console message.
    jsmntok_t tokens[32];

    // Hold the parse result returned by jsmn_parse().
    int token_count = 0;

    // Hold pointers into the received JSON text for the values we care about.
    const char *type = NULL;
    const char *device = NULL;
    const char *data = NULL;

    // Hold the token lengths because jsmn tokens are not null-terminated.
    int type_len = 0;
    int device_len = 0;
    int data_len = 0;

    // Hold the incoming sequence number from the JSON packet.
	int seq = 0;

	// Track whether a valid seq field was actually found and parsed.
	bool have_seq = false;

	// Remember the exact seq value token so we can patch only that slice
	// in the original received JSON when echoing the packet back.
	jsmntok_t *seq_value_tok = NULL;

	// Hold the echoed packet after we patch only the seq substring.
	char echo_buf[256];
	int echo_len = 0;

    // Ignore invalid UDP socket pointers.
    if (!udp_console)
        return;

    // Ask whether a new console UDP packet is waiting.
    packet_size = EthernetUDP_parsePacket(udp_console);

    // Stop immediately if no console packet is available.
    if (packet_size <= 0)
        return;

    // Read the waiting console packet into the dedicated console RX buffer.
    // Leave one extra byte so we can add a null terminator safely.
    len = EthernetUDP_read(
        udp_console,
        udp_console_rx_buf,
        sizeof(udp_console_rx_buf) - 1
    );

    // Stop if the UDP read failed or returned no payload.
    if (len <= 0)
        return;

    // Read the sender metadata for this packet.
    if (!EthernetUDP_remoteIP(udp_console, remote_ip) ||
        !EthernetUDP_remotePort(udp_console, &remote_port))
    {
        return;
    }

    // Null-terminate the received packet so helper code can safely treat
    // it like a C string where needed.
    udp_console_rx_buf[len] = '\0';

    // Initialize the jsmn parser before tokenizing this JSON text.
    jsmn_init(&parser);

    // Tokenize the received JSON text into the local token array.
    token_count = jsmn_parse(
        &parser,
        (const char *)udp_console_rx_buf,
        len,
        tokens,
        32
    );

    // Stop if the JSON packet failed to parse.
    if (token_count < 0)
        return;

    // Require the root token to be a JSON object.
    if (token_count < 1 || tokens[0].type != JSMN_OBJECT)
        return;

    // Walk through the returned token array looking for the object keys
        // "type", "device", "data", and "seq". When one of those keys is
        // found, the token immediately after it is that key's value.
    for (int i = 1; i < token_count; i++)
    {
        // Check whether this token is the key "type".
        if (jsoneq((const char *)udp_console_rx_buf, &tokens[i], "type") == 0)
        {
            // Point to the value token that follows the "type" key.
            jsmntok_t *value_tok = &tokens[i + 1];

            // Save the substring pointer and length for the type value.
            type = (const char *)udp_console_rx_buf + value_tok->start;
            type_len = value_tok->end - value_tok->start;

            // Skip the value token since we already consumed it.
            i++;
        }

        // Check whether this token is the key "device".
        else if (jsoneq((const char *)udp_console_rx_buf, &tokens[i], "device") == 0)
        {
            // Point to the value token that follows the "device" key.
            jsmntok_t *value_tok = &tokens[i + 1];

            // Save the substring pointer and length for the device value.
            device = (const char *)udp_console_rx_buf + value_tok->start;
            device_len = value_tok->end - value_tok->start;

            // Skip the value token since we already consumed it.
            i++;
        }

        // Check whether this token is the key "data".
        else if (jsoneq((const char *)udp_console_rx_buf, &tokens[i], "data") == 0)
        {
            // Point to the value token that follows the "data" key.
            jsmntok_t *value_tok = &tokens[i + 1];

            // Save the substring pointer and length for the data value.
            data = (const char *)udp_console_rx_buf + value_tok->start;
            data_len = value_tok->end - value_tok->start;

            // Skip the value token since we already consumed it.
            i++;
        }

        // Check whether this token is the key "seq".
		else if (jsoneq((const char *)udp_console_rx_buf, &tokens[i], "seq") == 0)
		{
			// Point to the value token that follows the "seq" key.
			jsmntok_t *value_tok = &tokens[i + 1];

			// Copy the numeric token into a temporary C string so atoi()
			// can parse it safely.
			char seq_buf[16];
			int seq_len = value_tok->end - value_tok->start;

			// Only parse sequence text that fits in our local temp buffer.
			if (seq_len > 0 && seq_len < (int)sizeof(seq_buf))
			{
				memcpy(seq_buf,
					   (const char *)udp_console_rx_buf + value_tok->start,
					   (size_t)seq_len);

				seq_buf[seq_len] = '\0';

				// Convert the JSON numeric token text into an integer.
				seq = atoi(seq_buf);
				have_seq = true;

				// Save the exact seq token location so the echo path can
				// replace only this numeric substring in the original packet.
				seq_value_tok = value_tok;
			}

			// Skip the value token since we already consumed it.
			i++;
		}
    }

    // Require all three expected JSON fields before continuing.
    if (!type || !device || !data || !have_seq || !seq_value_tok)
        return;

    // Only process packets whose "type" value is exactly "serial_tx".
    if (!(type_len == (int)strlen("serial_tx") &&
          strncmp(type, "serial_tx", type_len) == 0))
    {
        return;
    }

    // Route the outgoing text to SerialLIGHT when requested by name.
    if (device_len == (int)strlen("SerialLIGHT") &&
        strncmp(device, "SerialLIGHT", device_len) == 0)
    {
    	//Serial_println(&SerialLIGHT, "test\r\n");
        Serial_write(&SerialLIGHT, (const uint8_t *)data, (uint16_t)data_len);
    }

    // Route the outgoing text to SerialUSB when requested by name.
    else if (device_len == (int)strlen("SerialUSB") &&
             strncmp(device, "SerialUSB", device_len) == 0)
    {
        Serial_write(&SerialUSB, (const uint8_t *)data, (uint16_t)data_len);
    }

// Stop if the device name did not match any known serial port.
	else
	{
		return;
	}

	// Build the echo by copying the ORIGINAL received JSON packet and
	// replacing only the seq numeric token with seq + 1. This preserves
	// field order, spacing, and any extra keys we are not using here.
	{
		char seq_buf[16];
		int seq_out_len = 0;
		int prefix_len = 0;
		int suffix_len = 0;

		// Format the incremented sequence number into text.
		seq_out_len = snprintf(seq_buf, sizeof(seq_buf), "%d", seq + 1);

		// Stop if formatting failed or the new number would not fit.
		if (seq_out_len <= 0 || seq_out_len >= (int)sizeof(seq_buf))
			return;

		// Measure the packet segments before and after the original seq token.
		prefix_len = seq_value_tok->start;
		suffix_len = len - seq_value_tok->end;

		// Make sure the patched packet still fits in our local echo buffer.
		if ((prefix_len + seq_out_len + suffix_len) >= (int)sizeof(echo_buf))
			return;

		// Copy everything before seq exactly as received.
		memcpy(echo_buf,
			   udp_console_rx_buf,
			   (size_t)prefix_len);

		// Insert the incremented seq text.
		memcpy(echo_buf + prefix_len,
			   seq_buf,
			   (size_t)seq_out_len);

		// Copy everything after seq exactly as received.
		memcpy(echo_buf + prefix_len + seq_out_len,
			   udp_console_rx_buf + seq_value_tok->end,
			   (size_t)suffix_len);

		// Total number of bytes in the patched echo packet.
		echo_len = prefix_len + seq_out_len + suffix_len;
	}

	// Send the patched packet back to the same sender.
	if (EthernetUDP_beginPacket(udp_console, remote_ip, remote_port))
	{
		EthernetUDP_write(udp_console, echo_buf, (size_t)echo_len);
		EthernetUDP_endPacket(udp_console);
	}

	// Add more serial devices here with more else-if blocks as needed.
}


// -----------------------------------------------------------------------------
// pump_seriallight_monitor
//
// Drain any mirrored RX bytes from SerialLIGHT, JSON-escape them, and send
// them to the last remote endpoint associated with the console UDP socket.
//
// Current scope:
//   This first version only forwards SerialLIGHT monitor bytes.
//   We are keeping it explicit for now instead of building a registry.
//
// Important:
//   This uses the last remote IP and port currently latched in udp_console.
//   That means it will only transmit after a console client has already sent
//   us at least one packet on that socket.
// -----------------------------------------------------------------------------
static void pump_seriallight_monitor(EthernetUDP *udp_console)
{
    // Hold the last known console client endpoint.
    uint8_t remote_ip[4];
    uint16_t remote_port;

    // Hold raw mirrored UART bytes drained from SerialLIGHT.
    uint8_t raw_buf[96];
    int raw_len = 0;

    // Hold JSON-escaped serial data for the "data" field.
    char esc_buf[220];
    int esc_len = 0;

    // Hold the final outbound JSON packet.
    char tx_buf[320];
    int outlen = 0;

    // Ignore invalid UDP socket pointers.
    if (!udp_console)
        return;

    // Require a previously latched console remote endpoint before trying
    // to send any serial_rx traffic back out.
    if (!udp_console->has_remote)
        return;

    // Copy the last console sender endpoint directly from this UDP instance.
    memcpy(remote_ip, udp_console->remote_ip, sizeof(remote_ip));
    remote_port = udp_console->remote_port;

    // Drain a bounded chunk from the mirrored SerialLIGHT monitor buffer.
    // We intentionally send chunks rather than trying to wait for lines.
    while (Serial_monitor_available(&SerialLIGHT) > 0 && raw_len < (int)sizeof(raw_buf))
    {
        int c = Serial_monitor_read(&SerialLIGHT);

        // Stop if the monitor read failed unexpectedly.
        if (c < 0)
            break;

        raw_buf[raw_len++] = (uint8_t)c;
    }

    // Stop if there was nothing waiting in the mirrored monitor buffer.
    if (raw_len <= 0)
        return;

    // JSON-escape the drained raw serial bytes so quotes and backslashes
    // do not break the outbound JSON packet structure.
    for (int i = 0; i < raw_len; i++)
    {
        uint8_t c = raw_buf[i];

        // Escape backslash as \\ inside the JSON string.
        if (c == '\\')
        {
            if ((esc_len + 2) >= (int)sizeof(esc_buf))
                break;

            esc_buf[esc_len++] = '\\';
            esc_buf[esc_len++] = '\\';
        }

        // Escape quote as \" inside the JSON string.
        else if (c == '"')
        {
            if ((esc_len + 2) >= (int)sizeof(esc_buf))
                break;

            esc_buf[esc_len++] = '\\';
            esc_buf[esc_len++] = '"';
        }

        // Escape carriage return as \r so line endings survive JSON safely.
        else if (c == '\r')
        {
            if ((esc_len + 2) >= (int)sizeof(esc_buf))
                break;

            esc_buf[esc_len++] = '\\';
            esc_buf[esc_len++] = 'r';
        }

        // Escape newline as \n for the same reason.
        else if (c == '\n')
        {
            if ((esc_len + 2) >= (int)sizeof(esc_buf))
                break;

            esc_buf[esc_len++] = '\\';
            esc_buf[esc_len++] = 'n';
        }

        // Pass through normal printable and raw single-byte characters.
        else
        {
            if ((esc_len + 1) >= (int)sizeof(esc_buf))
                break;

            esc_buf[esc_len++] = (char)c;
        }
    }

    // Null-terminate the escaped data so snprintf can consume it safely.
    esc_buf[esc_len] = '\0';

    // Build one outbound serial_rx JSON packet for this drained chunk.
    outlen = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "{\"type\":\"serial_rx\",\"device\":\"SerialLIGHT\",\"data\":\"%s\",\"seq\":%lu}",
        esc_buf,
        (unsigned long)g_console_tx_seq++
    );

    // Stop if formatting failed or the packet did not fit.
    if (outlen <= 0 || outlen >= (int)sizeof(tx_buf))
        return;

    // Send the JSON packet to the last console client endpoint.
    if (EthernetUDP_beginPacket(udp_console, remote_ip, remote_port))
    {
        EthernetUDP_write(udp_console, tx_buf, (size_t)outlen);
        EthernetUDP_endPacket(udp_console);
    }
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the PWM light control.
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // Make sure flash doesn't start on
  flash_stop();

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

  // Set the pwm mode in the light itself to 1000k
  subc_mkii_set_dimming_frequency(&light_driver, 1000);


  EthernetUDP_set_logger(wiznet_log);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Service the flash timer
	  flash_update();

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

	    // Poll the dedicated console UDP socket and forward any valid
	    // JSON serial_tx packet to the requested serial device.
	    poll_udp_console(&udp_console);

	    // Drain mirrored SerialLIGHT RX bytes and forward them to the last
	   	// console client as serial_rx JSON packets on the console socket.
	   	pump_seriallight_monitor(&udp_console);

	   // ---------------- UDP command handling ----------------

		int packet_size = EthernetUDP_parsePacket(&udp);
		if (packet_size > 0)
		{
		   int len = EthernetUDP_read(&udp,
									  udp_rx_buf,
									  sizeof(udp_rx_buf));

		   if (len > 0)
		   {

			   // Force null termination so sscanf sees a clean C string and does not
			   	// read stale bytes left over from a previous longer packet.
			   	udp_rx_buf[len] = '\0';


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
				   else if(c == 'K'){
					   // probably the keepalive packet
					   // do nothing for now

				   }
				   else if (c == '0')
				   {
					   subc_mkii_set_brightness(&light_driver, 0);
					   outlen = snprintf(buf, sizeof(buf),
										 "Send OFF Command\r\n");
				   }
				   else if(c == 's'){
					   subc_mkii_assert_single_signal_mode(&light_driver);
				   }
				   else if(c == 'd'){
					   subc_mkii_set_dimming_frequency(&light_driver, 1000);
				   }
				   else if (c == 'f') {
				       int duty = 0;
				       int defaultDuty = 0;
				       int length_ms = 0;

				       if (sscanf((const char*)udp_rx_buf, "f,%d,%d,%d", &duty, &defaultDuty,  &length_ms) == 3) {
				           if (duty < 0) {
				               duty = 0;
				           }

				           if (duty > 255) {
				               duty = 255;
				           }

				           if (defaultDuty < 0) {
							   defaultDuty = 0;
						   }

						   if (defaultDuty > 255) {
							   defaultDuty = 255;
						   }

				           if (length_ms > 0) {
				               float dutyCycle = ((float)duty) / 255.0f;
				               flash_start(dutyCycle, (uint8_t)defaultDuty, (uint32_t)length_ms);
				        	   //flash_force_high_test((uint32_t)length_ms);
				           }
				       }
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
				   else if (c == 'S')
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
										 "Unknown cmd '%c'\r\n", (char)udp_rx_buf[0]);
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

			   else if(c == 's'){
				   subc_mkii_assert_single_signal_mode(&light_driver);
				   Serial_print(&SerialUSB, "subc_mkii_assert_single_signal_mode\r\n");
			   }

			   else if(c== 'q'){
				   flash_start(0.50f, 0, 25);
			   }
			   else if(c== 'a'){
			   	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
    // Bring up the Ethernet interface first so UDP sockets can be opened.
    if (!Ethernet_begin(&eth_cfg))
        return false;

    // Open the main application control socket on port 5000.
    if (!EthernetUDP_begin(&udp, 5000))
        return false;

    // Open the dedicated serial console socket on port 5001.
    if (!EthernetUDP_begin(&udp_console, 5001))
        return false;

    return true;
}


// -----------------------------------------------------------------------------
// flash_force_high_test
//
// TEMP TEST:
// Stops PWM on TIM3 CH4, reconfigures PB7 as a normal GPIO output, drives it
// steadily high for the requested time, then drives it low again.
// -----------------------------------------------------------------------------
void flash_force_high_test(uint32_t durationMs)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Stop PWM so TIM3 CH4 releases PB7.
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);

    // Reconfigure PB7 from TIM3 alternate function into plain GPIO output.
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Drive PB7 to a steady 3.3V.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    // Hold it high for the requested duration.
    HAL_Delay(durationMs);

    // Drive PB7 low again.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
}


// -----------------------------------------------------------------------------
// flash_pwm_for_time
//
// Applies a PWM duty cycle on TIM3 CH4 for a fixed number of milliseconds,
// then turns the output back off.
// duty_cycle: 0.0f to 1.0f
// duration_ms: how long to hold that duty cycle
// -----------------------------------------------------------------------------
void flash_pwm_for_time(float duty_cycle, uint32_t duration_ms)
{
    uint32_t period;
    uint32_t compare;

    // Clamp the requested duty cycle so invalid values do not go out of range.
    if (duty_cycle < 0.0f) {
        duty_cycle = 0.0f;
    }

    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    }

    // Read the timer period directly from the hardware setup.
    // This makes the function still work even if you later change ARR in CubeMX.
    period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    // Convert 0.0 to 1.0 into 0 to period.
    // Example with period = 255:
    // 0.0 -> 0
    // 0.5 -> about 128
    // 1.0 -> 255
    compare = (uint32_t)(duty_cycle * (float)period);

    // Apply the requested duty cycle to TIM3 channel 4.
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);

    // Hold that PWM value for the requested flash duration.
    HAL_Delay(duration_ms);

    // Turn the PWM output back off.
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}



// -----------------------------------------------------------------------------
// flash_set_duty
//
// Sets TIM3 CH4 duty cycle from a normalized value in the range 0.0 to 1.0.
// The timer period is read from the hardware so this still works if ARR changes
// later in CubeMX.
// -----------------------------------------------------------------------------
void flash_set_duty(float dutyCycle)
{
    uint32_t period;
    uint32_t compare;

    // Clamp duty cycle into the legal range.
    if (dutyCycle < 0.0f) {
        dutyCycle = 0.0f;
    }

    if (dutyCycle > 1.0f) {
        dutyCycle = 1.0f;
    }

    // Read the timer auto reload value.
    // If ARR = 255, this gives an Arduino like 8 bit scale.
    period = __HAL_TIM_GET_AUTORELOAD(&htim3);

    // Convert 0.0 to 1.0 into 0 to period.
    compare = (uint32_t)(dutyCycle * (float)period);

    // Apply the duty cycle to TIM3 channel 4.
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
}

// -----------------------------------------------------------------------------
// flash_start
//
// Starts the flash at the requested duty cycle and records when it must end.
// This function returns immediately, so it does not block the rest of firmware.
// -----------------------------------------------------------------------------
void flash_start(float dutyCycle, uint8_t defaultDutyCycle, uint32_t durationMs)
{
    // Apply the requested PWM duty right now.
    flash_set_duty(dutyCycle);

    g_flashAlwaysOnBrightness = defaultDutyCycle; //defaultDutyCycle;

    // Record the stop time using the HAL millisecond tick.
    g_flashEndTimeMs = HAL_GetTick() + durationMs;

    // Mark the flash as currently active.
    g_flashActive = true;
}

// -----------------------------------------------------------------------------
// flash_stop
//
// Forces the flash off immediately.
// -----------------------------------------------------------------------------
void flash_stop(void)
{
    // Set PWM duty to zero so the flash output turns off.
    flash_set_duty((float)g_flashAlwaysOnBrightness/255.0f);



    // Mark the flash as inactive.
    g_flashActive = false;

    //g_flashAlwaysOnBrightness = 0;
}

// -----------------------------------------------------------------------------
// flash_update
//
// Checks whether the active flash duration has expired. If so, it turns the
// flash off. Call this repeatedly from the main loop.
// -----------------------------------------------------------------------------
void flash_update(void)
{
    // Nothing to do if the flash is already off.
    if (!g_flashActive) {
        return;
    }

    // Use signed subtraction so tick rollover is handled safely.
    if ((int32_t)(HAL_GetTick() - g_flashEndTimeMs) >= 0) {
        flash_stop();
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
