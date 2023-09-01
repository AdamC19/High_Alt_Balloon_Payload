/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "hrtim.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h> //for va_list var arg functions
#include "adf4002.h"
#include "mcp9808.h"
#include "printf.h"
#include "ublox.h"
#include "ax25.h"
#include "aprs.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_FILE_COUNT 16

typedef enum radio_mode_enum {
  MODE_MANUAL,
  MODE_AUTO,
  MODE_TEST
}radio_mode_t;

typedef enum radio_states_enum {
  STATE_IDLE,
  STATE_POWER_ON_LO,
  STATE_WAIT_FOR_LO_LOCK,
  STATE_POWER_ON_PA,
  STATE_WAIT_FOR_PA,
  STATE_TRANSMIT,
  STATE_GOTO_IDLE
}radio_state_t;

typedef enum fields_enum {
  MODE,
  RADIO_STATE,
  PA_EN,
  LO_GEN_EN,
  LO_PLL_R_COUNTER_H,
  LO_PLL_R_COUNTER_L,
  LO_PLL_N_COUNTER_H,
  LO_PLL_N_COUNTER_L,
  LO_PLL_LOCK,
  CALLSIGN_0,
  CALLSIGN_1,
  CALLSIGN_2,
  CALLSIGN_3,
  CALLSIGN_4,
  CALLSIGN_5,
  TX_PERIOD_H,
  TX_PERIOD_L,
  LOG_INTERVAL,
  SD_CARD_OK,
  GPS_PPS_OK,
  DEBUG_MODE,
  SEND_PACKET_NOW,
  NUM_FIELDS
}FieldId_t;

typedef struct sd_card_struct {
  FATFS filesys; // fatfs handle
  FRESULT result;
  DWORD free_clusters;
  DWORD free_sectors;
  DWORD total_sectors;
}SdCard_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_NONE        0x00
#define CMD_READ        0x01
#define CMD_READ_BLOCK  0x02
#define CMD_WRITE       0x03
#define CMD_WRITE_BLOCK 0x04
#define CMD_COMMS_TEST  0x05
#define NUM_CMDS        (CMD_COMMS_TEST + 1)
#define CMD_ERROR       0x06

#define START_BYTE  0xA5
#define IND_CMD     1
#define LEN_IND     (IND_CMD + 1)
#define BODY_IND    (LEN_IND + 1)

#define MAX_PACKET_LEN  ((2*NUM_FIELDS) + BODY_IND)
#define UART_STATE_RCV_HDDR   0x00
#define UART_STATE_RCV_BODY   0x01
#define UART_STATE_RCV_ERROR  0x02

#define TX_PERIOD_SECONDS     300

#define PLL_LOCK_DURATION_US  50000

#define STARTUP_DELAY_US      1000000

#define POLL_GPS_INTERVAL     5000

#define TEMPERATURE_INTERVAl  1000000

#define CALLSIGN_LEN          6
#define FILE_CHUNK_SIZE       256
#define DEBUG_LINE_SIZE       128
#define DEBUG_BUF_SIZE        (DEBUG_LINE_SIZE + 20)
#define DEBUG_MODE_NORMAL     0
#define DEBUG_MODE_CONSOLE    1
#define UART_DEBUG_BUF_SIZE   256
#define DEFAULT_MARK_COUNT    2103
#define DEFAULT_SPACE_COUNT   2102

#define FRAME_SIZE_MAX        128
#define SPACE                 0
#define MARK                  1
#define FRAME_DELIM           0x7E

#define APRS_SCRATCH_SIZE     256
#define MAX_APRS_FRAME_COUNT  8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t uart_rx_buf[MAX_PACKET_LEN];
volatile uint8_t packet_lock = 0;
uint8_t packet[MAX_PACKET_LEN];
int uart_rx_ind = 0;
uint8_t uart_rx_state = 0;
uint8_t uart_tx_buf[MAX_PACKET_LEN];
bool update_sys = true;

/* field memory for debug interface */
uint8_t field_mem[NUM_FIELDS];
bool field_mem_writable[NUM_FIELDS];

MCP9808_t tsns_on_board;
MCP9808_t tsns_outside;

ADF4002_t lo_gen_pll;

volatile bool startup = true;

volatile uint64_t microseconds = 0;
volatile uint64_t last_pps_timestamp = 0;
volatile uint64_t pps_duration_us = 0;
volatile uint64_t last_tx_timestamp = 0;
volatile uint64_t begin_lock_timestamp = 0;
volatile uint64_t last_log_timestamp = 0;
volatile uint64_t poll_gps_timestamp = 0;
volatile uint64_t gps_microseconds = 0; // the actual count of microseconds according to GPS
volatile int64_t gps_local_discrepancy = 0;

/* SD CARD THINGS */
SdCard_t sdcard;
bool sd_init_success = false;
FIL config_file;

/* GPS THINGS */
MaxM8C_t gps;
FIL gps_logfile;
bool log_gps = false;
gnss_position_t gps_pos;

/* RADIO THINGS */
int hi_rate_period = 5; // period in seconds for high-rate telemetry
uint8_t tx_frame[FRAME_SIZE_MAX];
int tx_frame_len = 0;
int mark_or_space = 0;
int frame_byte_ind = -1;
int frame_bit_ind = 0;
int seq_ones_count = 0;
bool stuff_bit = false;
uint8_t* aprs_scratch[APRS_SCRATCH_SIZE];
Ax25Frame_t* aprs_frames;
int aprs_frame_count = 0;

/* HOUSEKEEPING THINGS */
const char* debug_fname = "debug.txt";
char debug_buf[DEBUG_BUF_SIZE]; // used by the handy dandy debug function
char debug_line[DEBUG_LINE_SIZE]; // used by user (dats me) to snprintf stuff to
char uart_debug_buf[UART_DEBUG_BUF_SIZE];
char uart_scratch_buf[DEBUG_BUF_SIZE];
volatile int uart_debug_buf_pos = 0;
volatile int uart_debug_tx_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void init_field_mem(uint8_t * mem, bool * writable);
void strstrip(char* str, char* loc);
void init_field_mem_from_config(uint8_t * mem, SdCard_t* sd, FIL* fp);
uint64_t get_microseconds();
bool init_sdcard(SdCard_t* sdcard);
int file_open_read(SdCard_t* sd, const TCHAR* fname, FIL* fp);
int file_open_write(SdCard_t* sd, const TCHAR* fname, FIL* fp);
void file_append_line(SdCard_t* sd, const TCHAR* fname, char* line);
void stop_fsk_output();
void start_fsk_output();
void set_fsk_freq_mark();
void set_fsk_freq_space();
void debug_all(char* line);
void debug(char* line);
void debug_uart(char* line);
void debug_gps(char* line);

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
  MX_HRTIM1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 1);

  // turn off RF amp power supplies
  HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 0);
  HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 0);

  // chip select lines high
  HAL_GPIO_WritePin(PLL_LE_GPIO_Port, PLL_LE_Pin, 0);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 1);

  init_field_mem(field_mem, field_mem_writable);

  lo_gen_pll.hspi = &hspi1;
  lo_gen_pll.ce_pin = PLL_CE_Pin;
  lo_gen_pll.ce_port = PLL_CE_GPIO_Port;
  lo_gen_pll.le_pin = PLL_LE_Pin;
  lo_gen_pll.le_port = PLL_LE_GPIO_Port;
  adf_disable_chip(&lo_gen_pll);
  adf_init_mem(&lo_gen_pll);
  adf_set_n_counter(&lo_gen_pll, 1422, false);
  adf_set_r_counter(&lo_gen_pll, 120, false);
  adf_init_chip(&lo_gen_pll);
  adf_disable_chip(&lo_gen_pll);

  // begin receiveing stuff from Debug port
  HAL_UART_Receive_DMA(&huart3, uart_rx_buf + uart_rx_ind, BODY_IND); 

  // start 10kHz clock
  HAL_TIM_Base_Start_IT(&htim2);
  
  // Init GPS Receiver
  gps.delay_ms = HAL_Delay;
  gps.console_print = debug_uart;
  ublox_init(&gps, &hi2c1, &huart1, GPS_RESETB_GPIO_Port, GPS_RESETB_Pin);
  ublox_hw_reset(&gps);
  HAL_Delay(500);
  ublox_config_i2c(&gps);
  ublox_config_gnss(&gps);
  HAL_Delay(500);
  ublox_enable_inf_messages(&gps);
  ublox_enable_utc_time_messages(&gps);
  ublox_enable_pvt_messages(&gps);
  ublox_enable_nav_messages(&gps);

  /* Temperature sensor things */
  tsns_on_board.hi2c = &hi2c1;
  tsns_on_board.i2c_addr = 0x30; // already left-shifted
  tsns_on_board.online = true;

  tsns_outside.hi2c = &hi2c1;
  tsns_outside.i2c_addr = 0x32; // already left-shifted
  tsns_outside.online = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* BEGIN PROCESSING DEBUG PACKET */
    if(packet_lock > 0){
      // process the packet

      // sanity check
      if(packet[0] == START_BYTE){
        uart_tx_buf[0] = START_BYTE;
        uart_tx_buf[IND_CMD] = packet[IND_CMD];
        switch(packet[IND_CMD]){
          case CMD_NONE:{
            // EZ clap
            break;
          }
          case CMD_READ:{
            // body is a list of indices 
            uint8_t* tx_body = uart_tx_buf + BODY_IND;
            int tx_body_ind = 0;
            for(int i = 0; i < packet[BODY_IND]; i++){
              tx_body[tx_body_ind] = packet[i]; // copy field id
              tx_body_ind++;
              tx_body[tx_body_ind] = field_mem[packet[i]]; // copy actual data
              tx_body_ind++;
            }
            uart_tx_buf[LEN_IND] = tx_body_ind;
            break;
          }
          case CMD_READ_BLOCK:{
            // body is a start index and a length
            uint8_t block_start = packet[BODY_IND];
            uint8_t block_len = packet[BODY_IND + 1];
            if(block_len > NUM_FIELDS - block_start){
              if(NUM_FIELDS - block_start < 0){
                block_len = 0;
              }else{
                block_len = NUM_FIELDS - block_start;
              }
            }

            // copy from field memory
            memcpy(uart_tx_buf + BODY_IND, field_mem + block_start, block_len);

            break;
          }
          case CMD_WRITE:{
            // body is a list of id:value pairs
            uint8_t* rx_body = packet + BODY_IND;
            uint8_t* tx_body = uart_tx_buf + BODY_IND;
            int rx_body_ind = 0;
            int tx_body_ind = 0;
            for(int i = 0; i < packet[LEN_IND]; i++){
              uint8_t field_id = rx_body[i];
              i++;
              if(field_mem_writable[field_id]){
                field_mem[field_id] = rx_body[i];
                tx_body[tx_body_ind] = field_id;
                tx_body_ind++;
              }
            }
            uart_tx_buf[LEN_IND] = tx_body_ind;
            if(tx_body_ind > 0){
              update_sys = true;
            }
            break;
          }
          case CMD_WRITE_BLOCK:{
            // body is a start id + block of data
            uint8_t block_start = packet[BODY_IND];
            uint8_t block_len = packet[LEN_IND] - 1;
            if(block_len > NUM_FIELDS - block_start){
              if(NUM_FIELDS - block_start < 0){
                block_len = 0;
              }else{
                block_len = NUM_FIELDS - block_start;
              }
            }
            uint8_t* block = packet + BODY_IND + 1;
            int field_id = block_start;
            uint8_t updated_len = 0;
            for(int i = 0; i < block_len; i++){
              if(field_mem_writable[field_id]){
                field_mem[field_id] = block[i];
                updated_len++;
              }
              field_id++;
            }
            uart_tx_buf[LEN_IND] = 1;
            uart_tx_buf[BODY_IND] = updated_len;
            if(updated_len > 0){
              update_sys = true;
            }
            break;
          }
          case CMD_COMMS_TEST:{
            uart_tx_buf[LEN_IND] = 4;
            uart_tx_buf[BODY_IND] = packet[BODY_IND];
            uart_tx_buf[BODY_IND+1] = packet[BODY_IND+1];
            uart_tx_buf[BODY_IND+2] = packet[BODY_IND+2];
            uart_tx_buf[BODY_IND+3] = packet[BODY_IND+3];
            // nothing else to do, just send empty packet
          }
        }

        packet_lock = 0; // release lock so we can receive new data

        // transmit
        HAL_UART_Transmit_DMA(&huart3, uart_tx_buf, BODY_IND + uart_tx_buf[LEN_IND]);
      }
      
    }
    /* END PROCESSING DEBUG PACKET */
    
    /* BEGIN CHECKING FOR FIELD MEM UPDATES */
    if(update_sys){
      if(field_mem[MODE] == MODE_MANUAL){
        if(field_mem[LO_GEN_EN]){
          uint16_t r_value = (field_mem[LO_PLL_R_COUNTER_H] << 8) | field_mem[LO_PLL_R_COUNTER_L];
          uint16_t n_value = (field_mem[LO_PLL_N_COUNTER_H] << 8) | field_mem[LO_PLL_N_COUNTER_L];
          adf_set_r_counter(&lo_gen_pll, r_value, true);
          adf_set_n_counter(&lo_gen_pll, n_value, true);
          adf_enable_chip(&lo_gen_pll);
          HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 1); // enable misc RF amps and such
        }else{
          adf_disable_chip(&lo_gen_pll);
          HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 0); // enable misc RF amps and such
        }
        if(field_mem[PA_EN]){
          HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 1); // enable misc RF amps and such
        }else{
          HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 0); // enable misc RF amps and such
        }
        
        if(field_mem[SEND_PACKET_NOW]){
          set_fsk_freq_mark();
          start_fsk_output();
        }else{
          stop_fsk_output();
        }
        
      }
      update_sys = false;
    }
    /* END CHECKING FOR FIELD MEM UPDATES */

    /* BEGIN CHECKING GPS LOCK */
    // check for PPS presence
    if (pps_duration_us == 0 ||
        get_microseconds() > (last_pps_timestamp + pps_duration_us + 2500) ){
      // we don't have GPS signal
      if(field_mem[GPS_PPS_OK]){
        debug_all("GPS PPS not present/lost");
        debug_all("");
      }
      field_mem[GPS_PPS_OK] = false;
      HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 0);
    }else{
      // we have a GPS signal!
      if(!field_mem[GPS_PPS_OK]){
        gps_microseconds = get_microseconds(); // start gps counter in agreement with XO
        debug_all("GPS PPS present");
      }
      field_mem[GPS_PPS_OK] = true;
      HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 1);
    }
    /* END GPS LOCK CHECKING */

    /* BEGIN STARTUP DELAY CHECKING */
    if(startup && get_microseconds() > STARTUP_DELAY_US){
      if(HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin) == 0){
        // an SD card is inserted
        // attempt to initialize the SD card
        debug_all("Attempting to intialize SD card...");
        field_mem[SD_CARD_OK] = init_sdcard(&sdcard);
      }
      if(field_mem[SD_CARD_OK]){
        // read config parameters from sd card
        if(file_open_read(&sdcard, "config.txt", &config_file) == 0){
          debug_all("Initializing fields from SD card...");
          init_field_mem_from_config(field_mem, &sdcard, &config_file);
          update_sys = true;
          f_close(&config_file);
        }

        // attempt to open GPS log file
        if((log_gps = (file_open_write(&sdcard, UBLOX_LOG_FILENAME, &gps_logfile) == 0))){
          f_close(&gps_logfile); // close until we need it
        }
      }else{
        debug_all("SD card init failed...");
      }
      startup = false;
    }
    /* END STARTUP DELAY CHECKING */


    /* BEGIN GPS DATA GRAB AND LOGGING */
    if(get_microseconds() - poll_gps_timestamp > POLL_GPS_INTERVAL){
      // debug_all("Polling GPS...");
      poll_gps_timestamp = get_microseconds();
      int avail = ublox_bytes_available_i2c(&gps);
      if(avail > 0){
        // snprintf(debug_line, DEBUG_LINE_SIZE, "GPS bytes available: %d", avail);
        // debug_all(debug_line);
        // HAL_Delay(10);
        ublox_read_all_data_i2c(&gps, 0);
        ublox_parse_messages(&gps);
        ublox_print_all_messages(&gps, debug_all); //debug_line);
        // // debug_all(debug_line);

        // TODO do real things with the GPS messages 
        for(int i = 0; i < gps.msg_count; i++){
          ubx_msg_t* msg = gps.msgs + i;
          if(msg->class == UBX_NAV_CLASS){
            
            if(msg->id == 0x21){
              // UBX-NAV-TIMEUTC
              gnss_timestamp_t* ts = &(gps.ts);
              ublox_get_gnss_timestamp(msg->payload, ts);
              // TODO packetize and all that
            }else if(msg->id == 0x14){
              // UBX-NAV-HPPOSLLH
              ublox_parse_hpposllh(msg, &gps_pos);
              
            }
          }
        }

        ublox_cleanup_messages(&gps);

        ublox_poll_message_i2c(&gps, UBX_NAV_CLASS, 0x35); // poll satellite information
        HAL_Delay(10);

        ublox_read_all_data_i2c(&gps, 0);
        ublox_parse_messages(&gps);
        ublox_print_all_messages(&gps, debug_all); //debug_line);
        ublox_cleanup_messages(&gps);
      }else if(avail < 0){
        // debug_all("GPS likely offline.");
      }
      
      
    }

    // write generic data to the log file
    if(get_microseconds() - last_log_timestamp > (field_mem[LOG_INTERVAL] * 1000000)){
      last_log_timestamp = get_microseconds();
    }
    /* END GPS DATA GRAB AND LOGGING */

    /* BEGIN TEMPERATURE AND BAROMETRIC PRESSURE DATA GRAB */
    if( (tsns_on_board.online && get_microseconds() - tsns_on_board.update_ts > TEMPERATURE_INTERVAl) || 
        (!tsns_on_board.online && get_microseconds() - tsns_on_board.update_ts > 10*TEMPERATURE_INTERVAl)
    ){
      mcp9808_get_temperature(&tsns_on_board);
      tsns_on_board.update_ts = get_microseconds();
      if(!tsns_on_board.online){
        debug_all("On-board temp sensor offline :(");
      }else{
        // snprintf(debug_line, DEBUG_LINE_SIZE, "(T-ONBOARD) %.2f°C", tsns_on_board.temp);
        // debug_all(debug_line);
      }
    }
    if( (tsns_outside.online && get_microseconds() - tsns_outside.update_ts > TEMPERATURE_INTERVAl) || 
        (!tsns_outside.online && get_microseconds() - tsns_outside.update_ts > 10*TEMPERATURE_INTERVAl)
    ){
      mcp9808_get_temperature(&tsns_outside);
      tsns_outside.update_ts = get_microseconds();
      if(!tsns_outside.online){
        debug_all("Outside temp sensor offline :(");
      }else{
        // snprintf(debug_line, DEBUG_LINE_SIZE, "(T-OUTSIDE) %.2f°C", tsns_outside.temp);
        // debug_all(debug_line);
      }
    }
    
    /* END TEMPERATURE AND BAROMETRIC PRESSURE DATA GRAB */

    /* BEGIN FORMING APRS PACKETS */
    uint64_t elapsed_us = get_microseconds() - last_tx_timestamp;
    uint16_t tx_period = (field_mem[TX_PERIOD_H] << 8) | field_mem[TX_PERIOD_L];
    if ((elapsed_us / 1000000) >= tx_period){
      // 
      debug_all("Forming APRS packet...");
      int offset = 0;
      /* LAT/LONG POSITION REPORT */
      /* with timestamp, with APRS messaging, zulu time, with comment */
      offset += snprintf(aprs_scratch + offset, APRS_SCRATCH_SIZE - offset, "%c", APRS_POS_WITH_TS_MSG);

      /* append UTC timestamp (zulu time) */
      offset += snprintf(aprs_scratch + offset, APRS_SCRATCH_SIZE - offset, "%02d%02d%02dz", gps.ts.day, gps.ts.hour, gps.ts.minute);

      /* append latitude */
      int lat_degrees = (int)gps_pos.lat; 
      float lat_minutes = (gps_pos.lat - 1.0*lat_degrees)*60.0; // 60 minutes per degree
      char n_or_s = (lat_degrees > 0 ? 'N' : 'S');
      char symbol_table_id = '/'; // primary symbol table
      offset += snprintf(aprs_scratch + offset, APRS_SCRATCH_SIZE - offset, "%02d%05.2f%c%c", lat_degrees, lat_minutes, n_or_s, symbol_table_id);

      /* append longitude */
      int lon_degrees = (int)gps_pos.lon; 
      float gps_lon_dec = gps_pos.lon;
      char w_or_e = 'E';
      if(lon_degrees < 0){
        lon_degrees = 0 - lon_degrees;
        gps_lon_dec = 0.0 - gps_lon_dec;
        w_or_e = 'W';
      }
      float lon_minutes = (gps_lon_dec - 1.0*lon_degrees)*60.0; // 60 minutes per degree
      char symbol_code = 'O'; // balloon
      offset += snprintf(aprs_scratch + offset, APRS_SCRATCH_SIZE - offset, "%02d%05.2f%c%c", lon_degrees, lon_minutes, w_or_e, symbol_code);

      /* append comment */
      int alt_ft = (int)(gps_pos.altitude * 3.2808);
      char* comment = "Test";
      offset += snprintf(aprs_scratch + offset, APRS_SCRATCH_SIZE - offset, "%s /A=%06d", comment, alt_ft);

      debug_all(aprs_scratch);
    }
    /* END FORMING APRS PACKETS */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

void init_field_mem(uint8_t * mem, bool* writable){
  memset(writable, 0x00, NUM_FIELDS);
  memset(mem, 0x00, NUM_FIELDS);
  mem[MODE] = MODE_MANUAL;
  writable[MODE] = true;

  mem[RADIO_STATE] = STATE_GOTO_IDLE;
  writable[RADIO_STATE] = true; // may want to change this later

  mem[PA_EN] = 0;
  writable[PA_EN] = true;

  mem[LO_GEN_EN] = 0;
  writable[LO_GEN_EN] = true;

  mem[LO_PLL_R_COUNTER_H] = 0;
  writable[LO_PLL_R_COUNTER_H] = true;
  mem[LO_PLL_R_COUNTER_L] = 120;
  writable[LO_PLL_R_COUNTER_L] = true;

  mem[LO_PLL_N_COUNTER_H] = (1422 >> 8) & 0xFF;
  writable[LO_PLL_N_COUNTER_H] = true;
  mem[LO_PLL_N_COUNTER_L] = (1422 & 0xFF);
  writable[LO_PLL_N_COUNTER_L] = true;
  
  mem[CALLSIGN_0] = 'K';
  writable[CALLSIGN_0] = true;
  mem[CALLSIGN_1] = 'E';
  writable[CALLSIGN_1] = true;
  mem[CALLSIGN_2] = '8';
  writable[CALLSIGN_2] = true;
  mem[CALLSIGN_3] = 'I';
  writable[CALLSIGN_3] = true;
  mem[CALLSIGN_4] = 'T';
  writable[CALLSIGN_4] = true;
  mem[CALLSIGN_5] = 'F';
  writable[CALLSIGN_5] = true;

  mem[LOG_INTERVAL] = 5;
  writable[LOG_INTERVAL] = true;

  mem[TX_PERIOD_H] = (TX_PERIOD_SECONDS >> 8) & 0xFF;
  writable[TX_PERIOD_H] = true;
  mem[TX_PERIOD_L] = TX_PERIOD_SECONDS & 0xFF;
  writable[TX_PERIOD_L] = true;

  mem[SD_CARD_OK] = false; // default to this
  mem[GPS_PPS_OK] = false; // default

  mem[DEBUG_MODE] = DEBUG_MODE_NORMAL;
  writable[DEBUG_MODE] = true;

  mem[SEND_PACKET_NOW] = 0;
  writable[SEND_PACKET_NOW] = true;

  // populate the tx_frame with our callsign
  tx_frame_len = 7;
  for(int i = 0; i < tx_frame_len; i++){
    tx_frame[i] = mem[CALLSIGN_0 + i];
  }
  tx_frame[6] = ' ';
} 


/**
 * strip whitespace from str storing result in loc
 * loc must have at least as much space as str
*/
void strstrip(char* str, char* loc){
  int i = 0;
  while(str[i] == ' ' || str[i] == '\n' || str[i] == '\r' || str[i] == '\t'){
    i++;
  }
  strcpy(loc, str + i); // copy over everything except leading whitespace

  i = strlen(loc) - 1; // get index of last character
  while(i >= 0 && (loc[i] == ' ' || loc[i] == '\n' || loc[i] == '\r' || loc[i] == '\t')){
    loc[i] = '\0'; // set this whitespace to null termination
    i--;
  }
}


/**
 * Initialize fields from a file on the SD card
 * Assumes fp represents an open file
*/
void init_field_mem_from_config(uint8_t * mem, SdCard_t* sd, FIL* fp){
  BYTE readbuf[FILE_CHUNK_SIZE];
  UINT bytes_read;
  char* token;

  do{
    sd->result = f_read(fp, readbuf, FILE_CHUNK_SIZE, &bytes_read);
    if(sd->result != FR_OK){
      // TODO indicate failure
      break;
    }
    
    BYTE* word = (BYTE*)malloc(bytes_read); // likely more space than we need
    token = strtok(readbuf, "="); // get first KEY
    while(token != NULL){
      strstrip(token, word); // word now holds the KEY
      
      // get VALUE for this key
      token = strtok(NULL, "\n");

      if(strcmp(word, "MODE") == 0){
        // radio mode
        strstrip(token, word);
        if(strcmp(word, "AUTO") == 0){
          mem[MODE] = MODE_AUTO;
        }else if(strcmp(word, "TEST") == 0){
          mem[MODE] = MODE_TEST;
        }else{
          // for any other crap, just default to manual
          mem[MODE] = MODE_MANUAL;
        }
      }else if(strcmp(word, "CALLSIGN") == 0){
        strstrip(token, word);
        // copy over callsign
        memcpy(mem + CALLSIGN_0, word, CALLSIGN_LEN);
      }else if(strcmp(word, "TX_PERIOD") == 0){
        strstrip(token, word);
        int period = atoi(word);
        mem[TX_PERIOD_H] = (period >> 8) & 0xFF;
        mem[TX_PERIOD_L] = period & 0xFF;
      }else if(strcmp(word, "LOG_INTERVAL") == 0){
        strstrip(token, word);
        mem[LOG_INTERVAL] = atoi(word);
      }else if(strcmp(word, "DEBUG_MODE") == 0){
        strstrip(token, word);
        if(strcmp(word, "CONSOLE") == 0){
          mem[DEBUG_MODE] = DEBUG_MODE_CONSOLE;
        }else{
          mem[DEBUG_MODE] = DEBUG_MODE_NORMAL;
        }
      }else if(strcmp(word, "LO_N_DIV") == 0){
        strstrip(token, word);
        int n_div = atoi(word);
        mem[LO_PLL_N_COUNTER_H] = (n_div >> 8) & 0xFF;
        mem[LO_PLL_N_COUNTER_L] = n_div & 0xFF;
      }else if(strcmp(word, "MARK_COUNT") == 0){
        strstrip(token, word);
        int count = atoi(word);
        // mem[LO_PLL_N_COUNTER_H] = (n_div >> 8) & 0xFF;
        // mem[LO_PLL_N_COUNTER_L] = n_div & 0xFF;
      }else if(strcmp(word, "SAMPLE_PACKET") == 0){
        strstrip(token, word);
        memcpy(tx_frame + 6, word, strlen(word));
        tx_frame_len = 6 + strlen(word);
      }else if(strcmp(word, "LAT") == 0){
        strstrip(token, word);
        gps_pos.lat = atof(word);
      }else if(strcmp(word, "LON") == 0){
        strstrip(token, word);
        gps_pos.lon = atof(word);
      }else if(strcmp(word, "ALT") == 0){
        strstrip(token, word);
        gps_pos.alt = atof(word);
      }else if(strcmp(word, "HI_RATE_PER") == 0){
        strstrip(token, word);
        hi_rate_period = atoi(word);
      }
      // get next KEY
      token = strtok(NULL, "=");
    }
    free(word);
  }while(bytes_read == FILE_CHUNK_SIZE);
  // TODO handle files bigger than FILE_CHUNK_SIZE

}


/**
 * Get microseconds since startup
*/
uint64_t get_microseconds(){
  return microseconds;
}



	// FR_OK = 0,				/* (0) Succeeded */
	// FR_DISK_ERR,			/* (1) A hard error occurred in the low level disk I/O layer */
	// FR_INT_ERR,				/* (2) Assertion failed */
	// FR_NOT_READY,			/* (3) The physical drive cannot work */
	// FR_NO_FILE,				/* (4) Could not find the file */
	// FR_NO_PATH,				/* (5) Could not find the path */
	// FR_INVALID_NAME,		/* (6) The path name format is invalid */
	// FR_DENIED,				/* (7) Access denied due to prohibited access or directory full */
	// FR_EXIST,				/* (8) Access denied due to prohibited access */
	// FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */
	// FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */
	// FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */
	// FR_NOT_ENABLED,			/* (12) The volume has no work area */
	// FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume */
	// FR_MKFS_ABORTED,		/* (14) The f_mkfs() aborted due to any parameter error */
	// FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */
	// FR_LOCKED,				/* (16) The operation is rejected according to the file sharing policy */
	// FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */
	// FR_TOO_MANY_OPEN_FILES,	/* (18) Number of open files > _FS_SHARE */
	// FR_INVALID_PARAMETER	/* (19) Given parameter is invalid */
void sdcard_print_error(SdCard_t* sd){
  int offset = snprintf(debug_line, DEBUG_LINE_SIZE, "SD Card Error: ");
  if(sd->result == FR_INVALID_DRIVE){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_INVALID_DRIVE");
  }else if(sd->result == FR_NOT_ENABLED){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_NOT_ENABLED");
  }else if(sd->result == FR_WRITE_PROTECTED){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_WRITE_PROTECTED");
  }else if(sd->result == FR_DISK_ERR){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_DISK_ERR");
  }else if(sd->result == FR_NO_FILESYSTEM){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_NO_FILESYSTEM");
  }else if(sd->result == FR_INT_ERR){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_INT_ERR");
  }else if(sd->result == FR_NOT_READY){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_NOT_READY");
  }else if(sd->result == FR_DENIED){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_DENIED");
  }else if(sd->result == FR_LOCKED){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_LOCKED");
  }else if(sd->result == FR_INVALID_NAME){
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "FR_INVALID_NAME");
  }else{
    snprintf(debug_line + offset, DEBUG_LINE_SIZE - offset, "Unknown error: %d", sd->result);
  }
  
  debug_uart(debug_line);
}


/**
 * Initialize SD card, including getting statistics
 * Return true if there is an SD card attached
*/
bool init_sdcard(SdCard_t* sd){

  char* path = {'\0','\0','\0','\0'};

  sd->result = f_mount(&(sd->filesys), "", 1);
  if (sd->result != FR_OK){
    sdcard_print_error(sd);
    return false;
  }

  FATFS* get_free_fs;
  sd->result = f_getfree("", &(sd->free_clusters), &get_free_fs);
  if (sd->result != FR_OK){
    debug_uart("SD Card getfree failed.");
    return false;
  }

  sd->total_sectors = (get_free_fs->n_fatent - 2) * get_free_fs->csize;
  sd->free_sectors = sd->free_clusters * get_free_fs->csize;

  return true;
}


/**
 * Opens file fname for reading (if it exists)
 * Returns -1 if file doesn't exist.
 * Returns index of file handle if file exists
*/
int file_open_read(SdCard_t* sd, const TCHAR* fname, FIL* fp){
  int retval = -1;
  sd->result = f_open(fp, fname, FA_READ);
  if(sd->result != FR_OK){
    // TODO set some error in field mem
    return retval;
  }
  // else, success
  return 0;
}


/**
 * Attempts to open a file for writing, creating file if
 * it doesn't exist.
 * Returns -1 of failure. 0 on success
*/
int file_open_write(SdCard_t* sd, const TCHAR* fname, FIL* fp){
  sd->result = f_open(fp, fname, FA_WRITE | FA_OPEN_ALWAYS);// | FA_CREATE_ALWAYS);
  if(sd->result != FR_OK){
    sd->result = f_open(fp, fname, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  }
  
  if(sd->result != FR_OK){
    return -1;
  }
  
  f_lseek(fp, fp->fsize);
  return 0;
}


/**
 * Opens file of name fname and appends line, writing a trailing newline
 * character if one is not included in line.
 * Closes file after operation is done.
*/
void file_append_line(SdCard_t* sd, const TCHAR* fname, char* line){
  FIL ftmp;
  if(file_open_write(sd, fname, &ftmp) != 0){
    return; // fail
  }
  UINT bytes_wrote;
  UINT len = strlen(line);
  sd->result = f_write(&ftmp, (BYTE*)line, len, &bytes_wrote);

  if(line[len - 1] != '\n'){
    BYTE tmp_byte = '\n';
    sd->result = f_write(&ftmp, &tmp_byte, 1, &bytes_wrote);
  }
  f_close(&ftmp);
}


void start_fsk_output(){
  HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_TIMER_C);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TC1);
}

void stop_fsk_output(){
  HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1);
  HAL_HRTIM_WaveformCountStop(&hhrtim1, HRTIM_TIMERID_TIMER_C);
}

void set_fsk_freq_mark(){
  hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = DEFAULT_MARK_COUNT >> 1;
  hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].PERxR = DEFAULT_MARK_COUNT;
}


void set_fsk_freq_space(){
  hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = DEFAULT_SPACE_COUNT >> 1;
  hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].PERxR = DEFAULT_SPACE_COUNT;
}


/**
 * Sends debug info to SD card and UART
*/
void debug_all(char* line){
  debug(line);
  debug_uart(line);
  // line[0] = '\0';
}


/**
 * Handy-dandy function for writing a timestamped debug line to a file
 * using the global sdcard object.
*/
void debug(char* line){
  if(field_mem[SD_CARD_OK]){
    double t_stamp_sec = (double)get_microseconds() / 1000000.0;
    snprintf(debug_buf, DEBUG_BUF_SIZE, "[%.3f]: %s\n", t_stamp_sec, line);
    file_append_line(&sdcard, debug_fname, debug_buf);
  }
}


/**
 * handy dandy function for writing a timestamped debug line over the UART3
 * hardware uart port. Send a NULL line to flush the uart debug buffer.
*/
void debug_uart(char* line){
  if(field_mem[DEBUG_MODE] == DEBUG_MODE_CONSOLE){

    if(line != NULL){
      double t_stamp_sec = (double)get_microseconds() / 1000000.0;
      int add_size = snprintf(uart_scratch_buf, DEBUG_BUF_SIZE, "[%.3f]: %s\r\n", t_stamp_sec, line);

      // tack on this line to our debug buffer, if it will fit :)
      if(uart_debug_buf_pos + add_size < UART_DEBUG_BUF_SIZE){
        strcpy(uart_debug_buf + uart_debug_buf_pos, uart_scratch_buf);
        uart_debug_buf_pos += add_size;
      }
    }

    // can we transmit now? If so, do it starting from position 0.
    if(huart3.gState == HAL_UART_STATE_READY){
      uart_debug_tx_len = uart_debug_buf_pos;
      HAL_UART_Transmit_DMA(&huart3, uart_debug_buf, uart_debug_tx_len);
    }
  }
}


/**
 * Writes a timestamped string to both the GPS logfile, and the UART
 * debug stream.
*/
void debug_gps(char* line){
  debug_uart(line);
  
  double t_stamp_sec = (double)get_microseconds() / 1000000.0;
  snprintf(debug_buf, DEBUG_BUF_SIZE, "[%12.3f]: %s\n", t_stamp_sec, line);
  file_append_line(&sdcard, UBLOX_LOG_FILENAME, debug_buf);
}


/**
 *
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART3){
    if(field_mem[DEBUG_MODE] == DEBUG_MODE_CONSOLE){
      if(uart_debug_buf_pos == uart_debug_tx_len){
        // user has added nothing since transmit was triggered
        uart_debug_buf_pos = 0;
        uart_debug_tx_len = 0;
      }else if(uart_debug_buf_pos > uart_debug_tx_len){
        // user has appended stuff later in the buffer since last tx.
        // So we need to start at uart_debug_tx_len and transmit up to
        // uart_debug_buf_pos. Then it's safe to reset uart_debug_buf_pos = 0
        int len = uart_debug_buf_pos - uart_debug_tx_len;
        int offset = uart_debug_tx_len;
        uart_debug_tx_len = uart_debug_buf_pos;
        HAL_UART_Transmit_DMA(huart, uart_debug_buf + offset, len);
      }else{
        // uhhh wut
      }
    }
  }
}


/**
 * Callback for handling completion of a UART receive operation
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == huart3.Instance){
    if(uart_rx_ind < BODY_IND){
      // we've just received the header, probably
      if(uart_rx_buf[0] == START_BYTE && uart_rx_buf[IND_CMD] < NUM_CMDS){
        uart_rx_ind += BODY_IND;

        int len = uart_rx_buf[LEN_IND];

        if(len <= MAX_PACKET_LEN - BODY_IND){
          HAL_UART_Receive_DMA(&huart3, uart_rx_buf + uart_rx_ind, len);
        }else{
          // error handling
        }
        
      }else{
        // error handling
      }
    }else{
      // we must've just received the body
      // if packet_lock is 0, copy over the new data and set packet_lock
      if(packet_lock == 0){
        memcpy(packet, uart_rx_buf, BODY_IND + uart_rx_buf[LEN_IND]);
        packet_lock = 1;
      }
      uart_rx_ind = 0;
      HAL_UART_Receive_DMA(&huart3, uart_rx_buf + uart_rx_ind, BODY_IND);
    }
  }
}


/**
 * callback for TIMER interrupts
 * This is where we will run the main state machine
 * This is also where we'll walk through the data to transmit
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == htim2.Instance){
    // 10kHz housekeeping clock
    microseconds += 100;

    if(field_mem[MODE] == MODE_MANUAL){
      return;
    }
    
    /* BEGIN MAIN STATE MACHINE */
    switch(field_mem[RADIO_STATE]){
      case STATE_GOTO_IDLE:{
        HAL_TIM_Base_Stop(&htim1); // stop the baud clock
        stop_fsk_output();
        // HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1); // stop the IF signal
        HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 0);
        HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 0);
        field_mem[RADIO_STATE] = STATE_IDLE;
        break;
      }
      case STATE_IDLE:{
        // what to do, what to do...
        uint64_t elapsed_us = get_microseconds() - last_tx_timestamp;
        uint16_t tx_period = (field_mem[TX_PERIOD_H] << 8) | field_mem[TX_PERIOD_L];
        if ((elapsed_us / 1000000) >= tx_period && tx_frame_len > 0){
          // get the ball rolling on transmitting
          field_mem[RADIO_STATE] = STATE_POWER_ON_LO;
        }
        break;
      }
      case STATE_POWER_ON_LO:{
        adf_enable_chip(&lo_gen_pll);
        HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 1); // enable misc RF amps and such
        begin_lock_timestamp = get_microseconds();
        field_mem[RADIO_STATE] = STATE_WAIT_FOR_LO_LOCK;
        break;
      }
      case STATE_WAIT_FOR_LO_LOCK:{
        if(get_microseconds() - begin_lock_timestamp >= PLL_LOCK_DURATION_US){
          field_mem[RADIO_STATE] = STATE_POWER_ON_PA;
        }
        break;
      }
      case STATE_POWER_ON_PA:{
        HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 1);
        begin_lock_timestamp = get_microseconds(); // reuse this timestamp variable
        field_mem[RADIO_STATE] = STATE_WAIT_FOR_PA;
        break;
      }
      case STATE_WAIT_FOR_PA:{
        if(get_microseconds() - begin_lock_timestamp >= 2500){
          // TODO: initialize things for the transmit data phase
          mark_or_space = SPACE;
          frame_byte_ind = -1;
          frame_bit_ind = 0;
          stuff_bit = false;
          seq_ones_count = 0;
          HAL_TIM_Base_Start_IT(&htim1);
          start_fsk_output();
          set_fsk_freq_space();
          last_tx_timestamp = get_microseconds();
          field_mem[RADIO_STATE] = STATE_TRANSMIT;
        }
        break;
      }
      case STATE_TRANSMIT:{
        // if TX done --> field_mem[RADIO_STATE] = STATE_GOTO_IDLE
        if(frame_byte_ind >= tx_frame_len){
           if(field_mem[MODE] == MODE_TEST){
            field_mem[RADIO_STATE] = STATE_WAIT_FOR_PA; // loop back to transmit immediately
          }else{
            field_mem[RADIO_STATE] = STATE_GOTO_IDLE; 
          }
          
        }
        break;
      }

    }
    /* END MAIN STATE MACHINE */

  }else if(htim->Instance == htim1.Instance){

    // baud clock (1.2kHz)
    if(field_mem[RADIO_STATE] == STATE_TRANSMIT){
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 1);
      // advance through the data

      bool change_state = false;

      if(frame_byte_ind < -3){
        set_fsk_freq_mark();
      }
      else if(frame_byte_ind < 0){
        // SEND FRAME DELIMITER
        uint8_t bit = (FRAME_DELIM >> frame_bit_ind) & 1;
        // change_state = !bit; // change state if bit == 0
        if (bit){
          set_fsk_freq_mark();
        }else{
          set_fsk_freq_space();
        }
      }else{
        // SEND FRAME DATA
        
        // determine if we've seen 5 consecutive 1's
        // if yes, insert a 0 here (stuff bit)
        stuff_bit = (seq_ones_count == 5); 
        
        if (!stuff_bit){
          uint8_t bit = (tx_frame[frame_byte_ind] >> frame_bit_ind) & 1;
          if (bit){
            // DON'T change fsk freq, but increment count of sequential ones
            set_fsk_freq_mark();
            seq_ones_count++;
          }else{
            set_fsk_freq_space();
            seq_ones_count = 0;
            change_state = true; // change fsk freq
          }
        }else{
          seq_ones_count = 0;
          change_state = true;
          set_fsk_freq_space();
        }
      }
      
      // if(change_state){
      //   mark_or_space = mark_or_space ^ 1; // toggle this
      //   if(mark_or_space == MARK){
      //     set_fsk_freq_mark();
      //   }else{
      //     set_fsk_freq_space();
      //   }
      // }

      if(!stuff_bit){
        frame_bit_ind++;
        if(frame_bit_ind > 7){
          frame_bit_ind = 0;
          frame_byte_ind++;
        }
      }
       HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);
    }
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPS_PPS_Pin){
    HAL_GPIO_TogglePin(LED_0_GPIO_Port, LED_0_Pin);
    /* get timestamp accurate to the ~microsecond (really +/-1us) */
    uint64_t add_us = (__HAL_TIM_GET_COUNTER(&htim2))/72;
    uint64_t stamp = get_microseconds() + add_us;
    // stamp is now the "extra-precise" XO time (what time we think it is)

    if(last_pps_timestamp > 0){
      pps_duration_us = stamp - last_pps_timestamp;
      gps_microseconds += 1000000; 
      // gps_microseconds is now the time it *actually* is according to GPS
    }else{
      // initial setting of these to something reasonable
      pps_duration_us = 1000000;
      gps_microseconds = stamp;
    }
    gps_local_discrepancy = gps_microseconds - stamp;
    if(gps_local_discrepancy > 2 || gps_local_discrepancy < -2){
      // if discrepancy is large enough to be real, correct it
      microseconds += gps_local_discrepancy;
    }
    last_pps_timestamp = stamp;
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
