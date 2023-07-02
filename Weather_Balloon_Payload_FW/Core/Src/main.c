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
#include "ublox.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_FILE_COUNT 16

typedef enum radio_mode_enum {
  MODE_MANUAL,
  MODE_AUTO
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
  SD_CARD_OK,
  GPS_PPS_OK,
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

#define PLL_LOCK_DURATION_US  10000

#define STARTUP_DELAY_US      50000

#define CALLSIGN_LEN          6
#define FILE_CHUNK_SIZE       128
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
bool update_sys = false;

/* field memory for debug interface */
uint8_t field_mem[NUM_FIELDS];
bool field_mem_writable[NUM_FIELDS];

ADF4002_t lo_gen_pll;

volatile bool startup = true;

volatile uint8_t radio_state = STATE_GOTO_IDLE;

volatile uint64_t microseconds = 0;

volatile uint64_t last_pps_timestamp = 0;

volatile uint64_t pps_duration_us = 0;

volatile uint64_t last_tx_timestamp = 0;

volatile uint64_t begin_lock_timestamp = 0;

/* SD CARD THINGS */
SdCard_t sdcard;
bool sd_init_success = false;
FIL config_file;

/* GPS THINGS */
MaxM8C_t gps;
FIL gps_logfile;
bool log_gps = false;

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
  adf_set_n_counter(&lo_gen_pll, 1412, false);
  adf_set_r_counter(&lo_gen_pll, 120, false);
  adf_init_chip(&lo_gen_pll);
  adf_disable_chip(&lo_gen_pll);

  // begin receiveing stuff from Debug port
  HAL_UART_Receive_DMA(&huart3, uart_rx_buf + uart_rx_ind, BODY_IND); 

  // start 10kHz clock
  HAL_TIM_Base_Start_IT(&htim2);
  
  // Init GPS Receiver
  gps.delay_ms = HAL_Delay;
  ublox_init(&gps, &hi2c1, &huart1);


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
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, BODY_IND + uart_tx_buf[LEN_IND]);
      }
      
    }
    /* END PROCESSING DEBUG PACKET */

    /* BEGIN CHECKING GPS LOCK */
    // check for PPS presence
    if (pps_duration_us == 0 ||
        get_microseconds() > (last_pps_timestamp + pps_duration_us + 1000) ){
      // we don't have GPS signal
      field_mem[GPS_PPS_OK] = false;
      HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 0);
    }else{
      // we have a GPS signal!
      field_mem[GPS_PPS_OK] = true;
      HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 1);
    }
    /* END GPS LOCK CHECKING */

    /* BEGIN STARTUP DELAY CHECKING */
    if(startup && get_microseconds() > STARTUP_DELAY_US){
      if(HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin) == 0){
        // an SD card is inserted
        // attempt to initialize the SD card
        field_mem[SD_CARD_OK] = init_sdcard(&sdcard);
      }
      if(field_mem[SD_CARD_OK]){
        // read config parameters from sd card
        if(file_open_read(&sdcard, "config.txt", &config_file) == 0){
          init_field_mem_from_config(field_mem, &sdcard, &config_file);
          f_close(&config_file);
        }

        // attempt to open GPS log file
        if((log_gps = (file_open_write(&sdcard, UBLOX_LOG_FILENAME, &gps_logfile) == 0))){
          f_close(&gps_logfile); // close until we need it
        }
      }
      startup = false;
    }
    /* END STARTUP DELAY CHECKING */

    /* BEGIN GPS DATA GRAB AND LOGGING */
    // TODO do GPS things
    /* END GPS DATA GRAB AND LOGGING */

  
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

  mem[LO_PLL_N_COUNTER_H] = (1412 >> 8) & 0xFF;
  writable[LO_PLL_N_COUNTER_H] = true;
  mem[LO_PLL_N_COUNTER_L] = (1412 & 0xFF);
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

  mem[TX_PERIOD_H] = (TX_PERIOD_SECONDS >> 8) & 0xFF;
  writable[TX_PERIOD_L] = TX_PERIOD_SECONDS & 0xFF;

  mem[SD_CARD_OK] = false; // default to this
  mem[GPS_PPS_OK] = false; // default
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

  int i = strlen(loc) - 1; // get index of last character
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
    token = strtok(readbuf, '='); // get first KEY
    while(token != NULL){
      strstrip(token, word); 
      
      // get VALUE for this key
      token = strtok(NULL, '\n');

      if(strcmp(word, "MODE") == 0){
        // radio mode
        strstrip(token, word);
        if(strcmp(word, "AUTO") == 0){
          mem[MODE] = MODE_AUTO;
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
      }
      // get next KEY
      token = strtok(NULL, '=');
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

/**
 * Initialize SD card, including getting statistics
 * Return true if there is an SD card attached
*/
bool init_sdcard(SdCard_t* sd){

  FATFS* get_free_fs;
  sd->result = f_getfree("", &(sd->free_clusters), &get_free_fs);
  if (sd->result != FR_OK){
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
  sd->result = f_open(fp, fname, FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if(sd->result != FR_OK){
    return -1;
  }
  return 0;
}


/**
 * Callback for handling completion of a UART receive operation
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(uart_rx_ind < BODY_IND){
    // we've just received the header, probably
    if(uart_rx_buf[0] == START_BYTE && uart_rx_buf[IND_CMD] < NUM_CMDS){
      uart_rx_ind += BODY_IND;

      int len = uart_rx_buf[LEN_IND];

      if(len <= MAX_PACKET_LEN - BODY_IND){
        HAL_UART_Receive_DMA(&huart1, uart_rx_buf + uart_rx_ind, len);
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
    HAL_UART_Receive_DMA(&huart1, uart_rx_buf + uart_rx_ind, BODY_IND);
  }
}

/**
 * callback for TIMER interrupts
 * This is where we will run the main state machine
 * This is also where we'll walk through the data to transmit
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM2){
    // 10kHz housekeeping clock
    microseconds += 100;

    if(field_mem[MODE] == MODE_MANUAL){
      return;
    }
    /* BEGIN MAIN STATE MACHINE */
    switch(field_mem[RADIO_STATE]){
      case STATE_GOTO_IDLE:{
        HAL_TIM_Base_Stop(&htim1); // stop the baud clock
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TC1); // stop the IF signal
        HAL_GPIO_WritePin(PA_EN_GPIO_Port, PA_EN_Pin, 0);
        HAL_GPIO_WritePin(EN_5P0_GPIO_Port, EN_5P0_Pin, 0);
        field_mem[RADIO_STATE] = STATE_IDLE;
        break;
      }
      case STATE_IDLE:{
        // what to do, what to do...
        uint64_t elapsed_us = get_microseconds() - last_tx_timestamp;
        uint16_t tx_period = (field_mem[TX_PERIOD_H] << 8) | field_mem[TX_PERIOD_L];
        if ((elapsed_us / 1000000) >= tx_period){
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
        begin_lock_timestamp = get_microseconds();
        field_mem[RADIO_STATE] = STATE_WAIT_FOR_PA;
        break;
      }
      case STATE_WAIT_FOR_PA:{
        if(get_microseconds() - begin_lock_timestamp >= 1000){
          // TODO: initialize things for the transmit data phase
          field_mem[RADIO_STATE] = STATE_TRANSMIT;
        }
        break;
      }
      case STATE_TRANSMIT:{
        // if TX done --> field_mem[RADIO_STATE] = STATE_GOTO_IDLE
        break;
      }

    }
    /* END MAIN STATE MACHINE */

  }else if(htim->Instance == TIM1){
    // baud clock (1.2kHz)
    if(field_mem[RADIO_STATE] == STATE_TRANSMIT){
      // advance through the data
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPS_PPS_Pin){
    uint64_t stamp = get_microseconds();
    pps_duration_us = stamp - last_pps_timestamp;
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
