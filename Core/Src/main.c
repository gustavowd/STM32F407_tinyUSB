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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp/board.h"
#include "tusb.h"
#include "microphone.h"
#include "FreeRTOS_CLI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USBD_STACK_SIZE    (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)
#define CDC_STACK_SZIE      3*configMINIMAL_STACK_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
void microphoneTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

TimerHandle_t blinky_tm;
TimerHandle_t green_led_blinky_tm;
TimerHandle_t button_tm;

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void* param)
{
  (void) param;

  // init device stack on configured roothub port
  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tud_init(BOARD_TUD_RHPORT);

  // RTOS forever loop
  while (1)
  {
    // put this thread to waiting state until there is new events
    tud_task();

    // following code only run if tud_task() process at least 1 event
    tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}


SemaphoreHandle_t cdc_tx_sem;
QueueHandle_t	  cdc_rx_queue;
QueueHandle_t 	  queue_button;

#define USB_PACKET_SIZE	64

void tud_cdc_send(uint8_t *buffer, uint32_t bufsize, TickType_t timeout){
	if (bufsize <= USB_PACKET_SIZE){
		tud_cdc_write((uint8_t *)buffer, bufsize);
	    tud_cdc_write_flush();
	    xSemaphoreTake(cdc_tx_sem, timeout);
	}else{
		uint32_t len = 0;
		while(bufsize){
			if (bufsize > USB_PACKET_SIZE){
				len = USB_PACKET_SIZE;
			}else{
				len = bufsize;
			}
			tud_cdc_write((uint8_t *)buffer, len);
			tud_cdc_write_flush();
			xSemaphoreTake(cdc_tx_sem, timeout);
			buffer += len;
			bufsize -= len;
		}
	}
}

uint32_t tud_cdc_receive(uint8_t *buffer, uint32_t bufsize, TickType_t timeout){
	uint32_t len;
	xQueueReceive(cdc_rx_queue, &len, timeout);
	if (len > bufsize){
		len = bufsize;
	}
	uint32_t count = tud_cdc_read(buffer, len);
	return count;
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
#if 0
void cdc_task(void* params)
{
  (void) params;
  uint8_t buffer[16];
  cdc_rx_queue = xQueueCreate(8, sizeof(uint32_t));
  cdc_tx_sem = xSemaphoreCreateBinary();

	do {
		vTaskDelay(10);
	}while (!tud_cdc_connected());

  // RTOS forever loop
  while ( 1 )
  {
		/* This implementation reads a single character at a time.  Wait in the
		Blocked state until a character is received. */
		// read
		uint32_t count = tud_cdc_receive(buffer, 1, portMAX_DELAY);

        // read and echo back
		if (count){
			tud_cdc_send(buffer, count, portMAX_DELAY);
		}
  }
}
#endif

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;
  (void) rts;

  // TODO set some indicator
  if ( dtr )
  {
    // Terminal connected
  }else
  {
    // Terminal disconnected
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	portBASE_TYPE high_priority_task_woken = pdFALSE;
	uint32_t len = tud_cdc_n_available(itf);
	xQueueSendToBackFromISR(cdc_rx_queue, &len, &high_priority_task_woken);
	portYIELD_FROM_ISR(high_priority_task_woken)
}


void tud_cdc_tx_complete_cb(uint8_t itf) {
	  (void) itf;
	  xSemaphoreGive(cdc_tx_sem);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinky_cb(TimerHandle_t xTimer)
{
  (void) xTimer;
  static bool led_state = false;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

void green_led_blinky_cb(TimerHandle_t xTimer)
{
  (void) xTimer;
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
}

BaseType_t read_button(uint8_t *button, TickType_t timeout){
	return xQueueReceive(queue_button, button, timeout);
}

void button_cb(TimerHandle_t xTimer)
{
	(void) xTimer;
	if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET) {
		// guardar em uma fila de botões
		uint8_t button = BUTTON_Pin;
		xQueueSend(queue_button, &button, 0);
	}
	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_Pin);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
    xTimerStartFromISR(button_tm, &pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
  }
}



//////////////////////////////////////////////////////////
//// Console
///////////////////////////////////////////////////////////
static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer,
                                          size_t xWriteBufferLen,
										  const char *pcCommandString )
{
    static BaseType_t state = 0;

    if (!state){
        /* For simplicity, this function assumes the output buffer is large enough
        to hold all the text generated by executing the vTaskList() API function,
        so the xWriteBufferLen parameter is not used. */
        char *head = "Name		State  Priority  Stack  Number\n\r";
        ( void ) xWriteBufferLen;

        /* pcWriteBuffer is used directly as the vTaskList() parameter, so the table
        generated by executing vTaskList() is written directly into the output
        buffer. */
        strcpy(pcWriteBuffer, head);
        vTaskList( &pcWriteBuffer[strlen(head)]);

        /* The entire table was written directly to the output buffer.  Execution
        of this command is complete, so return pdFALSE. */
        state = 1;
        return pdTRUE;
    }else{
        state = 0;
        strcpy(pcWriteBuffer, "\n\r");
        return pdFALSE;
    }
}


FRESULT res;
DIR dir;
FILINFO fno;
static BaseType_t prvLSCommand( char *pcWriteBuffer,
                                size_t xWriteBufferLen,
								const char *pcCommandString )
{
    //static BaseType_t state = 0;
    char buffer[128+32];
    char *path = "0:/";

    //if (!state){
        // Abre o diretório
        res = f_opendir(&dir, path);
        if (res == FR_OK) {
            int len = sprintf(buffer, "Conteúdo do diretório: %s\n", path);
            strcpy(pcWriteBuffer, buffer);
            pcWriteBuffer += len;
            memset(buffer, 0, 64);
            len = sprintf(buffer, "----------------------------------------\n");
            strcpy(pcWriteBuffer, buffer);
            pcWriteBuffer += len;
            memset(buffer, 0, 64);

            while (1) {
                        // Lê o próximo item do diretório
                        res = f_readdir(&dir, &fno);

                        // Se houve erro ou chegou ao fim do diretório
                        if (res != FR_OK || fno.fname[0] == 0) break;

                        // Verifica se é diretório ou arquivo
                        if (fno.fattrib & AM_DIR) {
                        	len = sprintf(buffer, "[DIR]  %s/\n", fno.fname);
                        } else {
                        	len = sprintf(buffer, "[FILE] %s (%lu bytes)\n", fno.fname, fno.fsize);
                        }
                        strcpy(pcWriteBuffer, buffer);
                        pcWriteBuffer += len;
                        memset(buffer, 0, 64);
            }

            len = sprintf(buffer, "----------------------------------------\n");
            strcpy(pcWriteBuffer, buffer);
			f_closedir(&dir);
		} else {
			sprintf(buffer, "Erro ao abrir diretório: %d\n", res);
			strcpy(pcWriteBuffer, buffer);
		}

        //return pdTRUE;
        return pdFALSE;
        /*
    }else{
        state = 0;
        strcpy(pcWriteBuffer, "\n\r");
        return pdFALSE;
    }
    */
}

static const CLI_Command_Definition_t xTasksCommand =
{
    "tasks",
	"tasks: Lists all the installed tasks\r\n\r\n",
	prvTaskStatsCommand,
    0
};

static const CLI_Command_Definition_t xLsCommand =
{
    "ls",
	"ls: List files into the SD card\r\n\r\n",
	prvLSCommand,
    0
};

void print_string(char *string, TickType_t timeout) {
	tud_cdc_send((uint8_t *)string, strlen(string), timeout);
}

void print_char(char character, TickType_t timeout) {
	tud_cdc_send((uint8_t *)&character, 1, timeout);
}

#define cmdPARAMTER_NOT_USED		( ( void * ) 0 )
#define MAX_INPUT_LENGTH    32
#define MAX_OUTPUT_LENGTH   512
static void cdc_task(void *param){
	char cRxedChar;
	BaseType_t cInputIndex = 0;
	BaseType_t xMoreDataToFollow;
	/* The input and output buffers are declared static to keep them off the stack. */
	static char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

	FreeRTOS_CLIRegisterCommand( &xTasksCommand );
	FreeRTOS_CLIRegisterCommand( &xLsCommand );

	cdc_rx_queue = xQueueCreate(8, sizeof(uint32_t));
	cdc_tx_sem = xSemaphoreCreateBinary();

	do {
		vTaskDelay(10);
	}while (!tud_cdc_connected());

	print_string("Welcome to FreeRTOS ", portMAX_DELAY);
	print_string(tskKERNEL_VERSION_NUMBER, portMAX_DELAY);
	print_string("\n\r", portMAX_DELAY);
	print_string(">>", portMAX_DELAY);

	while(1){
			/* This implementation reads a single character at a time.  Wait in the
	        Blocked state until a character is received. */
			(void)tud_cdc_receive((uint8_t *)&cRxedChar, 1, portMAX_DELAY);

	        if( cRxedChar == '\r' )
	        {
	            /* A newline character was received, so the input command string is
	            complete and can be processed.  Transmit a line separator, just to
	            make the output easier to read. */
	        	print_string("\n\r", portMAX_DELAY);

	            /* The command interpreter is called repeatedly until it returns
	            pdFALSE.  See the "Implementing a command" documentation for an
	            exaplanation of why this is. */
	            do
	            {
	                /* Send the command string to the command interpreter.  Any
	                output generated by the command interpreter will be placed in the
	                pcOutputString buffer. */
	                xMoreDataToFollow = FreeRTOS_CLIProcessCommand
	                              (
	                                  pcInputString,   /* The command string.*/
	                                  pcOutputString,  /* The output buffer. */
	                                  MAX_OUTPUT_LENGTH/* The size of the output buffer. */
	                              );

	                /* Write the output generated by the command interpreter to the
	                console. */
	                print_string(pcOutputString, portMAX_DELAY);
	                memset(pcOutputString, 0, MAX_OUTPUT_LENGTH);

	            } while( xMoreDataToFollow != pdFALSE );
                print_string(">>", portMAX_DELAY);

	            /* All the strings generated by the input command have been sent.
	            Processing of the command is complete.  Clear the input string ready
	            to receive the next command. */
	            cInputIndex = 0;
	            memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
	        }
	        else
	        {
	            /* The if() clause performs the processing after a newline character
	            is received.  This else clause performs the processing if any other
	            character is received. */

	            if( cRxedChar == '\n' )
	            {
	                /* Ignore carriage returns. */
	            }
	            //else if( cRxedChar == '\b' )
	            else if( cRxedChar == 0x7F )
	            {
	                /* Backspace was pressed.  Erase the last character in the input
	                buffer - if there are any. */
	                if( cInputIndex > 0 )
	                {
	                    cInputIndex--;
	                    pcInputString[ cInputIndex ] = (char)'\0';
		                print_char(cRxedChar, portMAX_DELAY);
	                }
	            }
	            else
	            {
	                /* A character was entered.  It was not a new line, backspace
	                or carriage return, so it is accepted as part of the input and
	                placed into the input buffer.  When a \n is entered the complete
	                string will be passed to the command interpreter. */
	                if( cInputIndex < MAX_INPUT_LENGTH )
	                {
	                    pcInputString[ cInputIndex ] = cRxedChar;
	                    cInputIndex++;
	                }
	                print_char(cRxedChar, portMAX_DELAY);
	            }
	        }
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
  board_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
  green_led_blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(500), true, NULL, green_led_blinky_cb);
  button_tm = xTimerCreate("Button timer", pdMS_TO_TICKS(100), pdFALSE, NULL, button_cb);
  xTimerStart(blinky_tm, 0);
  xTimerStart(green_led_blinky_tm, 0);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queue_button = xQueueCreate(32, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, microphoneTask, osPriorityBelowNormal, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate( cdc_task, "cdc", CDC_STACK_SZIE, NULL, configMAX_PRIORITIES-2, NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_HIGH;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin ORANGE_LED_Pin RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDCARD_DETECTION_Pin */
  GPIO_InitStruct.Pin = SDCARD_DETECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDCARD_DETECTION_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_microphoneTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_microphoneTask */
void microphoneTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  uint32_t file_number = 0;
  char filename[64];
  uint8_t button;

  mic_init();

  #if 0
  mic_start();
  while(1){
	  uint32_t bytes_read = mic_read();
	  vTaskDelay(1000);
  }
  #endif

  if (f_mount(&SDFatFS, SDPath, 1) != FR_OK){
	  vTaskSuspend(NULL);
  }

  /* Infinite loop */
  for(;;)
  {
	read_button(&button, portMAX_DELAY);
	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
	sprintf(filename, "audio_%ld.wav", file_number);
	record_wav(filename, 1);
	HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
	file_number++;
    //osDelay(10000);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
