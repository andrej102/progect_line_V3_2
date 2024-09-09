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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "event_groups.h"
#include "semphr.h"

#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SOFT_VER 03020000

// line specifications

#define LINE_DUMMY 20									// number dummy pixels
#define LINE_SIZE 1536									// number working pixels
#define LINE_SIZE_WITH_DUMMY (LINE_DUMMY + LINE_SIZE)	// number tolal pixels

#define MUX_SIZE 16

// switch work and debug screens

#define TOUCH_PERIOD 3000
#define NUM_TOUCH_FOR_SWITCH_PAGE 3

// debug mode switch

#define USE_DEBUG_MODE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim17_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

#define TRANSFER_FRAME_HEADER_SIZE 8

#define PIXEL_DIVIDER 8
#define QUEUE_NUM_LINES 100
#define LINE_DIV_LENGHT (LINE_SIZE / PIXEL_DIVIDER)
#define LINE_TRANS_LENGHT ((LINE_DIV_LENGHT/8) + TRANSFER_FRAME_HEADER_SIZE)

#define QUEUE_NUM_LINES_USB 100
#define QUEUE_NUM_LINES_UART 10

__attribute__((section(".ram_d2"))) uint32_t BufferCOMP1[LINE_SIZE_WITH_DUMMY];

uint32_t lines_buffer[QUEUE_NUM_LINES * LINE_DIV_LENGHT];

uint8_t lines_buffer_usb[QUEUE_NUM_LINES_USB * LINE_TRANS_LENGHT];
__attribute__((section(".ram_d2"))) uint8_t lines_buffer_uart[QUEUE_NUM_LINES_UART * LINE_TRANS_LENGHT];

QueueHandle_t xQueue_pLines_busy = NULL;
QueueHandle_t xQueue_pLines_empty = NULL;

QueueHandle_t xQueue_pLines_busy_usb = NULL;
QueueHandle_t xQueue_pLines_empty_usb = NULL;

QueueHandle_t xQueue_pLines_busy_uart = NULL;
QueueHandle_t xQueue_pLines_empty_uart = NULL;

QueueHandle_t xQueue_RX_uart = NULL;
QueueHandle_t xQueue_TX_uart = NULL;

uint32_t global_lines_counter = 0;
uint32_t queue_polling_lines_counter = 0;
uint32_t queue_send_lines_counter = 0;
uint32_t skip_lines_counter = 0;
uint32_t skip_lines_counter_2 = 0;

uint32_t clean_test_lines_buffer[LINE_DIV_LENGHT];

  //---------- scanner algorithm lines objects ------

typedef struct
{
	uint32_t area;
	uint32_t cont;
	uint32_t sl;
} line_object_t;

uint32_t current_line[LINE_DIV_LENGHT];
uint32_t NumObjectsInCurrentLine = 0;
line_object_t objects_current_line[LINE_DIV_LENGHT];
line_object_t *p_objects_current_line[LINE_DIV_LENGHT];

uint32_t last_line[LINE_DIV_LENGHT];
uint32_t NumObjectsInLastLine = 0;
line_object_t objects_last_line[LINE_DIV_LENGHT];
line_object_t *p_objects_last_line[LINE_DIV_LENGHT];

uint32_t counter_num_extra_count = 0;

uint32_t numObjects = 0;

//-----------------------------

uint32_t Objects_area[1000];
uint32_t num_show_object_area = 0;
uint32_t midle_area = 0;


uint32_t pices_time[NUM_PICES_PERIOD];
uint32_t system_time = 0;
uint32_t pice_period = 0;

uint8_t start = 0;
uint8_t data_tx_buffer[256];
uint8_t num_data_tx = 0;

uint32_t uart_rx_timeout = 0;
uint8_t uart_rx_buffer[256];
uint16_t uart_rx_buffer_pointer=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP1_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

xTaskHandle xTaskHandle_Scanner = NULL;
xTaskHandle xTaskHandle_TouchScreen = NULL;
xTaskHandle xTaskHandle_ContainerDetect = NULL;
xTaskHandle xTaskHandle_Display = NULL;
xTaskHandle xTaskHandle_USART_Service = NULL;
xTaskHandle xTaskHandle_UART_Line_TX = NULL;
xTaskHandle xTaskHandle_USB_Line_TX = NULL;
xTaskHandle xTaskHandle_Main = NULL;

void vTask_Main (void *pvParameters);
void vTask_Scanner (void *pvParameters);
void vTask_TouchScreen (void *pvParameters);
void vTask_ContainerDetect (void *pvParameters);
void vTask_Display (void *pvParameters);
void vTask_USART_Service (void *pvParameters);
void vTask_UART_Line_TX (void *pvParameters);
void vTask_USB_Line_TX (void *pvParameters);

void UARTsTunning(void);
void ComparatorsTuning(void);
void TimersTuning(void);
void SystemInterruptsTuning(void);
void DMATuning(void);

void StartScaner(void);
void StopScaner(void);
void Clear_Counter (void);

void tft_show_message(uint8_t msg);
void tft_show_nun_pices(uint16_t num_pices);
void tft_show_area_pices(uint16_t num_pice);
void tft_show_overcount(uint16_t state);
void tft_show_hide_counter(uint16_t state);
void tft_show_page(uint8_t page_num);
void tft_send_click(uint8_t num_but, uint8_t event);

void Delay_us(uint32_t us);

void tft_show_param(void);
void service_page_0(uint8_t but, uint8_t val);
void service_page_1(uint8_t but, uint8_t val);

EventGroupHandle_t xEventGroup_StatusFlags;

const EventBits_t Flag_Scanner_Busy =           	0x00000001;
const EventBits_t Flag_Over_Count =           		0x00000002;
const EventBits_t Flag_Mode_Transparent =           0x00000004;
const EventBits_t Flag_USART_TX =           		0x00000008;
const EventBits_t Flag_USART_RX =           		0x00000010;
const EventBits_t Flag_UART_RX_Buffer_Busy = 		0x00000020;
const EventBits_t Flag_Mode_Blue = 					0x00000040;
const EventBits_t Flag_Over_Count_Display =    		0x00000080;
const EventBits_t Flag_Container_Removed =    		0x00000100;
const EventBits_t Flag_Counter_Not_Visible =    	0x00000200;
const EventBits_t Flag_Touch_Key_Poling		 =    	0x00000400;
const EventBits_t Flag_USB_LINE_TX_Complete	=		0x00000800;
const EventBits_t Flag_UART_LINE_TX_Complete = 		0x00001000;
const EventBits_t Flag_UART_TX_Ready =		 		0x00002000;
const EventBits_t Flag_Reset_lines_counters =	 	0x00004000;
const EventBits_t Flag_Activity_Detect =	 		0x00008000;
const EventBits_t Flag_Idle_State =	 				0x00010000;
const EventBits_t Flag_Idle_Event = 				0x00020000;
const EventBits_t Flag_Scaner_State =				0x00040000;
const EventBits_t Flag_Scaner_Event =				0x00080000;
const EventBits_t Flag_Protect_State =				0x00100000;
const EventBits_t Flag_Protect_Event =				0x00200000;
const EventBits_t Flag_Scaner_Dirty			 =		0x00400000;
const EventBits_t Flag_Scaner_Dirty_Event	 =		0x00800000;

EventGroupHandle_t xEventGroup_StatusFlags_2;

const EventBits_t Flag_2_Need_Stop_Scaner	 =			0x00000001;
const EventBits_t Flag_2_Envent_Mode = 				0x00000002;
const EventBits_t Flag_2_Envent_Mode_Press =			0x00000004;
const EventBits_t Flag_2_Envent_Mode_Unpress =		0x00000008;
const EventBits_t Flag_2_Need_Mode_Event  =			0x00000010;
const EventBits_t Flag_2_Debug_Mode = 				0x00000020;

EventGroupHandle_t xEventGroup_ChangeScreenFlags;

const EventBits_t Flag_Show_Screen_0	 =			0x00000001;
const EventBits_t Flag_Show_Screen_1 = 				0x00000002;

SemaphoreHandle_t xSemaphoreMutex_Pice_Counter;

uint8_t active_page = 0;

uint8_t num_param = 0;
char param_str[32] = {0};

//------------ major tuning parameters of scanner ----------------------

#define PROTECT_SERVICE_ENABLE 1		// protect service enable
#define CLEAN_TEST_SERVICE_ENABLE 1		// clean test service enable
#define OVER_AREA 1500					// global max area
#define IDLE_STATE_TIMEOUT 1000  		// idle state timeout in seconds

float k_1 = 3.0; 						// for small tablets
float k_2 = 1.7; 						// for big tablets
uint32_t div_12 = 200;					// area separating small and big tablets
uint32_t min_area = 10;					// current smallest area to be taken into account
uint32_t max_area = 1500;				// current largest area to be taken into account without division

//----------------------------------

uint8_t change_num_param = 0;



// USB interface variables

extern USBD_HandleTypeDef hUsbDeviceHS;

uint8_t * p_pixel_parsel = NULL;
uint8_t temp_pixel_parsel[LINE_TRANS_LENGHT];
uint32_t pixel_parsel_counter = 0;

// after start scanner execute dummy scans for clear line,
// then do scan for determining whether the line is clean

uint32_t start_time =0;

#define INIT_DUMMY_SCAN_COUNTER_VALUE 		100
#define INIT_CLEAR_TEST_SCAN_COUNTER_VALUE 	100

uint32_t dummy_scan_counter = INIT_DUMMY_SCAN_COUNTER_VALUE;
uint32_t clean_test_scan_counter = INIT_CLEAR_TEST_SCAN_COUNTER_VALUE;



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

	void *p_line = NULL;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
//  MX_DMA_Init();
//  MX_COMP1_Init();
//  MX_TIM17_Init();
//  MX_USART1_UART_Init();
//  MX_USART3_UART_Init();
//  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  RCC->AHB2ENR |= RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN;
  RCC->APB4ENR |= RCC_APB4ENR_COMP12EN;
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM17_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  UARTsTunning();
  ComparatorsTuning();
  TimersTuning();

  DMATuning();
  SystemInterruptsTuning();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // create event grup

    xEventGroup_StatusFlags = xEventGroupCreate();
    xEventGroup_StatusFlags_2 = xEventGroupCreate();
    xEventGroup_ChangeScreenFlags = xEventGroupCreate();

    xEventGroupSetBits(xEventGroup_StatusFlags, Flag_UART_TX_Ready);

    // create semaphores

    xSemaphoreMutex_Pice_Counter = xSemaphoreCreateMutex();

    // create queues

    xQueue_pLines_busy = xQueueCreate( QUEUE_NUM_LINES, sizeof(uint32_t) );
    xQueue_pLines_empty = xQueueCreate( QUEUE_NUM_LINES, sizeof(uint32_t) );

    for(uint16_t i = 0; i < QUEUE_NUM_LINES; i++)
    {
  	  p_line = lines_buffer + (i * LINE_DIV_LENGHT);
  	  xQueueSend(xQueue_pLines_empty, &p_line, 0);
    }

    xQueue_pLines_busy_usb = xQueueCreate( QUEUE_NUM_LINES_USB, sizeof(uint32_t) );
    xQueue_pLines_empty_usb = xQueueCreate( QUEUE_NUM_LINES_USB, sizeof(uint32_t) );

    for(uint16_t i = 0; i < QUEUE_NUM_LINES_USB; i++)
    {
  	  p_line = lines_buffer_usb + (i * LINE_TRANS_LENGHT);
  	  xQueueSend(xQueue_pLines_empty_usb, &p_line, 0);
    }

    xQueue_pLines_busy_uart = xQueueCreate( QUEUE_NUM_LINES_UART, sizeof(uint32_t) );
    xQueue_pLines_empty_uart = xQueueCreate( QUEUE_NUM_LINES_UART, sizeof(uint32_t) );

    for(uint16_t i = 0; i < QUEUE_NUM_LINES_UART; i++)
    {
  	  p_line = lines_buffer_uart + (i * LINE_TRANS_LENGHT);
  	  xQueueSend(xQueue_pLines_empty_uart, &p_line, 0);
    }

    xQueue_RX_uart = xQueueCreate( 256, sizeof(uint8_t) );
    xQueue_TX_uart = xQueueCreate( 256, sizeof(uint8_t) );


    // vars zero

    memset(last_line, 0, sizeof(last_line));
    memset(objects_current_line, 0, sizeof(objects_current_line));

    memset(current_line, 0, sizeof(current_line));
    memset(objects_last_line, 0, sizeof(objects_last_line));

    memset(data_tx_buffer, 0, sizeof(data_tx_buffer));
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));

    active_page = 0;

    // create tasks

    xTaskCreate(vTask_Main,(char*)"Task Main", 1024, NULL, tskIDLE_PRIORITY + 3, &xTaskHandle_Main);
    xTaskCreate(vTask_Scanner,(char*)"Task Scanner", 1024, NULL, tskIDLE_PRIORITY + 5, &xTaskHandle_Scanner);
    xTaskCreate(vTask_TouchScreen,(char*)"Task TouchScreen", 1024, NULL, tskIDLE_PRIORITY + 3, &xTaskHandle_TouchScreen);
    xTaskCreate(vTask_ContainerDetect,(char*)"Task Container Detect", 1024, NULL, tskIDLE_PRIORITY + 3, &xTaskHandle_ContainerDetect);
    xTaskCreate(vTask_Display,(char*)"Task Display", 1024, NULL, tskIDLE_PRIORITY + 3, &xTaskHandle_Display);
    xTaskCreate(vTask_USART_Service,(char*)"USART Service", 1024, NULL, tskIDLE_PRIORITY + 3, &xTaskHandle_USART_Service);

      //xTaskCreate(vTask_UART_Line_TX,(char*)"UART Line TX", 512, NULL, tskIDLE_PRIORITY + 4, &xTaskHandle_UART_Line_TX);
    //xTaskCreate(vTask_USB_Line_TX,(char*)"USB Line TX", 1024, NULL, tskIDLE_PRIORITY + 4, &xTaskHandle_USB_Line_TX);

    StartScaner();

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_IO1;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO2;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 274;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 60;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 55;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 30;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  huart1.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_G_Pin|LED_R_Pin|LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OV_CNT_Pin|LED_BLUE_TIM3_CH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, S0_Pin|S1_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin LED_B_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUT_U_Pin LINE1_EOS_Pin SW_U_Pin */
  GPIO_InitStruct.Pin = BUT_U_Pin|LINE1_EOS_Pin|SW_U_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : U2_CTS_Pin U2_RTS_Pin U2_TX_Pin U2_RX_Pin */
/*  GPIO_InitStruct.Pin = U2_CTS_Pin|U2_RTS_Pin|U2_TX_Pin|U2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/

  /*Configure GPIO pin : ESP_RDY_Pin */
 /* GPIO_InitStruct.Pin = ESP_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ESP_RDY_GPIO_Port, &GPIO_InitStruct);*/

  /*Configure GPIO pin : LINE1_REF2_COMP1_INM_Pin */
 /* GPIO_InitStruct.Pin = LINE1_REF2_COMP1_INM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LINE1_REF2_COMP1_INM_GPIO_Port, &GPIO_InitStruct);*/

  /*Configure GPIO pins : OV_CNT_Pin LED_BLUE_TIM3_CH2_Pin */
  GPIO_InitStruct.Pin = OV_CNT_Pin|LED_BLUE_TIM3_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TRAY_DET_Pin MUX1_Pin MUX2_Pin MUX3_Pin
                           MUX4_Pin MUX5_Pin MUX6_Pin */
  GPIO_InitStruct.Pin = TRAY_DET_Pin|MUX1_Pin|MUX2_Pin|MUX3_Pin
                          |MUX4_Pin|MUX5_Pin|MUX6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  /*Configure GPIO pin : TIM17_CH1_LINE_CLK_Pin */
    GPIO_InitStruct.Pin = TIM17_CH1_LINE_CLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;
    HAL_GPIO_Init(TIM17_CH1_LINE_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TIM3_CH1_LINE_ST_Pin */
    GPIO_InitStruct.Pin = TIM3_CH1_LINE_ST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(TIM3_CH1_LINE_ST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TIM3_CH2_LIGTH_Pin */
    GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin */
 	GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
 	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 	GPIO_InitStruct.Pull = GPIO_NOPULL;
 	HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
 	HAL_GPIO_WritePin(TIM3_CH2_LIGHT_BLUE_GPIO_Port, TIM3_CH2_LIGHT_BLUE_Pin, 0);

 	 /*Configure GPIO pins : CONTAINER_DETECT_Pin */
	GPIO_InitStruct.Pin = CONTAINER_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CONTAINER_DETECT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 *
 */

void vTask_Main(void *pvParameters)
{
	uint32_t idle_timer=0;
	uint32_t previousTickCount = xTaskGetTickCount();

	while(1)
	{
		// check activity flag

		if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Activity_Detect)
		{
			xEventGroupClearBits( xEventGroup_StatusFlags, Flag_Activity_Detect | Flag_Idle_State | Flag_Protect_State | Flag_Scaner_Dirty);

			previousTickCount = xTaskGetTickCount();

			idle_timer = 0;

			if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Scaner_State))
			{
				memset(last_line, 0, sizeof(last_line));
				memset(objects_current_line, 0, sizeof(objects_current_line));

				memset(current_line, 0, sizeof(current_line));
				memset(objects_last_line, 0, sizeof(objects_last_line));

				if (!(xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode))
				{
				  Clear_Counter();
				}

				StartScaner();
				xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Scaner_State | Flag_Scaner_Event);
			}
		}
		else if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Protect_State))
		{
			if ((xTaskGetTickCount() - previousTickCount) >= 1000)
			{
				previousTickCount = xTaskGetTickCount();
				idle_timer++;
			}

			if ((idle_timer >= IDLE_STATE_TIMEOUT) && (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_State)))
			{
				StopScaner();
				xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Idle_State | Flag_Idle_Event);
			}
		}

		vTaskDelay(1000);
	}
}

/*
 *
 */

void vTask_Display(void *pvParameters)
{
  /* Infinite loop */

	static uint32_t timer_over_count_signal_display = 0;
	static uint32_t timer_counter_flashing_display = 0;

	vTaskDelay(2000);

	for(;;)
	{
		if (!active_page)
		{
			if (xEventGroupGetBits(xEventGroup_ChangeScreenFlags) & Flag_Show_Screen_1)
			{
				xEventGroupClearBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_1);
				tft_show_page(1);
			}
			else if (xEventGroupGetBits(xEventGroup_ChangeScreenFlags) & Flag_Show_Screen_0)
			{
				xEventGroupClearBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_0);
				tft_show_page(0);
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode_Press)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
				tft_show_message(4); //
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode_Unpress)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
				tft_show_message(5); //
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Scaner_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Scaner_Event);
				tft_show_message(0); // Active
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Idle_Event);
				tft_show_message(1); // Idle
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Protect_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Protect_Event);
				tft_show_message(2); // Protect
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Scaner_Dirty_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Scaner_Dirty_Event);
				tft_show_message(3); // Not Clear
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Need_Mode_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Need_Mode_Event);

				if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Mode_Blue)
				{
					if (!active_page)
					{
						tft_send_click(4, 1);
					}
					else
					{
						tft_send_click(12, 1);
					}
				}
				else
				{
					if (!active_page)
					{
						tft_send_click(4, 0);
					}
					else
					{
						tft_send_click(12, 0);
					}
				}

				if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_State)
				{
					tft_show_message(0);
				}
			}
			else
			{

				tft_show_nun_pices(numObjects);

				if (timer_over_count_signal_display)
				{
					timer_over_count_signal_display--;
					if (!timer_over_count_signal_display)
					{
						tft_show_overcount(0);
					}
				}

				if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Over_Count_Display)
				{
					xEventGroupClearBits( xEventGroup_StatusFlags, Flag_Over_Count_Display);

					if (!timer_over_count_signal_display)
					{
						tft_show_overcount(1);
					}

					timer_over_count_signal_display = 5;
				}

				//------
				if ( (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Protect_State)) /*&& (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_State))*/)
				{
					if (timer_counter_flashing_display)
					{
						timer_counter_flashing_display--;
					}

					if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Container_Removed)
					{
						if (!timer_counter_flashing_display)
						{
							if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Counter_Not_Visible)
							{
								tft_show_hide_counter(1);
							}
							else
							{
								tft_show_hide_counter(0);
							}

							timer_counter_flashing_display = 5;
						}
					}
					else
					{
						if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Counter_Not_Visible)
						{
							tft_show_hide_counter(1);
						}
					}
				}
			}
		}
		else if (active_page == 1)
		{
			if (xEventGroupGetBits(xEventGroup_ChangeScreenFlags) & Flag_Show_Screen_1)
			{
				xEventGroupClearBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_1);
				tft_show_page(1);
			}
			else if (xEventGroupGetBits(xEventGroup_ChangeScreenFlags) & Flag_Show_Screen_0)
			{
				xEventGroupClearBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_0);
				tft_show_page(0);
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode_Press)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
				tft_show_message(4); //
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode_Unpress)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
				tft_show_message(5); //
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Scaner_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Scaner_Event);
				tft_show_message(0); // Active
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Idle_Event);
				tft_show_message(1); // Idle
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Protect_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Protect_Event);
				tft_show_message(2); // Protect
			}
			else if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Scaner_Dirty_Event)
			{
				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Scaner_Dirty_Event);
				tft_show_message(3); // Not Clear
			}
			else
			{

				tft_show_nun_pices(numObjects);

				tft_show_area_pices(num_show_object_area);

				if (timer_over_count_signal_display)
				{
					timer_over_count_signal_display--;
					if (!timer_over_count_signal_display)
					{
						tft_show_overcount(0);
					}
				}

				if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Over_Count_Display)
				{
					xEventGroupClearBits( xEventGroup_StatusFlags, Flag_Over_Count_Display);

					if (!timer_over_count_signal_display)
					{
						tft_show_overcount(1);
					}

					timer_over_count_signal_display = 5;
				}

				//------
				if ( (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Protect_State)) /*&& (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Idle_State))*/)
				{
					if (timer_counter_flashing_display)
					{
						timer_counter_flashing_display--;
					}

					if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Container_Removed)
					{
						if (!timer_counter_flashing_display)
						{
							if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Counter_Not_Visible)
							{
								tft_show_hide_counter(1);
							}
							else
							{
								tft_show_hide_counter(0);
							}

							timer_counter_flashing_display = 5;
						}
					}
					else
					{
						if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Counter_Not_Visible)
						{
							tft_show_hide_counter(1);
						}
					}
				}
			}
		}

		vTaskDelay(100);

	/*	tft_show_nun_pices(numObjects);
		vTaskDelay(1000);*/


	}

}

/*
 *
 */

void service_page_1(uint8_t but, uint8_t val)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	static uint32_t last_time_touch = 0;
	static uint8_t touch_counter = 0;

	switch(but)
	{
		case 15 :
		{
			if (val)
			{
				if ((HAL_GetTick() - last_time_touch) > TOUCH_PERIOD)
				{
					touch_counter = 1;
				}
				else
				{
					touch_counter++;
					if (touch_counter == NUM_TOUCH_FOR_SWITCH_PAGE)
					{
						touch_counter = 0;
						xEventGroupSetBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_0);
					}
				}

				last_time_touch = HAL_GetTick();
			}

			break;
		}

		case 3 :
		{
		  if (val)
		  {
			  if(num_show_object_area < numObjects)
			  {
				  num_show_object_area++;
			  }
			  else
			  {
				  num_show_object_area = 1;
			  }
		  }

		  break;
		}

		case 2 :
		{
		  if (val)
		  {
			  if(num_show_object_area > 1)
			  {
				  num_show_object_area--;
			  }
		  }

		  break;
		}

		case 16 :
		{
			if (val)
			{
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode);
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
			}
			else
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode);
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
			}

			break;
		}

		case 12 :
		{
		  if (val)
		  {
			  xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Mode_Blue);

			  COMP1->CFGR |= COMP_CFGRx_INMSEL_0; // PC4 -

			  /*Configure GPIO pin : TIM3_CH2_LIGTH_Pin */
				GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);
				HAL_GPIO_WritePin(TIM3_CH2_LIGHT_GPIO_Port, TIM3_CH2_LIGHT_Pin, 0);

				 /*Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin */
				GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
				GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
				HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
		  }
		  else if (!val)
		  {
			  COMP1->CFGR &= ~COMP_CFGRx_INMSEL_0; // PB1 -

			  /*Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin */
				 GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
				 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
				 GPIO_InitStruct.Pull = GPIO_NOPULL;
				 HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
				 HAL_GPIO_WritePin(TIM3_CH2_LIGHT_BLUE_GPIO_Port, TIM3_CH2_LIGHT_BLUE_Pin, 0);

				 /*Configure GPIO pin : TIM3_CH2_LIGTH_Pin */
				GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
				GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
				GPIO_InitStruct.Pull = GPIO_NOPULL;
				GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
				GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
				HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);

				xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Mode_Blue);
		  }

		  break;
		}

		case 17 :
		{
			if (val)
			{
				StopScaner();
				Clear_Counter();
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Debug_Mode);
				if(!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Container_Removed))
				{
					xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Activity_Detect);
				}
			}
			else
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Debug_Mode);
			}

			break;
		}


		default : break;
	}
}

/*
 *
 */

void service_page_0(uint8_t but, uint8_t val)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	static uint32_t last_time_touch = 0;
	static uint8_t touch_counter = 0;

	switch(but)
	{
		case 4 :
		{
			//if (!numObjects)
			//{
			//	StopScaner();

				if (val)
				{
					xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Mode_Blue);

					COMP1->CFGR |= COMP_CFGRx_INMSEL_0; // PC4 -

					//Configure GPIO pin : TIM3_CH2_LIGTH_Pin
					GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);
					HAL_GPIO_WritePin(TIM3_CH2_LIGHT_GPIO_Port, TIM3_CH2_LIGHT_Pin, 0);

					 //Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin
					GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
					HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
				}
				else if (!val)
				{
					COMP1->CFGR &= ~COMP_CFGRx_INMSEL_0; // PB1 -

					  //Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin
					GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
					GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
					HAL_GPIO_WritePin(TIM3_CH2_LIGHT_BLUE_GPIO_Port, TIM3_CH2_LIGHT_BLUE_Pin, 0);

					 //Configure GPIO pin : TIM3_CH2_LIGTH_Pin
					GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
					GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
					GPIO_InitStruct.Pull = GPIO_NOPULL;
					GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
					GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
					HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);

					xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Mode_Blue);
				 }

				// xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Activity_Detect);
			// }

			// xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Need_Mode_Event);

			 break;
		}

		case 5 :
		{
			if (val)
			{
				if ((HAL_GetTick() - last_time_touch) > TOUCH_PERIOD)
				{
					touch_counter = 1;
				}
				else
				{
					touch_counter++;
					if (touch_counter == NUM_TOUCH_FOR_SWITCH_PAGE)
					{
						touch_counter = 0;
						xEventGroupSetBits(xEventGroup_ChangeScreenFlags, Flag_Show_Screen_1);
					}
				}

				last_time_touch = HAL_GetTick();
			}

			break;
		}

		case 6 :
		{
			if (val)
			{
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode);
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
			}
			else
			{
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode);
				xEventGroupClearBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Press);
				xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Envent_Mode_Unpress);
			}

			break;
		}

		default : break;
	}

}

/*
 *
 */


void vTask_TouchScreen(void *pvParameters)
{

	for(;;)
	{
		xEventGroupWaitBits(xEventGroup_StatusFlags, Flag_UART_RX_Buffer_Busy, pdFALSE, pdFALSE, portMAX_DELAY );

		if (uart_rx_buffer_pointer == 7)
		{
			  if (uart_rx_buffer[0] == 0x65)
			  {
				  if (uart_rx_buffer[1] == 0x00)
				  {
					  service_page_0(uart_rx_buffer[2], uart_rx_buffer[3]);
				  }
				  else if (uart_rx_buffer[1] == 0x01)
				  {
					  service_page_1(uart_rx_buffer[2], uart_rx_buffer[3]);
				  }
			  }
		}
		else if (uart_rx_buffer_pointer == 5)
		{
			  if (uart_rx_buffer[0] == 0x66)
			  {
				  if (uart_rx_buffer[1] == 0x00)
				  {
					  active_page = 0;
				  }
				  else if (uart_rx_buffer[1] == 0x01)
				  {
					  active_page = 1;
				  }
			  }
		}

		uart_rx_buffer_pointer = 0;

		xEventGroupClearBits(xEventGroup_StatusFlags, Flag_UART_RX_Buffer_Busy);

	}
}

/*
 *
 */

void vTask_ContainerDetect(void *pvParameters)
{
	static uint8_t previous_state = 1, event_state = 0, timer_over_count_signal = 0;

	/* Infinite loop */
  for(;;)
  {
	  if (HAL_GPIO_ReadPin(CONTAINER_DETECT_GPIO_Port, CONTAINER_DETECT_Pin))
	  {
		  if(previous_state)
		  {
			  if(!event_state)
			  {
				  xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Container_Removed);
				  event_state = 1;

				  if (!(xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Envent_Mode))
				  {
					  Clear_Counter();
				  }

				  xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Activity_Detect);
			  }
		  }
		  else event_state = 0;

		  previous_state = 1;
	  }
	  else
	  {
		  if(!previous_state)
		  {
			  if(!event_state)
			  {
				  xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Container_Removed);
				  StopScaner();
				  event_state = 1;
				  xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Need_Mode_Event);
			  }
		  }
		  else
		  {
			  event_state = 0;
		  }

		  previous_state = 0;
	  }

	  osDelay(500);
  }

}

/*
 *
 */

void vTask_Scanner(void *pvParameters)
{
	uint32_t j, p, i;
	uint32_t lastbit = 0;
	uint32_t numObjects_temp =0;
	uint32_t transparent_object_start = 0;
	uint32_t transparent_object_overtime = 0;
	uint32_t transparent_object_current_line_overtime = 0;
	uint32_t r = 0, k = 0;
	uint32_t *p_line = NULL;

	uint32_t clear_tester = 0;

	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(xQueue_pLines_busy, &p_line, portMAX_DELAY);

		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);

		if (xEventGroupGetBits(xEventGroup_StatusFlags_2) & Flag_2_Debug_Mode)
		{
			dummy_scan_counter = INIT_DUMMY_SCAN_COUNTER_VALUE;
			if(!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Container_Removed))
			{
				xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Activity_Detect);
			}
		}

		if (xQueueReceive(xQueue_pLines_empty_usb, &p_pixel_parsel, 0) != pdTRUE)
		{
			p_pixel_parsel = temp_pixel_parsel;
		}

#ifndef CLEAN_TEST_SERVICE_ENABLE
		clean_test_scan_counter =0;
#endif		// CLEAN_TEST_SERVICE_ENABLE

		if(dummy_scan_counter) // dummy scans for normal start line
		{
			if(dummy_scan_counter == INIT_DUMMY_SCAN_COUNTER_VALUE)
			{
				memset((uint8_t*)last_line, 0, sizeof(last_line));
			}

			dummy_scan_counter--;

			k = 0; r = 8;

			for (j = 0; j < LINE_DIV_LENGHT; j++)
			{
				*(p_pixel_parsel + r) &= ~( 1 << k++);

				if(k == 8)
				{
					k = 0;
					r++;
				}
			}
		}
		else // active dummy scans
		{
			if(clean_test_scan_counter)
			{
				if(clean_test_scan_counter == INIT_CLEAR_TEST_SCAN_COUNTER_VALUE) //initial zeros clean line test
				{
					memset((uint8_t*)clean_test_lines_buffer, 0, sizeof(clean_test_lines_buffer));
					memset((uint8_t*)last_line, 0, sizeof(last_line));
				}

				clean_test_scan_counter--;

				clear_tester = 0;

				for (j = 0; j < LINE_DIV_LENGHT; j++)
				{
					if((*(p_line + j) & COMP_SR_C1VAL) || (j < 8))
					{
						*(p_pixel_parsel + r) &= ~( 1 << k++);
					}
					else
					{
						*(p_pixel_parsel + r) |= ( 1 << k++);

						StopScaner();
						xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Scaner_Dirty | Flag_Scaner_Dirty_Event);
						break;
					}
				}
			}
			else
			{
				NumObjectsInCurrentLine = 0;
				lastbit = 0;

				k = 0; r = 8;

				for (j = 0; j < LINE_DIV_LENGHT; j++)
				{
					if((*(p_line + j) & COMP_SR_C1VAL) || (j < 8))
					{
						current_line[j] = 0;
						lastbit = 0;

						*(p_pixel_parsel + r) &= ~( 1 << k++);
					}
					else
					{
						*(p_pixel_parsel + r) |= ( 1 << k++);

						if(!lastbit)
						{
							NumObjectsInCurrentLine++;
							p_objects_current_line[NumObjectsInCurrentLine-1] = &objects_current_line[NumObjectsInCurrentLine-1];
							p_objects_current_line[NumObjectsInCurrentLine-1]->area = 0;
						}

						current_line[j] = NumObjectsInCurrentLine;
						p_objects_current_line[NumObjectsInCurrentLine-1]->area++;
						lastbit = 1;

						if(last_line[j])
						{
							p_objects_last_line[last_line[j]-1]->cont = 1;

							if (!p_objects_last_line[last_line[j]-1]->sl)
							{
								p_objects_last_line[last_line[j]-1]->sl = current_line[j];
								p_objects_current_line[current_line[j]-1]->area += p_objects_last_line[last_line[j]-1]->area;
							}
							else
							{
								if (p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl - 1] != p_objects_current_line[current_line[j]-1])
								{
									p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl-1]->area += p_objects_current_line[current_line[j]-1]->area;
									p_objects_current_line[current_line[j]-1] = p_objects_current_line[p_objects_last_line[last_line[j]-1]->sl - 1];
								}
							}
						}
					}

					// we analyze the connectivity of the objects of the current line with the objects of the previous line
					// and arrange the corresponding signs (connectivity and continuation)

					if(k == 8) {k = 0; r++;}

					last_line[j] = current_line[j];
				}

				if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_Mode_Transparent)
				{
					// Transparent mode

					if (!transparent_object_start)
					{
						if (NumObjectsInCurrentLine)
						{
							transparent_object_start = 1;
							Objects_area[numObjects] = 0;
							transparent_object_overtime = HAL_GetTick();
							transparent_object_current_line_overtime = HAL_GetTick();
						}
					}

					if(transparent_object_start)
					{
						if ((HAL_GetTick() - transparent_object_overtime > TRANSPARENT_OBJECT_OVERTIME) || ((HAL_GetTick() - transparent_object_current_line_overtime > TRANSPARENT_OBJECT_CURRENT_LINE_OVERTIME)))
						{
							transparent_object_start = 0;
							numObjects++;
						}
						else
						{
							if (NumObjectsInCurrentLine)
							{
								transparent_object_current_line_overtime = HAL_GetTick();

								for(j = 0; j < NumObjectsInCurrentLine; j++)
								{
									Objects_area[numObjects] += p_objects_current_line[j]->area;
								}
							}
						}
					}
				}
				else
				{
							// Common mode

					// check if there are completed objects on the previous line

					line_object_t * previous_p_objects_last_line = NULL;

					for (j=0; j < NumObjectsInLastLine; j++)
					{
						// check over area
						if (p_objects_last_line[j]->area > OVER_AREA)
						{
	#ifdef PROTECT_SERVICE_ENABLE
							StopScaner();
							xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Protect_State |  Flag_Protect_Event);
	#endif // PROTECT_SERVICE_ENABLE
							p_objects_last_line[j]->area = 0;
							continue;
						}

						if (!p_objects_last_line[j]->cont)
						{
							if (p_objects_last_line[j] == previous_p_objects_last_line)
							{
								continue;
							}
							else
							{
								previous_p_objects_last_line = p_objects_last_line[j];
							}

							// check over area
							if (p_objects_last_line[j]->area > OVER_AREA)
							{
	#ifdef PROTECT_SERVICE_ENABLE
								StopScaner();
								xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Protect_State |  Flag_Protect_Event);
	#endif // PROTECT_SERVICE_ENABLE
								p_objects_last_line[j]->area = 0;
								continue;
							}

							// check under area
							if (p_objects_last_line[j]->area < min_area)
							{
								p_objects_last_line[j]->area = 0;
								continue;
							}
							numObjects_temp = numObjects;

							while (p_objects_last_line[j]->area)
							{
								if (p_objects_last_line[j]->area > max_area)
								{
									//if (midle_area >= div_12)
									//{

		#ifndef OVER_RATE_ENABLE
										xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Over_Count | Flag_Over_Count_Display);
		#endif //OVER_RATE_ENABLE
										Objects_area[numObjects] = max_area;
										p_objects_last_line[j]->area -= max_area;
									//}
									//else
									//{
									//	Objects_area[numObjects] = p_objects_last_line[j]->area;
									//	p_objects_last_line[j]->area = 0;
									//}
								}
								else
								{
									Objects_area[numObjects] = p_objects_last_line[j]->area;
									p_objects_last_line[j]->area = 0;
								}

								/*if(numObjects)
								{
									if (Objects_area[numObjects] == Objects_area[numObjects - 1])
									{
										numObjects--;
									}
								}*/

								numObjects++;

								xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Activity_Detect);

								if (numObjects == NUM_PICES_FOR_EXECUTE_MIDLE)
								{
									midle_area = 0;
									for (i=0; i < NUM_PICES_FOR_EXECUTE_MIDLE; i++) midle_area += Objects_area[i];
									midle_area /= NUM_PICES_FOR_EXECUTE_MIDLE;
									max_area = (midle_area < div_12) ? (midle_area * k_1) : (midle_area * k_2);
									min_area = (midle_area*15)/100;
								}

								if(numObjects > 1000)
								{
									numObjects = 0;
								}
							}

		//#ifdef OVER_RATE_ENABLE
							if ((numObjects_temp != numObjects) && (midle_area < div_12) && numObjects > NUM_PICES_FOR_EXECUTE_MIDLE)
							{
								for (p=1; p < NUM_PICES_PERIOD; p++)
								{
									pices_time[p-1] = pices_time[p];
								}

								pices_time[NUM_PICES_PERIOD - 1]  = HAL_GetTick();

								if (numObjects > (NUM_PICES_PERIOD - 1))
								{
									pice_period = (pices_time[NUM_PICES_PERIOD - 1] - pices_time[0]) / NUM_PICES_PERIOD;
									if (pice_period < MIN_PICE_PERIOD)
									{
										counter_num_extra_count++;
										xEventGroupSetBits( xEventGroup_StatusFlags, Flag_Over_Count | Flag_Over_Count_Display);

										for (p=0; p < NUM_PICES_PERIOD; p++)
										{
											pices_time[p] = 0;
										}
									}
								}
							}
		//#endif // OVER_RATE_ENABLE
						}
					}
				}

				// transfer objects of current line to last line

				for (j=0; j < NumObjectsInCurrentLine; j++)
				{
					p_objects_last_line[j] = &objects_last_line[0] + (p_objects_current_line[j] - &objects_current_line[0]);

					p_objects_last_line[j]->area = p_objects_current_line[j]->area;
					p_objects_last_line[j]->cont = 0;
					p_objects_last_line[j]->sl = 0;
				}

				NumObjectsInLastLine = NumObjectsInCurrentLine;
			}
		}

		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);

		xQueueSend(xQueue_pLines_empty, &p_line, 0);

		queue_polling_lines_counter++;

		if (p_pixel_parsel != temp_pixel_parsel)
		{
			*(uint32_t*)p_pixel_parsel = 0xAAAAAAAA;
			*(uint32_t*)(p_pixel_parsel + 4) = pixel_parsel_counter;

			xQueueSend(xQueue_pLines_busy_usb, &p_line, 0);
		}

		pixel_parsel_counter++;
  }

}

/*
 *
 */

void vTask_UART_Line_TX(void *pvParameters)
{
	uint8_t *p_line_uart = NULL;

	xEventGroupSetBits( xEventGroup_StatusFlags, Flag_UART_LINE_TX_Complete);

	while(1)
	{
		xQueueReceive(xQueue_pLines_busy_uart, &p_line_uart, portMAX_DELAY);

		xEventGroupWaitBits( xEventGroup_StatusFlags, Flag_UART_LINE_TX_Complete, pdFALSE, pdFALSE, 100 );

		if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_UART_LINE_TX_Complete)
		{
			xEventGroupClearBits( xEventGroup_StatusFlags, Flag_UART_LINE_TX_Complete);

			// config DMA channel

			while(DMA1_Stream1->CR & DMA_SxCR_EN)
			{
				DMA1_Stream1->CR &= ~DMA_SxCR_EN;
			}

			DMA1->LIFCR |= (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);

			DMA1_Stream1->PAR = (uint32_t)&USART3->TDR;
			DMA1_Stream1->M0AR = (uint32_t)p_line_uart;
			DMA1_Stream1->NDTR = LINE_TRANS_LENGHT;
			DMA1_Stream1->CR = DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE;
			DMAMUX1_Channel1->CCR = ( 46 << DMAMUX_CxCR_DMAREQ_ID_Pos);

			USART3->ICR |= USART_ICR_TCCF;

			DMA1_Stream1->CR &= ~DMA_SxCR_EN;

			// wait end transmission

			xEventGroupWaitBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete, pdFALSE, pdFALSE, 100);
		}
	}
}

/*
 *
 */

void vTask_USB_Line_TX(void *pvParameters)
{
	uint8_t *p_line_usb = NULL;

	xEventGroupSetBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete);

	while(1)
	{
		xQueueReceive(xQueue_pLines_busy_usb, &p_line_usb, portMAX_DELAY);

		if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED)
		{
			xEventGroupWaitBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete, pdFALSE, pdFALSE, 100);

			if (xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USB_LINE_TX_Complete)
			{
				xEventGroupClearBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete);

				if(CDC_Transmit_HS(p_line_usb, LINE_TRANS_LENGHT) == USBD_OK)
				{
					xEventGroupWaitBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete, pdFALSE, pdFALSE, 100);
				}
			}

			xEventGroupSetBits( xEventGroup_StatusFlags, Flag_USB_LINE_TX_Complete);
		}

		xQueueSend(xQueue_pLines_empty_usb, &p_line_usb, 0);

	}
}

/*
 *
 */

void vTask_USART_Service (void *pvParameters)
{
	uint32_t tx_timeout = 0;
	uint8_t num_data_send = 0;
	uint8_t byte = 0;

	for(;;)
	{
		xEventGroupWaitBits(xEventGroup_StatusFlags, Flag_USART_TX | Flag_USART_RX, pdFALSE, pdFALSE, portMAX_DELAY );

		if(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_RX)
		{
			xEventGroupClearBits(xEventGroup_StatusFlags, Flag_USART_RX);
		}

		if(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX)
		{
			tx_timeout = xTaskGetTickCount();
			num_data_send = 0;

			while(num_data_tx)
			{
				vTaskDelay(1);

				while ((USART1->ISR & USART_ISR_TXE_TXFNF) && num_data_tx)
				{
					USART1->TDR = *(data_tx_buffer + num_data_send);
					num_data_tx--;
					num_data_send++;

					while(!(USART1->ISR & USART_ISR_TC))
					{
						vTaskDelay(1);
					}

					USART1->ICR |= USART_ICR_TCCF;
				}


			}

			/*while(!(USART1->ISR & USART_ISR_TC))
			{
				vTaskDelay(1);
			}*/

			//xEventGroupSetBits(xEventGroup_StatusFlags, Flag_UART_TX_Ready);

			xEventGroupClearBits(xEventGroup_StatusFlags, Flag_USART_TX);
		}
	}
}

/*
 *
 */




/*
 *
 */

void tft_show_message(uint8_t msg)
{
	uint32_t protect_counter = HAL_GetTick();

	if (msg < 4)
	{
		while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
		{
			vTaskDelay(10);
		}

		if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
		{
			switch(msg)
			{
				case 0 : // Active
					sprintf((char*)data_tx_buffer, "page%u.n0.pco=0", active_page);
					break;
				case 1 : // Idle
					sprintf((char*)data_tx_buffer, "page%u.n0.pco=65520", active_page);
					break;
				case 2 :  // Protect
					sprintf((char*)data_tx_buffer, "page%u.n0.pco=65520", active_page);
					break;
				case 3 : // Dirty
					sprintf((char*)data_tx_buffer, "page%u.n0.pco=65520", active_page); //63488
					break;
			}

			num_data_tx = strlen((char*)data_tx_buffer);
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;

			xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
		}
	}

	if (msg < 6)
	{
		protect_counter = HAL_GetTick();

		while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
		{
			vTaskDelay(10);
		}

		if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
		{
			switch(msg)
			{
				case 0 : // Active
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\" \"", active_page);
					break;
				case 1 : // Idle
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\"Sleep\"", active_page);
					break;
				case 2 :  // Protect
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\"Protect\"", active_page);
					break;
				case 3 : // Dirty
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\"Clean Window\"", active_page);
					break;

				case 4 : // Dirty
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\"Inventory\"", active_page);
					break;
				case 5 : // Dirty
					sprintf((char*)data_tx_buffer, "page%u.t4.txt=\" \"", active_page);
					break;
			}

			num_data_tx = strlen((char*)data_tx_buffer);
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;

			xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
		}
	}


}

/*
 *
 */

void tft_show_overcount(uint16_t state)
{
	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		if(state)
		{
			sprintf((char*)data_tx_buffer, "page%u.n0.bco=63488", active_page);
		}
		else
		{
			sprintf((char*)data_tx_buffer, "page%u.n0.bco=65520", active_page);
		}
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}

	protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		if(state)
		{
			sprintf((char*)data_tx_buffer, "page%u.t4.txt=\"SLOWLY \"", active_page);
		}
		else
		{
			sprintf((char*)data_tx_buffer, "page%u.t4.txt=\" \"", active_page);
		}
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}


	protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		if(state)
		{
			sprintf((char*)data_tx_buffer, "page%u.wav0.en=1", active_page);
			num_data_tx = strlen((char*)data_tx_buffer);
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;
			data_tx_buffer[num_data_tx++] = 0xff;
			xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
		}
	}
}



/*
 *
 */

void tft_show_hide_counter(uint16_t state)
{
	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		if(state)
		{
			sprintf((char*)data_tx_buffer, "page%u.n0.pco=0", active_page);
			xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Counter_Not_Visible);
		}
		else
		{
			sprintf((char*)data_tx_buffer, "page%u.n0.pco=65520", active_page);
			xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Counter_Not_Visible);
		}
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}
}

/*
 *
 */

void tft_show_nun_pices(uint16_t num_pices)
{
	//xEventGroupWaitBits(xEventGroup_StatusFlags,Flag_UART_TX_Ready, pdFALSE, pdFALSE, portMAX_DELAY );
	//xEventGroupClearBits(xEventGroup_StatusFlags, Flag_UART_TX_Ready);

	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		sprintf((char*)data_tx_buffer, "page%u.n0.val=%u", active_page, num_pices);
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;

		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}

}

void tft_show_area_pices(uint16_t num_pice)
{
	//xEventGroupWaitBits(xEventGroup_StatusFlags,Flag_UART_TX_Ready, pdFALSE, pdFALSE, portMAX_DELAY );
	//xEventGroupClearBits(xEventGroup_StatusFlags, Flag_UART_TX_Ready);

	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		if(!num_pice)
		{
			sprintf((char*)data_tx_buffer, "page1.n1.val=%d", 0);
		}
		else
		{
			sprintf((char*)data_tx_buffer, "page1.n1.val=%d", Objects_area[num_pice-1]);
		}
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		sprintf((char*)(data_tx_buffer + num_data_tx), "page1.n2.val=%d", num_pice);
		num_data_tx += strlen((char*)(data_tx_buffer + num_data_tx));
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		sprintf((char*)(data_tx_buffer + num_data_tx), "page1.n3.val=%d", max_area);
		num_data_tx += strlen((char*)(data_tx_buffer + num_data_tx));
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		sprintf((char*)(data_tx_buffer + num_data_tx), "page1.n4.val=%d", midle_area);
		num_data_tx += strlen((char*)(data_tx_buffer + num_data_tx));
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}
}

/*
 *
 */

void tft_show_page(uint8_t page_num)
{
	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		sprintf((char*)data_tx_buffer, "page %d", page_num);
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}
}

void tft_send_click(uint8_t num_but, uint8_t event)
{
	uint32_t protect_counter = HAL_GetTick();

	while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
	{
		vTaskDelay(10);
	}

	if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
	{
		//sprintf((char*)data_tx_buffer, "click %u,%u", num_but, event);
		sprintf((char*)data_tx_buffer, "click bt1,%u", event);
		num_data_tx = strlen((char*)data_tx_buffer);
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		data_tx_buffer[num_data_tx++] = 0xff;
		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
	}
}

/*
 * *************** General Purpose Functions ***************
 */

/*
 *
 */

void StartScaner(void)
{
	//COMP1->CFGR |= COMP_CFGRx_INMSEL_0;

	dummy_scan_counter = INIT_CLEAR_TEST_SCAN_COUNTER_VALUE;
	clean_test_scan_counter = INIT_CLEAR_TEST_SCAN_COUNTER_VALUE;

	TIM17->CR1 |= TIM_CR1_CEN;
	TIM17->CCER = TIM_CCER_CC1E;

	TIM3->CR1 |= TIM_CR1_CEN;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
}

/*
 *
 */

void StopScaner(void)
{
	xEventGroupClearBits( xEventGroup_StatusFlags, Flag_Scaner_State);
//	xEventGroupSetBits(xEventGroup_StatusFlags_2, Flag_2_Need_Stop_Scaner);

	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);

	TIM17->CR1 &= ~TIM_CR1_CEN;
	TIM17->CCER &= ~TIM_CCER_CC1E;
}

/*
 *
 */

void Clear_Counter (void)
{
	uint8_t p;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

 	if (xSemaphoreTake(xSemaphoreMutex_Pice_Counter, 10) == pdTRUE)
	{
 		xEventGroupSetBits(xEventGroup_StatusFlags, Flag_Reset_lines_counters);

 		counter_num_extra_count = 0;
		numObjects = 0;
		num_show_object_area = 0;
		max_area = 3000;//max_area = 0;
		midle_area = 0;

		for (p=0; p < NUM_PICES_PERIOD; p++)
		{
			pices_time[p] = 0;
		}

		xSemaphoreGive(xSemaphoreMutex_Pice_Counter);

		 /*Configure GPIO pin : TIM3_CH2_LIGTH_BLUE_Pin */
		 GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_BLUE_Pin;
		 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		 GPIO_InitStruct.Pull = GPIO_NOPULL;
		 HAL_GPIO_Init(TIM3_CH2_LIGHT_BLUE_GPIO_Port, &GPIO_InitStruct);
		 HAL_GPIO_WritePin(TIM3_CH2_LIGHT_BLUE_GPIO_Port, TIM3_CH2_LIGHT_BLUE_Pin, 0);

		 /*Configure GPIO pin : TIM3_CH2_LIGTH_Pin */
		GPIO_InitStruct.Pin = TIM3_CH2_LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(TIM3_CH2_LIGHT_GPIO_Port, &GPIO_InitStruct);

		xEventGroupClearBits(xEventGroup_StatusFlags, Flag_Mode_Blue);

		uint32_t protect_counter = HAL_GetTick();

		while ((xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX) && ((HAL_GetTick() - protect_counter) < 1000))
		{
			vTaskDelay(10);
		}

		if (!(xEventGroupGetBits(xEventGroup_StatusFlags) & Flag_USART_TX))
		{
			sprintf((char*)data_tx_buffer, "page%u.bt1.val=0", active_page);
			num_data_tx = strlen((char*)data_tx_buffer);
			data_tx_buffer[num_data_tx] = 0xff;
			num_data_tx++;
			data_tx_buffer[num_data_tx] = 0xff;
			num_data_tx++;
			data_tx_buffer[num_data_tx] = 0xff;
			num_data_tx++;
			data_tx_buffer[num_data_tx] = 0x00;
			xEventGroupSetBits(xEventGroup_StatusFlags, Flag_USART_TX);
		}
	}
}

/*
 *
 */

void TimersTuning(void)
{
    TIM3->PSC = 279;
    TIM3->ARR = 400;
    TIM3->CCR1 = 50;
    TIM3->CCR2 = 60;
    TIM3->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC1FE | TIM_CCMR1_OC2FE;
    //TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->DIER = TIM_DIER_CC1IE;

    TIM17->PSC = 0;
    TIM17->ARR = 56;
    TIM17->CCR1 = 31;
    TIM17->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1FE;
    //TIM17->CCER = TIM_CCER_CC1E;
    TIM17->BDTR = TIM_BDTR_MOE;
    TIM17->DIER = TIM_DIER_CC1DE;
}

/*
 *
 */

void UARTsTunning(void)
{
	USART1->BRR = 14583; 			//  140 MHz / 9600 bout
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE|/* USART_CR1_FIFOEN |*/ USART_CR1_RXNEIE_RXFNEIE /*|  USART_CR1_TXFNFIE*/ /*| TCIE*/;

	USART3->BRR = 152; 			//  140 MHz / 921600 bout
	USART3->CR3 = USART_CR3_DMAT;
	USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/*
 *
 */

void ComparatorsTuning(void)
{
	//internal reference 1.180 - 1.216 - 1.255 V
		// 0000 = 1/4 VREF_COMP; 0001 = 1/2 VREF_COMP; 0010 = 3/4 VREF_COMP; 0011 = VREF_COMP; 0110 = COMP1_INM6 (PB1); 0111 = COMP1_INM7 (PC4)
    COMP1->CFGR = COMP_CFGRx_EN | COMP_CFGRx_HYST_1 | COMP_CFGRx_INMSEL_1 | COMP_CFGRx_INMSEL_2 | COMP_CFGRx_INPSEL ; //COMP1 enable , HighSpeed, Medium hysteresis, PB2 +, P1 -
    COMP2->CFGR = COMP_CFGRx_EN | COMP_CFGRx_HYST_1 | COMP_CFGRx_INMSEL_1 | COMP_CFGRx_INMSEL_2 | COMP_CFGRx_INPSEL ; //COMP1 enable , HighSpeed, Medium hysteresis, PB2 +, P1 -
}

/*
 *
 */

void SystemInterruptsTuning(void)
{
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 10);

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 11);

    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(DMA1_Stream0_IRQn, 10);
}

/*
 *
 */

void DMATuning(void)
{
    // for change addresses DMA chanal must disable !!!

	DMA1_Stream0->PAR = (uint32_t)&COMP12->SR;
	DMA1_Stream0->M0AR = (uint32_t)BufferCOMP1;
	DMA1_Stream0->NDTR = LINE_SIZE_WITH_DUMMY;
	DMA1_Stream0->CR = DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_PL_0 | DMA_SxCR_PL_1;
	DMAMUX1_Channel0->CCR = ( 111 << DMAMUX_CxCR_DMAREQ_ID_Pos);
}

/*
 *
 */

void TIM3_IRQHandler(void)
{
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->SR &= ~TIM_SR_CC1IF;

    DMA1_Stream0->NDTR = LINE_SIZE_WITH_DUMMY;
    DMA1_Stream0->CR |= DMA_SxCR_EN;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/*
 *
 *
 */

void DMA1_Stream0_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;
    uint32_t r = 0, k = 0;

    DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;

    xHigherPriorityTaskWoken = pdFALSE;

  //  if (xEventGroupGetBitsFromISR(xEventGroup_2_StatusFlags) & Flag_2_Need_Stop_Scaner)
//	{
//		TIM3->CR1 &= ~TIM_CR1_CEN;
//		TIM17->CR1 &= ~TIM_CR1_CEN;
//		xEventGroupClearBitsFromISR(xEventGroup_2_StatusFlags, Flag_2_Need_Stop_Scaner);
//	}
//	else
//	{
		uint32_t *p_line = NULL;

		if (!start_time)
		{
			start_time = xTaskGetTickCount();
		}

		if (xEventGroupGetBitsFromISR(xEventGroup_StatusFlags) & Flag_Reset_lines_counters)
		{
			xEventGroupClearBitsFromISR(xEventGroup_StatusFlags, Flag_Reset_lines_counters);

			global_lines_counter = 0;
			queue_polling_lines_counter = 0;
			queue_send_lines_counter = 0;
			skip_lines_counter_2 = 0;
			skip_lines_counter = 0;
		}

		if (xQueueReceiveFromISR(xQueue_pLines_empty, &p_line, &xHigherPriorityTaskWoken) == pdTRUE)
		{
			for (uint32_t y=LINE_DUMMY, z=0; y < (LINE_SIZE + LINE_DUMMY); y += PIXEL_DIVIDER, z +=1)
			{
				*(p_line + z) = BufferCOMP1[y];
			}

			if (xQueueSendFromISR(xQueue_pLines_busy, &p_line, &xHigherPriorityTaskWoken) == pdTRUE)
			{
				queue_send_lines_counter++;
			}
			else
			{
				skip_lines_counter_2++;
			}
		}
		else
		{
			skip_lines_counter++;
		}
//	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 *
 */

void DMA1_Stream1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken, xResult;

	DMA1->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1;

	xHigherPriorityTaskWoken = pdFALSE;

	xResult = xEventGroupSetBitsFromISR(xEventGroup_StatusFlags, Flag_UART_LINE_TX_Complete, &xHigherPriorityTaskWoken);

	if(xResult != pdFAIL)
	{
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

/*
 *
 */

void USART1_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken= pdFALSE;
	uint8_t rx_byte = 0;

	/*while(USART1->ISR & USART_ISR_RXNE_RXFNE)
	{
		rx_byte = USART1->RDR;

		xQueueSendFromISR(xQueue_RX_uart, &rx_byte, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);*/

	if (USART1->ISR & USART_ISR_RXNE_RXFNE)
	{
		if (!(xEventGroupGetBitsFromISR(xEventGroup_StatusFlags) & Flag_UART_RX_Buffer_Busy))
		{
			if (!uart_rx_timeout)
			{
				uart_rx_timeout = 1;
				uart_rx_buffer_pointer = 0;
			}

			while(USART1->ISR & USART_ISR_RXNE_RXFNE)
			{
				uart_rx_buffer[uart_rx_buffer_pointer] = USART1->RDR;

				if (uart_rx_buffer[uart_rx_buffer_pointer] == 0xff)
				{
					uart_rx_timeout++;
				}

				if (uart_rx_buffer_pointer < sizeof(uart_rx_buffer))
				{
					uart_rx_buffer_pointer++;
				}
			}

			if (uart_rx_timeout >= 4)
			{
				uart_rx_timeout = 0;
				xEventGroupSetBitsFromISR(xEventGroup_StatusFlags, Flag_UART_RX_Buffer_Busy, &xHigherPriorityTaskWoken);
			}
		}
		else
		{
			while(USART1->ISR & USART_ISR_RXNE_RXFNE)
			{
				rx_byte = USART1->RDR;
			}
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
  if (htim->Instance == TIM1) {
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
