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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <bme280.h>
#include <max30102_for_stm32_hal.h>
#include <algorithm_by_rf.h>
#include "FreeRTOS.h"
#include "semphr.h"
uint8_t txByte;
uint8_t rxByte;
max30102_t max30102;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TX_queue */
osMessageQueueId_t TX_queueHandle;
const osMessageQueueAttr_t TX_queue_attributes = {
  .name = "TX_queue"
};
/* Definitions for RX_queue */
osMessageQueueId_t RX_queueHandle;
const osMessageQueueAttr_t RX_queue_attributes = {
  .name = "RX_queue"
};
/* Definitions for myQueue03 */
osMessageQueueId_t myQueue03Handle;
const osMessageQueueAttr_t myQueue03_attributes = {
  .name = "myQueue03"
};
/* Definitions for myTimer03 */
osTimerId_t myTimer03Handle;
const osTimerAttr_t myTimer03_attributes = {
  .name = "myTimer03"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
const osMutexAttr_t myMutex02_attributes = {
  .name = "myMutex02"
};
/* Definitions for myMutex03 */
osMutexId_t myMutex03Handle;
const osMutexAttr_t myMutex03_attributes = {
  .name = "myMutex03"
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
const osSemaphoreAttr_t myBinarySem01_attributes = {
  .name = "myBinarySem01"
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
const osSemaphoreAttr_t myBinarySem02_attributes = {
  .name = "myBinarySem02"
};
/* Definitions for myBinarySem03 */
osSemaphoreId_t myBinarySem03Handle;
const osSemaphoreAttr_t myBinarySem03_attributes = {
  .name = "myBinarySem03"
};
/* Definitions for myBinarySem04 */
osSemaphoreId_t myBinarySem04Handle;
const osSemaphoreAttr_t myBinarySem04_attributes = {
  .name = "myBinarySem04"
};
/* Definitions for myBinarySem05 */
osSemaphoreId_t myBinarySem05Handle;
const osSemaphoreAttr_t myBinarySem05_attributes = {
  .name = "myBinarySem05"
};
/* USER CODE BEGIN PV */

SemaphoreHandle_t sem_btn;
SemaphoreHandle_t sem_motor;
TaskHandle_t bmeTaskHandle;
void bme_task(void *argument);
#define RX_BUF_SIZE 64
#define MAX_LINE    64
#define TIMEOUT_MS  5000
uint8_t rxByte;
char rxLine[MAX_LINE];
uint8_t rxIndex = 0;


// channel button
typedef struct {
    GPIO_TypeDef *btn_port;
    uint16_t btn_pin;
    uint8_t channel_index;
} ButtonMap;

ButtonMap buttons[] = {
    {GPIOB, GPIO_PIN_10, 0},
    {GPIOB, GPIO_PIN_11, 1},
    {GPIOB, GPIO_PIN_0, 2},
    {GPIOB, GPIO_PIN_1, 3},
};
#define NUM_BUTTONS (sizeof(buttons)/sizeof(buttons[0]))

#define PWM_50_PERCENT  512
#define PWM_100_PERCENT 1023
#define PWM_STOP 0

// channel motor
typedef struct {
    const char *cmd;
    GPIO_TypeDef *port_out;
    uint16_t pin_out;

    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;

    GPIO_TypeDef *port_in;
    uint16_t pin_in;

    uint8_t waiting;
    uint32_t start_time;
} GpioChannel;


GpioChannel channels[] = {
    {"CMD_HEAD_UP", GPIOA, GPIO_PIN_6, &htim3, TIM_CHANNEL_1, GPIOA, GPIO_PIN_4, 0, 0},

    {"CMD_HEAD_DOWN", GPIOA, GPIO_PIN_7, &htim3, TIM_CHANNEL_2, GPIOA, GPIO_PIN_5, 0, 0},

    {"CMD_TAIL_UP", GPIOA, GPIO_PIN_1, &htim2, TIM_CHANNEL_2, GPIOB, GPIO_PIN_3, 0, 0},

    {"CMD_TAIL_DOWN", GPIOA, GPIO_PIN_2, &htim2, TIM_CHANNEL_3, GPIOB, GPIO_PIN_9, 0, 0},
};

#define NUM_CHANNELS (sizeof(channels)/sizeof(channels[0]))

osTimerId_t timers[NUM_CHANNELS];

// interrup ngắt ĐC
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        if (GPIO_Pin == channels[i].pin_in)
        {
        	//osSemaphoreRelease(myBinarySem01Handle);
        	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        	xSemaphoreGiveFromISR(sem_motor, &xHigherPriorityTaskWoken);

        	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        }
    }
    for (int i = 0; i < NUM_BUTTONS; i++)
    {
    	if(GPIO_Pin ==buttons[i].btn_pin)
    	{
    		//osSemaphoreRelease(myBinarySem02Handle);
    		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    		xSemaphoreGiveFromISR(sem_btn, &xHigherPriorityTaskWoken);

    		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    	}
    }
    if (GPIO_Pin == GPIO_PIN_8)
    {
    	max30102_on_interrupt(&max30102);
    	osSemaphoreRelease(myBinarySem04Handle);
    	return;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        osSemaphoreRelease(myBinarySem03Handle);
    }
}

typedef struct {
    char cmd[MAX_LINE];
} CommandMsg_t;

typedef enum {
    PART_UP      = 1,
    PART_DOWN    = 0,
} PartStatus;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rxByte == '\r' || rxByte == '\n')
        {
            if (rxIndex > 0)
            {
                rxLine[rxIndex] = '\0';

                CommandMsg_t msg;
                strcpy(msg.cmd, rxLine);

                osMessageQueuePut(RX_queueHandle, &msg, 0, 0);

                rxIndex = 0;
            }
        }
        else
        {
            if (rxByte >= 32 && rxByte <= 126)  // 🔥 lọc ký tự
            {
                if (rxIndex < MAX_LINE - 1)
                {
                    rxLine[rxIndex++] = rxByte;
                }
            }
        }
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void TX_UART(void *argument);
void button_dc(void *argument);
void RX_UART(void *argument);
void interrup_dc(void *argument);
void sensor(void *argument);
void Callback02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sendUart(char *str)
{
    while (*str)
    {
        uint16_t data = (uint16_t)(*str++);
        osMessageQueuePut(TX_queueHandle, &data, 0, osWaitForever);
    }
}

void TimeoutCallback(void *argument)
{
    int i = (int)argument;
    if (HAL_GPIO_ReadPin(channels[i].port_in, channels[i].pin_in)== RESET)
    {
    	__HAL_TIM_SET_COMPARE(channels[i].htim, channels[i].tim_channel, PWM_STOP);
        HAL_TIM_PWM_Stop(channels[i].htim, channels[i].tim_channel);
        channels[i].waiting = 0;
        sendUart("CHECK=0\n");
    }
}

/*for (int i = 0; i < NUM_CHANNELS; i++)
{
    timers[i] = osTimerNew(TimeoutCallback, osTimerOnce, (void*)i, NULL);
}*/

/*HAL_UART_Receive_IT(&huart1, &rxByte, 1);
osTimerStart(myTimer03Handle, 10000);
int_max30102();*/

typedef struct {
    float temp;
    float hum;
} SensorMsg_t;


// cảm biến nhịp tim
 void int_max30102(){
	  max30102_init(&max30102, &hi2c1);
	  max30102_reset(&max30102);
	  max30102_clear_fifo(&max30102);
	  max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7);

	  // Sensor settings
	  max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
	  max30102_set_adc_resolution(&max30102, max30102_adc_2048);
	  max30102_set_sampling_rate(&max30102, max30102_sr_50);
	  max30102_set_led_current_1(&max30102, 6.2);
	  max30102_set_led_current_2(&max30102, 6.2);

	  // Enter SpO2 mode
	  max30102_set_mode(&max30102, max30102_spo2);
	  max30102_set_a_full(&max30102, 1);
	  max30102_set_ppg_rdy(&max30102, 1);
	  // Initiate 1 temperature measurement
	  max30102_set_die_temp_en(&max30102, 1);
	  max30102_set_die_temp_rdy(&max30102, 1);

	  uint8_t en_reg[2] = {0};
	  max30102_read(&max30102, 0x00, en_reg, 1);
 }

 #define FILTER_WINDOW 8

 typedef struct {
     uint32_t ir;
     uint32_t red;
 } MaxSample_t;

 osMessageQueueId_t maxQueue;

 void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
 {
	    MaxSample_t sample = {
	        .ir = ir_sample,
	        .red = red_sample
	    };
	    osMessageQueuePut(myQueue03Handle, &sample, 0, 0);
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* creation of myMutex02 */
  myMutex02Handle = osMutexNew(&myMutex02_attributes);

  /* creation of myMutex03 */
  myMutex03Handle = osMutexNew(&myMutex03_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */

  myBinarySem01Handle = osSemaphoreNew(1, 0, &myBinarySem01_attributes);

  myBinarySem02Handle = osSemaphoreNew(1, 0, &myBinarySem02_attributes);

  myBinarySem03Handle = osSemaphoreNew(1, 0, &myBinarySem03_attributes);

  myBinarySem04Handle = osSemaphoreNew(1, 0, &myBinarySem04_attributes);

  myBinarySem05Handle = osSemaphoreNew(1, 0, &myBinarySem05_attributes);

  sem_btn   = xSemaphoreCreateBinary();
  sem_motor = xSemaphoreCreateBinary();

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of myTimer03 */
  myTimer03Handle = osTimerNew(Callback02, osTimerPeriodic, NULL, &myTimer03_attributes);

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
      timers[i] = osTimerNew(TimeoutCallback, osTimerOnce, (void*)i, NULL);
  }
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of TX_queue */
  TX_queueHandle = osMessageQueueNew (128, sizeof(uint16_t), &TX_queue_attributes);

  /* creation of RX_queue */
  RX_queueHandle = osMessageQueueNew (16, sizeof(CommandMsg_t), &RX_queue_attributes);

  /* creation of myQueue03 */
  myQueue03Handle = osMessageQueueNew (128, sizeof(MaxSample_t), &myQueue03_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(TX_UART, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(button_dc, NULL, &myTask02_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(RX_UART, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(interrup_dc, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(sensor, NULL, &myTask06_attributes);

  xTaskCreate(bme_task, "bme_task", 256, NULL, 2, &bmeTaskHandle );

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  HAL_UART_Receive_IT(&huart1, &rxByte, 1);
  osTimerStart(myTimer03Handle, 60000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB3 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_TX_UART */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TX_UART */
void TX_UART(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint16_t data;
  for (;;)
  {
      if (osMessageQueueGet(TX_queueHandle, &data, NULL, osWaitForever) == osOK)
      {
          txByte = (uint8_t)data;
          osMutexAcquire(myMutex01Handle, osWaitForever);
          HAL_UART_Transmit_IT(&huart1, &txByte, 1);
          osSemaphoreAcquire(myBinarySem03Handle, osWaitForever);
          osMutexRelease(myMutex01Handle);
      }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_button_dc */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_button_dc */

int channel_busy(void)
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        if (channels[i].waiting)
            return 1;
    }
    return 0;
}

void button_dc(void *argument)
{
  /* USER CODE BEGIN button_dc */
  /* Infinite loop */
  for(;;)
  {
	//osSemaphoreAcquire(myBinarySem02Handle, osWaitForever);
	xSemaphoreTake(sem_btn, portMAX_DELAY);
	for (int i = 0; i < NUM_BUTTONS; i++)
	{
		if ((HAL_GPIO_ReadPin(buttons[i].btn_port, buttons[i].btn_pin) == GPIO_PIN_RESET) &&
				((HAL_GPIO_ReadPin(channels[i].port_in, channels[i].pin_in) == GPIO_PIN_SET)))
	    {
			if (!channel_busy())
	        {
				HAL_TIM_PWM_Start(channels[i].htim, channels[i].tim_channel);

	            __HAL_TIM_SET_COMPARE(channels[i].htim, channels[i].tim_channel, PWM_100_PERCENT);
	            channels[i].waiting = 1;

	            // CHECKTIMEOUT TIMER
	            osTimerStart(timers[i], TIMEOUT_MS);
	        }
	     }
	}
  }
  /* USER CODE END button_dc */
}

/* USER CODE BEGIN Header_RX_UART */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RX_UART */
void RX_UART(void *argument)
{
  /* USER CODE BEGIN RX_UART */
  /* Infinite loop */
    CommandMsg_t rxmsg;

    for(;;)
    {
        if (osMessageQueueGet(RX_queueHandle, &rxmsg, NULL, osWaitForever) == osOK)
        {
            uint8_t found = 0;

            if (strcmp(rxmsg.cmd, "SYNC") == 0)
            {
                PartStatus headStatus, tailStatus;
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
                    headStatus = PART_UP;
                else
                    headStatus = PART_DOWN;
                if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET)
                    tailStatus = PART_UP;
                else
                    tailStatus = PART_DOWN;
                char msg[32];
                sprintf(msg, "STATUS=%d,%d\n", headStatus, tailStatus);
                sendUart(msg);
                found = 1;
            }

            /*for (int i = 0; i < NUM_CHANNELS; i++)
            {
                if (strcmp(rxmsg.cmd, channels[i].cmd) == 0)
                {
                    if (channels[i].waiting)
                    {
                        sendUart("BUSY\n");
                        found = 1;
                        break;
                    }

                    HAL_TIM_PWM_Start(channels[i].htim, channels[i].tim_channel);
                    __HAL_TIM_SET_COMPARE(channels[i].htim, channels[i].tim_channel, PWM_50_PERCENT);

                    sendUart("OK_CMD\n");

                    channels[i].waiting = 1;
                    channels[i].start_time = HAL_GetTick();
                    found = 1;
                    break;
                }
            }*/

            if (!found)
                sendUart("ERR_CMD\n");
        }
    }
  /* USER CODE END RX_UART */
}

/* USER CODE BEGIN Header_interrup_dc */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_interrup_dc */
void interrup_dc(void *argument)
{
  /* USER CODE BEGIN interrup_dc */
  /* Infinite loop */
  for(;;)
  {
	//osSemaphoreAcquire(myBinarySem01Handle, osWaitForever);
	xSemaphoreTake(sem_motor, portMAX_DELAY);
	for (int i = 0; i < NUM_CHANNELS; i++)
	{
		if (HAL_GPIO_ReadPin(channels[i].port_in, channels[i].pin_in) == GPIO_PIN_RESET && channels[i].waiting)
		{
			__HAL_TIM_SET_COMPARE(channels[i].htim, channels[i].tim_channel, PWM_STOP);
	        HAL_TIM_PWM_Stop(channels[i].htim, channels[i].tim_channel);
	        channels[i].waiting = 0;
	        break;
		}
	}
  }
  /* USER CODE END interrup_dc */
}

/* USER CODE BEGIN Header_sensor */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor */
void sensor(void *argument)
{
  /* USER CODE BEGIN sensor */

  /* Infinite loop */
    /* ===== INIT SENSOR ===== */
    vTaskDelay(pdMS_TO_TICKS(1000));
    int_max30102();
    sendUart("MAX30102 OK\r\n");

    /* ===== BUFFER ===== */
    MaxSample_t sample;

    static uint32_t ir_buffer[BUFFER_SIZE];
    static uint32_t red_buffer[BUFFER_SIZE];

    static uint32_t ir_filter_buf[FILTER_WINDOW] = {0};
    static uint32_t red_filter_buf[FILTER_WINDOW] = {0};

    static uint32_t ir_sum = 0;
    static uint32_t red_sum = 0;

    static uint8_t filter_index = 0;
    static uint32_t sample_count = 0;

    float spo2;
    int8_t spo2_valid;
    int8_t hr_valid;
    int32_t heart_rate;
    float ratio, correl;

    /* ===== LOOP ===== */
    for(;;)
    {
        /* chờ interrupt từ MAX30102 */
        if(osSemaphoreAcquire(myBinarySem04Handle, osWaitForever) == osOK)
        {
            osMutexAcquire(myMutex02Handle, osWaitForever);
            max30102_interrupt_handler(&max30102);
            osMutexRelease(myMutex02Handle);
        }
        if(osMessageQueueGet(myQueue03Handle, &sample, NULL, 0) == osOK)
        {
            /* moving average filter */
            ir_sum  -= ir_filter_buf[filter_index];
            red_sum -= red_filter_buf[filter_index];

            ir_filter_buf[filter_index]  = sample.ir;
            red_filter_buf[filter_index] = sample.red;

            ir_sum  += sample.ir;
            red_sum += sample.red;

            filter_index++;
            if(filter_index >= FILTER_WINDOW)
                filter_index = 0;

            uint32_t ir_avg  = ir_sum  / FILTER_WINDOW;
            uint32_t red_avg = red_sum / FILTER_WINDOW;

            if(ir_avg < 5000)
            {
                sample_count = 0;
                continue;
            }

            if(sample_count < BUFFER_SIZE)
            {
                ir_buffer[sample_count]  = ir_avg;
                red_buffer[sample_count] = red_avg;
                sample_count++;
            }

            if(sample_count >= BUFFER_SIZE)
            {
                rf_heart_rate_and_oxygen_saturation(ir_buffer, BUFFER_SIZE, red_buffer, &spo2, &spo2_valid, &heart_rate, &hr_valid, &ratio, &correl);

                if(hr_valid && spo2_valid)
                {
                    char msg[64];
                    sprintf(msg, "BPM=%ld, SpO2=%.0f%%\r\n", heart_rate, spo2);
                    sendUart(msg);
                }
                sample_count = 0;
            }
        }
    vTaskDelay(10);
    }
  /* USER CODE END sensor */
}
void bme_task(void *argument)
{
    BME280_HandleTypedef bme = {
        .hi2c = &hi2c1,
        .dev_addr = 0x76 << 1
    };

    osMutexAcquire(myMutex03Handle, osWaitForever);
    if (!BME280_Init(&bme)) {
        sendUart("BME INIT FAIL\n");
    } else {
        sendUart("BME INIT OK\n");
    }
    osMutexRelease(myMutex03Handle);

    for (;;)
    {
        osSemaphoreAcquire(myBinarySem05Handle, osWaitForever);

        float temp, hum;

        osMutexAcquire(myMutex03Handle, osWaitForever);

        if (BME280_ReadTemperature(&bme, &temp) &&
            BME280_ReadHumidity(&bme, &hum))
        {
            char msg[64];
            sprintf(msg, "T=%.1f H=%.1f\r\n", temp, hum);
            sendUart(msg);
        }

        osMutexRelease(myMutex03Handle);
    }
}
/* Callback02 function */
void Callback02(void *argument)
{
  /* USER CODE BEGIN Callback02 */
	osSemaphoreRelease(myBinarySem05Handle);
  /* USER CODE END Callback02 */
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
