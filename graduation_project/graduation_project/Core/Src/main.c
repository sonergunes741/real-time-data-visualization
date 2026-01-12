/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (ROBUST VERSION)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

/* Extern declarations for FreeRTOS port functions */
extern void xPortSysTickHandler(void);
extern void xPortPendSVHandler(void);
extern void vPortSVCHandler(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId SensorTaskHandle;
/* USER CODE BEGIN PV */

// Operation Mode: 'F' = FreeRTOS, 'B' = Bare Metal
volatile char currentMode = 'F';

// UART Receive
volatile uint8_t rxByte = 0;
volatile uint8_t rxComplete = 0;

// Timestamp (ms)
volatile uint32_t systemTimestamp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */
void delay_us(uint16_t us);
uint32_t HCSR04_Read(void);
void vProducerTask(void *pvParameters);   // Priority 3 - sensor read + validation
void vConsumerTask(void *pvParameters);   // Priority 1 - data display
void vAlertTask(void *pvParameters);      // Priority 3 - critical alert handler
void RunBareMetalMode(void);
void ProcessUARTCommand(uint8_t cmd);
void SendSensorData(uint32_t distance, uint32_t latency);

// Double buffer for Producer-Consumer (lock-free)
typedef struct {
    uint32_t distance;
    uint32_t timestamp;
    uint8_t ready;
} SensorBuffer;

volatile SensorBuffer sharedBuffer = {0, 0, 0};

// Critical threshold detection (immediate trigger, no spike protection)
volatile uint32_t criticalThreshold = 10;   // Default 10cm, user configurable
volatile uint8_t alertActive = 0;           // Alert state
volatile uint32_t alertStartTime = 0;       // When alert started
volatile uint32_t alertSampleCount = 0;     // Total samples during alert
volatile uint8_t alertAcknowledged = 0;     // User acknowledged

// Semaphore for Alert Task
SemaphoreHandle_t xAlertSemaphore = NULL;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Microsecond delay (using TIM1)
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

// Ultrasonic Sensor Read - Simple and reliable
uint32_t HCSR04_Read(void)
{
    uint32_t time = 0;
    uint32_t timeout = 1000000;

    // Trigger pulse
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

    // Wait for Echo HIGH
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
        if (timeout-- == 0) return 0;
    }

    // Start measurement
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    // Wait for Echo LOW
    timeout = 1000000;
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) {
        if (timeout-- == 0) return 0;
    }

    time = __HAL_TIM_GET_COUNTER(&htim1);
    return time / 58; // Convert to cm
}

// Format and send sensor data
void SendSensorData(uint32_t distance, uint32_t latency)
{
    char buffer[80];
    // Format: MODE:X,TIME:XXXXXX,DIST:XXX,LAT:XXX\n
    sprintf(buffer, "MODE:%c,TIME:%lu,DIST:%lu,LAT:%lu\r\n", 
            currentMode, systemTimestamp, distance, latency);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}

// Read mode from Backup Register
char ReadModeFromBackup(void) {
    // Enable clocks (required for reading)
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    
    // Read register directly
    uint16_t val = BKP->DR1;
    if ((val & 0xFF) == 'B') return 'B';
    return 'F'; // Default
}

// Write mode to Backup Register
void WriteModeToBackup(char mode) {
    // Enable clocks and disable write protection
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    
    // Write register directly
    BKP->DR1 = (uint16_t)mode;
    
    // Ensure write completion
    __DSB(); // Data Synchronization Barrier
}

// Process UART Command
void ProcessUARTCommand(uint8_t cmd)
{
    char response[50];
    
    switch(cmd) {
        case 'F':
        case 'f':
            WriteModeToBackup('F');
            sprintf(response, "ACK:MODE_FREERTOS\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            HAL_Delay(50); // Wait for UART/BKP completion
            NVIC_SystemReset();
            break;
            
        case 'B':
        case 'b':
            WriteModeToBackup('B');
            sprintf(response, "ACK:MODE_BAREMETAL\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            HAL_Delay(50); // Wait for UART/BKP completion
            NVIC_SystemReset();
            break;
            
        case 'S':
        case 's':
            sprintf(response, "STATUS:MODE_%c,UPTIME:%lu\r\n", currentMode, systemTimestamp);
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        
        case 'A':
        case 'a':
            // Acknowledge alert from UI
            alertAcknowledged = 1;
            sprintf(response, "ACK:ALERT_ACK\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        
        // Threshold commands: 5, 10, 15, 20, 25, 30 cm
        case 'T':
            criticalThreshold = 5;
            sprintf(response, "ACK:THRESHOLD_5\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        case 'U':
            criticalThreshold = 10;
            sprintf(response, "ACK:THRESHOLD_10\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        case 'V':
            criticalThreshold = 15;
            sprintf(response, "ACK:THRESHOLD_15\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        case 'W':
            criticalThreshold = 20;
            sprintf(response, "ACK:THRESHOLD_20\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        case 'X':
            criticalThreshold = 25;
            sprintf(response, "ACK:THRESHOLD_25\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
        case 'Y':
            criticalThreshold = 30;
            sprintf(response, "ACK:THRESHOLD_30\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
            break;
            
        default:
            break;
    }
}
/* ... Sensor fonksiyonları ... satırı hatası düzeltildi */

// ========== PRODUCER TASK (PRIORITY 3 - HIGHEST) ==========
// Reads sensor, validates critical threshold, never blocks
// Uses double buffer - no mutex, no priority inversion
void vProducerTask(void *pvParameters)
{
    for(;;)
    {
        uint32_t startTime = HAL_GetTick();
        
        // Read sensor (~3ms)
        uint32_t distance = HCSR04_Read();
        
        // Write to shared buffer (lock-free)
        sharedBuffer.distance = distance;
        sharedBuffer.timestamp = startTime;
        sharedBuffer.ready = 1;
        
        // === CRITICAL THRESHOLD VALIDATION ===
        // Triggers immediately when threshold crossed - no spike protection
        if (distance < criticalThreshold && distance > 0) {
            if (!alertActive) {
                alertActive = 1;
                alertStartTime = HAL_GetTick();
                alertSampleCount = 1;
                alertAcknowledged = 0;
                
                // Wake Alert Task immediately
                if (xAlertSemaphore != NULL) {
                    xSemaphoreGive(xAlertSemaphore);
                }
            } else {
                alertSampleCount++;
            }
        }
        
        // LED toggle
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========== CONSUMER TASK (LOW PRIORITY) ==========
// Reads from buffer, calculates latency, sends to UART
// Same priority as dummy tasks, so gets delayed when busy-wait is active!
void vConsumerTask(void *pvParameters)
{
    for(;;)
    {
        // Check if data is ready
        if(sharedBuffer.ready) {
            // Read data
            uint32_t distance = sharedBuffer.distance;
            uint32_t produceTime = sharedBuffer.timestamp;
            sharedBuffer.ready = 0;
            
            // NOW - Producer Write Time = REAL LATENCY
            // Latency = time from producer write to consumer read
            uint32_t consumeTime = HAL_GetTick();
            uint32_t latency = consumeTime - produceTime;
            
            systemTimestamp = consumeTime;
            
            SendSensorData(distance, latency);
            
            // Check UART commands
            if(rxComplete) {
                ProcessUARTCommand(rxByte);
                rxComplete = 0;
                HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxByte, 1);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ========== ALERT TASK (PRIORITY 3 - HIGHEST) ==========
// Sleeps until woken by Producer when critical threshold exceeded
// Sends alert to UI immediately, never blocked by Consumer
void vAlertTask(void *pvParameters)
{
    char alertMsg[128];
    
    for(;;)
    {
        // Wait for Producer to wake us (blocking wait - saves CPU)
        if (xSemaphoreTake(xAlertSemaphore, portMAX_DELAY) == pdTRUE) {
            
            // Send alert message to UI
            while (alertActive && !alertAcknowledged) {
                uint32_t duration = HAL_GetTick() - alertStartTime;
                
                sprintf(alertMsg, "ALERT:SAMPLES:%lu,DURATION:%lu,THRESHOLD:%lu\r\n", 
                        alertSampleCount, duration, criticalThreshold);
                HAL_UART_Transmit(&huart2, (uint8_t*)alertMsg, strlen(alertMsg), 100);
                
                // Update every 200ms while alert is active
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            
            // Alert acknowledged by user, send confirmation
            if (alertAcknowledged) {
                sprintf(alertMsg, "ALERT:CLEARED\r\n");
                HAL_UART_Transmit(&huart2, (uint8_t*)alertMsg, strlen(alertMsg), 100);
                
                // Reset alert state
                alertActive = 0;
                alertSampleCount = 0;
            }
        }
    }
}

// Bare Metal Mode - Runs without FreeRTOS
void RunBareMetalMode(void)
{
    uint32_t distance;
    uint32_t startTime, endTime, latency;
    
    // Startup message
    char msg[] = "BARE_METAL_MODE_STARTED\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    
    // Start UART RX interrupt
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxByte, 1);
    
    while(1)
    {
        // Start time
        startTime = HAL_GetTick();
        
        // Read sensor
        distance = HCSR04_Read();
        
        // End time and latency
        endTime = HAL_GetTick();
        latency = endTime - startTime;
        
        // Update timestamp
        systemTimestamp = HAL_GetTick();
        
        // Send data
        SendSensorData(distance, latency);
        
        // LED toggle
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        
        // Check UART commands
        if(rxComplete) {
            ProcessUARTCommand(rxByte);
            rxComplete = 0;
            HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxByte, 1);
        }
        
        // Wait 100ms
        HAL_Delay(100);
    }
}

// UART RX Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) {
        rxComplete = 1;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  
  // Enable Power Clock and Backup Access (must be done early)
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  // DEBUG: Read raw BKP value
  uint16_t bkpRaw = BKP->DR1;
  
  // Read mode from Backup Register
  currentMode = ReadModeFromBackup();
  
  // DEBUG: Display BKP value
  char debugMsg[80];
  sprintf(debugMsg, "DEBUG:BKP_RAW=0x%04X,MODE=%c\r\n", bkpRaw, currentMode);
  HAL_UART_Transmit(&huart2, (uint8_t*)debugMsg, strlen(debugMsg), 100);
  
  // Startup message
  char startMsg[60];
  sprintf(startMsg, "STM32_READY:MODE_%c\r\n", currentMode);
  HAL_UART_Transmit(&huart2, (uint8_t*)startMsg, strlen(startMsg), 100);
  
  // Start UART RX interrupt
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxByte, 1);
  
  // Run based on selection
  if(currentMode == 'B') {
    // Bare Metal Mode
    RunBareMetalMode();  // Does not return
  }
  else {
    // FreeRTOS Mode (Default)
    // Default to 'F' if invalid
    currentMode = 'F';
    
    // PRODUCER Task - HIGH PRIORITY (3)
    // Reads sensor, writes to buffer with timestamp
    xTaskCreate(
      vProducerTask,
      "Producer",
      256,
      NULL,
      3,                // HIGH PRIORITY
      NULL
    );
    
    // CONSUMER Task - Priority 1 (Low)
    xTaskCreate(
      vConsumerTask,
      "Consumer",
      256,
      NULL,
      1,
      NULL
    );
    
    // Create Alert Semaphore
    xAlertSemaphore = xSemaphoreCreateBinary();
    
    // ALERT Task - Priority 3 (High, same as Producer)
    // Sleeps until critical threshold exceeded
    xTaskCreate(
      vAlertTask,
      "Alert",
      256,
      NULL,
      3,
      NULL
    );

    // Start FreeRTOS Scheduler
    vTaskStartScheduler();
  }

  // Should never reach here
  while(1) { }
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
  /* definition and creation of SensorTask */
  osThreadDef(SensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  SensorTaskHandle = osThreadCreate(osThread(SensorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  // FreeRTOS tick handler - generate tick with TIM2
  if (htim->Instance == TIM2)
  {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
      xPortSysTickHandler();
    }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  while (1)
  {
    // Fast blink - error indicator
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    for(volatile uint32_t i = 0; i < 50000; i++);
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
