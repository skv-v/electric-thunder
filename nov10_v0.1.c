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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define HOMING_STEP_DEGREE 5.0
#define HOMING_DELAY_TIME_US 1000
#define MOTOR_DELAY_TIME_US 1000
#define HOMING_CHECK_DELAY_MS 5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_COUNT 6  // Number of motors
#define LIMIT_SWITCH_COUNT 6  // Number of limit switches 
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    GPIO_TypeDef* dirPort;          // Direction pin port
    uint16_t dirPin;                // Direction pin
    GPIO_TypeDef* pulPort;          // Step pin port
    uint16_t pulPin;                // Step pin
    int totalPulsesPerRevolution;   // Pulses per revolution
    float pulsesPerMM;              // Pulses per mm (specific to linear motors)
    float accumulatedRemainder;     // To handle fractional pulses
    uint8_t isLinear;               // Flag to indicate if the motor is linear (1 for linear, 0 for rotational)
} MotorConfig;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int pulse_count = 0;
float motor_min_limits[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Min angles
float motor_max_limits[MOTOR_COUNT] = {200.0, 90.0, 120.0, 550.0, 60.0, 300.0}; // Max angles

// Motor direction configuration (0 means towards limit switch, 1 means away from limit switch)
int motor_directions[MOTOR_COUNT] = {0, 0, 1, 1, 0, 0};  // Motors 1, 2, 5, 6 -> 0 (towards limit), Motors 3, 4 -> 1 (away from limit)

// Array to store current positions of each motor
float motor_current_positions[MOTOR_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Assume starting at 0 (home position)

// Array to store the state of each limit switch (0 = not triggered, 1 = triggered)
int limit_switch_states[LIMIT_SWITCH_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void delay_us (uint16_t us);
void run_motor(uint8_t motor_number, float degree, uint8_t direction, uint16_t delay_time_us);
void read_limit_switches(void);
void print_limit_switch_status(void);
void home_motor(uint8_t motor_number, uint8_t direction);
void home_all_motors(void);
bool motorStoppedByLimit[6] = {false, false, false, false, false, false};
void print_motorStoppedByLimit(void);
bool homing_operation = false;
bool homing_operation_inv = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MotorConfig motors[MOTOR_COUNT] = { 
    {GPIOB, DIR1_Pin , GPIOA, PUL1_Pin, 16000, 0, 0.0, 0},   // Motor 1 gear ratio : 10 
    {GPIOB, DIR2_Pin, GPIOB, PUL2_Pin, 40000, 0, 0.0, 0},   // Motor 2 gear ratio : 50
    {GPIOA, DIR3_Pin, GPIOB, PUL3_Pin, 20000, 0, 0.0, 0},   // Motor 3 gear ratio : 50
    {DIR4_GPIO_Port, DIR4_Pin, GPIOA, PUL4_Pin, 6400, 0, 0.0, 0},  // Motor 4 gear ratio : 16
    {GPIOA, DIR5_Pin, GPIOB, PUL5_Pin, 0 , 25, 0.0, 1},   // Motor 5 (Linear motor)
    {GPIOA, DIR6_Pin, GPIOA, PUL6_Pin, 8000, 0, 0.0, 0},  // Motor 6 gear ratio : 20
};
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

/* move all arms to home position */
  home_all_motors();

  HAL_Delay(1000);
  for(int motor_number=0; motor_number< MOTOR_COUNT; motor_number++){
    motor_current_positions[motor_number - 1] = 0.0;
    HAL_Delay(50);
  }
    HAL_Delay(50);

    run_motor(1, 160.0, 1, MOTOR_DELAY_TIME_US);  
    HAL_Delay(50);
    run_motor(2, 27.0, 1, MOTOR_DELAY_TIME_US);  
    HAL_Delay(50);
    run_motor(3, 50.0, 0, MOTOR_DELAY_TIME_US);  
    HAL_Delay(50);
    run_motor(4, 470.0, 0, MOTOR_DELAY_TIME_US);  
    HAL_Delay(50);
    run_motor(5, 10.0, 0, MOTOR_DELAY_TIME_US);  
    HAL_Delay(50);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        // Print the status of each limit switch
          read_limit_switches(); // DEBUG
          print_limit_switch_status();  //DEBUG
    HAL_Delay(50);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  htim1.Init.Prescaler = 47;
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
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR6_Pin|PUL6_Pin|DIR5_Pin|DIR3_Pin
                          |PUL4_Pin|PUL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUL3_Pin|DIR1_Pin|DIR2_Pin|PUL2_Pin
                          |PUL5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR6_Pin PUL6_Pin DIR5_Pin DIR3_Pin
                           PUL4_Pin PUL1_Pin */
  GPIO_InitStruct.Pin = DIR6_Pin|PUL6_Pin|DIR5_Pin|DIR3_Pin
                          |PUL4_Pin|PUL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LS3_Pin LS2_Pin */
  GPIO_InitStruct.Pin = LS3_Pin|LS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL3_Pin DIR1_Pin DIR2_Pin PUL2_Pin
                           PUL5_Pin */
  GPIO_InitStruct.Pin = PUL3_Pin|DIR1_Pin|DIR2_Pin|PUL2_Pin
                          |PUL5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LS6_Pin */
  GPIO_InitStruct.Pin = LS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LS6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR4_Pin */
  GPIO_InitStruct.Pin = DIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LS1_Pin */
  GPIO_InitStruct.Pin = LS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LS5_Pin LS4_Pin */
  GPIO_InitStruct.Pin = LS5_Pin|LS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

/**
  * @brief  Retargets the C library printf function to the USART.
     *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void run_motor(uint8_t motor_number, float value, uint8_t direction, uint16_t delay_time_us)
{
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return;  // Invalid motor number
    }

    float new_position = 0;

    MotorConfig* motor = &motors[motor_number - 1];  // Get the motor configuration

#if 1
if(!homing_operation){
    // Update the motor's current position based on direction
      if (motor_directions[motor_number - 1] == direction) {
          new_position = motor_current_positions[motor_number -1] - value;  // Move in the specified direction
      } else {
          new_position = motor_current_positions[motor_number -1] + value;  // Move in the opposite direction
      }
    // Now check if the new position exceeds the motor's limit
    if (new_position <= motor_max_limits[motor_number - 1] && new_position >= 0){ // If the position is within the allowed limit, update the current position
      motor_current_positions[motor_number - 1] = new_position;
    }
    else{
      new_position = motor_max_limits[motor_number - 1] - motor_current_positions[motor_number - 1];  // Max position for away from limit
      motor_current_positions[motor_number - 1] = new_position;
      value = new_position;
    }
}
#endif

    if (motor->isLinear) {
        // Linear motor (Motor 5)
        float pulses_per_mm = motor->pulsesPerMM;
        int whole_pulses = value * pulses_per_mm;  // 'value' is the distance in mm
        float remainder = (value * pulses_per_mm) - whole_pulses;

        // Accumulate the remainder
        motor->accumulatedRemainder += remainder;
        if (motor->accumulatedRemainder >= 1.0) {
            whole_pulses += 1;
            motor->accumulatedRemainder -= 1.0;
        }

        // Set direction
        if (direction) {
            HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_RESET);
        }

        // Variables for acceleration/deceleration control
        uint16_t pulse_count = whole_pulses;
        uint16_t current_pulse = 0;
        uint16_t initial_delay = delay_time_us;
        uint16_t max_delay = delay_time_us * 2;  // Maximum delay for deceleration

        // Accelerate: Gradually decrease pulse rate (increase delay time)
        while (current_pulse < pulse_count / 2) {
            // Accelerating phase
            if(homing_operation && limit_switch_states[motor_number -1] && !homing_operation_inv){
              break;
            }
            if(!limit_switch_states[motor_number -1] || homing_operation == true){
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_SET);
              delay_us(initial_delay);
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_RESET);
              delay_us(initial_delay);
              read_limit_switches();
            }

            // Gradually increase delay for smoother movement (acceleration)
            if (initial_delay < max_delay) {
                initial_delay += 10;  // Increment delay to slow down
            }
            current_pulse++;
          }

          // Decelerate: Gradually increase pulse rate (decrease delay time)
          while (current_pulse < pulse_count) {
              // Decelerating phase
            if(homing_operation && limit_switch_states[motor_number -1] && !homing_operation_inv){
              break;
            }
            if(!limit_switch_states[motor_number -1] || homing_operation == true){
                HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_SET);
                delay_us(initial_delay);
                HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_RESET);
                delay_us(initial_delay);
                read_limit_switches();
              }

              // Gradually decrease delay for smoother movement (deceleration)
              if (initial_delay > delay_time_us) {
                  initial_delay -= 10;  // Decrease delay to speed up
              }
              current_pulse++;
          }
    } else {
        // Rotational motor (Motors 1, 2, 3, 4, 6)
        if (motor->totalPulsesPerRevolution == 0) {
            return;  // Skip if motor is not configured
        }

        float pulses_per_degree = motor->totalPulsesPerRevolution / 360.0;
        int whole_pulses = value * pulses_per_degree;  // 'value' is the degree
        float remainder = (value * pulses_per_degree) - whole_pulses;

        // Accumulate the remainder
        motor->accumulatedRemainder += remainder;
        if (motor->accumulatedRemainder >= 1.0) {
            whole_pulses += 1;
            motor->accumulatedRemainder -= 1.0;
        }

        // Set direction
        if (direction) {
            HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, GPIO_PIN_RESET);
        }

        // Variables for acceleration/deceleration control
        uint16_t pulse_count = whole_pulses;
        uint16_t current_pulse = 0;
        uint16_t initial_delay = delay_time_us;
        uint16_t max_delay = delay_time_us * 2;  // Maximum delay for deceleration

        // Accelerate: Gradually decrease pulse rate (increase delay time)
        while (current_pulse < pulse_count / 2) {
            // Accelerating phase
            //if(!motorStoppedByLimit[motor_number -1])
            if(homing_operation && limit_switch_states[motor_number -1] && !homing_operation_inv){
              break;
            }
            if(!limit_switch_states[motor_number -1] || homing_operation == true){
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_SET);
              delay_us(initial_delay);
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_RESET);
              delay_us(initial_delay);
              read_limit_switches();
            }

            // Gradually increase delay for smoother movement (acceleration)
            if (initial_delay < max_delay) {
                initial_delay += 10;  // Increment delay to slow down
            }
            current_pulse++;
        }

        // Decelerate: Gradually increase pulse rate (decrease delay time)
        while (current_pulse < pulse_count) {
            // Decelerating phase
            //if(!motorStoppedByLimit[motor_number -1])
            if(homing_operation && limit_switch_states[motor_number -1] && !homing_operation_inv){
              break;
            }
            if(!limit_switch_states[motor_number -1] || homing_operation == true){
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_SET);
              delay_us(initial_delay);
              HAL_GPIO_WritePin(motor->pulPort, motor->pulPin, GPIO_PIN_RESET);
              delay_us(initial_delay);
              read_limit_switches();
            }

            // Gradually decrease delay for smoother movement (deceleration)
            if (initial_delay > delay_time_us) {
                initial_delay -= 10;  // Decrease delay to speed up
            }
            current_pulse++;
        }
    }
}

// Function to read the state of each limit switch
void read_limit_switches(void)
{
    // Read each limit switch input pin and store the state
    // Here, assuming the limit switches are connected to the GPIO pins LS1_Pin, LS2_Pin, ..., LS6_Pin
    limit_switch_states[0] = HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin); // Reading the first limit switch
    limit_switch_states[1] = HAL_GPIO_ReadPin(GPIOC, LS2_Pin); // Reading the second limit switch
    limit_switch_states[2] = HAL_GPIO_ReadPin(GPIOC, LS3_Pin); // Reading the third limit switch
    limit_switch_states[3] = HAL_GPIO_ReadPin(GPIOA, LS4_Pin); // Reading the fourth limit switch
    limit_switch_states[4] = HAL_GPIO_ReadPin(GPIOA, LS5_Pin); // Reading the fifth limit switch
    limit_switch_states[5] = HAL_GPIO_ReadPin(LS6_GPIO_Port, LS6_Pin); // Reading the sixth limit switch
}

// Function to print the status of limit switches
void print_limit_switch_status(void)
{
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        printf("Limit switch %d state: %d %s\n\r", i + 1, limit_switch_states[i], limit_switch_states[i] ? "Triggered" : "Not Triggered");
    }
    printf("\n\r");
}

// Function to print the status of motorStoppedByLimit
void print_motorStoppedByLimit(void)
{
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        printf("motorStoppedByLimit %d state: %d %s\n\r", i + 1, motorStoppedByLimit[i], motorStoppedByLimit[i] ? "TRUE" : "FALSE");
    }
    printf("\n\r");
}

void home_motor(uint8_t motor_number, uint8_t direction) {
    homing_operation = true;
    if (motor_number < 1 || motor_number > MOTOR_COUNT) {
        return;  // Invalid motor number
    }
    uint8_t inv_direction = direction ? 0 : 1;
    // Check if the limit switch is in the correct state (active low)
    while (!limit_switch_states[motor_number - 1]) 
    {
        // Move motor step by step towards the limit switch
        run_motor(motor_number, HOMING_STEP_DEGREE , direction, HOMING_DELAY_TIME_US );
        read_limit_switches();
        HAL_Delay(HOMING_CHECK_DELAY_MS);
    }
    homing_operation_inv = true;
    run_motor(motor_number, HOMING_STEP_DEGREE*3 , inv_direction, HOMING_DELAY_TIME_US );
    homing_operation_inv = false;
    printf("Motor %d has reached home position.\n\r", motor_number + 1);
    motor_current_positions[motor_number - 1] = 0.0;
    homing_operation = false;
}

void home_all_motors(void){

  //home_motor(6,0); // update : during unit testing, motor driver was damages, uncomment after replacement.
    HAL_Delay(50);
  home_motor(5,0); 
    HAL_Delay(50);
  home_motor(4,1);
    HAL_Delay(50);
  home_motor(2,0);
    HAL_Delay(50);
  home_motor(3,1);
    HAL_Delay(50);
  home_motor(1,0);
}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LS1_Pin) // If The INT Source Is from limit switch 1
    {
        motorStoppedByLimit[0] = true;  // Motor 1 stopped by limit switch
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
