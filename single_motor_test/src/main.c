#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// LED is connected to PA5 on Nucleo-F446RE (green user LED)
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA

// Stepper motor pin definitions
#define STEP_PIN GPIO_PIN_6    // STEP pin connected to PA6
#define DIR_PIN GPIO_PIN_7     // DIR pin connected to PA7
#define ENABLE_PIN GPIO_PIN_8  // ENABLE pin connected to PA8
#define STEPPER_PORT GPIOA

// Stepper motor parameters
#define STEPS_PER_REVOLUTION 200  // 17HS19-2004S1 is 1.8° per step = 200 steps/rev

// UART handle for serial communication
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void GPIO_Init(void);
void UART_Init(void);
void Error_Handler(void);
void DWT_Init(void);
void delayMicroseconds(uint32_t us);
void rotateMotor(int steps, int delayTime);

int main(void)
{
    char msg[100];

    // Initialize HAL Library
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize GPIO for LED and stepper motor
    GPIO_Init();

    // Initialize UART for serial communication
    UART_Init();

    // Initialize DWT for microsecond delays
    DWT_Init();

    // Send startup message
    sprintf(msg, "=================================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    sprintf(msg, "Stepper Motor Test - 17HS19-2004S1\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    sprintf(msg, "=================================\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_Delay(1000);

    // Main loop
    while (1)
    {
        // Test 1: Slow rotation clockwise
        sprintf(msg, "\r\nTest 1: Slow CW rotation (1 rev)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_SET);  // Clockwise
        rotateMotor(STEPS_PER_REVOLUTION, 2000);  // 2000us delay = slow
        HAL_Delay(2000);

        // Test 2: Slow rotation counterclockwise
        sprintf(msg, "Test 2: Slow CCW rotation (1 rev)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_RESET);  // Counterclockwise
        rotateMotor(STEPS_PER_REVOLUTION, 2000);
        HAL_Delay(2000);

        // Test 3: Medium speed clockwise
        sprintf(msg, "Test 3: Medium CW rotation (2 rev)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_SET);
        rotateMotor(STEPS_PER_REVOLUTION * 2, 1000);  // 1000us = medium speed
        HAL_Delay(2000);

        // Test 4: Fast rotation counterclockwise
        sprintf(msg, "Test 4: Fast CCW rotation (2 rev)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_RESET);
        rotateMotor(STEPS_PER_REVOLUTION * 2, 500);  // 500us = fast
        HAL_Delay(2000);

        // Test 5: Very slow precise movement
        sprintf(msg, "Test 5: Precise positioning (90 degree steps)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        for(int i = 0; i < 4; i++)
        {
            HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_SET);
            rotateMotor(50, 3000);  // 50 steps = 90 degrees, very slow
            HAL_Delay(1000);
        }
        HAL_Delay(2000);

        // Test 6: Continuous rotation with direction changes
        sprintf(msg, "Test 6: Back and forth motion\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        for(int i = 0; i < 3; i++)
        {
            HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_SET);
            rotateMotor(100, 800);
            HAL_Delay(500);
            HAL_GPIO_WritePin(STEPPER_PORT, DIR_PIN, GPIO_PIN_RESET);
            rotateMotor(100, 800);
            HAL_Delay(500);
        }

        sprintf(msg, "\r\n=== Test cycle complete ===\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        sprintf(msg, "Pausing 5 seconds before next cycle...\r\n\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        HAL_Delay(5000);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initialize the RCC Oscillators according to the specified parameters
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // Initialize the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pin for LED (PA5)
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // Configure GPIO pins for stepper motor (PA6, PA7, PA8)
    GPIO_InitStruct.Pin = STEP_PIN | DIR_PIN | ENABLE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // High speed for step pulses

    HAL_GPIO_Init(STEPPER_PORT, &GPIO_InitStruct);

    // Enable the stepper driver (LOW = enabled, HIGH = disabled)
    HAL_GPIO_WritePin(STEPPER_PORT, ENABLE_PIN, GPIO_PIN_RESET);
}

void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable USART2 and GPIOA clocks
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure UART pins (PA2 = TX, PA3 = RX)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure UART parameters
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
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
}

void Error_Handler(void)
{
    // Error handling - disable interrupts and loop forever
    __disable_irq();
    while (1)
    {
    }
}

// Initialize DWT (Data Watchpoint and Trace) for precise timing
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable TRC
    DWT->CYCCNT = 0;  // Reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Enable cycle counter
}

// Precise microsecond delay using DWT cycle counter
void delayMicroseconds(uint32_t us)
{
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - startTick) < delayTicks);
}

// Function to rotate stepper motor
void rotateMotor(int steps, int delayTime)
{
    for(int x = 0; x < steps; x++)
    {
        HAL_GPIO_WritePin(STEPPER_PORT, STEP_PIN, GPIO_PIN_SET);
        delayMicroseconds(delayTime);
        HAL_GPIO_WritePin(STEPPER_PORT, STEP_PIN, GPIO_PIN_RESET);
        delayMicroseconds(delayTime);
    }
}

// Required for HAL library
void SysTick_Handler(void)
{
    HAL_IncTick();
}
