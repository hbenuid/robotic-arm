#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

// LED is connected to PA5 on Nucleo-F446RE (green user LED)
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA

// UART handle for serial communication
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void GPIO_Init(void);
void UART_Init(void);
void Error_Handler(void);

int main(void)
{
    char msg[100];

    // Initialize HAL Library
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize GPIO for LED
    GPIO_Init();

    // Initialize UART for serial communication
    UART_Init();

    // Send startup message
    sprintf(msg, "STM32 Blink Test Started\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Main loop - blink LED rapidly three times
    int loop_count = 0;
    while (1)
    {
        loop_count++;
        sprintf(msg, "Starting blink sequence #%d\r\n", loop_count);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        // Blink rapidly three times
        for (int i = 0; i < 3; i++)
        {
            sprintf(msg, "  Blink %d - ON\r\n", i + 1);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);    // LED ON
            HAL_Delay(100);  // Wait 100ms

            sprintf(msg, "  Blink %d - OFF\r\n", i + 1);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);  // LED OFF
            HAL_Delay(100);  // Wait 100ms
        }

        sprintf(msg, "Pausing...\r\n\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        // Pause before repeating the pattern
        HAL_Delay(1000);  // Wait 1 second
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

// Required for HAL library
void SysTick_Handler(void)
{
    HAL_IncTick();
}
