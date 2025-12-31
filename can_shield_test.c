#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* --- PIN DEFINITIONS --- */
// User LED (Green) -> Visual Indicator
#define LED_PIN         GPIO_PIN_5
#define LED_PORT        GPIOA

// CAN Shield TX Pin (Manual Page 4)
// We treat the CAN TX pin as a simple GPIO Output to manualy drive the bus voltage.
#define CAN_TX_PIN      GPIO_PIN_9  // D14 (PB9)
#define CAN_PORT        GPIOB

UART_HandleTypeDef huart2;

/* Function Prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void UART_Init(void);
void Error_Handler(void);

int main(void)
{
    char msg[128];

    // 1. Initialize HAL & Clock (Using your working config)
    HAL_Init();
    SystemClock_Config();

    // 2. Initialize GPIO (LED + CAN TX)
    GPIO_Init();

    // 3. Initialize UART
    UART_Init();

    // 4. Send Startup Message
    sprintf(msg, "\r\n--- CAN SHIELD VOLTAGE TESTER ---\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    sprintf(msg, "Focus ONLY on the CANH and CANL Screw Terminals.\r\n\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    while (1)
    {
        // ============================================
        // STATE 1: RECESSIVE (IDLE)
        // ============================================
        // Logic: HIGH (3.3V) on CAN TX pin = Recessive Bus State
        
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);     // LED OFF
        HAL_GPIO_WritePin(CAN_PORT, CAN_TX_PIN, GPIO_PIN_SET);    // CAN TX HIGH

        // Print Status to Serial
        sprintf(msg, "[ LED OFF ] STATE: RECESSIVE (Idle)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        sprintf(msg, "    -> CANH to GND: ~2.5V\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        sprintf(msg, "    -> CANL to GND: ~2.5V\r\n\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(5000); // Wait 5 seconds

        // ============================================
        // STATE 2: DOMINANT (ACTIVE)
        // ============================================
        // Logic: LOW (0V) on CAN TX pin = Dominant Bus State
        
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);       // LED ON
        HAL_GPIO_WritePin(CAN_PORT, CAN_TX_PIN, GPIO_PIN_RESET);  // CAN TX LOW

        // Print Status to Serial
        sprintf(msg, "[ LED ON  ] STATE: DOMINANT (Active)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        sprintf(msg, "    -> CANH to GND: ~3.5V (Voltage goes UP)\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        sprintf(msg, "    -> CANL to GND: ~1.5V (Voltage goes DOWN)\r\n\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(5000); // Wait 5 seconds
    }
}

// ---------------------------------------------------------
// GPIO INITIALIZATION
// ---------------------------------------------------------
void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE(); // For LED
    __HAL_RCC_GPIOB_CLK_ENABLE(); // For CAN

    // 1. Configure LED (PA5)
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

    // 2. Configure CAN TX Pin (PB9 / D14)
    // We configure it as a Push-Pull Output to manually drive the transceiver
    GPIO_InitStruct.Pin = CAN_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CAN_PORT, &GPIO_InitStruct);
}

// ---------------------------------------------------------
// SYSTEM CLOCK CONFIG (From your working code)
// ---------------------------------------------------------
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

// ---------------------------------------------------------
// UART INITIALIZATION (From your working code)
// ---------------------------------------------------------
void UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    __disable_irq();
    while (1) { }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}