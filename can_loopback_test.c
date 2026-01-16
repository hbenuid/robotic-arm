/**
 * Simple CAN Bus Loopback Test for STM32 Nucleo F446RE
 * 
 * Tests CAN1 in loopback mode (no external hardware needed).
 * Uses HSI clock (16MHz) for simplicity.
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* Handles */
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* Printf redirect to USART2 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void Error_Handler(void) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

/**
 * Simple clock config - just use HSI (16 MHz)
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    __HAL_RCC_PWR_CLK_ENABLE();
    
    /* Use HSI (16 MHz internal) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    /* HCLK = SYSCLK, APB1 = HCLK/1, APB2 = HCLK/1 (all 16 MHz) */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * USART2 Init - 115200 baud @ 16 MHz
 */
void USART2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /* PA2=TX, PA3=RX */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * LED Init - PA5
 */
void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * CAN1 Init - 500 Kbit/s @ 16 MHz APB1, Loopback mode
 */
void CAN1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    CAN_FilterTypeDef canFilter;
    
    printf("Enabling CAN1 clock...\r\n");
    
    /* Enable clocks */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    printf("CAN1 MCR before init: 0x%08lX\r\n", CAN1->MCR);
    printf("CAN1 MSR before init: 0x%08lX\r\n", CAN1->MSR);
    
    /* CAN1 GPIO: PB8=RX, PB9=TX */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    printf("GPIO configured\r\n");
    
    /*
     * CAN Bit Timing @ 16 MHz APB1:
     * Baud = APB1 / (Prescaler * (1 + BS1 + BS2))
     * 500000 = 16000000 / (2 * (1 + 13 + 2))
     * 500000 = 16000000 / 32 = 500000 ✓
     */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;
    hcan1.Init.Mode = CAN_MODE_LOOPBACK;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    printf("Calling HAL_CAN_Init...\r\n");
    HAL_StatusTypeDef status = HAL_CAN_Init(&hcan1);
    if (status != HAL_OK) {
        printf("HAL_CAN_Init FAILED: status=%d, error=0x%08lX\r\n", 
               status, HAL_CAN_GetError(&hcan1));
        Error_Handler();
    }
    printf("HAL_CAN_Init OK\r\n");
    
    /* Filter - accept all */
    canFilter.FilterBank = 0;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilter.FilterActivation = ENABLE;
    canFilter.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK) {
        printf("HAL_CAN_ConfigFilter FAILED\r\n");
        Error_Handler();
    }
    printf("CAN Filter configured\r\n");
}

/**
 * CAN Loopback Test
 */
int CAN_Loopback_Test(void) {
    CAN_TxHeaderTypeDef txHeader;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t txData[8] = {0xCA, 0xFE, 0xBA, 0xBE, 0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t rxData[8] = {0};
    uint32_t txMailbox;
    uint32_t startTick;
    HAL_StatusTypeDef status;
    
    /* TX message config */
    txHeader.StdId = 0x123;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    
    printf("Starting CAN...\r\n");
    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK) {
        printf("HAL_CAN_Start FAILED: status=%d, error=0x%08lX\r\n", 
               status, HAL_CAN_GetError(&hcan1));
        
        /* Check CAN status register */
        printf("CAN MSR: 0x%08lX\r\n", CAN1->MSR);
        printf("CAN MCR: 0x%08lX\r\n", CAN1->MCR);
        printf("CAN ESR: 0x%08lX\r\n", CAN1->ESR);
        return 0;
    }
    printf("CAN Started OK\r\n");
    
    printf("Sending: ID=0x%03lX Data=", txHeader.StdId);
    for (int i = 0; i < 8; i++) printf("%02X ", txData[i]);
    printf("\r\n");
    
    /* Send */
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK) {
        printf("TX failed\r\n");
        return 0;
    }
    
    /* Wait TX complete */
    startTick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
        if ((HAL_GetTick() - startTick) > 100) {
            printf("TX timeout\r\n");
            return 0;
        }
    }
    printf("TX complete\r\n");
    
    /* Wait RX */
    startTick = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        if ((HAL_GetTick() - startTick) > 100) {
            printf("RX timeout\r\n");
            return 0;
        }
    }
    
    /* Receive */
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        printf("RX failed\r\n");
        return 0;
    }
    
    printf("Received: ID=0x%03lX Data=", rxHeader.StdId);
    for (int i = 0; i < 8; i++) printf("%02X ", rxData[i]);
    printf("\r\n");
    
    HAL_CAN_Stop(&hcan1);
    
    /* Verify */
    if (rxHeader.StdId != txHeader.StdId) return 0;
    if (rxHeader.DLC != txHeader.DLC) return 0;
    if (memcmp(txData, rxData, 8) != 0) return 0;
    
    return 1;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    LED_Init();
    USART2_Init();
    
    printf("\r\n================================\r\n");
    printf("CAN Bus Loopback Test\r\n");
    printf("================================\r\n");
    printf("SYSCLK: %lu Hz\r\n", HAL_RCC_GetSysClockFreq());
    printf("PCLK1:  %lu Hz\r\n", HAL_RCC_GetPCLK1Freq());
    printf("\r\n");
    
    CAN1_Init();
    
    printf("\r\nRunning loopback test...\r\n");
    if (CAN_Loopback_Test()) {
        printf("\r\n*** TEST PASSED ***\r\n");
    } else {
        printf("\r\n*** TEST FAILED ***\r\n");
    }
    
    printf("\nBlinking LED...\r\n");
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}