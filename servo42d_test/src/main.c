/**
 * MKS SERVO42D CAN Control for STM32 Nucleo F446RE
 * 
 * Controls MKS SERVO42D via CAN bus using SN65HVD230 transceiver
 * CAN pins: PB8 (RX), PB9 (TX)
 * 
 * Before using:
 * 1. Set servo to SR_vFOC mode: Menu -> Mode -> SR_vFOC
 * 2. Set CAN rate to 500K: Menu -> CanRate -> 500K
 * 3. Set CAN ID to 01: Menu -> CanID -> 01
 * 4. Calibrate motor if not done: Menu -> CAL
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* Handles */
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* MKS SERVO42D CAN Protocol Definitions */
#define MKS_CAN_ID          0x01    /* Default servo CAN ID */

/* Command codes */
#define CMD_READ_ENCODER    0x30    /* Read encoder value (carry) */
#define CMD_READ_ENCODER2   0x31    /* Read encoder value (addition) */
#define CMD_READ_SPEED      0x32    /* Read motor speed (RPM) */
#define CMD_READ_PULSES     0x33    /* Read pulses received */
#define CMD_READ_ERROR      0x39    /* Read shaft angle error */
#define CMD_READ_EN_STATUS  0x3A    /* Read enable status */
#define CMD_QUERY_STATUS    0xF1    /* Query motor status */
#define CMD_ENABLE_MOTOR    0xF3    /* Enable/disable motor */
#define CMD_SPEED_MODE      0xF6    /* Speed mode control */
#define CMD_POSITION_MODE1  0xFD    /* Position mode (pulses) */
#define CMD_POSITION_MODE2  0xF4    /* Position mode (relative axis) */
#define CMD_POSITION_MODE3  0xF5    /* Position mode (absolute axis) */

/* Function prototypes */
void SystemClock_Config(void);
void USART2_Init(void);
void LED_Init(void);
void CAN1_Init(void);
void Error_Handler(void);

/* MKS Servo functions */
uint8_t MKS_CalcChecksum(uint8_t canId, uint8_t *data, uint8_t len);
HAL_StatusTypeDef MKS_SendCommand(uint8_t *data, uint8_t len);
HAL_StatusTypeDef MKS_ReceiveResponse(uint8_t *data, uint8_t *len, uint32_t timeout);
int32_t MKS_ReadEncoder(void);
int16_t MKS_ReadSpeed(void);
uint8_t MKS_QueryStatus(void);
HAL_StatusTypeDef MKS_EnableMotor(uint8_t enable);
HAL_StatusTypeDef MKS_SpeedMode(uint8_t dir, uint16_t speed, uint8_t acc);
HAL_StatusTypeDef MKS_StopMotor(uint8_t acc);
HAL_StatusTypeDef MKS_PositionMode(uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses);

/* Printf redirect */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void Error_Handler(void) {
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

/* ========== MKS SERVO42D Functions ========== */

/**
 * Calculate checksum: (ID + all data bytes) & 0xFF
 */
uint8_t MKS_CalcChecksum(uint8_t canId, uint8_t *data, uint8_t len) {
    uint16_t sum = canId;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/**
 * Send CAN command to servo
 */
HAL_StatusTypeDef MKS_SendCommand(uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    
    txHeader.StdId = MKS_CAN_ID;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = len;
    txHeader.TransmitGlobalTime = DISABLE;
    
    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
}

/**
 * Receive CAN response from servo
 */
HAL_StatusTypeDef MKS_ReceiveResponse(uint8_t *data, uint8_t *len, uint32_t timeout) {
    CAN_RxHeaderTypeDef rxHeader;
    uint32_t startTick = HAL_GetTick();
    
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        if ((HAL_GetTick() - startTick) > timeout) {
            return HAL_TIMEOUT;
        }
    }
    
    HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data);
    if (status == HAL_OK) {
        *len = rxHeader.DLC;
    }
    return status;
}

/**
 * Read encoder value (addition format)
 * Returns encoder position (0x4000 = one full rotation)
 */
int32_t MKS_ReadEncoder(void) {
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    txData[0] = CMD_READ_ENCODER2;
    txData[1] = MKS_CalcChecksum(MKS_CAN_ID, txData, 1);
    
    if (MKS_SendCommand(txData, 2) != HAL_OK) {
        return 0;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return 0;
    }
    
    /* Response: code(1) + value(6) + CRC(1) */
    if (rxLen >= 7 && rxData[0] == CMD_READ_ENCODER2) {
        int64_t value = 0;
        /* 48-bit signed value, big-endian */
        for (int i = 1; i <= 6; i++) {
            value = (value << 8) | rxData[i];
        }
        /* Sign extend from 48-bit */
        if (value & 0x800000000000LL) {
            value |= 0xFFFF000000000000LL;
        }
        return (int32_t)value;
    }
    return 0;
}

/**
 * Read motor speed in RPM
 */
int16_t MKS_ReadSpeed(void) {
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    txData[0] = CMD_READ_SPEED;
    txData[1] = MKS_CalcChecksum(MKS_CAN_ID, txData, 1);
    
    if (MKS_SendCommand(txData, 2) != HAL_OK) {
        return 0;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return 0;
    }
    
    /* Response: code(1) + speed(2) + CRC(1) */
    if (rxLen >= 3 && rxData[0] == CMD_READ_SPEED) {
        return (int16_t)((rxData[1] << 8) | rxData[2]);
    }
    return 0;
}

/**
 * Query motor status
 * Returns: 0=fail, 1=stopped, 2=accelerating, 3=decelerating, 4=full speed, 5=homing
 */
uint8_t MKS_QueryStatus(void) {
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    txData[0] = CMD_QUERY_STATUS;
    txData[1] = MKS_CalcChecksum(MKS_CAN_ID, txData, 1);
    
    if (MKS_SendCommand(txData, 2) != HAL_OK) {
        return 0;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return 0;
    }
    
    if (rxLen >= 2 && rxData[0] == CMD_QUERY_STATUS) {
        return rxData[1];
    }
    return 0;
}

/**
 * Enable or disable motor
 */
HAL_StatusTypeDef MKS_EnableMotor(uint8_t enable) {
    uint8_t txData[3];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    txData[0] = CMD_ENABLE_MOTOR;
    txData[1] = enable ? 0x01 : 0x00;
    txData[2] = MKS_CalcChecksum(MKS_CAN_ID, txData, 2);
    
    if (MKS_SendCommand(txData, 3) != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return HAL_TIMEOUT;
    }
    
    if (rxLen >= 2 && rxData[0] == CMD_ENABLE_MOTOR && rxData[1] == 0x01) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
 * Run motor in speed mode
 * dir: 0=CCW, 1=CW
 * speed: 0-3000 RPM
 * acc: 0-255 (0=instant, higher=slower acceleration)
 */
HAL_StatusTypeDef MKS_SpeedMode(uint8_t dir, uint16_t speed, uint8_t acc) {
    uint8_t txData[5];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    /* byte2: dir(bit7) + speed high nibble(bit3-0) */
    /* byte3: speed low byte */
    txData[0] = CMD_SPEED_MODE;
    txData[1] = (dir ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
    txData[2] = speed & 0xFF;
    txData[3] = acc;
    txData[4] = MKS_CalcChecksum(MKS_CAN_ID, txData, 4);
    
    if (MKS_SendCommand(txData, 5) != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return HAL_TIMEOUT;
    }
    
    if (rxLen >= 2 && rxData[0] == CMD_SPEED_MODE && rxData[1] == 0x01) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
 * Stop motor
 * acc: 0=immediate stop, >0=decelerate
 */
HAL_StatusTypeDef MKS_StopMotor(uint8_t acc) {
    return MKS_SpeedMode(0, 0, acc);
}

/**
 * Run motor in position mode (relative pulses)
 * dir: 0=CCW, 1=CW
 * speed: 0-3000 RPM
 * acc: 0-255
 * pulses: number of pulses (3200 pulses = 1 rotation at 16 subdivisions)
 */
HAL_StatusTypeDef MKS_PositionMode(uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses) {
    uint8_t txData[8];
    uint8_t rxData[8];
    uint8_t rxLen;
    
    txData[0] = CMD_POSITION_MODE1;
    txData[1] = (dir ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
    txData[2] = speed & 0xFF;
    txData[3] = acc;
    txData[4] = (pulses >> 16) & 0xFF;
    txData[5] = (pulses >> 8) & 0xFF;
    txData[6] = pulses & 0xFF;
    txData[7] = MKS_CalcChecksum(MKS_CAN_ID, txData, 7);
    
    if (MKS_SendCommand(txData, 8) != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK) {
        return HAL_TIMEOUT;
    }
    
    if (rxLen >= 2 && rxData[0] == CMD_POSITION_MODE1) {
        /* 0=fail, 1=starting, 2=complete */
        return (rxData[1] > 0) ? HAL_OK : HAL_ERROR;
    }
    return HAL_ERROR;
}

/* ========== Peripheral Init Functions ========== */

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    __HAL_RCC_PWR_CLK_ENABLE();
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
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

void USART2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
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

void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void CAN1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    CAN_FilterTypeDef canFilter;
    
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* CAN1 GPIO: PB8=RX, PB9=TX */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /*
     * CAN Bit Timing @ 16 MHz APB1 for 500 Kbit/s:
     * Baud = 16MHz / (2 * (1 + 13 + 2)) = 500 KHz
     */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;
    hcan1.Init.Mode = CAN_MODE_NORMAL;  /* Normal mode for real bus */
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        printf("CAN Init failed!\r\n");
        Error_Handler();
    }
    
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
        printf("CAN Filter config failed!\r\n");
        Error_Handler();
    }
    
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        printf("CAN Start failed!\r\n");
        Error_Handler();
    }
    
    printf("CAN initialized at 500 Kbit/s\r\n");
}

/* ========== Main ========== */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    LED_Init();
    USART2_Init();
    
    printf("\r\n========================================\r\n");
    printf("MKS SERVO42D CAN Controller\r\n");
    printf("========================================\r\n\r\n");
    
    CAN1_Init();
    
    printf("\r\nMake sure SERVO42D is set to:\r\n");
    printf("  - Mode: SR_vFOC\r\n");
    printf("  - CanRate: 500K\r\n");
    printf("  - CanID: 01\r\n\r\n");
    
    HAL_Delay(1000);
    
    /* Test: Query motor status */
    printf("Querying motor status...\r\n");
    uint8_t status = MKS_QueryStatus();
    printf("Motor status: %d ", status);
    switch (status) {
        case 0: printf("(query failed)\r\n"); break;
        case 1: printf("(stopped)\r\n"); break;
        case 2: printf("(accelerating)\r\n"); break;
        case 3: printf("(decelerating)\r\n"); break;
        case 4: printf("(full speed)\r\n"); break;
        case 5: printf("(homing)\r\n"); break;
        default: printf("(unknown)\r\n"); break;
    }
    
    /* Test: Read encoder */
    printf("Reading encoder...\r\n");
    int32_t encoder = MKS_ReadEncoder();
    printf("Encoder value: %ld (0x%08lX)\r\n", encoder, encoder);
    
    /* Test: Enable motor */
    printf("Enabling motor...\r\n");
    if (MKS_EnableMotor(1) == HAL_OK) {
        printf("Motor enabled OK\r\n");
    } else {
        printf("Failed to enable motor\r\n");
    }
    
    HAL_Delay(500);
    
    /* Test: Run motor forward at 100 RPM */
    printf("\r\nRunning motor forward at 100 RPM...\r\n");
    if (MKS_SpeedMode(0, 100, 10) == HAL_OK) {
        printf("Speed command OK\r\n");
    } else {
        printf("Speed command FAILED\r\n");
    }
    
    /* Run for 3 seconds */
    for (int i = 0; i < 6; i++) {
        HAL_Delay(500);
        int16_t speed = MKS_ReadSpeed();
        encoder = MKS_ReadEncoder();
        printf("  Speed: %d RPM, Encoder: %ld\r\n", speed, encoder);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
    
    /* Stop motor */
    printf("\r\nStopping motor...\r\n");
    if (MKS_StopMotor(10) == HAL_OK) {
        printf("Stop command OK\r\n");
    } else {
        printf("Stop command FAILED\r\n");
    }
    
    HAL_Delay(1000);
    
    /* Test: Position mode - rotate 1 full turn (3200 pulses at 16 microsteps) */
    printf("\r\nPosition mode: 1 rotation forward...\r\n");
    if (MKS_PositionMode(0, 200, 10, 3200) == HAL_OK) {
        printf("Position command OK\r\n");
    } else {
        printf("Position command FAILED\r\n");
    }
    
    /* Wait for completion */
    HAL_Delay(3000);
    
    printf("\r\nTest complete. Blinking LED.\r\n");
    
    /* Blink LED */
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}