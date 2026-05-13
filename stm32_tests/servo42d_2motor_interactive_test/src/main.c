/**
 * MKS SERVO42D Interactive CAN Controller - Dual Motor Version
 *
 * Real-time control via serial terminal (115200 baud)
 * Supports two motors with CAN IDs 0x01 and 0x02
 *
 * Commands:
 *   1/2     - Select motor 1 or 2
 *   *       - Select both motors (broadcast mode)
 *   w/s     - Increase/decrease speed (10 RPM steps)
 *   W/S     - Increase/decrease speed (100 RPM steps)
 *   a/d     - Jog CCW/CW (200 pulses)
 *   A/D     - Jog CCW/CW (1600 pulses = half turn)
 *   space   - Stop motor(s)
 *   0       - Set current position as zero
 *   e       - Toggle motor enable
 *   ?       - Show status (current motor or both)
 *   h       - Show help
 *
 * Or type numbers:
 *   +500    - Run CW at 500 RPM
 *   -200    - Run CCW at 200 RPM
 *   p3200   - Move 3200 pulses CW (1 rotation)
 *   p-3200  - Move 3200 pulses CCW
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Handles */
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* Motor Configuration */
#define NUM_MOTORS 2
#define MOTOR_ID_1 0x01
#define MOTOR_ID_2 0x02
#define MOTOR_ID_BOTH 0xFF /* Special value for broadcast mode */

/* MKS Command Codes */
#define CMD_READ_ENCODER2 0x31
#define CMD_READ_SPEED 0x32
#define CMD_QUERY_STATUS 0xF1
#define CMD_ENABLE_MOTOR 0xF3
#define CMD_SPEED_MODE 0xF6
#define CMD_POSITION_MODE1 0xFD
#define CMD_SET_ZERO 0x92

/* Per-motor state */
typedef struct
{
    uint8_t canId;
    int16_t targetSpeed;
    uint8_t enabled;
    int32_t lastEncoder;
    int16_t lastSpeed;
    uint8_t lastStatus;
} MotorState;

MotorState motors[NUM_MOTORS] = {
    {.canId = MOTOR_ID_1, .targetSpeed = 0, .enabled = 1, .lastEncoder = 0, .lastSpeed = 0, .lastStatus = 0},
    {.canId = MOTOR_ID_2, .targetSpeed = 0, .enabled = 1, .lastEncoder = 0, .lastSpeed = 0, .lastStatus = 0}};

/* Current selection: 0 = motor 1, 1 = motor 2, 0xFF = both */
uint8_t selectedMotor = 0;

/* Function prototypes */
void SystemClock_Config(void);
void USART2_Init(void);
void LED_Init(void);
void CAN1_Init(void);
void Error_Handler(void);

/* MKS functions - now take motor ID as parameter */
uint8_t MKS_CalcChecksum(uint8_t canId, uint8_t *data, uint8_t len);
HAL_StatusTypeDef MKS_SendCommand(uint8_t canId, uint8_t *data, uint8_t len);
HAL_StatusTypeDef MKS_ReceiveResponse(uint8_t *data, uint8_t *len, uint32_t timeout);
void MKS_FlushRx(void);
int32_t MKS_ReadEncoder(uint8_t canId);
int16_t MKS_ReadSpeed(uint8_t canId);
uint8_t MKS_QueryStatus(uint8_t canId);
HAL_StatusTypeDef MKS_EnableMotor(uint8_t canId, uint8_t enable);
HAL_StatusTypeDef MKS_SpeedMode(uint8_t canId, uint8_t dir, uint16_t speed, uint8_t acc);
HAL_StatusTypeDef MKS_StopMotor(uint8_t canId, uint8_t acc);
HAL_StatusTypeDef MKS_PositionMode(uint8_t canId, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses);
HAL_StatusTypeDef MKS_SetZero(uint8_t canId);

/* Helper functions */
MotorState *GetMotorByIndex(uint8_t index);
MotorState *GetMotorById(uint8_t canId);
void ForEachSelectedMotor(void (*func)(MotorState *));
void PrintPrompt(void);

/* Printf redirect */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void Error_Handler(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

/* ========== Helper Functions ========== */

MotorState *GetMotorByIndex(uint8_t index)
{
    if (index < NUM_MOTORS)
    {
        return &motors[index];
    }
    return NULL;
}

MotorState *GetMotorById(uint8_t canId)
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        if (motors[i].canId == canId)
        {
            return &motors[i];
        }
    }
    return NULL;
}

void PrintPrompt(void)
{
    if (selectedMotor == 0xFF)
    {
        printf("M*> ");
    }
    else
    {
        printf("M%d> ", selectedMotor + 1);
    }
}

/* ========== MKS SERVO42D Functions ========== */

uint8_t MKS_CalcChecksum(uint8_t canId, uint8_t *data, uint8_t len)
{
    uint16_t sum = canId;
    for (int i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

HAL_StatusTypeDef MKS_SendCommand(uint8_t canId, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint32_t startTick = HAL_GetTick();

    /* Wait for free mailbox */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - startTick) > 10)
        {
            return HAL_TIMEOUT;
        }
    }

    txHeader.StdId = canId;
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = len;
    txHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
}

HAL_StatusTypeDef MKS_ReceiveResponse(uint8_t *data, uint8_t *len, uint32_t timeout)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeout)
    {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
        {
            HAL_StatusTypeDef status = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data);
            if (status == HAL_OK)
            {
                *len = rxHeader.DLC;
                return HAL_OK;
            }
        }
    }
    return HAL_TIMEOUT;
}

void MKS_FlushRx(void)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, data);
    }
}

int32_t MKS_ReadEncoder(uint8_t canId)
{
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_READ_ENCODER2;
    txData[1] = MKS_CalcChecksum(canId, txData, 1);

    if (MKS_SendCommand(canId, txData, 2) != HAL_OK)
        return 0;
    if (MKS_ReceiveResponse(rxData, &rxLen, 50) != HAL_OK)
        return 0;

    if (rxLen >= 7 && rxData[0] == CMD_READ_ENCODER2)
    {
        int64_t value = 0;
        for (int i = 1; i <= 6; i++)
        {
            value = (value << 8) | rxData[i];
        }
        if (value & 0x800000000000LL)
        {
            value |= 0xFFFF000000000000LL;
        }
        return (int32_t)value;
    }
    return 0;
}

int16_t MKS_ReadSpeed(uint8_t canId)
{
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_READ_SPEED;
    txData[1] = MKS_CalcChecksum(canId, txData, 1);

    if (MKS_SendCommand(canId, txData, 2) != HAL_OK)
        return 0;
    if (MKS_ReceiveResponse(rxData, &rxLen, 50) != HAL_OK)
        return 0;

    if (rxLen >= 3 && rxData[0] == CMD_READ_SPEED)
    {
        return (int16_t)((rxData[1] << 8) | rxData[2]);
    }
    return 0;
}

uint8_t MKS_QueryStatus(uint8_t canId)
{
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_QUERY_STATUS;
    txData[1] = MKS_CalcChecksum(canId, txData, 1);

    if (MKS_SendCommand(canId, txData, 2) != HAL_OK)
        return 0;
    if (MKS_ReceiveResponse(rxData, &rxLen, 50) != HAL_OK)
        return 0;

    if (rxLen >= 2 && rxData[0] == CMD_QUERY_STATUS)
    {
        return rxData[1];
    }
    return 0;
}

HAL_StatusTypeDef MKS_EnableMotor(uint8_t canId, uint8_t enable)
{
    uint8_t txData[3];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_ENABLE_MOTOR;
    txData[1] = enable ? 0x01 : 0x00;
    txData[2] = MKS_CalcChecksum(canId, txData, 2);

    if (MKS_SendCommand(canId, txData, 3) != HAL_OK)
        return HAL_ERROR;
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK)
        return HAL_TIMEOUT;

    return (rxLen >= 2 && rxData[0] == CMD_ENABLE_MOTOR && rxData[1] == 0x01) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MKS_SpeedMode(uint8_t canId, uint8_t dir, uint16_t speed, uint8_t acc)
{
    uint8_t txData[5];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_SPEED_MODE;
    txData[1] = (dir ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
    txData[2] = speed & 0xFF;
    txData[3] = acc;
    txData[4] = MKS_CalcChecksum(canId, txData, 4);

    if (MKS_SendCommand(canId, txData, 5) != HAL_OK)
        return HAL_ERROR;
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK)
        return HAL_TIMEOUT;

    return (rxLen >= 2 && rxData[0] == CMD_SPEED_MODE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MKS_StopMotor(uint8_t canId, uint8_t acc)
{
    return MKS_SpeedMode(canId, 0, 0, acc);
}

HAL_StatusTypeDef MKS_PositionMode(uint8_t canId, uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    uint8_t txData[8];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_POSITION_MODE1;
    txData[1] = (dir ? 0x80 : 0x00) | ((speed >> 8) & 0x0F);
    txData[2] = speed & 0xFF;
    txData[3] = acc;
    txData[4] = (pulses >> 16) & 0xFF;
    txData[5] = (pulses >> 8) & 0xFF;
    txData[6] = pulses & 0xFF;
    txData[7] = MKS_CalcChecksum(canId, txData, 7);

    if (MKS_SendCommand(canId, txData, 8) != HAL_OK)
        return HAL_ERROR;
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK)
        return HAL_TIMEOUT;

    return (rxLen >= 2 && rxData[0] == CMD_POSITION_MODE1 && rxData[1] > 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MKS_SetZero(uint8_t canId)
{
    uint8_t txData[2];
    uint8_t rxData[8];
    uint8_t rxLen;

    MKS_FlushRx();
    txData[0] = CMD_SET_ZERO;
    txData[1] = MKS_CalcChecksum(canId, txData, 1);

    if (MKS_SendCommand(canId, txData, 2) != HAL_OK)
        return HAL_ERROR;
    if (MKS_ReceiveResponse(rxData, &rxLen, 100) != HAL_OK)
        return HAL_TIMEOUT;

    return (rxLen >= 2 && rxData[0] == CMD_SET_ZERO && rxData[1] == 0x01) ? HAL_OK : HAL_ERROR;
}

/* ========== Peripheral Init ========== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

void USART2_Init(void)
{
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

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void CAN1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    CAN_FilterTypeDef canFilter;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 2;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
        Error_Handler();

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

    if (HAL_CAN_ConfigFilter(&hcan1, &canFilter) != HAL_OK)
        Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();
}

/* ========== Command Processing ========== */

void SetMotorSpeed(MotorState *motor, int16_t speed)
{
    motor->targetSpeed = speed;
    if (speed == 0)
    {
        MKS_StopMotor(motor->canId, 50);
    }
    else if (speed > 0)
    {
        MKS_SpeedMode(motor->canId, 1, (uint16_t)speed, 50);
    }
    else
    {
        MKS_SpeedMode(motor->canId, 0, (uint16_t)(-speed), 50);
    }
}

void SetSelectedSpeed(int16_t speed)
{
    if (speed > 3000)
        speed = 3000;
    if (speed < -3000)
        speed = -3000;

    if (selectedMotor == 0xFF)
    {
        /* Both motors */
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            SetMotorSpeed(&motors[i], speed);
        }
    }
    else
    {
        SetMotorSpeed(&motors[selectedMotor], speed);
    }
}

int16_t GetSelectedTargetSpeed(void)
{
    if (selectedMotor == 0xFF)
    {
        return motors[0].targetSpeed; /* Return motor 1's speed as reference */
    }
    return motors[selectedMotor].targetSpeed;
}

void AdjustSelectedSpeed(int16_t delta)
{
    if (selectedMotor == 0xFF)
    {
        /* Adjust both motors */
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            int16_t newSpeed = motors[i].targetSpeed + delta;
            if (newSpeed > 3000)
                newSpeed = 3000;
            if (newSpeed < -3000)
                newSpeed = -3000;
            SetMotorSpeed(&motors[i], newSpeed);
        }
        printf("Speed: M1=%d M2=%d\r\n", motors[0].targetSpeed, motors[1].targetSpeed);
    }
    else
    {
        MotorState *motor = &motors[selectedMotor];
        int16_t newSpeed = motor->targetSpeed + delta;
        if (newSpeed > 3000)
            newSpeed = 3000;
        if (newSpeed < -3000)
            newSpeed = -3000;
        SetMotorSpeed(motor, newSpeed);
        printf("Speed: %d\r\n", motor->targetSpeed);
    }
}

void StopSelectedMotors(void)
{
    if (selectedMotor == 0xFF)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motors[i].targetSpeed = 0;
            MKS_StopMotor(motors[i].canId, 50);
        }
    }
    else
    {
        motors[selectedMotor].targetSpeed = 0;
        MKS_StopMotor(motors[selectedMotor].canId, 50);
    }
}

void JogSelectedMotors(uint8_t dir, uint16_t speed, uint8_t acc, uint32_t pulses)
{
    if (selectedMotor == 0xFF)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            MKS_PositionMode(motors[i].canId, dir, speed, acc, pulses);
        }
    }
    else
    {
        MKS_PositionMode(motors[selectedMotor].canId, dir, speed, acc, pulses);
    }
}

void SetZeroSelectedMotors(void)
{
    if (selectedMotor == 0xFF)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            if (MKS_SetZero(motors[i].canId) == HAL_OK)
            {
                printf("M%d: Zero OK  ", i + 1);
            }
            else
            {
                printf("M%d: Zero FAIL  ", i + 1);
            }
        }
        printf("\r\n");
    }
    else
    {
        if (MKS_SetZero(motors[selectedMotor].canId) == HAL_OK)
        {
            printf("Zero set OK\r\n");
        }
        else
        {
            printf("Zero set FAILED\r\n");
        }
    }
}

void ToggleEnableSelectedMotors(void)
{
    if (selectedMotor == 0xFF)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            motors[i].enabled = !motors[i].enabled;
            MKS_EnableMotor(motors[i].canId, motors[i].enabled);
            printf("M%d: %s  ", i + 1, motors[i].enabled ? "ON" : "OFF");
        }
        printf("\r\n");
    }
    else
    {
        MotorState *motor = &motors[selectedMotor];
        motor->enabled = !motor->enabled;
        MKS_EnableMotor(motor->canId, motor->enabled);
        printf("Motor %s\r\n", motor->enabled ? "ENABLED" : "DISABLED");
    }
}

void PrintMotorStatus(MotorState *motor, int index)
{
    int32_t enc = MKS_ReadEncoder(motor->canId);
    int16_t spd = MKS_ReadSpeed(motor->canId);
    uint8_t stat = MKS_QueryStatus(motor->canId);

    motor->lastEncoder = enc;
    motor->lastSpeed = spd;
    motor->lastStatus = stat;

    float degrees = (float)enc * 360.0f / 16384.0f;

    printf("--- Motor %d (ID 0x%02X) ---\r\n", index + 1, motor->canId);
    printf("Encoder: %ld (%.1f deg)\r\n", enc, degrees);
    printf("Speed: %d RPM (target: %d)\r\n", spd, motor->targetSpeed);
    printf("Status: %d ", stat);
    switch (stat)
    {
    case 1:
        printf("(stopped)");
        break;
    case 2:
        printf("(accel)");
        break;
    case 3:
        printf("(decel)");
        break;
    case 4:
        printf("(running)");
        break;
    default:
        printf("(?)");
        break;
    }
    printf("\r\nEnabled: %s\r\n", motor->enabled ? "YES" : "NO");
}

void PrintStatus(void)
{
    printf("\r\n");
    if (selectedMotor == 0xFF)
    {
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            PrintMotorStatus(&motors[i], i);
            printf("\r\n");
        }
    }
    else
    {
        PrintMotorStatus(&motors[selectedMotor], selectedMotor);
    }
    printf("--------------------------\r\n");
}

void PrintHelp(void)
{
    printf("\r\n");
    printf("=== MKS SERVO42D Dual Motor Controller ===\r\n");
    printf("Motor Selection:\r\n");
    printf("  1       - Select motor 1 (ID 0x01)\r\n");
    printf("  2       - Select motor 2 (ID 0x02)\r\n");
    printf("  *       - Select both motors\r\n");
    printf("\r\n");
    printf("Control (applies to selected motor(s)):\r\n");
    printf("  w/s     - Speed up/down (10 RPM steps)\r\n");
    printf("  W/S     - Speed up/down (100 RPM steps)\r\n");
    printf("  a/d     - Jog CCW/CW (200 pulses)\r\n");
    printf("  A/D     - Jog CCW/CW (1600 pulses = half turn)\r\n");
    printf("  space   - Stop motor(s)\r\n");
    printf("  0       - Set current position as zero\r\n");
    printf("  e       - Toggle enable\r\n");
    printf("  ?       - Show status\r\n");
    printf("  h       - Show this help\r\n");
    printf("\r\n");
    printf("Typed Commands:\r\n");
    printf("  +500    - Run CW at 500 RPM\r\n");
    printf("  -200    - Run CCW at 200 RPM\r\n");
    printf("  p3200   - Move 3200 pulses CW (1 turn)\r\n");
    printf("  p-6400  - Move 6400 pulses CCW (2 turns)\r\n");
    printf("==========================================\r\n\r\n");
}

void ProcessCommand(char *cmd)
{
    if (cmd[0] == '+' || cmd[0] == '-' || (cmd[0] >= '0' && cmd[0] <= '9'))
    {
        /* Speed command: +500, -200, 300 */
        int speed = atoi(cmd);
        if (speed > 3000)
            speed = 3000;
        if (speed < -3000)
            speed = -3000;
        printf("Speed: %d RPM\r\n", speed);
        SetSelectedSpeed(speed);
    }
    else if (cmd[0] == 'p' || cmd[0] == 'P')
    {
        /* Position command: p3200, p-3200 */
        int pulses = atoi(&cmd[1]);
        uint8_t dir = (pulses >= 0) ? 1 : 0;
        uint32_t absPulses = (pulses >= 0) ? pulses : -pulses;
        printf("Position: %d pulses %s\r\n", (int)absPulses, dir ? "CW" : "CCW");
        JogSelectedMotors(dir, 2000, 5, absPulses);
    }
}

/* ========== Main ========== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    LED_Init();
    USART2_Init();
    CAN1_Init();

    printf("\r\n\r\n");
    PrintHelp();

    /* Enable both motors */
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        MKS_EnableMotor(motors[i].canId, 1);
        motors[i].enabled = 1;
        printf("Motor %d (ID 0x%02X) enabled\r\n", i + 1, motors[i].canId);
    }

    printf("\r\nReady! Type 'h' for help, '?' for status\r\n\r\n");
    PrintPrompt();

    char cmdBuffer[32];
    uint8_t cmdIndex = 0;
    uint32_t lastStatusTime = 0;

    while (1)
    {
        /* Check for serial input */
        uint8_t ch;
        if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
        {

            /* Single character commands */
            if (cmdIndex == 0)
            {
                switch (ch)
                {
                case '1': /* Select motor 1 */
                    selectedMotor = 0;
                    printf("Motor 1 selected\r\n");
                    PrintPrompt();
                    continue;

                case '2': /* Select motor 2 */
                    selectedMotor = 1;
                    printf("Motor 2 selected\r\n");
                    PrintPrompt();
                    continue;

                case '*': /* Select both motors */
                    selectedMotor = 0xFF;
                    printf("Both motors selected\r\n");
                    PrintPrompt();
                    continue;

                case 'w': /* Speed up 10 */
                    AdjustSelectedSpeed(10);
                    PrintPrompt();
                    continue;

                case 'W': /* Speed up 100 */
                    AdjustSelectedSpeed(100);
                    PrintPrompt();
                    continue;

                case 's': /* Speed down 10 */
                    AdjustSelectedSpeed(-10);
                    PrintPrompt();
                    continue;

                case 'S': /* Speed down 100 */
                    AdjustSelectedSpeed(-100);
                    PrintPrompt();
                    continue;

                case ' ': /* Stop */
                    StopSelectedMotors();
                    printf("STOP\r\n");
                    PrintPrompt();
                    continue;

                case 'a': /* Jog CCW small */
                    printf("Jog CCW 200\r\n");
                    JogSelectedMotors(0, 200, 50, 200);
                    PrintPrompt();
                    continue;

                case 'A': /* Jog CCW large */
                    printf("Jog CCW 1600\r\n");
                    JogSelectedMotors(0, 300, 30, 1600);
                    PrintPrompt();
                    continue;

                case 'd': /* Jog CW small */
                    printf("Jog CW 200\r\n");
                    JogSelectedMotors(1, 200, 50, 200);
                    PrintPrompt();
                    continue;

                case 'D': /* Jog CW large */
                    printf("Jog CW 1600\r\n");
                    JogSelectedMotors(1, 300, 30, 1600);
                    PrintPrompt();
                    continue;

                case '0': /* Set zero */
                    SetZeroSelectedMotors();
                    PrintPrompt();
                    continue;

                case 'e': /* Toggle enable */
                    ToggleEnableSelectedMotors();
                    PrintPrompt();
                    continue;

                case '?': /* Status */
                    PrintStatus();
                    PrintPrompt();
                    continue;

                case 'h':
                case 'H': /* Help */
                    PrintHelp();
                    PrintPrompt();
                    continue;
                }
            }

            /* Build command buffer for multi-char commands */
            if (ch == '\r' || ch == '\n')
            {
                if (cmdIndex > 0)
                {
                    cmdBuffer[cmdIndex] = '\0';
                    printf("\r\n");
                    ProcessCommand(cmdBuffer);
                    cmdIndex = 0;
                }
                PrintPrompt();
            }
            else if (ch == 127 || ch == 8)
            { /* Backspace */
                if (cmdIndex > 0)
                {
                    cmdIndex--;
                    printf("\b \b");
                }
            }
            else if (ch >= 32 && ch < 127 && cmdIndex < sizeof(cmdBuffer) - 1)
            {
                cmdBuffer[cmdIndex++] = ch;
                printf("%c", ch); /* Echo */
            }
        }

        /* Periodic status update (every 500ms if any motor running) */
        uint8_t anyRunning = 0;
        for (int i = 0; i < NUM_MOTORS; i++)
        {
            if (motors[i].targetSpeed != 0)
                anyRunning = 1;
        }

        if (anyRunning && (HAL_GetTick() - lastStatusTime) > 500)
        {
            lastStatusTime = HAL_GetTick();

            printf("\r[");
            for (int i = 0; i < NUM_MOTORS; i++)
            {
                motors[i].lastEncoder = MKS_ReadEncoder(motors[i].canId);
                motors[i].lastSpeed = MKS_ReadSpeed(motors[i].canId);
                printf("M%d:%ld/%d ", i + 1, motors[i].lastEncoder, motors[i].lastSpeed);
            }
            printf("] ");
            PrintPrompt();

            /* Redraw any partial command */
            for (int i = 0; i < cmdIndex; i++)
            {
                printf("%c", cmdBuffer[i]);
            }
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }
}