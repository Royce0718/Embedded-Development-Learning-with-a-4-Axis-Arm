/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   1.The configuration of the CAN instances
 *          2.The configuration of the CAN Filter
 *          3.The HAL_CAN_RxFifo0MsgPendingCallback and Receive
 *          4.The CAN SendMessage
 ******************************************************************************
 * @instruction
 *	@filter  In terms of my targets,the four of the 16bits ID list is enough.
 *					 The below is specific functions of filter.
 *          1.ID1: Reset
 *				   2.ID2: Set (Angle,Velocity,Torque) (0xFF:remaining the current value)
 *					 3.ID3: Sendback (Angle,Velocity)
 *          4.ID4: Enable or Disenable the Motor
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart5;
// 11bits
uint16_t STID1 = 0x0001;
uint16_t STID2 = 0x0002;
uint16_t STID3 = 0x0003;
uint16_t STID4 = 0x0004;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 3;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = (STID1 << 5) | CAN_RTR_DATA | CAN_ID_STD;
  sFilterConfig.FilterIdLow = (STID2 << 5) | CAN_RTR_DATA | CAN_ID_STD;
  sFilterConfig.FilterMaskIdHigh = (STID3 << 5) | CAN_RTR_DATA | CAN_ID_STD;
  sFilterConfig.FilterMaskIdLow = (STID4 << 5) | CAN_RTR_DATA | CAN_ID_STD;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1)
  {
    uint8_t RxData[8];
    CAN_RxHeaderTypeDef CAN_RxHeader;

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN_RxHeader, RxData) == HAL_OK)
    {
      // print info for verifying
      HAL_UART_Transmit(&huart5, (uint8_t *)CAN_RxHeader.StdId, 1, 0xffff);
      HAL_UART_Transmit(&huart5, RxData, CAN_RxHeader.DLC, 0xffff);
    }
  }
}

// before sending info please configure variable of CAN_TxHeaderTypeDef
uint8_t HAL_CAN_SendTxMessage(CAN_TxHeaderTypeDef TxHeader, uint8_t *aData)
{
  uint32_t TxMailBox;
  uint8_t FreeTxMailBoxNum;

  while (0 == FreeTxMailBoxNum)
  {
    FreeTxMailBoxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
  }

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, aData, &TxMailBox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
  return 1;
}
/* USER CODE END 1 */
