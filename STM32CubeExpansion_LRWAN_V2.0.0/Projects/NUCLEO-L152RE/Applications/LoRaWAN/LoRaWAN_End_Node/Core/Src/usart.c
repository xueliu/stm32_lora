/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef husart2;
DMA_HandleTypeDef hdma_tx;

/* USART2 init function */

void MX_USART2_Init(void)
{

  husart2.Instance        = USART2;
  husart2.Init.BaudRate   = USART_BAUDRATE;
  husart2.Init.WordLength = UART_WORDLENGTH_8B;
  husart2.Init.StopBits   = UART_STOPBITS_1;
  husart2.Init.Parity     = UART_PARITY_NONE;
  husart2.Init.Mode       = UART_MODE_TX;
  husart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;

  if (HAL_UART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */

    /* USER CODE END USART2_MspInit 0 */

    /* Enable peripherals and GPIO Clocks */
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();

    /* Enable USARTx clock */
    __USART2_CLK_ENABLE();

    /* Enable DMA clock */
    DMAx_CLK_ENABLE();

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();



    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PA2     ------> USART2_TX
      */
    GPIO_InitStruct.Pin = USARTx_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;
    HAL_GPIO_Init(USARTx_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USARTx_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;
    HAL_GPIO_Init(USARTx_RX_GPIO_Port, &GPIO_InitStruct);


    /* USART2 DMA Init */
    /* USART2_TX Init */
    /* Configure the DMA handler for Transmission process */
    hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
    hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_tx.Init.Mode                = DMA_NORMAL;
    hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

    if (HAL_DMA_Init(&hdma_tx) != HAL_OK)
    {
      Error_Handler();
    }


    /* Associate the initialized DMA handle to the UART handle */
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

    /* Configure the NVIC for DMA */
    /* NVIC configuration for DMA transfer complete interrupt (USART1_TX) */
    HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, USARTx_Priority, 1);
    HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

    /* NVIC for USART, to catch the TX complete */
    HAL_NVIC_SetPriority(USARTx_IRQn, USARTx_DMA_Priority, 1);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);

    /* USER CODE BEGIN USART2_MspInit 1 */

    /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */

    /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA3     ------> USART2_RX
    PA2     ------> USART2_TX
      */
    HAL_GPIO_DeInit(GPIOA, USARTx_RX_Pin | USARTx_TX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    /* USER CODE BEGIN USART2_MspDeInit 1 */

    /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
