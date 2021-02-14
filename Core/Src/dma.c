/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USER CODE BEGIN 2 */
void ConfigDMA()
{
	LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_7,
	                         (uint32_t)aTXBuffer,
	                         LL_USART_DMA_GetRegAddr(USART1),
	                         LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_7));
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, 6);


	 LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_2,
	                         LL_USART_DMA_GetRegAddr(USART1),
	                         (uint32_t)aRXBuffer,
	                         LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));

	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, 4);

	  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
	  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);
	  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
	  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);
}
/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
