/*
 * Lpuart.h
 *
 *  Created on: Jan 23, 202
 *      Author: andre
 */

#ifndef LPUART_H_
#define LPUART_H_


#include "S32K146.h"


#define LPUART0_INSTANCE 0U
#define LPUART1_INSTANCE 1U
#define LPUART2_INSTANCE 2U






void Lpuart_Init(const uint8_t Instance, uint32_t Baudrate, uint32_t OversamplingRatio);
void Lpuart_SendData(const uint8_t Instance, const uint8_t Data);
uint8_t Lpuart_GetData(const uint8_t Instance);
void Lpuart_TransmitString(const uint8_t Instance, char DataString[]);
void Lpuart_TransmitBuffer(const uint8_t Instance, uint8_t* const DataString, uint32_t Length);
void Lpuart_TransmitEnable(const uint8_t Instance, const uint8_t Enable);
void Lpuart_RxInterruptEnable(const uint8_t Instance, const uint8_t Enable);
void Lpuart_TxInterruptEnable(const uint8_t Instance, const uint8_t Enable);

#endif /* LPUART_H_ */
