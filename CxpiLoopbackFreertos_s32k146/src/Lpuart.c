/*
 * Lpuart.c
 *
 *  Created on: Jan 23, 2026
 *      Author: andre
 */


#include "../include/Lpuart.h"

const LPUART_Type* const LpuartBasePtr[LPUART_INSTANCE_COUNT] = {LPUART0_BASE, LPUART1_BASE, LPUART2_BASE};

const uint8_t LpuartPccIndex[LPUART_INSTANCE_COUNT] = {PCC_LPUART0_INDEX, PCC_LPUART1_INDEX, PCC_LPUART2_INDEX};



void Lpuart_Init(const uint8_t Instance, uint32_t Baudrate, uint32_t OversamplingRatio)
{

	uint32_t Sbr = 0u;
	float Sbr_f = 0.0f;

	LPUART_Type* const BasePtr = (LPUART_Type* const)LpuartBasePtr[Instance];
	const uint8_t PccIndex = LpuartPccIndex[Instance];


	PCC->PCCn[PccIndex] &= ~PCC_PCCn_CGC_MASK;    /* Ensure clk disabled for config */
	PCC->PCCn[PccIndex] |= PCC_PCCn_PCS(1u)       /* Clock Src= 1 (SOSCDIV2_CLK) */
	                       |  PCC_PCCn_CGC_MASK;     /* Enable clock for LPUART1 regs */


	/* baud = clock / ((osr + 1) * sbr) => baud * (osr + 1) * sbr = clock => sbr = clock / (baud * (osr + 1)) */
	/* SOSCDIV2_CLK = 8_000_000U */

	if(OversamplingRatio >= 3 && OversamplingRatio <= 31)
	{
		Sbr_f = (8000000.0f / (float)(Baudrate * (OversamplingRatio + 1)));
	}

	if((uint32_t)(Sbr_f + 0.5f) > (uint32_t)Sbr_f)
	{
		Sbr = (uint32_t)Sbr_f + 1u;
	}
	else
	{
		Sbr = (uint32_t)Sbr_f;
	}

    if(Sbr >= 1u && Sbr <= 8191u)
    {
    	BasePtr->BAUD = (OversamplingRatio << 24u | Sbr);

        /* BasePtr->BAUD = 0x0B000023;  */ /* Initialize for 19200 baud, 1 stop: */
    		                               /* SBR=35 (0x23): baud divisor = 8M/19200/12 = ~35 */
    		                               /* OSR=11: Over sampling ratio = 11+1=12 */
    		                               /* SBNS=0: One stop bit */
    		                               /* BOTHEDGE=0: receiver samples only on rising edge */
    		                               /* M10=0: Rx and Tx use 7 to 9 bit data characters */
    		                               /* RESYNCDIS=0: Resync during rec'd data word supported */
    		                               /* LBKDIE, RXEDGIE=0: interrupts disable */
    		                               /* TDMAE, RDMAE, TDMAE=0: DMA requests disabled */
    		                               /* MAEN1, MAEN2,  MATCFG=0: Match disabled */

		 BasePtr->CTRL=0x002C0000;     /* Enable transmitter & receiver, no parity, 8 bit char: */
									   /* RE=1: Receiver enabled */
									   /* TE=1: Transmitter enabled */
									   /* PE,PT=0: No hw parity generation or checking */
									   /* M7,M,R8T9,R9T8=0: 8-bit data characters*/
									   /* DOZEEN=0: LPUART enabled in Doze mode */
									   /* ORIE,NEIE,FEIE,PEIE,TIE,TCIE,RIE,ILIE,MA1IE,MA2IE=0: no IRQ*/
									   /* TxDIR=0: TxD pin is input if in single-wire mode */
									   /* TXINV=0: TRansmit data not inverted */
									   /* RWU,WAKE=0: normal operation; rcvr not in statndby */
									   /* IDLCFG=0: one idle character */
									   /* ILT=0: Idle char bit count starts after start bit */
									   /* SBK=0: Normal transmitter operation - no break char */
									   /* LOOPS,RSRC=0: no loop back */
    }
    else
    {
    	/* do nothing, initialization failed */
    }
}


void Lpuart_SendData(const uint8_t Instance, const uint8_t Data)
{
	LPUART_Type* const BasePtr = (LPUART_Type* const)LpuartBasePtr[Instance];
	while((BasePtr->STAT & LPUART_STAT_TDRE_MASK)>>LPUART_STAT_TDRE_SHIFT==0);
	                                   /* Wait for transmit buffer to be empty */

    /* Wait for TX buffer empty */
    while (!(BasePtr->STAT & LPUART_STAT_TDRE_MASK));

	BasePtr->DATA=Data;                /* Send data */

    /* Wait for transmission complete */
    while (!(BasePtr->STAT & LPUART_STAT_TC_MASK));
}


uint8_t Lpuart_GetData(const uint8_t Instance)
{
	uint8_t ReceiveData;
	const LPUART_Type* const BasePtr = LpuartBasePtr[Instance];
	while((BasePtr->STAT & LPUART_STAT_RDRF_MASK)>>LPUART_STAT_RDRF_SHIFT==0);
	                                     /* Wait for received buffer to be full */
	ReceiveData= BasePtr->DATA;          /* Read received data*/
	while((BasePtr->STAT & LPUART_STAT_RDRF_MASK)>>LPUART_STAT_RDRF_SHIFT==0);
	return ReceiveData;
}


void Lpuart_TransmitString(const uint8_t Instance, char DataString[])
{
  uint32_t Index = 0;
  while(DataString[Index] != '\0')
  {
	  Lpuart_SendData(Instance, (uint8_t)DataString[Index]);
	  Index++;
  }
}

void Lpuart_TransmitBuffer(const uint8_t Instance, uint8_t* const DataString, uint32_t Length)
{
  uint32_t Index = 0;
  if((void*)(0u) != DataString)
  {
	  while(Index < Length)
	  {
		  Lpuart_SendData(Instance, DataString[Index]);
		  Index++;
	  }
  }
  else
  {
  }
}


void Lpuart_TransmitEnable(const uint8_t Instance, const uint8_t Enable)
{
	LPUART_Type* const BasePtr = (LPUART_Type* const)LpuartBasePtr[Instance];
	BasePtr->CTRL = (BasePtr->CTRL & ~LPUART_CTRL_TE_MASK) | ((Enable ? 1U : 0U) << LPUART_CTRL_TE_SHIFT);
	while(((BasePtr->CTRL & LPUART_CTRL_TE_MASK) != 0U) != Enable){};
}


void Lpuart_RxInterruptEnable(const uint8_t Instance, const uint8_t Enable)
{
	LPUART_Type* const BasePtr = (LPUART_Type* const)LpuartBasePtr[Instance];
	BasePtr->CTRL = (BasePtr->CTRL & ~LPUART_CTRL_RIE_MASK) | ((Enable ? 1U : 0U) << LPUART_CTRL_RIE_SHIFT);
	while(((BasePtr->CTRL & LPUART_CTRL_RIE_MASK) != 0U) != Enable){};
}

void Lpuart_TxInterruptEnable(const uint8_t Instance, const uint8_t Enable)
{
	LPUART_Type* const BasePtr = (LPUART_Type* const)LpuartBasePtr[Instance];
	BasePtr->CTRL = (BasePtr->CTRL & ~LPUART_CTRL_TIE_MASK) | ((Enable ? 1U : 0U) << LPUART_CTRL_TIE_SHIFT);
	while(((BasePtr->CTRL & LPUART_CTRL_TIE_MASK) != 0U) != Enable){};
}













