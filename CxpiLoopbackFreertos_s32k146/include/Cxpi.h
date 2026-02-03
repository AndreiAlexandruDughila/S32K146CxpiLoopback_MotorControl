/*
 * Cxpi.h
 *
 *  Created on: Feb 1, 2026
 *      Author: andre
 */

#ifndef CXPI_H_
#define CXPI_H_


#include "S32K146.h"
#include "stdbool.h"
#include "stddef.h"


typedef enum
{
	MASTER_NODE,
	SLAVE_NODE
} Cxpi_Node_t;

typedef enum
{
	EVENT_MODE,
	POLLING_MODE
} Cxpi_Communication_t;


typedef enum
{
    CXPI_FRAME_TX,
    CXPI_FRAME_RX
} Cxpi_FrameDir_t;

typedef struct
{
    uint8_t          pid;        /* Protected Identifier */
    uint8_t          datalength;
    Cxpi_FrameDir_t  direction;  /* TX or RX */
    uint8_t         *data;       /* Pointer to data buffer, includes info, actual data and crc */
} Cxpi_FrameTableEntry_t;

/* Maximum allowed entries */
#define CXPI_FRAME_TABLE_MAX_ENTRIES  0xFu

/* Runtime-modifiable frame table */
extern Cxpi_FrameTableEntry_t Cxpi_FrameTable[];

/* Current number of entries */
extern uint8_t Cxpi_FrameTable_Size;



typedef enum
{
	NO_ERROR,
	FRAMERROR,
	OVERUNERROR
} Cxpi_Erros_t;

typedef struct
{
	uint8_t Byte;
	Cxpi_Erros_t Error;
} Cxpi_Byte_t;


typedef struct
{
	uint8_t pid;
	uint8_t dlc;
	uint8_t nm;
	uint8_t ct;
	uint8_t* data;
	uint8_t crc;
} Cxpi_Frame_Structure_t;

typedef enum
{
	CXPI_NO_PENDING,
	CXPI_IDLE,
	CXPI_WAIT_ID,
	CXPI_WAIT_INFO,
	CXPI_WAIT_DATA,
	CXPI_WAIT_CRC
} Cxpi_Frame_NodeStatus_t;

typedef enum
{
	CXPI_NO_INDICATION,
	CXPI_RX_INDICATION,
	CXPI_TX_INDICATION
} Cxpi_Event;


typedef enum
{
	CHECK_PARITY,
	COMPUTE_PARITY
} Cxpi_ComputeParity;



typedef struct
{
	bool IdleBus;
	bool InitNode;
	bool TxPending;
	bool RxPending;
	uint16_t ByteCnt;
	Cxpi_Byte_t TxByte;
	Cxpi_Byte_t RxByte;
	Cxpi_Node_t NodeType;
	Cxpi_Frame_NodeStatus_t FrameNodeStatus;
	Cxpi_Communication_t CommunicationType;
	Cxpi_Frame_Structure_t CurrentFrame;
	uint8_t LpuartInstance;
} Cxpi_Node_Status;




uint8_t CxpiComputeParity(uint8_t PID, Cxpi_ComputeParity ComputeParity);
uint8_t CxpiCrc8(const uint8_t *data, uint8_t len);
void CxpiProcessRxByte(uint8_t RxByte, Cxpi_Node_t nodetype);


#endif /* CXPI_H_ */
