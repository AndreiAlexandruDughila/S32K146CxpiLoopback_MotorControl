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
    uint8_t          dlc;        /* Data Length Code */
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
	uint8_t info;
	uint8_t* data;
	uint8_t crc;
} Cxpi_Frame_Structure_t;



typedef struct
{
	bool IdleBus;
	bool InitNode;
	uint16_t ByteCnt;
	bool TxPending;
	bool RxPending;
	Cxpi_Byte_t TxByte;
	Cxpi_Byte_t RxByte;
	Cxpi_Node_t NodeType;
	Cxpi_Communication_t CommunicationType;
	Cxpi_Frame_Structure_t CurrentFrame;
	Cxpi_FrameTableEntry_t* Cxpi_FrameTable;
	uint8_t Cxpi_FrameTable_Size;
} Cxpi_Node_Status;







#endif /* CXPI_H_ */
