/*
 * Cxpi.c
 *
 *  Created on: Feb 1, 2026
 *      Author: andre
 */





#include "../include/Cxpi.h"


/* Frame data buffers (RAM) */
static uint8_t Cxpi_Frame0_DataMaster[2] = {0x01u, 0x28u};
static uint8_t Cxpi_Frame1_DataMaster[2] = {0x01u, 0x46u};
static uint8_t Cxpi_Frame2_DataMaster[2] = {0x02u, 0x14u};
static uint8_t Cxpi_Frame3_DataMaster[1] = {0x00u};


static uint8_t Cxpi_Frame0_DataSlave[2];
static uint8_t Cxpi_Frame1_DataSlave[2];
static uint8_t Cxpi_Frame2_DataSlave[2];
static uint8_t Cxpi_Frame3_DataSlave[1];

/* Frame table (RAM, FLASH) */
Cxpi_FrameTableEntry_t Cxpi_FrameTableMaster[CXPI_FRAME_TABLE_MAX_ENTRIES] =
{
    /* Initial entries */
    { 0x22, 0x02u, CXPI_FRAME_TX, Cxpi_Frame0_DataMaster },
    { 0x23, 0x02u, CXPI_FRAME_TX, Cxpi_Frame1_DataMaster },
    { 0x24, 0x02u, CXPI_FRAME_TX, Cxpi_Frame2_DataMaster },
    { 0x25, 0x01u, CXPI_FRAME_TX, Cxpi_Frame3_DataMaster }

};

/* Frame table (RAM, FLASH) */
Cxpi_FrameTableEntry_t Cxpi_FrameTableSlave[CXPI_FRAME_TABLE_MAX_ENTRIES] =
{
    /* Initial entries */
	/* Initial entries */
	{ 0x22, 0x02u, CXPI_FRAME_RX, Cxpi_Frame0_DataSlave },
	{ 0x23, 0x02u, CXPI_FRAME_RX, Cxpi_Frame1_DataSlave },
	{ 0x24, 0x02u, CXPI_FRAME_RX, Cxpi_Frame2_DataSlave },
	{ 0x25, 0x01u, CXPI_FRAME_RX, Cxpi_Frame3_DataSlave }
};

/* Current number of valid entries */
uint8_t Cxpi_FrameTableMaster_Size = 4;

/* Current number of valid entries */
uint8_t Cxpi_FrameTableSlave_Size = 4;

extern Cxpi_Event EventMaster;
extern Cxpi_Event EventSlave;


bool Cxpi_FrameTable_Add(uint8_t pid, Cxpi_FrameDir_t dir, uint8_t length, uint8_t *data, Cxpi_Node_t nodetype)
{
	bool ReturnValue = false;
	if(nodetype == MASTER_NODE)
	{
	    if (Cxpi_FrameTableMaster_Size >= CXPI_FRAME_TABLE_MAX_ENTRIES)
	    {
	    	ReturnValue = false;
	    }
	    else
	    {
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].pid         = pid;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].datalength  = length;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].direction   = dir;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].data        = data;
	        Cxpi_FrameTableMaster_Size++;
	    	ReturnValue = true;
	    }
	}
	else
	if(nodetype == SLAVE_NODE)
	{
	    if (Cxpi_FrameTable_Size >= CXPI_FRAME_TABLE_MAX_ENTRIES)
	    {
	    	ReturnValue = false;
	    }
	    else
	    {
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].pid         = pid;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].datalength  = length;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].direction   = dir;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].data        = data;
	        Cxpi_FrameTableSlave_Size++;
	    	ReturnValue = true;
	    }
	}
    return ReturnValue;
}

Cxpi_FrameTableEntry_t* Cxpi_FrameTable_FindByPid(uint8_t pid, Cxpi_Node_t nodetype)
{
	Cxpi_FrameTableEntry_t* TableEntry = (void*)0u;
	if(nodetype == MASTER_NODE)
	{
	    for (uint8_t i = 0; i < Cxpi_FrameTableMaster_Size; i++)
	    {
	        if (Cxpi_FrameTableMaster[i].pid == pid)
	        {
	        	TableEntry = &Cxpi_FrameTableMaster[i];
	        }
	    }

	}
	else
	if(nodetype == SLAVE_NODE)
	{

	}
    for (uint8_t i = 0; i < Cxpi_FrameTableSlave_Size; i++)
    {
	    for (uint8_t i = 0; i < Cxpi_FrameTableSlave_Size; i++)
	    {
	        if (Cxpi_FrameTableSlave[i].pid == pid)
	        {
	        	TableEntry = &Cxpi_FrameTableSlave[i];
	        }
	    }
    }
    return TableEntry;
}





Cxpi_Node_Status CxpiMasterNode =
{
		true,
		true,
		false,
		false,
		0u,
		{
			0u,
			NO_ERROR,
		},
		{
			0u,
			NO_ERROR,
		},
		MASTER_NODE,
		CXPI_NO_PENDING,
		EVENT_MODE,
		{
			0u,
			0u,
			0u,
			0u,
			(void*)0u,
			0u,
		},
		2u
};


Cxpi_Node_Status CxpiSlaveNode =
{
		true,
		false,
		false,
		false,
		0u,
		{
			0u,
			NO_ERROR,
		},
		{
			0u,
			NO_ERROR,
		},
		SLAVE_NODE,
		CXPI_NO_PENDING,
		EVENT_MODE,
		{
			0u,
			0u,
			0u,
			0u,
			(void*)0u,
			0u,
		},
		0u
};



void CxpiProcessRxByte(uint8_t RxByte, Cxpi_Node_t nodetype)
{
	Cxpi_Node_Status* NodeStatus = (void*)0u;
	Cxpi_FrameTableEntry_t* TableEntry;
	uint8_t Info;
	if(nodetype == MASTER_NODE && (void*)0u != Cxpi_FrameTableMaster)
	{
		NodeStatus = &CxpiMasterNode;
	}
	else
	if(nodetype == SLAVE_NODE && (void*)0u != Cxpi_FrameTableSlave)
	{
		NodeStatus = &CxpiSlaveNode;
	}
	else
	{
	}

	switch(NodeStatus->FrameNodeStatus)
	{
		case CXPI_NO_PENDING:
		case CXPI_IDLE:
		{
			TableEntry = Cxpi_FrameTable_FindByPid(RxByte & 0x7F, nodetype);
			NodeStatus->CurrentFrame.pid = TableEntry->pid;
			NodeStatus->CurrentFrame.data = TableEntry->data;

			NodeStatus->FrameNodeStatus = CXPI_WAIT_INFO;

			break;
		}
		case CXPI_WAIT_ID:
		{
			if(NodeStatus->TxPending)
			{
				PTA->PTOR = (1 << 25);
				Info = (NodeStatus->CurrentFrame.dlc << 4u) | (NodeStatus->CurrentFrame.nm << 2u) | (NodeStatus->CurrentFrame.ct);
				Lpuart_SendData(NodeStatus->LpuartInstance, Info);
				NodeStatus->FrameNodeStatus = CXPI_WAIT_INFO;
			}
			else
			if(NodeStatus->RxPending)
			{

			}
			break;
		}
		case CXPI_WAIT_INFO:
		{
			if(NodeStatus->TxPending)
			{

				if(NodeStatus->ByteCnt < NodeStatus->CurrentFrame.dlc )
				{
					Lpuart_SendData(NodeStatus->LpuartInstance, (NodeStatus->CurrentFrame.data[NodeStatus->ByteCnt]));
					NodeStatus->ByteCnt++;
					NodeStatus->FrameNodeStatus = CXPI_WAIT_DATA;
				}
				else
				{
					Lpuart_SendData(NodeStatus->LpuartInstance, (NodeStatus->CurrentFrame.crc));
					NodeStatus->FrameNodeStatus = CXPI_WAIT_CRC;
				}
			}
			else
			if(NodeStatus->RxPending)
			{
				NodeStatus->CurrentFrame.dlc = (RxByte >> 4u);
				NodeStatus->CurrentFrame.nm = (RxByte >> 2u) & 0x03u;
				NodeStatus->CurrentFrame.ct = (RxByte) & 0x03u;
				if(NodeStatus->ByteCnt < NodeStatus->CurrentFrame.dlc )
				{
					NodeStatus->FrameNodeStatus = CXPI_WAIT_DATA;
				}
				else
				{
					NodeStatus->FrameNodeStatus = CXPI_WAIT_CRC;
				}
			}
			else
			{

			}
			break;
		}

		case CXPI_WAIT_DATA:
		{
			if(NodeStatus->TxPending)
			{
				if(NodeStatus->ByteCnt < NodeStatus->CurrentFrame.dlc )
				{
					Lpuart_SendData(NodeStatus->LpuartInstance, (NodeStatus->CurrentFrame.data[NodeStatus->ByteCnt]));
					NodeStatus->ByteCnt++;
					NodeStatus->FrameNodeStatus = CXPI_WAIT_DATA;
				}
				else
				{
					Lpuart_SendData(NodeStatus->LpuartInstance, (NodeStatus->CurrentFrame.crc));
					NodeStatus->FrameNodeStatus = CXPI_WAIT_CRC;
				}
			}
			else
			if(NodeStatus->RxPending)
			{

				NodeStatus->CurrentFrame.data[NodeStatus->ByteCnt] = RxByte;
				NodeStatus->ByteCnt++;
				if(NodeStatus->ByteCnt < NodeStatus->CurrentFrame.dlc)
				{
					NodeStatus->FrameNodeStatus = CXPI_WAIT_DATA;
				}
				else
				{
					NodeStatus->FrameNodeStatus = CXPI_WAIT_CRC;
				}

			}
			else
			{

			}
			break;
		}
		case CXPI_WAIT_CRC:
		{
			if(NodeStatus->TxPending)
			{
				NodeStatus->ByteCnt = 0u;
				NodeStatus->IdleBus = true;
				NodeStatus->TxPending = false;
				NodeStatus->RxPending = false;
				NodeStatus->FrameNodeStatus = CXPI_IDLE;
				if(nodetype == MASTER_NODE)
				{
					EventMaster = CXPI_TX_INDICATION;
				}
				else
				{
					EventSlave = CXPI_TX_INDICATION;
				}

			}
			else
			if(NodeStatus->RxPending)
			{
				NodeStatus->CurrentFrame.crc = RxByte;

				NodeStatus->ByteCnt = 0u;
				NodeStatus->IdleBus = true;
				NodeStatus->TxPending = false;
				NodeStatus->RxPending = false;
				NodeStatus->FrameNodeStatus = CXPI_IDLE;
				if(nodetype == MASTER_NODE)
				{
					EventMaster = CXPI_RX_INDICATION;
				}
				else
				{
					EventSlave = CXPI_RX_INDICATION;
				}
			}
		}
	}
}


void CxpiSendFrame(uint8_t IndexFrame, Cxpi_Node_t nodetype)
{
	Cxpi_Node_Status* NodeStatus = (void*)0u;
	uint8_t index;

	if(nodetype == MASTER_NODE && (void*)0u != Cxpi_FrameTableMaster)
	{
		NodeStatus = &CxpiMasterNode;
	}
	if(nodetype == SLAVE_NODE && (void*)0u != Cxpi_FrameTableSlave)
	{
		NodeStatus = &CxpiSlaveNode;
	}
	else
	{

	}


	if(nodetype == MASTER_NODE && (NodeStatus->IdleBus == true) && (NodeStatus->TxPending != true) && (NodeStatus->RxPending != true))
	{
		if(IndexFrame < Cxpi_FrameTableMaster_Size)
		{
			index = IndexFrame;
			IndexFrame++;
		}
		else
		{
			IndexFrame = 0u;
			index = 0u;
		}

		NodeStatus->CurrentFrame.pid = CxpiComputeParity(Cxpi_FrameTableMaster[index].pid, COMPUTE_PARITY);
		NodeStatus->CurrentFrame.dlc = Cxpi_FrameTableMaster[index].datalength;
		NodeStatus->CurrentFrame.nm = 0u;
		NodeStatus->CurrentFrame.ct = 0u;
		NodeStatus->CurrentFrame.data = &Cxpi_FrameTableMaster[index].data[0];
		NodeStatus->CurrentFrame.crc = CxpiCrc8(Cxpi_FrameTableMaster[index].data, Cxpi_FrameTableMaster[index].datalength);
		NodeStatus->IdleBus = false;
		NodeStatus->TxPending = true;
		NodeStatus->RxPending = false;
		NodeStatus->FrameNodeStatus = CXPI_WAIT_ID;
		Lpuart_SendData(NodeStatus->LpuartInstance, NodeStatus->CurrentFrame.pid);
	}
	else
    if(nodetype == SLAVE_NODE && (NodeStatus->IdleBus == true) && (NodeStatus->TxPending != true) && (NodeStatus->RxPending != true))
	{
    	/* to do */
	}
}


uint8_t CxpiComputeParity(uint8_t PID, Cxpi_ComputeParity ComputeParity)
{
	uint8_t PidParityBit = PID >> 7u;
	uint8_t CalculatedParityBit = 0x00;
	uint8_t CalculatdPid = 0x00u;
	if(COMPUTE_PARITY == ComputeParity)
	{
		if(PidParityBit == 1u)
		{
			CalculatdPid = 0xFF;
		}
		else
		{
			for(uint8_t i=0; i<6; i++)
			{
				CalculatedParityBit ^= (PID >> i) & 0x01;
			}
			CalculatdPid = (CalculatedParityBit << 7) | PID;
		}
	}
	else
	if(CHECK_PARITY == ComputeParity)
	{
		for(uint8_t i=0; i<6; i++)
		{
			CalculatedParityBit ^= (PID >> i) & 0x01;
		}
		if(PidParityBit == CalculatedParityBit)
		{
			CalculatdPid = (CalculatedParityBit << 7) | PID;
		}
		else
		{
			CalculatdPid = 0xFF;
		}
	}
	else
	{
		CalculatdPid = 0xFF;
	}
	return CalculatdPid;
}


uint8_t CxpiCrc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;

    while (len--)
    {
        crc ^= *data++;

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x1D);
            else
                crc <<= 1;
        }
    }

    return (uint8_t)(crc ^ 0xFF);
}


