/*
 * Cxpi.c
 *
 *  Created on: Feb 1, 2026
 *      Author: andre
 */





#include "../include/Cxpi.h"


/* Frame data buffers (RAM) */
static uint8_t Cxpi_Frame0_Data[8];
static uint8_t Cxpi_Frame1_Data[8];

/* Frame table (RAM, FLASH) */
Cxpi_FrameTableEntry_t Cxpi_FrameTableMaster[CXPI_FRAME_TABLE_MAX_ENTRIES] =
{
    /* Initial entries */
    { 0x22, 8, CXPI_FRAME_TX, Cxpi_Frame0_Data },
    { 0x24, 8, CXPI_FRAME_RX, Cxpi_Frame1_Data }
};

/* Frame table (RAM, FLASH) */
Cxpi_FrameTableEntry_t Cxpi_FrameTableSlave[CXPI_FRAME_TABLE_MAX_ENTRIES] =
{
    /* Initial entries */
    { 0x22, 8, CXPI_FRAME_RX, Cxpi_Frame0_Data },
    { 0x24, 8, CXPI_FRAME_TX, Cxpi_Frame1_Data }
};

/* Current number of valid entries */
uint8_t Cxpi_FrameTableMaster_Size = 2;

/* Current number of valid entries */
uint8_t Cxpi_FrameTableSlave_Size = 2;

bool Cxpi_FrameTable_Add(uint8_t pid, uint8_t dlc, Cxpi_FrameDir_t dir, uint8_t *data, Cxpi_Node_t nodetype)
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
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].pid       = pid;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].dlc       = dlc;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].direction = dir;
	        Cxpi_FrameTableMaster[Cxpi_FrameTableMaster_Size].data      = data;
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
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].pid       = pid;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].dlc       = dlc;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].direction = dir;
	        Cxpi_FrameTableSlave[Cxpi_FrameTableSlave_Size].data      = data;
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
		0u,
		false,
		false,
		{
			0u,
			NO_ERROR,
		},
		{
			0u,
			NO_ERROR,
		},
		MASTER_NODE,
		EVENT_MODE,
		{
			0u,
			0u,
			(void*)0u,
			0u,
		},
		(void*)0u,
		0u
};


Cxpi_Node_Status CxpiSlaveNode =
{
		true,
		false,
		0u,
		false,
		false,
		{
			0u,
			NO_ERROR,
		},
		{
			0u,
			NO_ERROR,
		},
		SLAVE_NODE,
		EVENT_MODE,
		{
			0u,
			0u,
			(void*)0u,
			0u,
		},
		(void*)0u,
		0u
};


void CxpiInit(Cxpi_Node_t nodetype)
{
	if(nodetype == MASTER_NODE && (void*)0u != Cxpi_FrameTableMaster)
	{
		CxpiMasterNode.Cxpi_FrameTable = &Cxpi_FrameTableMaster[0U];
		CxpiMasterNode.Cxpi_FrameTable_Size = Cxpi_FrameTableMaster_Size;
	}
	else
	if(nodetype == SLAVE_NODE && (void*)0u != Cxpi_FrameTableSlave)
	{
		CxpiSlaveNode.Cxpi_FrameTable = &Cxpi_FrameTableSlave[0U];
		CxpiSlaveNode.Cxpi_FrameTable_Size = Cxpi_FrameTableSlave_Size;
	}
	else
	{

	}
}


void CxpiProcessFrame(Cxpi_Node_t nodetype)
{
	Cxpi_Node_Status* NodeStatus;
}



