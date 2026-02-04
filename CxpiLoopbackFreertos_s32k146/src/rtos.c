/*
 * Copyright (c) 2015 - 2016 , Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * main-blinky.c is included when the "Blinky" build configuration is used.
 * main-full.c is included when the "Full" build configuration is used.
 *
 * main-blinky.c (this file) defines a very simple demo that creates two tasks,
 * one queue, and one timer.  It also demonstrates how Cortex-M4 interrupts can
 * interact with FreeRTOS tasks/timers.
 *
 * This simple demo project runs 'stand alone' (without the rest of the tower
 * system) on the Freedom Board or Validation Board, which is populated with a
 * S32K146 Cortex-M4 microcontroller.
 *
 * The idle hook function:
 * The idle hook function demonstrates how to query the amount of FreeRTOS heap
 * space that is remaining (see vApplicationIdleHook() defined in this file).
 *
 * The main() Function:
 * main() creates one software timer, one queue, and two tasks.  It then starts
 * the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds, before sending the value 100 to the queue that
 * was created within main().  Once the value is sent, the task loops back
 * around to block for another 200 milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop that causes it to
 * repeatedly attempt to read data from the queue that was created within
 * main().  When data is received, the task checks the value of the data, and
 * if the value equals the expected 100, toggles the green LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the task
 * should be held in the Blocked state indefinitely to wait for data to be
 * available on the queue.  The queue receive task will only leave the Blocked
 * state when the queue send task writes to the queue.  As the queue send task
 * writes to the queue every 200 milliseconds, the queue receive task leaves the
 * Blocked state every 200 milliseconds, and therefore toggles the blue LED
 * every 200 milliseconds.
 *
 * The LED Software Timer and the Button Interrupt:
 * The user button BTN1 is configured to generate an interrupt each time it is
 * pressed.  The interrupt service routine switches the red LED on, and
 * resets the LED software timer.  The LED timer has a 5000 millisecond (5
 * second) period, and uses a callback function that is defined to just turn the
 * LED off again.  Therefore, pressing the user button will turn the LED on, and
 * the LED will remain on until a full five seconds pass without the button
 * being pressed.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* SDK includes. */
#include "interrupt_manager.h"
#include "sdk_project_config.h"
#include "../include/Lpuart.h"
#include "../include/Pwm_Ftm.h"
#include "../include/Cxpi.h"

#include "BoardDefines.h"


/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )



#define	mainQUEUE_Slave_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_Master_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )


/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 200 / portTICK_PERIOD_MS )

/* The LED will remain on until the button has not been pushed for a full
5000ms. */
#define mainBUTTON_LED_TIMER_PERIOD_MS		( 5000UL / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 32 )

/* The LED toggle by the queue receive task (blue). */
#define mainTASK_CONTROLLED_LED				( 1UL << 0UL )

/* The LED turned on by the button interrupt, and turned off by the LED timer
(green). */
#define mainTIMER_CONTROLLED_LED			( 1UL << 1UL )

/* The vector used by the GPIO port C.  Button SW7 is configured to generate
an interrupt on this port. */
#define mainGPIO_C_VECTOR					( 61 )

/* A block time of zero simply means "don't block". */
#define mainDONT_BLOCK						( 0UL )

/*-----------------------------------------------------------*/

 PORT_Type* PORTx[PORT_INSTANCE_COUNT] = {
     PORTA, PORTB, PORTC, PORTD, PORTE
 };

 GPIO_Type* PTx[GPIO_INSTANCE_COUNT] = {
     PTA, PTB, PTC, PTD, PTE
 };

 uint32_t PCC_PORTx_INDEX[PORT_INSTANCE_COUNT] = {
     PCC_PORTA_INDEX,
     PCC_PORTB_INDEX,
     PCC_PORTC_INDEX,
     PCC_PORTD_INDEX,
     PCC_PORTE_INDEX
 };

typedef struct
{
	isr_t NewHandler;
	uint8_t Priority;
} IrqInfoHandler;


typedef enum
{
	MOTOR_STOPPED,
	MOTOR_FORWARD,
	MOTOR_REVERSE
} CxpiMotorRotation;

/*
 * Setup the NVIC, LED outputs, and button inputs.
 */
static void prvSetupHardware( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void CxpiMasterTask(void *pvParameters);
static void CxpiSlaveTask(void *pvParameters);


/*-----------------------------------------------------------*/

/* The queue used by Master Task. */
static QueueHandle_t xQueueCxpiUartRxMaster = NULL;

/* The queue used by Slave Task. */
static QueueHandle_t xQueueCxpiUartRxSlave = NULL;

/* The queue used by Master Task. */
static QueueHandle_t xQueueCxpiButton = NULL;

/* The queue used by Master Task */
static QueueHandle_t xQueueCxpiGpioInterruptMaster = NULL;

/* The queue used by Slave Task. */
static QueueHandle_t xQueueCxpiGpioInterruptSlave = NULL;

Cxpi_Event EventMaster;
Cxpi_Event EventSlave;

extern uint8_t Cxpi_FrameTableMaster_Size;

static void MasterPTA11InterruptHandler(void)
{
	static bool MasterInterrupt = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (PORTA->ISFR & (1 << 11u))
    {
        /* ---- USER CODE HERE ---- */
        /* Example: set a flag, toggle LED, etc. */

        /* Clear interrupt flag */
        PORTA->ISFR = (1 << 11u);

		xQueueSendFromISR(
			xQueueCxpiGpioInterruptMaster,
			&MasterInterrupt,
			&xHigherPriorityTaskWoken
		);
    }
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


/* button */
static void ButtonPTC13InterruptHandler(void)
{
	static bool ButtonPressed = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (PORTC->ISFR & (1 << 13u))
    {
        /* ---- USER CODE HERE ---- */
        /* Example: set a flag, toggle LED, etc. */

        /* Clear interrupt flag */
        PORTC->ISFR = (1 << 13u);

		xQueueSendFromISR(
			xQueueCxpiButton,
			&ButtonPressed,
			&xHigherPriorityTaskWoken
		);
    }
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}


static void SlavePTE16InterruptHandler(void)
{
	static bool SlaveInterrupt = false;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (PORTE->ISFR & (1 << 16u))
    {
        /* ---- USER CODE HERE ---- */
        /* Example: set a flag, toggle LED, etc. */

        /* Clear interrupt flag */
        PORTE->ISFR = (1 << 16u);

		xQueueSendFromISR(
			xQueueCxpiGpioInterruptSlave,
			&SlaveInterrupt,
			&xHigherPriorityTaskWoken
		);
    }
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}



const IrqInfoHandler IrqInfoButton =
{
	ButtonPTC13InterruptHandler,
	5u
};

const IrqInfoHandler IrqInfoGpioMaster =
{
	MasterPTA11InterruptHandler,
	3u
};

const IrqInfoHandler IrqInfoGpioSlave =
{
	SlavePTE16InterruptHandler,
	4u
};


void GpioInputInterruptInit(const uint32_t Index, uint8_t Pin, IrqInfoHandler HandlerInfo, IRQn_Type IrqNumber)
{
    PCC->PCCn[PCC_PORTx_INDEX[Index]] |= PCC_PCCn_CGC_MASK;
    PORT_Type* portx = (PORT_Type*)PORTx[Index];
    GPIO_Type* ptx = (GPIO_Type*)PTx[Index];

    /* Configure PTxPIN as GPIO with interrupt */
    portx->PCR[Pin] =
          PORT_PCR_MUX(1)        /* GPIO */
        | PORT_PCR_PE_MASK       /* Pull enable */
        | PORT_PCR_PS_MASK       /* Pull-up */
        | PORT_PCR_IRQC(0b1010); /* Interrupt on falling edge */

    /* Set PTx Pin as input */
    ptx->PDDR &= ~(1 << 11u);

    /* Clear any pending interrupt flag */
    portx->ISFR = (1 << 11u);

    INT_SYS_InstallHandler(IrqNumber, HandlerInfo.NewHandler, (isr_t *)NULL);
    INT_SYS_SetPriority(IrqNumber, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + HandlerInfo.Priority);
	INT_SYS_EnableIRQ(IrqNumber);
}


void GpioOutputInit(const uint32_t Index, uint8_t Pin)
{
    /* Enable clock for PORTx*/
    PCC->PCCn[PCC_PORTx_INDEX[Index]] |= PCC_PCCn_CGC_MASK;
    PORT_Type* portx = (PORT_Type*)PORTx[Index];
    GPIO_Type* ptx = (GPIO_Type*)PTx[Index];

    /* Set ptx pin mux to GPIO */
    portx->PCR[Pin] = PORT_PCR_MUX(1);

    /* Configure as output */
    ptx->PDDR |= (1 << Pin);

    /* Optional: start LOW */
    ptx->PCOR = (1 << Pin);
}


void GpioOutputSetHigh(const uint32_t Index, uint8_t Pin)
{
    GPIO_Type* ptx = (GPIO_Type*)PTx[Index];
	ptx->PSOR = (1 << Pin);
}


void GpioOutputSetLow(const uint32_t Index, uint8_t Pin)
{
    GPIO_Type* ptx = (GPIO_Type*)PTx[Index];
	ptx->PCOR = (1 << Pin);
}

void GpioOutputToggle(const uint32_t Index, uint8_t Pin)
{
    GPIO_Type* ptx = (GPIO_Type*)PTx[Index];
	ptx->PTOR = (1 << Pin);
}



static inline void MasterGpioInterruptDisable(void)
{
	INT_SYS_DisableIRQ(PORTA_IRQn);
}

static inline void MasterGpioInterruptEnable(void)
{
	INT_SYS_EnableIRQ(PORTA_IRQn);
}

static inline void SlaveGpioInterruptDisable(void)
{
	INT_SYS_DisableIRQ(PORTE_IRQn);
}

static inline void SlaveGpioInterruptEnable(void)
{
	INT_SYS_EnableIRQ(PORTE_IRQn);
}


static inline void MasterGpioInterruptInit(void)
{
	GpioInputInterruptInit(0u, 11u, IrqInfoGpioMaster, PORTA_IRQn);
}

static inline void SlaveGpioInterruptInit(void)
{
	GpioInputInterruptInit(4u, 16u, IrqInfoGpioSlave, PORTE_IRQn);
}



static inline void MasterEnableInit()
{
	GpioOutputInit(0u, 27u);
}


static inline void SlaveEnableInit(void)
{
	GpioOutputInit(0u, 28u);
}

static inline void SlaveEnable(void)
{
	GpioOutputSetHigh(0u, 28);
}

static inline void SlaveDisable(void)
{
	GpioOutputSetHigh(0u, 27);
}


static inline void MasterEnable()
{
	GpioOutputSetHigh(0u, 27u);
}

static inline void MasterDisable()
{
	GpioOutputSetLow(0u, 27u);
}



static inline void ButtonInit(void)
{
	GpioInputInterruptInit(2u, 13u, IrqInfoButton, PORTC_IRQn);
	GpioOutputInit(3u, 0u);    /* Prepare Led */
	GpioOutputSetHigh(3u, 0u); /* Set Led Pin High */
}


static void MotorInit(void)
{
	GpioOutputInit(2u, 30u);
	GpioOutputSetHigh(2u, 30u);
	GpioOutputInit(2u, 31u);
	GpioOutputSetLow(2u, 31u);
	CxpiPWMCenterAlignInitMotor();
}


static void MotorSpeed(uint8_t Speed, CxpiMotorRotation MotorRotation)
{
	if(MOTOR_STOPPED == MotorRotation)
	{
		CxpiPwmMotorSpeed(0u, false);
		GpioOutputSetLow(2u, 31u);
	}
	else
	if(MOTOR_REVERSE == MotorRotation)
	{
		CxpiPwmMotorSpeed(Speed, true);
		GpioOutputSetHigh(2u, 31u);
	}
	if(MOTOR_FORWARD == MotorRotation)
	{
		CxpiPwmMotorSpeed(Speed, false);
		GpioOutputSetLow(2u, 31u);
	}
	else
	{

	}
}

static void MotorControl(void)
{
	MotorSpeed(DutyCycle, (CxpiMotorRotation)Direction);
}

void rtos_start( void )
{
	/* Configure the NVIC, GPIO interrup, LPUART for Master and Slave interrupt. */
	prvSetupHardware();

	/* Create the queue. */
	xQueueCxpiUartRxMaster = xQueueCreate( 1u, sizeof( uint8_t ) );

	/* Create the queue. */
	xQueueCxpiUartRxSlave = xQueueCreate( 1u, sizeof( uint8_t ) );

	/* Create the queue. */
	xQueueCxpiButton = xQueueCreate( 1u, sizeof( uint8_t ) );

	/* Create the queue. */
	xQueueCxpiGpioInterruptMaster = xQueueCreate( 1u, sizeof( bool ) );

	/* Create the queue. */
	xQueueCxpiGpioInterruptSlave = xQueueCreate( 1u, sizeof( bool ) );




	if(NULL != xQueueCxpiUartRxMaster &&
	   NULL != xQueueCxpiUartRxSlave &&
	   NULL != xQueueCxpiGpioInterruptMaster &&
	   NULL != xQueueCxpiGpioInterruptSlave &&
	   NULL != xQueueCxpiButton)
	{
		(void)xTaskCreate(CxpiMasterTask, "CXPI Master Task", 512, NULL, mainQUEUE_Master_TASK_PRIORITY, NULL);
		(void)xTaskCreate(CxpiSlaveTask, "CXPI Slave Task", 512, NULL, mainQUEUE_Slave_TASK_PRIORITY, NULL);

	}


	vTaskStartScheduler();

}
/*-----------------------------------------------------------*/






/* The ISR executed when the user button is pushed. */
void vLpuart0_ISRHandler( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t rxByte;

	/* RX data register full */
	if (LPUART0->STAT & LPUART_STAT_RDRF_MASK)
	{
		rxByte = (uint8_t)LPUART0->DATA;


		xQueueSendFromISR(
			xQueueCxpiUartRxMaster,
			&rxByte,
			&xHigherPriorityTaskWoken
		);
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/


/* The ISR executed when the user button is pushed. */
void vLpuart2_ISRHandler( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static uint8_t rxByte;

	/* RX data register full */
	if (LPUART2->STAT & LPUART_STAT_RDRF_MASK)
	{
		rxByte = (uint8_t)LPUART2->DATA;


		xQueueSendFromISR(
			xQueueCxpiUartRxSlave,
			&rxByte,
			&xHigherPriorityTaskWoken
		);
	}

	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/


static void CxpiMasterTask(void *pvParameters)
{
	static uint8_t rxByte;
	static bool ButtonPressed;

	uint8_t LedLightCnt = 0u;
	uint8_t BtnCnt = 0u;
	bool Init = false;


	for (;;)
	{
		if (xQueueReceive(xQueueCxpiUartRxMaster, &rxByte, pdMS_TO_TICKS(0x10)) == pdPASS)
		{
			CxpiProcessRxByte(rxByte, MASTER_NODE);
			if(CXPI_TX_INDICATION == EventMaster)
			{

			}
			else
			if(CXPI_RX_INDICATION == EventMaster)
			{

			}
		}

		if(xQueueReceive(xQueueCxpiButton, &ButtonPressed, pdMS_TO_TICKS(0x10)) == pdPASS)
		{
			if(false == Init)
			{
				Init = true;
				/*
				MasterEnableInit();
				SlaveEnableInit();
				MasterEnable();
				SlaveEnable();
				*/
			}
			else
			{
				if(LedLightCnt > 5u)
				{
					CxpiSendFrame(BtnCnt, MASTER_NODE);
					BtnCnt++;
					if(BtnCnt == Cxpi_FrameTableMaster_Size)
					{
						BtnCnt = 0;
					}
					else
					{

					}
				}
			}

		}

		if(true == Init)
		{
			if(LedLightCnt <= 5u)
			{
				vTaskDelay(pdMS_TO_TICKS(200));
				/*
					GpioOutputToggle(0u, 27u);
					GpioOutputToggle(0u, 25u);
					GpioOutputToggle(3u, 0u);
				*/
				GpioOutputToggle(0u, 25u);

				LedLightCnt++;
			}
		}

	}
}

static void CxpiSlaveTask(void *pvParameters)
{
	static uint8_t rxByte;

	for (;;)
	{
		if (xQueueReceive(xQueueCxpiUartRxSlave, &rxByte, pdMS_TO_TICKS(0x100)) == pdPASS)
		{
			CxpiProcessRxByte(rxByte, SLAVE_NODE);
			if(CXPI_TX_INDICATION == EventMaster)
			{

			}
			else
			if(CXPI_RX_INDICATION == EventMaster)
			{
				MotorControl();
				EventMaster = CXPI_NO_INDICATION;
			}
		}
	}
}



static void prvSetupHardware( void )
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                   g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_AGREEMENT);


	PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;  /* Enable clock for PORTB */
	PORTB->PCR[0] |= PORT_PCR_MUX(2);                 /* Port B0: MUX = ALT2, UART0 RX */
	PORTB->PCR[1] |= PORT_PCR_MUX(2);                 /* Port B1: MUX = ALT2, UART0 TX */

	PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;  /* Enable clock for PORTD */
	PORTD->PCR[6] |= PORT_PCR_MUX(2);                 /* Port D6: MUX = ALT2, UART2 RX */
	PORTD->PCR[7] |= PORT_PCR_MUX(2);                 /* Port D7: MUX = ALT2, UART2 TX */


    INT_SYS_InstallHandler(LPUART0_RxTx_IRQn, vLpuart0_ISRHandler, (isr_t *)NULL);
    INT_SYS_InstallHandler(LPUART2_RxTx_IRQn, vLpuart2_ISRHandler, (isr_t *)NULL);

	INT_SYS_EnableIRQ(LPUART0_RxTx_IRQn);
	INT_SYS_EnableIRQ(LPUART2_RxTx_IRQn);


    INT_SYS_SetPriority( LPUART0_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    INT_SYS_SetPriority( LPUART2_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

    Lpuart_Init(0U, 19600, 15U);
    Lpuart_Init(2U, 19600, 15U);


	MasterEnableInit();
	MasterEnable();
	SlaveEnableInit();
	SlaveEnable();


	GpioOutputInit(0u, 25u);
	GpioOutputSetHigh(0u, 25u);

	for(volatile uint32_t x = 0u; x <= 21000; x++)
	{
		x++;
	}



	for(volatile uint32_t x = 0u; x <= 21000; x++)
	{
		x++;
	}


	CxpiPWMCenterAlignInit();
	CxpiPWMStart();
	MotorInit();

	GpioInputInterruptInit(2u, 13u, IrqInfoButton, PORTC_IRQn);


}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
    volatile size_t xFreeHeapSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	if( xFreeHeapSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}

}
/*-----------------------------------------------------------*/

/* The Blinky build configuration does not include run time stats gathering,
however, the Full and Blinky build configurations share a FreeRTOSConfig.h
file.  Therefore, dummy run time stats functions need to be defined to keep the
linker happy. */
void vMainConfigureTimerForRunTimeStats( void ) {}
unsigned long ulMainGetRunTimeCounterValue( void ) { return 0UL; }

/* A tick hook is used by the "Full" build configuration.  The Full and blinky
build configurations share a FreeRTOSConfig.h header file, so this simple build
configuration also has to define a tick hook - even though it does not actually
use it for anything. */
void vApplicationTickHook( void ) {}

/*-----------------------------------------------------------*/
