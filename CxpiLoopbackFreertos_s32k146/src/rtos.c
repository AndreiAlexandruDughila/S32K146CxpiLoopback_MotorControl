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



void Pta11interruptHandler(void)
{
    if (PORTA->ISFR & (1 << 11u))
    {
        /* ---- USER CODE HERE ---- */
        /* Example: set a flag, toggle LED, etc. */

        /* Clear interrupt flag */
        PORTA->ISFR = (1 << 11u);
    }
}


static inline void PTA11InterruptInit(void)
{
    /* Enable clock for PORTA */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Configure PTA11 as GPIO with interrupt */
    PORTA->PCR[BTN_PIN] =
          PORT_PCR_MUX(1)        /* GPIO */
        | PORT_PCR_PE_MASK       /* Pull enable */
        | PORT_PCR_PS_MASK       /* Pull-up */
        | PORT_PCR_IRQC(0b1010); /* Interrupt on falling edge */

    /* Set PTA11 as input */
    PTA->PDDR &= ~(1 << 11u);

    /* Clear any pending interrupt flag */
    PORTA->ISFR = (1 << 11u);

    /* Enable PORTA interrupt in NVIC */


    INT_SYS_InstallHandler(PORTA_IRQn, Pta11interruptHandler, (isr_t *)NULL);
    INT_SYS_SetPriority( PORTA_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 3);
    NVIC_EnableIRQ(PORTA_IRQn);
}


void Pta17interruptHandler(void)
{
    if (PORTA->ISFR & (1 << 17u))
    {
        /* ---- USER CODE HERE ---- */
        /* Example: set a flag, toggle LED, etc. */

        /* Clear interrupt flag */
        PORTA->ISFR = (1 << 17u);
    }
}


static inline void PTA17InterruptInit(void)
{
    /* Enable clock for PORTA */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Configure PTA11 as GPIO with interrupt */
    PORTA->PCR[17u] =
          PORT_PCR_MUX(1)        /* GPIO */
        | PORT_PCR_PE_MASK       /* Pull enable */
        | PORT_PCR_PS_MASK       /* Pull-up */
        | PORT_PCR_IRQC(0b1010); /* Interrupt on falling edge */

    /* Set PTA11 as input */
    PTA->PDDR &= ~(1 << 17u);

    /* Clear any pending interrupt flag */
    PORTA->ISFR = (1 << 17u);

    /* Enable PORTA interrupt in NVIC */


    INT_SYS_InstallHandler(PORTA_IRQn, Pta11interruptHandler, (isr_t *)NULL);
    INT_SYS_SetPriority( PORTA_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 4);
    NVIC_EnableIRQ(PORTA_IRQn);
}

static inline void PTA27InitOutput(void)
{
    /* Enable clock for PORTA */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Set PTA27 mux to GPIO */
    PORTA->PCR[27u] = PORT_PCR_MUX(1);

    /* Configure as output */
    PTA->PDDR |= (1 << 27u);

    /* Optional: start LOW */
    PTA->PCOR = (1 << 27u);
}

static inline void PTA17SetHigh(void)
{
	PTA->PSOR = (1 << 27u);
}


static inline void PTA17SetLow(void)
{
	PTA->PCOR = (1 << 27u);
}

static inline void PTA17Toggle(void)
{
	PTA->PTOR = (1 << 27u);
}


static inline void PTD18InitOutput(void)
{
    /* Enable clock for PORTA */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Set PTA27 mux to GPIO */
    PORTD->PCR[18u] = PORT_PCR_MUX(1);

    /* Configure as output */
    PTD->PDDR |= (1 << 18u);

    /* Optional: start LOW */
    PTD->PCOR = (1 << 18u);
}

static inline void PTD18SetHigh(void)
{
	PTD->PSOR = (1 << 18u);
}


static inline void PTD18SetLow(void)
{
	PTD->PCOR = (1 << 18u);
}

static inline void PTD18Toggle(void)
{
	PTD->PTOR = (1 << 18u);
}


static inline void PTD15InitOutput(void)
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Set PTA15 mux to GPIO */
    PORTD->PCR[15u] = PORT_PCR_MUX(1);

    /* Configure as output */
    PTD->PDDR |= (1 << 15u);

    /* Optional: start LOW */
    PTD->PCOR = (1 << 15u);
}

static inline void PTD15SetHigh(void)
{
	PTD->PSOR = (1 << 15u);
}


static inline void PTD15SetLow(void)
{
	PTD->PCOR = (1 << 15u);
}

static inline void PTD15Toggle(void)
{
	PTD->PTOR = (1 << 15u);
}



static inline void PTD0InitOutput(void)
{
    /* Enable clock for PORTD */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Set PTD0 mux to GPIO */
    PORTD->PCR[0u] = PORT_PCR_MUX(1);

    /* Configure as output */
    PTD->PDDR |= (1 << 0u);

    /* Optional: start LOW */
    PTD->PCOR = (1 << 0u);
}

static inline void PTD0SetHigh(void)
{
	PTD->PSOR = (1 << 0u);
}


static inline void PTD0SetLow(void)
{
	PTD->PCOR = (1 << 0u);
}

static inline void PTD0Toggle(void)
{
	PTD->PTOR = (1 << 0u);
}




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

/* The queue used by both tasks. */
static QueueHandle_t xQueueCxpiRxMaster = NULL;
/* The queue used by both tasks. */
static QueueHandle_t xQueueCxpiRxUartSlave = NULL;
/* The queue used by both tasks. */
static QueueHandle_t xQueueCxpiRxSpiTableMaster = NULL;
/* The queue used by both tasks. */
static QueueHandle_t xQueueCxpiButtonMaster = NULL;

/* The LED software timer.  This uses prvButtonLEDTimerCallback() as its callback
function. */
static TimerHandle_t xButtonLEDTimer = NULL;

/*-----------------------------------------------------------*/

void rtos_start( void )
{
	/* Configure the NVIC, GPIO interrup, LPUART for Master and Slave interrupt. */
	prvSetupHardware();

	/* Create the queue. */
	xQueueCxpiRxMaster = xQueueCreate( 1u, sizeof( uint8_t ) );
	/* Create the queue. */
	xQueueCxpiRxUartSlave = xQueueCreate( 1u, sizeof( uint8_t ) );
	/* Create the queue. */
	xQueueCxpiRxSpiTableMaster = xQueueCreate( 1u, sizeof( uint8_t ) );

	/* Create the queue. */
	xQueueCxpiButtonMaster = xQueueCreate( 1u, sizeof( uint8_t ) );



	if(NULL != xQueueCxpiRxMaster &&
	   NULL != xQueueCxpiRxUartSlave &&
	   NULL != xQueueCxpiRxSpiTableMaster &&
	   NULL != xQueueCxpiButtonMaster)
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
			xQueueCxpiRxMaster,
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
			xQueueCxpiRxUartSlave,
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

	for (;;)
	{
		if (xQueueReceive(xQueueCxpiRxMaster, &rxByte, pdMS_TO_TICKS(10)) == pdPASS)
		{

		}
		CxpiPWMStop();
		vTaskDelay(pdMS_TO_TICKS(1000));
		Lpuart_SendData(0, 0x55);
		CxpiPWMStart();
		PTD0Toggle();
	}
}

static void CxpiSlaveTask(void *pvParameters)
{
	static uint8_t rxByte;

	for (;;)
	{
		if (xQueueReceive(xQueueCxpiRxUartSlave, &rxByte, pdMS_TO_TICKS(10)) == pdPASS)
		{

		}
		vTaskDelay(pdMS_TO_TICKS(10));
		Lpuart_SendData(2, 0x55);
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
	PORTD->PCR[6] |= PORT_PCR_MUX(2);                 /* Port D6: MUX = ALT2, UART2 TX */
	PORTD->PCR[7] |= PORT_PCR_MUX(2);                 /* Port D7: MUX = ALT2, UART2 RX */


    INT_SYS_InstallHandler(LPUART0_RxTx_IRQn, vLpuart0_ISRHandler, (isr_t *)NULL);
    INT_SYS_InstallHandler(LPUART2_RxTx_IRQn, vLpuart2_ISRHandler, (isr_t *)NULL);

	INT_SYS_EnableIRQ(LPUART0_RxTx_IRQn);
	INT_SYS_EnableIRQ(LPUART2_RxTx_IRQn);


    INT_SYS_SetPriority( LPUART0_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    INT_SYS_SetPriority( LPUART2_RxTx_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);

    Lpuart_Init(0U, 19200U, 11U);
    Lpuart_Init(2U, 19200U, 11U);
    PTD0InitOutput();
    PTD0SetHigh();
    CxpiPWMCenterAlignInit();
    CxpiPWMCenterAlignInitMotor();
	CxpiPWMStartMotor();
    CxpiPWMCenterAlignInitMotor1();
	CxpiPWMStartMotor1();


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
