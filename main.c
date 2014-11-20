/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 ***NOTE*** The exception to the GPL is included to allow you to distribute
    a combined work that includes FreeRTOS without being obliged to provide the
    source code for proprietary components outside of the FreeRTOS kernel.
    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
 */

/*
 * Yeonil Yoo
 * Zack Parker
 * Rich Hemingway
 * Antonio Orozco
 */

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


/* Demo includes. */
#include "basic_io.h"

/* Library includes. */
#include "LPC17xx.h"

/* The interrupt number to use for the software interrupt generation.  This
could be any unused number.  In this case the first chip level (non system)
interrupt is used, which happens to be the watchdog on the LPC1768. */
#define mainSW_INTERRUPT_ID		( 0 )

/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()	NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()	NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

/* The priority of the software interrupt.  The interrupt service routine uses
an (interrupt safe) FreeRTOS API function, so the priority of the interrupt must
be equal to or lower than the priority set by
configMAX_SYSCALL_INTERRUPT_PRIORITY - remembering that on the Cortex-M3 high
numeric values represent low priority values, which can be confusing as it is
counter intuitive. */
#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 5 )

/* Dimensions the buffer into which messages destined for stdout are placed. */
#define mainMAX_MSG_LEN	( 80 )

#define LED0_GPIO_PORT_NUM                      0
#define LED0_GPIO_BIT_NUM                       22

/* The tasks to be created. */
static void vReadADCTask( void *pvParameters );
static void vPeriodicTask( void *pvParameters );
static void vHandlerTask( void *pvParameters );
static void vFormatTask( void *pvParameters );
static void vPrintTask( void *pvParameters );
static void vCalculateFreqTask( void *pvParameters );
static void vTrackFreqTask( void *pvParameters );
static void vButtonHandlerTask( void *pvParameters );

void buttonInterrupt(void);
static int filterADC(int adc_val);

static void prvSetupSoftwareInterrupt();
static void prvSetupHardware(void);

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */
void vSoftwareInterruptHandler( void );


/*-----------------------------------------------------------*/

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore, xReadADCSemaphore, xFilterSemaphore,
				 xTrackSemaphore, xCaclculateSemaphore, xButtonPressSemaphore,
				 xMutexFreq, xMutexTime, xMutexReadADC, xMutexSampleRate;

/**
 * Queue that used at vHandlerTask and vFormatTask. vHandlerTask will queue ADC value,
 * and vFormatTask will dequeue the ADC value.
 */
xQueueHandle xPrintQueue;

/**
 * Clock that will stored by vPeriodicTask using Mutex, and
 * the clock will by used and reset by vFormatTask also by using Mutex
 */
volatile uint32_t rtc_clock_ctr;

/**
 * Frequency that will be calculated and stored by vFormatTask using Mutex,
 * and vPrintTask will take data by using Mutex also.
 */
volatile unsigned int filterVal;

/**
 * Real Time value since last button interrupt.
 */
volatile unsigned int time;

/**
 * Frequency value calculated from latest ADC readings.
 */
volatile unsigned int currentFreq;

/**
 * ADC Sampling rate determined by the latest frequency readings, adjusted for
 * accuracy, and time and energy savings.
 */
volatile unsigned int sampleRate;

/*-----------------------------------------------------------*/
/* Define the strings that the tasks and interrupt will print out via the gatekeeper. */
//static char *pcStringToPrint[] =
//{
//	"\nCurrent Frequency: ",
//	"\nFrequency Change: "
//};
int main( void )
{

	NVIC_SetPriority(21, 4);
	NVIC_EnableIRQ(21);

	LPC_SC->PCONP |= 1 << 12; // set PCADC bit
	LPC_PINCON->PINSEL1 = (LPC_PINCON->PINSEL1 & ~(0X3 << 14)) | (0X1 << 14); // set 15:14 to 01
	LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~(0X3 << 14)) | (0X1 << 15); // set 15:14 to 10
	LPC_ADC->ADCR = 1 | (2 << 8) | (1 << 14) | (1 << 21);

	LPC_GPIO2->FIODIR &= ~(0x1<<12); //GPIO2[12] Setting direction as input
	LPC_GPIO0->FIOSET &= (0x1<<22);

	//Enables the interrupt on GPIO 0
	LPC_GPIOINT->IO0IntEnR = 1;

	xPrintQueue = xQueueCreate(5, sizeof(char *));

	//Before a semaphore is used it must be explicitly created.
	xMutexClock = xSemaphoreCreateMutex();
	xMutexData = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary( xBinarySemaphore );
	vSemaphoreCreateBinary( xPrintSemaphore );

	/* Check the semaphore was created successfully. */
	if( xBinarySemaphore != NULL )
	{
		/* Enable the software interrupt and set its priority. */
		prvSetupSoftwareInterrupt();

		/* Create the 'handler' task.  This is the task that will be synchronized
        with the interrupt.  The handler task is created with a high priority to
        ensure it runs immediately after the interrupt exits.  In this case a
        priority of 3 is chosen. */
		xTaskCreate( vReadADCTask, "ReadADC", 240, NULL, 3, NULL );
		xTaskCreate( vPeriodicTask, "ISR", 240, NULL, 0, NULL );
		xTaskCreate ( vHandlerTask, "Handler", 240, NULL, 3, NULL);
		xTaskCreate ( vFormatTask, "Format", 240, NULL, 3, NULL);
		xTaskCreate ( vPrintTask, "Print", 240, NULL, 3, NULL);
		xTaskCreate ( vCalculateFreqTask, "CalcFreq", 240, NULL, 3, NULL);
		xTaskCreate ( vTrackFreqTask, "TrackFreq", 240, NULL, 3, NULL);
		xTaskCreate ( vButtonHandlerTask, "ButtonHandler", 240, NULL, 3, NULL);


		/*Create the gatekeeper task. This is the only task that is permitted
		 * to directly access standard out
		 */
		//        xTaskCreate ( prvStdioGateKeeperTask, "GateKeeper", 240, NULL, 3, NULL);
		/* Create the task that will periodically generate a software interrupt.
        This is created with a priority below the handler task to ensure it will
        get preempted each time the handler task exits the Blocked state. */
		xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 1, NULL );

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	/* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */
	for( ;; );
	return 0;
}

void buttonInterrupt(void) {
	printf("Frequency Measured");
	LPC_GPIOINT->IO0IntClr = 1;
	NVIC_ClearPendingIRQ(21);
	NVIC_DisableIRQ(21);
	portEND_SWITCHING_ISR(pdFalse);
}
/*-----------------------------------------------------------*/
/**
 * Once vPeriodic task throws interrupt, vHandlerTask catches xBinarySemaphore,
 * read data from ADC and queue, which will be dequeue by vFormatTask
 */
static void vReadADCTask( void *pvParameters )
{
	unsigned long ul;
	int count = 0;
	int adc_val;
	/* As per most tasks, this task is implemented within an infinite loop.

    Take the semaphore once to start with so the semaphore is empty before the
    infinite loop is entered.  The semaphore was created before the scheduler
    was started so before this task ran for the first time.*/
	xSemaphoreTake( xBinarySemaphore, 0 );

	for( ;; )
	{
		/* Use the semaphore to wait for the event.  The task blocks
        indefinitely meaning this function call will only return once the
        semaphore has been successfully obtained - so there is no need to check
        the returned value. */
		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );

		/* To get here the event must have occurred.  Process the event (in this
        case we just print out a message). */
		//vPrintString( "Handler task - Processing event.\n" );

		LPC_ADC->ADCR |= 1 << 24; // start conversion
		while((LPC_ADC->ADDR0 & (1 << 31)) == 0); // wait for conversion to finish
		adc_val = (LPC_ADC->ADDR0 >> 4) & 0xfff; // read value
		xQueueSendToBack(xDataQueue, &adc_val, 0);

		//This print need to be deleted later
		//printf("%8d : %d\n", count++, adc_val);
	}
}
/*-----------------------------------------------------------*/
/**
 * vPeriodicTask will generate simulate interrupt periodically
 */
static void vPeriodicTask( void *pvParameters )

{

	/* As per most tasks, this task is implemented within an infinite loop. */
	for( ;; )
	{
		/* This task is just used to 'simulate' an interrupt.  This is done by
        periodically generating a software interrupt. */
		vTaskDelay( 1 / portTICK_RATE_MS );

		mainTRIGGER_INTERRUPT();
		//}
	}
}

static void vHandlerTask(void *pvParameters )
{
	unsigned int n;

	unsigned int countMS = 0;

	/* As per most tasks, this task is implemented within an infinite loop.
	Take the semaphore once to start with so the semaphore is empty before the
	infinite loop is entered.  The semaphore was created before the scheduler
	was started so before this task ran for the first time.*/
	xSemaphoreTake( xBinarySemaphore, 0 );

	for( ;; )
	{
		//Taking semaphore from vFormatTask
		xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
			countMS++;
			xSemaphoreTake( xMutexTime, portMAX_DELAY);
			{
				time = countMS;
			}
			xSemaphoreGive( xMutexTime);

			//Using mutex to grab data that vFormatTask stored
			xSemaphoreTake( xMutexSampleRate, portMAX_DELAY);
			{
				n = sampleRate;
			}
			xSemaphoreGive( xMutexSampleRate);

			if(countMS % n == 0)
			{
				xSemaphoreGive(xReadADCSemaphore);
			}
		xSemaphoreGive( xBinarySemaphore);
	}
}
/*-----------------------------------------------------------*/
/**
 * vFormatTask dequeue the data that vHandlerTask has queued,
 * keeps track of counts when data crosses middle point, and when button is pressed,
 * take time(clock) and reset time using mutex, and calculate the frequency, then store
 * using mutex
 */
static void vFormatTask( void *pvParameters )
{
	//Max is 4095 or 3915 Min is 0
	int pcInt;
	portBASE_TYPE xStatus;
	int released = 1;
	int clock = 0;
	int counter = 0;
	int old = 0;
	char tempFreq[0];
	for(;;)
	{
		//Taking data from queue
		xStatus = xQueueReceive( xDataQueue, &pcInt, portMAX_DELAY );
		if(xStatus == pdPASS) {
			//Checking if data passed middle point
			counter += (pcInt <= 1200 && old >= 1200 || pcInt >= 1200 && old <= 1200) ? 1 : 0;
			//Keeping tracking of last data value as old value
			old = pcInt;
			//Button if statement
			if(LPC_GPIO2->FIOPIN != (LPC_GPIO2->FIOPIN & ~(0x1<<12))) {
				//				printf("Button Pushed");
				//checking if button is edge of pressed
				if(released) {
					released = 0;
					//using Mutex to take time(clock) and resetting to zero
					xSemaphoreTake( xMutexClock, portMAX_DELAY );
					{
						clock = rtc_clock_ctr;
						rtc_clock_ctr = 0;
					}
					xSemaphoreGive( xMutexClock );
					//using Mutex to store data which will be used by vPrintTask
					xSemaphoreTake( xMutexData, portMAX_DELAY);
					{
						freq = (double)counter * 500.0 / (double)clock;
					}
					xSemaphoreGive( xMutexData );

					//Print out debugging
					//double temp = (double)counter * 500.0 / (double)clock;
					//printf("PRESSED %d counter *** %d data *** %d Clock *** %f freq\n", counter, pcInt, clock, temp);

					//reseting counter
					counter = 0;
					//signal vPrintTask to get to work
					//					xSemaphoreTake( xMutexData, portMAX_DELAY);
					//					{
					//						memcpy(&tempFreq,&freq,1);
					//						xQueueSendToBack(xPrintQueue, &( pcStringToPrint[0]), 0);
					//						xQueueSendToBack(xPrintQueue, &tempFreq[0], 0);
					//					}
					//					xSemaphoreGive(xMutexData);
					xSemaphoreGive(xPrintSemaphore);
				}
			} else {
				//Button is released
				released = 1;
			}
		} else {
			printf("Error in vFormatTask Queue\n");
		}
	}
}
/**
 * vPrintTask requires vFormatTask to release PrintSemaphore, and take data
 * from vFormatTask stored in globe by using Mutex and print with Mutex.
 */
static void vPrintTask (void *pvParameters )
{
	xSemaphoreTake( xPrintSemaphore, 0 );
	double temp;
	for( ;; )
	{
		//Taking semaphore from vFormatTask
		xSemaphoreTake( xPrintSemaphore, portMAX_DELAY );
		//Using mutex to grab data that vFormatTask stored
		xSemaphoreTake( xMutexData, portMAX_DELAY);
		{
			temp = freq;
		}
		xSemaphoreGive( xMutexData );
		//Printing Frequency
		printf("Current Frequency: %f\n", temp);
	}
}
static void vCalculateFreqTask(void *pvParameters)
{

}
static void vTrackFreqTask(void *pvParameters)
{

}
static void vButtonHandlerTask(void *pvParameters)
{

}

static int filterADC(int adc_val)
{
	return (adc_val >> 2) + (filterADC-(filterADC >> 2));
}
//static void prvStdioGateKeeperTask (void *pvParameters)
//{
//	char *pcMessageToPrint;
//
//	static char cBuffer[mainMAX_MSG_LEN];
//	xSemaphoreTake( xPrintSemaphore, portMAX_DELAY);
//
//		for(;;)
//		{
//			xSemaphoreTake( xPrintSemaphore, portMAX_DELAY);
//			while(uxQueueMessagesWaiting(xPrintQueue) != 0)
//			{
//				xQueueReceive( xPrintQueue, &pcMessageToPrint, portMAX_DELAY);
//				printf("%f", pcMessageToPrint);
//
////				sprintf(cBuffer, "%s", pcMessageToPrint);
////				consoleprint(cBuffer);
//			}
//			xSemaphoreGive(xPrintSemaphore);
//		}
//
//}

static void prvSetupSoftwareInterrupt()
{
	/* The interrupt service routine uses an (interrupt safe) FreeRTOS API
	function so the interrupt priority must be at or below the priority defined
	by configSYSCALL_INTERRUPT_PRIORITY. */
	NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );

	/* Enable the interrupt. */
	NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/

void vSoftwareInterruptHandler( void )
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* 'Give' the semaphore to unblock the task. */
	xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );

	/* Clear the software interrupt bit using the interrupt controllers
    Clear Pending register. */
	mainCLEAR_INTERRUPT();

	/* Giving the semaphore may have unblocked a task - if it did and the
    unblocked task has a priority equal to or above the currently executing
    task then xHigherPriorityTaskWoken will have been set to pdTRUE and
    portEND_SWITCHING_ISR() will force a context switch to the newly unblocked
    higher priority task.

    NOTE: The syntax for forcing a context switch within an ISR varies between
    FreeRTOS ports.  The portEND_SWITCHING_ISR() macro is provided as part of
    the Cortex-M3 port layer for this purpose.  taskYIELD() must never be called
    from an ISR! */
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
}
/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}
