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

#define mainSOFTWARE_INTERRUPT_PRIORITY 		( 5 )

/* Dimensions the buffer into which messages destined for stdout are placed. */
#define mainMAX_MSG_LEN	( 80 )

#define LED0_GPIO_PORT_NUM                      0
#define LED0_GPIO_BIT_NUM                       22

/* The tasks to be created. */
static void vReadADC( void *pvParameters );
//static void vPeriodicTask( void *pvParameters );
static void vClock( void *pvParameters );
static void vFormat( void *pvParameters );
static void vPrint( void *pvParameters );
static void vCalculate( void *pvParameters );
static void vTrack( void *pvParameters );


void ISR_Button(void);
static int filterADC(int adc_val, int previous);

static void prvSetupSoftwareInterrupt();
static void prvSetupHardware(void);

/* The service routine for the interrupt.  This is the interrupt that the
task will be synchronized with. */
void vSoftwareInterruptHandler( void );


/*-----------------------------------------------------------*/

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize a task with an interrupt. */
xSemaphoreHandle xBinarySemaphore, xReadADCSemaphore, xReadSemaphore,
xWriteSemaphore, xCalculateSemaphore, xButtonPressSemaphore,
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

volatile unsigned int time;

volatile unsigned int currentFreq;

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
	/* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
	xMutexFreq = xSemaphoreCreateMutex();		//
	xMuteTime = xSemaphoreCreateMutex();		//
	xMuteReadADC = xSemaphoreCreateMutex();		//
	xMuteSampleRate = xSemaphoreCreateMutex();	//
	vSemaphoreCreateBinary( xBinarySemaphore );		//Semaphore for from ISR to Handler
	vSemaphoreCreateBinary( xReadADCSemaphore );	//Semaphore for Handler to signal to start Read ADC Task
	vSemaphoreCreateBinary( xCalculateSemaphore );	//Semaphore for ReadADC Task to signal to Calculate from ADC value
	vSemaphoreCreateBinary( xReadSemaphore );		//Semaphore that Calculate task to flag writing mutex is done and ready to read
	vSemaphoreCreateBinary( xWriteSemaphore );		//Semaphore that Track task to flag reading Mutex is done and read to write
	vSemaphoreCreateBinary( xButtonPressSemaphore );//
	/* Check the semaphore was created successfully. */
	if( xBinarySemaphore != NULL )
	{
		/* Enable the software interrupt and set its priority. */
		prvSetupSoftwareInterrupt();

		xTaskCreate( vReadADC, "Handler", 240, NULL, 3, NULL );
		xTaskCreate( vFormatTask, "Format", 240, NULL, 0, NULL );
		xTaskCreate ( vPrintCurrentFreq, "PrintFreq", 240, NULL, 3, NULL);
		xTaskCreate( vPeriodicTask, "Periodic", 240, NULL, 1, NULL );

		/* Start the scheduler so the created tasks start executing. */
		vTaskStartScheduler();
	}

	for( ;; );
	return 0;
}

void buttonInterrupt(void) {
	printf("Frequency Measured");
	LPC_GPIOINT->IO0IntClr = 1;
	NVIC_ClearPendingIRQ(21);
	NVIC_DisableIRQ(21);
	portEND_SWITCHING_ISR(pdFalse);
	xSemaphoreGive(xButtonPressSemaphore);
}

///*-----------------------------------------------------------*/
///**
// * vPeriodicTask will generate simulate ISR 1ms periodically
// */
//static void vPeriodicTask( void *pvParameters )
//
//{
//	for( ;; )
//	{
//		vTaskDelay( 1 / portTICK_RATE_MS );
//
//		mainTRIGGER_INTERRUPT();
//	}
//}

static void vClock(void *pvParameters )
{
	unsigned int n;

	unsigned int countMS = 0;
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
	}
}

static void vReadADC( void *pvParameters )
{
	unsigned long ul;
	int count = 0;
	int adc_val;
	int previous = 0;
	xSemaphoreTake( xReadADCSemaphore, 0 );

	for( ;; )
	{
		xSemaphoreTake( xReadADCSemaphore, portMAX_DELAY );

		LPC_ADC->ADCR |= 1 << 24; // start conversion
		while((LPC_ADC->ADDR0 & (1 << 31)) == 0); // wait for conversion to finish
		adc_val = (LPC_ADC->ADDR0 >> 4) & 0xfff; // read value
		//Filter and store in previous
		previous = filterADC(adc_valu, previous);

		xSemaphoreTake(xMutexReadADC, portMAX_DELAY);
		{
			filterVal = previous;
		}
		xSemaphoreGive(xMutexReadADC);

		xSemaphoreGive(xCalculateSemaphore);
		//printf("%8d : %d\n", count++, adc_val); //For debugging purpose
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
static void vCalculate(void *pvParameters)
{

}
static void vTrack(void *pvParameters)
{
	int tracker = 10;

	xSemaphoreTake( xFreq, 0 );
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
static void vISR_Button(void *pvParameters)
{
	xSemaphoreTake(xButtonPressSemaphore, 0);

	for(;;) {
		xSemaphoreTake(xButtonPressSemaphore, portMAX_DELAY);
	}
}

static int filterADC(int adc_val, int previous)
{
	return (adc_val >> 2) + (previous-(previous>>2));
}


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
