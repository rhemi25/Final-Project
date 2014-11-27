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
#define mainSW_INTERRUPT_ID        ( 0 )

/* Macro to force an interrupt. */
#define mainTRIGGER_INTERRUPT()    NVIC_SetPendingIRQ( mainSW_INTERRUPT_ID )

/* Macro to clear the same interrupt. */
#define mainCLEAR_INTERRUPT()    NVIC_ClearPendingIRQ( mainSW_INTERRUPT_ID )

#define mainSOFTWARE_INTERRUPT_PRIORITY         ( 5 )

/* Dimensions the buffer into which messages destined for stdout are placed. */
#define mainMAX_MSG_LEN    ( 80 )

#define LED0_GPIO_PORT_NUM                      0
#define LED0_GPIO_BIT_NUM                       22

//-----------------------------------------------------------------------------------------
// Software ISR
//-----------------------------------------------------------------------------------------
static void SoftwareISR( void *pvParameters );
//-----------------------------------------------------------------------------------------
// The Tasks
//-----------------------------------------------------------------------------------------
static void vClock( void *pvParameters );
static void vReadADC( void *pvParameters );
static void vCalculate( void *pvParameters );
static void vTrackFreq( void *pvParameters );
static void vPrintTask( void *pvParameters );
//-----------------------------------------------------------------------------------------
// Button Hardware Interrupt
//-----------------------------------------------------------------------------------------
void ISR_Button(void);
//-----------------------------------------------------------------------------------------
// Filerts the ADC value
//-----------------------------------------------------------------------------------------
static int filteringADC(int adc_val, int previous);
//-----------------------------------------------------------------------------------------
// Configuration of Interrupts
//-----------------------------------------------------------------------------------------
static void prvSetupSoftwareInterrupt();
static void prvSetupHardware(void);
//The service routine for the interrupt.  This is the interrupt that the
//task will be synchronized with.
void vSoftwareInterruptHandler( void );
//-----------------------------------------------------------------------------------------
// Semaphores
//-----------------------------------------------------------------------------------------
xSemaphoreHandle xBinarySemaphore;
xSemaphoreHandle xReadADCSemaphore; //ISRHandle --> vReadADC
xSemaphoreHandle xButtonPressSemaphore;
xSemaphoreHandle xReadADCSemaphoreCon, xCalTrackSemaphoreCon;
//vReadADC(P)-->| xMutexFilterADC |-->vCalculate(C)
//xSemaphoreHandle xReadADCSemaphorePro, xReadADCSemaphoreCon;
//vCalculate(P)-->| xMutexFreq |-->vTrackFreq(C)
//xSemaphoreHandle xCalTrackSemaphorePro, xCalTrackSemaphoreCon;
//vTrackFreq(P)-->| xMutexSampleRate |-->vClock(C)
//xSemaphoreHandle xClockTrackSemaphorePro, xClockTrackSemaphoreCon;
//vClock(P)-->| xMutexTime |-->vCalculate(C)
//xSemaphoreHandle xClockCalSemaphorePro, xClockCalSemaphoreCon;
xSemaphoreHandle xPrintSemaphore;
//-----------------------------------------------------------------------------------------
// Mutexes
//-----------------------------------------------------------------------------------------
xSemaphoreHandle xMutexFilterADC, xMutexFreq, xMutexSampleRate, xMutexTime;
xSemaphoreHandle xMutexPrintTime, xMutexPrintFreq;
xSemaphoreHandle xMutexTimeFlag, xMutexButtonFlag, xMutexTrackFlag;
//-----------------------------------------------------------------------------------------
// Queue
//-----------------------------------------------------------------------------------------
//xQueueHandle xPrintQueue;
//-----------------------------------------------------------------------------------------
// Clock that will stored by vPeriodicTask using Mutex, and
// the clock will by used and reset by vFormatTask also by using Mutex
//-----------------------------------------------------------------------------------------
volatile uint32_t rtc_clock_ctr;
//-----------------------------------------------------------------------------------------
// Frequency that will be calculated and stored by vFormatTask using Mutex,
// and vPrintTask will take data by using Mutex also.
//-----------------------------------------------------------------------------------------
volatile unsigned int filterADC;
volatile unsigned int time;
volatile double currentFreq;
volatile unsigned int sampleRate = 1;

volatile double printFreq;
volatile unsigned int printTime;

volatile unsigned int timeFlag = 0;
volatile unsigned int buttonFlag = 0;
volatile unsigned int trackFlag = 0;

int main( void ) {
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

    //xPrintQueue = xQueueCreate(5, sizeof(char *));
    /* Before a semaphore is used it must be explicitly created.  In this example
    a binary semaphore is created. */
    xMutexFreq = xSemaphoreCreateMutex();        //
    xMutexTime = xSemaphoreCreateMutex();        //
    xMutexSampleRate = xSemaphoreCreateMutex();        //
    xMutexFilterADC = xSemaphoreCreateMutex();    //
    //Flag mutex for print
    xMutexTimeFlag = xSemaphoreCreateMutex();
    xMutexButtonFlag = xSemaphoreCreateMutex();
    xMutexTrackFlag = xSemaphoreCreateMutex();
    //Data mutex for pint
    xMutexPrintTime = xSemaphoreCreateMutex();
    xMutexPrintFreq = xSemaphoreCreateMutex();

    vSemaphoreCreateBinary( xBinarySemaphore );        //Semaphore for from ISR to Handler
    vSemaphoreCreateBinary( xReadADCSemaphore );
    //vSemaphoreCreateBinary( xReadADCSemaphorePro );
    vSemaphoreCreateBinary( xReadADCSemaphoreCon );
    //vSemaphoreCreateBinary( xCalTrackSemaphorePro );
    vSemaphoreCreateBinary( xCalTrackSemaphoreCon );

    vSemaphoreCreateBinary( xPrintSemaphore );
    /* Check the semaphore was created successfully. */
    if( xBinarySemaphore != NULL )
    {

        /* Enable the software interrupt and set its priority. */
        prvSetupSoftwareInterrupt();
        xTaskCreate( vTrackFreq, "vTrackFreq", 240, NULL, 3, NULL );
        xTaskCreate( vCalculate, "vCalculate", 240, NULL, 3, NULL );
        xTaskCreate( vReadADC, "vReadADC", 240, NULL, 3, NULL );
        xTaskCreate( vClock, "vClock", 240, NULL, 3, NULL );
        xTaskCreate( SoftwareISR, "SoftwareISR", 240, NULL, 1, NULL );
        xTaskCreate( vPrintTask, "vPrintTask", 240, NULL, 3, NULL );

        /* Start the scheduler so the created tasks start executing. */
        vTaskStartScheduler();
    }

    for( ;; );
    return 0;
}
//-----------------------------------------------------------------------------------------
// SoftwareISR will generate simulate ISR 1ms periodically
//-----------------------------------------------------------------------------------------
static void SoftwareISR ( void *pvParameters ) {
    for( ;; ) {
        vTaskDelay( 1 / portTICK_RATE_MS );
        mainTRIGGER_INTERRUPT();
    }
}
//-----------------------------------------------------------------------------------------
// vClock is a timer. It also keeps track of the sampling rate.
//-----------------------------------------------------------------------------------------
static void vClock(void *pvParameters ) {
    unsigned int n;
    unsigned int countMS = 0;
    xSemaphoreTake( xBinarySemaphore, 0 ); //SoftwareISR_Handler gave this semaphore. Clear

    for( ;; ) {
        xSemaphoreTake( xBinarySemaphore, portMAX_DELAY );
        countMS++;
        xSemaphoreTake( xMutexTime, portMAX_DELAY);
        {
            time = countMS;
        }
        xSemaphoreGive( xMutexTime);

        xSemaphoreTake( xMutexSampleRate, portMAX_DELAY);
        {
            n = sampleRate;
        }
        xSemaphoreGive( xMutexSampleRate);
        if(countMS % 1000 == 0) {
            //NVIC_EnableIRQ(21);
            xSemaphoreTake( xMutexTimeFlag, portMAX_DELAY);
            {
                timeFlag = 1;
            }
            xSemaphoreGive( xMutexTimeFlag );

            xSemaphoreTake( xMutexPrintTime, portMAX_DELAY);
            {
                printTime = countMS;
            }
            xSemaphoreGive( xMutexPrintTime );

            xSemaphoreGive( xPrintSemaphore );
        }
        if(countMS % n == 0)
            xSemaphoreGive(xReadADCSemaphore);
    }
}
//-----------------------------------------------------------------------------------------
// vReadADC task reads the ADC raw value, filters it via software implemented filter
//-----------------------------------------------------------------------------------------
static void vReadADC( void *pvParameters ) {
    unsigned long ul;
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
        //previous = filteringADC(adc_val, previous);
        previous = adc_val;
        xSemaphoreTake(xMutexFilterADC, portMAX_DELAY);
        {
            filterADC = previous;
        }
        xSemaphoreGive(xMutexFilterADC);
        xSemaphoreGive(xReadADCSemaphoreCon);
        //printf("%d\n", adc_val); //For debugging purpose
    }
}
//-----------------------------------------------------------------------------------------
// filterADC software implemented filer some of the noise from the signal.
//-----------------------------------------------------------------------------------------
static int filteringADC(int adc_val, int previous) {
    return (adc_val >> 2) + (previous-(previous>>2));
}
//-----------------------------------------------------------------------------------------
// vCalculate task reads the ADC raw value, filters it via software implemented filter
//-----------------------------------------------------------------------------------------
static void vCalculate(void *pvParameters) {
    //Max is 4095 or 3915 Min is 0
    unsigned int adcVal = 0;
    unsigned int old = 0;
    unsigned int current_time = 0;
    unsigned int previous_time = 0;
    double freq = 0;
    unsigned int count = 0;
    for(;;)
    {
        xSemaphoreTake( xReadADCSemaphoreCon, portMAX_DELAY);
        old = adcVal;
        xSemaphoreTake( xMutexFilterADC, portMAX_DELAY);
        {
            adcVal = filterADC;
        }
        xSemaphoreGive( xMutexFilterADC );
        //printf("%d\n", adcVal); //debugging purpose
        count += (adcVal <= 1200 && old >= 1200) ? 1:0;
        if(count == 500) {
            count = 0;
            //keep track of time
            previous_time = current_time;
            xSemaphoreTake( xMutexTime, portMAX_DELAY);
            {
                current_time = time;
            }
            xSemaphoreGive( xMutexTime );
            //checking if it is not first time crossing
            if(previous_time != 0) {
                freq = 500.0 * 1000.0 / (double)(current_time - previous_time);
                //printf("\nCurrent Freq: %f ", freq);//debugging purpose
                xSemaphoreTake( xMutexFreq, portMAX_DELAY);
                {
                    currentFreq = freq;
                }
                xSemaphoreGive( xMutexFreq );
                xSemaphoreTake( xMutexPrintFreq , portMAX_DELAY);
                {
                    printFreq = freq;
                }
                xSemaphoreGive( xMutexPrintFreq  );
                xSemaphoreGive(xCalTrackSemaphoreCon);
            }
        }
    }
}

static void vTrackFreq(void *pvParameters) {
    unsigned int tracker = 10;
    double old_freq = 0;
    double current_freq = 0;
    unsigned int rate = 1;

    double temp;
    for( ;; )
    {
        xSemaphoreTake( xCalTrackSemaphoreCon, portMAX_DELAY );
        xSemaphoreTake( xMutexFreq , portMAX_DELAY);
        {
            current_freq = currentFreq;
        }
        xSemaphoreGive( xMutexFreq  );

        if(current_freq - old_freq > 10 || old_freq -  current_freq> 10) {
            //Need to flag vPrintTask
            //printf("Changed Freq: %f", current_freq);//debugging purpose
            xSemaphoreTake( xMutexTrackFlag, portMAX_DELAY);
            {
                trackFlag = 1;
            }
            xSemaphoreGive( xMutexTrackFlag );
            xSemaphoreGive( xPrintSemaphore );
            //keep tracking freq
            old_freq = current_freq;
        }

        xSemaphoreTake( xMutexTime , portMAX_DELAY);
        {
            sampleRate = rate;
        }
        xSemaphoreGive( xMutexTime  );
    }
}

void ISR_Button (void) {
    LPC_GPIOINT->IO0IntClr = 1;
    //printf("Frequency Measured"); //Don't forget to enable in main
    NVIC_ClearPendingIRQ(21);
    NVIC_DisableIRQ(21);
    portEND_SWITCHING_ISR(pdFALSE);
    //*
    xSemaphoreTake( xMutexButtonFlag, portMAX_DELAY);
    {
        buttonFlag = 1;
    }
    xSemaphoreGive( xMutexButtonFlag );
    xSemaphoreGive( xPrintSemaphore ); //*/
    //printf("measure 1");

    //portEND_SWITCHING_ISR(0);
    //xSemaphoreGive(xButtonPressSemaphore);
}

static void vPrintTask (void *pvParameters ) {
    double freq;
    unsigned int localtime;
    unsigned int sec;
    unsigned int min;
    unsigned int hr;
    unsigned int localtimeflag;
    unsigned int localtrackflag;
    unsigned int localbuttonflag;

    for( ;; )
    {
        xSemaphoreTake( xPrintSemaphore, portMAX_DELAY );
        xSemaphoreTake( xMutexTimeFlag, portMAX_DELAY);
        {
            localtimeflag = timeFlag;
            timeFlag = 0;
        }
        xSemaphoreGive( xMutexTimeFlag );
        xSemaphoreTake( xMutexTrackFlag, portMAX_DELAY);
        {
            localtrackflag = trackFlag;
            trackFlag = 0;
        }
        xSemaphoreGive( xMutexTrackFlag );
        xSemaphoreTake( xMutexButtonFlag, portMAX_DELAY);
        {
            localbuttonflag = buttonFlag;
            buttonFlag = 0;
        }
        xSemaphoreGive( xMutexButtonFlag );
        //printf("\nvPrintTask %d %d %d", localtimeflag, localtrackflag, localbuttonflag);
        if(localtimeflag == 1) {
            xSemaphoreTake( xMutexPrintTime, portMAX_DELAY);
            {
                localtime = printTime;
            }
            xSemaphoreGive( xMutexPrintTime );
            localtime = localtime / 1000;
            min = localtime / 60;
            sec = localtime % 60;
            hr = min / 60;
            min = min % 60;
            //vPrintString(hr + ":" + "min" + ":" + sec);
            printf("\n%d:%d:%d  ", hr, min, sec);
            NVIC_EnableIRQ(21);
        }
        if(localbuttonflag == 1) {
            //NVIC_EnableIRQ(21);
            xSemaphoreTake( xMutexPrintFreq , portMAX_DELAY);
            {
                freq = printFreq;
            }
            xSemaphoreGive( xMutexPrintFreq  );
            //vPrintString("Current frequency: " + freq);
            printf("Current frequency: %f  ", freq);
        }
        if(localtrackflag  == 1) {
            xSemaphoreTake( xMutexPrintFreq , portMAX_DELAY);
            {
                freq = printFreq;
            }
            xSemaphoreGive( xMutexPrintFreq  );
            //vPrintString("Frequency change: " + freq);
            printf("Frequency change: %f  ", freq);
        }
    }
}

static void prvSetupSoftwareInterrupt()    {
    /* The interrupt service routine uses an (interrupt safe) FreeRTOS API
    function so the interrupt priority must be at or below the priority defined
    by configSYSCALL_INTERRUPT_PRIORITY. */
    NVIC_SetPriority( mainSW_INTERRUPT_ID, mainSOFTWARE_INTERRUPT_PRIORITY );
    /* Enable the interrupt. */
    NVIC_EnableIRQ( mainSW_INTERRUPT_ID );
}
/*-----------------------------------------------------------*/

void vSoftwareInterruptHandler( void )    {
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

void vApplicationMallocFailedHook( void ) {
    /* This function will only be called if an API call to create a task, queue
    or semaphore fails because there is too little heap RAM remaining. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName ) {
    /* This function will only be called if a task overflows its stack.  Note
    that stack overflow checking does slow down the context switch
    implementation. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
    /* This example does not use the idle hook to perform any processing. */
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void ) {
    /* This example does not use the tick hook to perform any processing. */
}
/* Sets up system hardware */
static void prvSetupHardware(void) {
    SystemCoreClockUpdate();
    Board_Init();
    /* Initial LED0 state is off */
    //Board_LED_Set(0, false);
    Board_LED_Set(0, 0);
}


