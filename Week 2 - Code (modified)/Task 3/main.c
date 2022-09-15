/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*Include of queue and event group and serial header files*/
#include "queue.h"
#include "serial.h"
#include "string.h"

/*Creating an enum that holdes the state of the edge*/
typedef enum 
{
	RISING,
	FALLING
} EN_Edge_t;

/*Creating a structure that holdes the data
* It holds ID of the Sender, Edge & Strig
*/
typedef struct 
{
	uint8_t ID;
	EN_Edge_t edge;
	uint8_t string[20];
} ST_Data_t;

/*Task Handler*/
TaskHandle_t Task1_Handle = NULL ;
TaskHandle_t Task2_Handle = NULL ;
TaskHandle_t Task3_Handle = NULL;
TaskHandle_t Task4_Handle = NULL;

/*Queue handler*/
QueueHandle_t Queue;


/*Task to detect rising or falling edges on button 1*/
void Task_1( void * pvParameters )
{
	/*Giving this task ID=0*/
	ST_Data_t Data_Sent= {0};
	EN_Edge_t Detected_Edge;
    for( ;; )
    {
			/* Task code goes here. */
			/*Reading the pin state*/
			if (GPIO_read(PORT_0,PIN0) == PIN_IS_LOW)
			{	
				Detected_Edge =FALLING;
			}
			else 
			{
				Detected_Edge=RISING;
			}
			/*Store the value of the edge in structure*/
			Data_Sent.edge = Detected_Edge;
			/*Send structure to queue*/
			xQueueSend(Queue,&Data_Sent,portMAX_DELAY);
			vTaskDelay(10);
		}
}

/*Task to detect rising or falling edges on button 2*/
void Task_2( void * pvParameters )
{
	/*Giving this task ID=1*/
  ST_Data_t Data_Sent= {1};
	EN_Edge_t Detected_Edge;
  	for( ;; )
    {
			/* Task code goes here. */
			/*Reading the pin state*/
			if (GPIO_read(PORT_0,PIN1) == PIN_IS_LOW)
			{		
				Detected_Edge =FALLING;
			}
			else 
			{
				Detected_Edge =RISING;
			}
			/*Store the value of the edge in structure*/
			Data_Sent.edge = Detected_Edge;
			/*Send structure to queue*/
			xQueueSend(Queue,&Data_Sent,portMAX_DELAY);
			vTaskDelay(20);
		}
}

/*Task to send periodic string every 100ms on UART*/
void Task_3( void * pvParameters )
{
	/*Giving this task ID=2*/
	ST_Data_t Data_Sent= {2,0,"Periodic String\n"};
	TickType_t xLastWakeTime;
  const TickType_t xFrequency = 100;
 // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			// Wait for the next cycle.
       vTaskDelayUntil( &xLastWakeTime, xFrequency );
			/* Task code goes here. */
			/*Send the structure to the queue*/
			xQueueSend(Queue,&Data_Sent,portMAX_DELAY);
		}
}

/*Task to collect all data and send it on UART*/
void Task_4( void * pvParameters )
{
	/*Structure that recieves the data from the queue*/
	ST_Data_t Data_recieved;
	/*Used to store the previous states of buttons*/
	EN_Edge_t Previous1 = FALLING;
	EN_Edge_t Previous2 = FALLING;
    for( ;; )
    {
		/* Task code goes here. */
			/*Check if queue is not empty*/
		if( Queue != NULL )
   {
		 /*Recieve the structure*/
      if( xQueueReceive( Queue,&( Data_recieved ),portMAX_DELAY ) == pdPASS )
      {
				/*Check which ID Sent tha data to decide what to do with it*/
         if(Data_recieved.ID == 0)
				 {
					 if (Data_recieved.edge == RISING)
					 {
						 if (Previous1 == Data_recieved.edge)
						 {
							 /*Do nothing*/
						 }
						 else 
						 {
							 vSerialPutString("First Edge is Rising\n",21);
							 Previous1 = Data_recieved.edge;
						 }
					 }
					 else 
					 {
						 if (Previous1 == Data_recieved.edge)
						 {
							 /*Do nothing*/
						 }
						 else 
						 {
							 vSerialPutString("First Edge is FALLING\n",22);
							 Previous1 = Data_recieved.edge;
						 }
					 }
				 }
				 else if (Data_recieved.ID == 1)
				 {
					 if (Data_recieved.edge == RISING)
					 {
						 if (Previous2 == Data_recieved.edge)
						 {
							 /*Do nothing*/
						 }
						 else 
						 {
							 vSerialPutString("Second Edge is Rising\n",22);
							 Previous2 = Data_recieved.edge;
						 }
					 }
					 else 
					 {
						 if (Previous2 == Data_recieved.edge)
						 {
							 /*Do nothing*/
						 }
						 else 
						 {
							 vSerialPutString("Second Edge is FALLING\n",23);
							 Previous2 = Data_recieved.edge;
						 }
					 }
				 }
				 else if (Data_recieved.ID == 2)
				 {
					 vSerialPutString(Data_recieved.string,strlen(Data_recieved.string));
				 }
				 else
				 {
					 /*Do Nothing*/
				 }
      }
   }
		vTaskDelay(30);
		}
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{

	/*Create queue*/
	Queue = xQueueCreate(4, sizeof (ST_Data_t));
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

    /* Create Tasks here */
    /* Create the task, storing the handle. */
	/*Create Task 1 to detect rising or falling edges on button 1 */
				xTaskCreate(
                    Task_1,       /* Function that implements the task. */
                    "Task_1",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    0,/* Priority at which the task is created. */
                    &Task1_Handle );      /* Used to pass out the created task's handle. */
										
		    /* Create the task, storing the handle. */
	/*Create Task 2 to detect rising or falling edges on button 2*/
				xTaskCreate(
                    Task_2,       /* Function that implements the task. */
                    "Task_2",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    0,/* Priority at which the task is created. */
                    &Task2_Handle );      /* Used to pass out the created task's handle. */
    /* Create the task, storing the handle. */
	/*Create Task 3 to send periodic string every 100ms on UART*/
				xTaskCreate(
                    Task_3,       /* Function that implements the task. */
                    "Task_3",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    0,/* Priority at which the task is created. */
                    &Task3_Handle );      /* Used to pass out the created task's handle. */
										
		    /* Create the task, storing the handle. */
/*Create Task 4 to collect all data and send it on UART*/
				xTaskCreate(
                    Task_4,       /* Function that implements the task. */
                    "Task_4",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    0,/* Priority at which the task is created. */
                    &Task4_Handle );      /* Used to pass out the created task's handle. */
										
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


