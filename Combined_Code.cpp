#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define STACK_SIZE	200
SemaphoreHandle_t semaphore = NULL;
SemaphoreHandle_t semaphoreFull = NULL;
SemaphoreHandle_t semaphoreEmpty = NULL;
SemaphoreHandle_t semaphoreMutex = NULL;


#define POTENTIOMETER 0
#define TASK_PERIOD 5000 //ms
unsigned long lastTime = 0;
unsigned long currentTime;

#define bufferSize 4
int arrayBuffer[bufferSize];
int prodIndex = 0;
int consIndex = 0 ;

static signed portBASE_TYPE xHigherPriorityTaskWoken;


void INT0_ISR() {
	xHigherPriorityTaskWoken = pdFALSE;
	currentTime = millis();
	if (currentTime - lastTime > 300) {
		lastTime = currentTime;
		/* Unblock the task by releasing the semaphore. */
		xSemaphoreGiveFromISR( semaphore, &xHigherPriorityTaskWoken );
	}
	taskYIELD();
}

void producer(void *p) {
	for(;;){
		if( xSemaphoreTake( semaphore, (TickType_t) portMAX_DELAY) == pdTRUE ) {
			if(xSemaphoreTake(semaphoreFull, (TickType_t) portMAX_DELAY) == pdTRUE){
				if(xSemaphoreTake(semaphoreMutex, (TickType_t) portMAX_DELAY) == pdTRUE)
				{
					arrayBuffer[prodIndex] = analogRead(POTENTIOMETER);
					prodIndex = (prodIndex + 1)%4;
					xSemaphoreGive(semaphoreMutex);
					xSemaphoreGive(semaphoreEmpty);
				}

			}
		}

	}
}

void consumer(void *p) {
	for(;;){
		if(xSemaphoreTake(semaphoreEmpty, (TickType_t) portMAX_DELAY) == pdTRUE)
		{
			if(xSemaphoreTake(semaphoreMutex, (TickType_t) portMAX_DELAY) == pdTRUE){
				//Read from circular buffer
				int valuePrint = arrayBuffer[consIndex];
				consIndex = (consIndex + 1)%4;
				Serial.println(valuePrint);

				xSemaphoreGive(semaphoreMutex);
				xSemaphoreGive(semaphoreFull);
			}

		}
		vTaskDelay(TASK_PERIOD);
	}
}

void setup(){
	Serial.begin(115200);
	attachInterrupt(0, INT0_ISR, RISING);
	semaphore = xSemaphoreCreateBinary();
	semaphoreFull = xSemaphoreCreateCounting(4, 4);
	semaphoreEmpty = xSemaphoreCreateCounting(4,0);
	semaphoreMutex = xSemaphoreCreateMutex();

}

void loop() {
	xTaskCreate(producer, "Producer", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(consumer, "Consumer", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
