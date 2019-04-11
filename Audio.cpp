#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

extern HardwareSerial Serial;

#define STACK_SIZE	200
#define MAX_PWM_VOLTAGE 240
#define REARLED_PIN 2
#define BUZZER_PIN 8
#define CLOCK_SHIFTREG 13
#define LATCH 4
#define DATA 7
#define BIN_1 3
#define AIN_2 5
#define BIN_3 10
#define AIN_4 11
#define bufferSize 4

SemaphoreHandle_t semaphore = NULL;

int connectionEstablished = 0;

int myState = 0;
char blueToothVal;
char lastValue;
xQueueHandle queueAudio;
unsigned long lastMovingRear = 0;
unsigned long lastStopRear = 0;
unsigned long lastMovingFront  = 0;
int rearMovingstate = 0;
int rearStopState = 0;

void tSerial(void *p) {
	while(1){
		if(Serial.available()){
			blueToothVal = Serial.read();
		}
		xQueueSendToBack(queueAudio, (void *) &blueToothVal, (TickType_t) 10);
		vTaskDelay(1);

	}
}

void beep(int note, int duration) {
  unsigned long startTime = millis();
  unsigned long period = 1000000L/note/2;
  while(millis() - startTime < duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(period);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(period);
  }
}

void ConnectionEstablishedAudio() {
  beep(659, 250);
  beep(659, 250);
  beep(659, 375);
  beep(659, 250);
  beep(659, 250);
  beep(659, 375);
  beep(659, 250);
  beep(784, 250);
  beep(523, 250);
  beep(587, 250);
  beep(659, 750);
  beep(659, 250);
  beep(659, 250);
  beep(659, 375);
  beep(659, 250);
  beep(659, 250);
  beep(659, 375);
  beep(659, 250);
  beep(784, 250);
  beep(523, 250);
  beep(587, 250);
  beep(659, 750);
}

void BabyShark() {
  beep(587, 250);
  beep(659, 250);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(587, 250);
  beep(659, 250);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(587, 250);
  beep(659, 250);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 375);
  beep(784, 250);
  beep(784, 250);
  beep(740, 375);
}

void EndTune() {
  beep(659, 375);
  beep(698, 375);
  beep(698, 375);
  beep(698, 375);
  beep(698, 375);
  beep(659, 375);
  beep(698, 375);
  beep(784, 375);
  beep(698, 375);
  beep(698, 375);
  beep(659, 375);
  beep(698, 375);
  beep(698, 375);
  beep(698, 375);
  beep(698, 375);
  beep(659, 250);
  beep(698, 250);
  beep(784, 375);
  beep(698, 375);
  beep(659, 250);
  beep(698, 250);
  beep(587, 375);
  beep(698, 375);
  beep(698, 375);
  beep(698, 250);
  beep(659, 250);
  beep(659, 250);
  beep(523, 375);
  beep(523, 250);
  beep(523, 250);
  beep(587, 375);
}

void tAudio(void *p) {
	while(1) {
		char valueRun;
		if(xQueueReceive(queueAudio, &valueRun, (TickType_t) 10)){
			if(valueRun == 'n' && connectionEstablished) {
				ConnectionEstablishedAudio();
			}
			else if(myState == 1 && (valueRun == 'O' || valueRun == 'P' || valueRun == '-'
					|| valueRun == '.' || valueRun == 'Z' || valueRun == '[')) {
				BabyShark();
			}
			else if(myState == 1 && valueRun == 'F'){
				EndTune();
			}
		}
	}
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CLOCK_SHIFTREG,OUTPUT);
  pinMode(LATCH,OUTPUT);
  pinMode(DATA,OUTPUT);
  pinMode(REARLED_PIN, OUTPUT);
  queueAudio = xQueueCreate(STACK_SIZE, sizeof(int));
}

void loop(){

	ConnectionEstablishedAudio();
	BabyShark();
	EndTune();
	delay(100);
	xTaskCreate(tSerial, "Serial", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tAudio, "Audio", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
