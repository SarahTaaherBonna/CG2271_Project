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
SemaphoreHandle_t semaphoreY = NULL;
xQueueHandle queue;
int connectionEstablished = 0;
int myState = 0;
int index = 0;
int indexAud = 0;
unsigned long lastMovingRear = 0;
unsigned long lastStopRear = 0;
unsigned long lastMovingFront  = 0;
unsigned long lastBabyShark = 0;
int rearMovingstate = 0;
int rearStopState = 0;
char blueToothVal;
char lastValue;

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

int babyshark[] = {587, 659, 784, 784, 784, 784,
					784, 784, 784, 784, 784, 784,
					784, 784, 784, 784, 587, 659,
					784, 784, 784, 784, 784, 784,
					784, 784, 784, 784, 784, 784,
					784, 784, 587, 659, 784, 784,
					784, 784, 784, 784, 784, 784,
					784, 784, 784, 784, 784, 784,
					784, 784, 784, 784, 740, 740
};

void mazeAudio() {
	unsigned long int count = millis() - lastBabyShark;
	if(count%100 == 0) {
		beep(babyshark[indexAud], 250);
		index++;
		if(index >= 54)
			index = 0;
		lastBabyShark = millis();
	}
}


void BabyShark() {
  beep(587, 250);//D5
  beep(659, 250);//E5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(587, 250);//D5
  beep(659, 250);//E5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(587, 250);//D5
  beep(659, 250);//E5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 375);//G5
  beep(784, 375);//G5
  beep(784, 375);//G5

  beep(784, 250);//G5
  beep(784, 250);//G5
  beep(740, 375);//FS5
}


/*
void beepA() {
	int i = 0;
	while(i < 500) {
		digitalWrite(BUZZER_PIN, HIGH);
		delay(3);
		digitalWrite(BUZZER_PIN, LOW);
		delay(3);
	}
}
*/

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

void EndTune() {
	//D - D     E  -  D     G    F#
  beep(294, 500); //D -
  beep(294, 500); //D
  beep(330, 500); //E -
  beep(294, 500); //D
  beep(392, 750); //G
  beep(740, 750); //F#
}

void tSerial(void *p) {
	while(1){
		if(Serial.available()){
			blueToothVal = Serial.read();
		}
		xQueueSendToBack(queue, (void *) &blueToothVal, (TickType_t) 10);
		vTaskDelay(1);

	}
}

void moveForward(){
	analogWrite(BIN_1, 240);
	analogWrite(AIN_2, 240);
	analogWrite(BIN_3, 240);
	analogWrite(AIN_4, 240);
	delay(100);
}

void stop(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, 0);
	delay(100);
}

void move90Left(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, MAX_PWM_VOLTAGE);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, MAX_PWM_VOLTAGE);
	delay(600);
}

void move90Right(){
	analogWrite(BIN_1, MAX_PWM_VOLTAGE);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, MAX_PWM_VOLTAGE);
	analogWrite(AIN_4, 0);
	delay(600);
}

void move45Left(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, MAX_PWM_VOLTAGE);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, MAX_PWM_VOLTAGE);
	delay(300);
}

void move45Right(){
	analogWrite(BIN_1, MAX_PWM_VOLTAGE);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, MAX_PWM_VOLTAGE);
	analogWrite(AIN_4, 0);
	delay(300);
}

void tMotorControl(void *p) {
	while(1){
		char value;
		if(xQueueReceive(queue, &value, (TickType_t) 10) && (lastValue != value)
				&& (xSemaphoreTake(semaphore, (TickType_t) 100) == pdFALSE)){
				lastValue = value;
				xSemaphoreGive(semaphore);
				if(value == 'O') {
					int i=10;
					myState = 1;
					while(i) {
						moveForward();
						xSemaphoreGive(semaphore);
						i--;
					}
				}
				else if(value == 'P') {
					//backwards
					myState = 1;
					xSemaphoreGive(semaphore);
				}
				else if(value == '-') {
					move45Left();
					myState = 1;
					xSemaphoreGive(semaphore);
				}
				else if(value == '.') {
					move45Right();
					myState = 1;
					xSemaphoreGive(semaphore);
				}
				else if(value == 'Z') {
					move90Left();
					myState = 1;
					xSemaphoreGive(semaphore);
				}
				else if(value == '[') {
					move90Right();
					myState = 1;
					xSemaphoreGive(semaphore);
				}
				else if(value == 'z' || value == 'F')
					stop();
				stop();
				myState = 0;
		}
	}
}


void tAudio(void *p) {
	while(1) {
		char valueRun;
		if(xQueueReceive(queue, &valueRun, (TickType_t) 100) ){
			//xSemaphoreGive(semaphoreY);
			if(valueRun == 'n' && !connectionEstablished ) {
				ConnectionEstablishedAudio();
				connectionEstablished = 1;
			}
			else if(!connectionEstablished) {

			}
			else if(valueRun == 'n' && connectionEstablished) {

			}
			if((valueRun == 'O' || valueRun == 'P' || valueRun == '-' || valueRun == '.'
					|| valueRun == 'Z' || valueRun == '[' || valueRun == 'z')
					&& connectionEstablished && lastValue != valueRun) {
				lastBabyShark = millis();
				//BabyShark();
				mazeAudio();
				//xSemaphoreGive(semaphoreY);
			}
			else if(valueRun == 'F' && connectionEstablished && lastValue != valueRun)
				EndTune();
			else {
				delay(100);
				lastStopRear = millis();
				//xSemaphoreGive(semaphoreY);
			}
		}
	}
}

void setup(){
	Serial.begin(9600);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(CLOCK_SHIFTREG,OUTPUT);
	pinMode(LATCH,OUTPUT);
	pinMode(DATA,OUTPUT);
	pinMode(REARLED_PIN, OUTPUT);
	semaphore = xSemaphoreCreateBinary();
	semaphoreY = xSemaphoreCreateBinary();
	queue = xQueueCreate(STACK_SIZE, sizeof(int));
	xSemaphoreGive(semaphore);
	xSemaphoreGive(semaphoreY);
}

void loop() {
	xTaskCreate(tSerial, "Serial", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tMotorControl, "Motor", STACK_SIZE, (void *) 1, 1, NULL);
	xTaskCreate(tAudio, "Audio", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
