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

byte moving[]=
	{
			B00000001,
			B00000010,
			B00000100,
			B00001000,
			B00010000,
			B00100000,
			B01000000,
			B10000000,
			B01000000,
			B00100000,
			B00010000,
			B00001000,
			B00000100,
			B00000010,
	};

void setDown() {
	byte pattern = B00000000;
	digitalWrite(LATCH, LOW);
	shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,pattern);
	digitalWrite(LATCH,HIGH);
}

void connectionEstablishedFront(){
	byte pattern1 = B11000000;
	digitalWrite(LATCH,LOW);
	shiftOut(DATA, CLOCK_SHIFTREG, MSBFIRST, B00000000);
	shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,pattern1);
	digitalWrite(LATCH,HIGH);
	delay(100);
	setDown();
	delay(100);
}

void connectionEstablishedRear(){
	digitalWrite(REARLED_PIN, LOW);
	delay(200);
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


void movingModeFront(){
	unsigned long int count1 = millis() - lastMovingFront;
	if(count1 % 100 == 0) {
		digitalWrite(LATCH,LOW);
		shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,moving[index]); //front green LEDs Running Mode
		digitalWrite(LATCH,HIGH);
		index++;
		if(index >= 14)
			index=0;
	}
}

void movingModeRear(){
	unsigned long int count2 = millis() - lastMovingRear;
	if(count2 % 10 == 0) {
		if(rearMovingstate == 0){
			digitalWrite(REARLED_PIN, HIGH);
			rearMovingstate = 1;
		}
		else {
			digitalWrite(REARLED_PIN, LOW);
			rearMovingstate = 0;
		}
		lastMovingRear = millis();
	}
}

void stopModeRear(){
	unsigned long int count2 = millis() - lastStopRear;
	if(count2 % 250 == 0) {
		if(rearStopState == 0){
			digitalWrite(REARLED_PIN, HIGH);
			rearStopState = 1;
		}
		else {
			digitalWrite(REARLED_PIN, LOW);
			rearStopState = 0;
		}
		lastStopRear = millis();
	}
}

void stopModeFront(){
	byte pattern = B11111111;
			digitalWrite(LATCH,LOW);
			shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,pattern);
			digitalWrite(LATCH,HIGH);
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


void tLED(void *p) {
	while(1) {
		char valueRun;
		if(xQueueReceive(queue, &valueRun, (TickType_t) 100) ){
			//xSemaphoreGive(semaphoreY);
			if(valueRun == 'n' && !connectionEstablished ) {
				connectionEstablishedFront();
				connectionEstablishedRear();
				delay(100);
				connectionEstablishedFront();
				connectionEstablishedRear();
				delay(100);
				setDown();
				//xSemaphoreGive(semaphoreY);
				connectionEstablished = 1;
			}
			else if(!connectionEstablished) {
				setDown();
				connectionEstablishedRear();
			}
			else if(valueRun == 'n' && connectionEstablished) {
				setDown();
				connectionEstablishedRear();
			}
			if((valueRun == 'O' || valueRun == 'P' || valueRun == '-' || valueRun == '.' || valueRun == 'Z' || valueRun == '[') && connectionEstablished && lastValue != valueRun) {
				lastMovingRear = millis();
				movingModeRear();
				lastMovingFront = millis();
				movingModeFront();
				//xSemaphoreGive(semaphoreY);
			}
			else {
				delay(100);
				lastStopRear = millis();
				stopModeRear();
				stopModeFront();
				//xSemaphoreGive(semaphoreY);
			}
		}
	}
}

void setup(){
	Serial.begin(9600);
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
	xTaskCreate(tLED, "LED", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
