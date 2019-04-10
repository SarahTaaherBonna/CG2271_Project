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

char blueToothVal;
char lastValue;
xQueueHandle queueMotor;
xQueueHandle queueLED;

void tSerial(void *p) {
	while(1){
		if(Serial.available()){
			blueToothVal = Serial.read();
		}
		if(lastValue != blueToothVal){
			xQueueSendToBack(queueMotor, (void *) &blueToothVal, (TickType_t) 10);
			lastValue = blueToothVal;
			vTaskDelay(1);
		}
	}
}

void moveForward(){
	Serial.println(F("queeud"));
	analogWrite(BIN_1, 240);
	analogWrite(AIN_2, 240);
	analogWrite(BIN_3, 240);
	analogWrite(AIN_4, 240);
	delay(1000);
}

void stop(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, 0);
	delay(100);
}

void moveLeft(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, MAX_PWM_VOLTAGE);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, MAX_PWM_VOLTAGE);
	delay(600);
}

void moveRight(){
		analogWrite(BIN_1, MAX_PWM_VOLTAGE);
		analogWrite(AIN_2, 0);
		analogWrite(BIN_3, MAX_PWM_VOLTAGE);
		analogWrite(AIN_4, 0);
		delay(600);
}

void tMotorControl(void *p) {
	while(1){
		char value;
		if(xQueueReceive(queueMotor, &value, (TickType_t) 10)){
				if(value == 'O') {
					moveForward();
					stop();
				}
				if(value == 'Z') {
					moveLeft();
					stop();
				}
				if(value == '[') {
					moveRight();
					stop();
				}
				if(value == 'z')
					stop();
		}
	}
}

void movingModeVisual(){
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

	int index = 0;
	unsigned long count = 0;
	int rearstate = 0;

	while(1) {
		count += 100;
		digitalWrite(LATCH,LOW);
		shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,moving[index]);
		digitalWrite(LATCH,HIGH);
		index++;
		if(index >= 14)
			index=0;
		if(count % 500 == 0) {
			if(rearstate == 0){
				digitalWrite(REARLED_PIN, HIGH);
				rearstate = 1;
			}
			else {
				digitalWrite(REARLED_PIN, LOW);
				rearstate = 0;
			}
		}
		delay(100);
	}
}

void stopModeVisual(){
	byte pattern = B11111111;
	int rearstate = 0;
	unsigned long count = 0;
	while(1)
	{
		count += 50;
		if(count % 100 == 0)
		{
			digitalWrite(LATCH,LOW);
			shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,pattern);
			digitalWrite(LATCH,HIGH);
		}
		if(count % 250 == 0)
		{
			if(rearstate == 0){
				digitalWrite(REARLED_PIN, HIGH);
				rearstate = 1;
			}
			else
			{
				digitalWrite(REARLED_PIN, LOW);
				rearstate = 0;
			}
		}
		delay(50);
	}
}


void tLED(void *p) {
	while(1) {
		int valueRun;
		if(xQueueReceive(queueLED, &valueRun, (TickType_t) 10)){
			if(valueRun == 'O' || valueRun == 'P' || valueRun == 'Z' || valueRun == '[')
				movingModeVisual();
			else
				stopModeVisual();
		}
	}
}


void setup(){
	Serial.begin(9600);
	queueMotor = xQueueCreate(STACK_SIZE, sizeof(int));
	queueLED = xQueueCreate(STACK_SIZE, sizeof(int));
}

void loop() {
	xTaskCreate(tSerial, "Serial", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tMotorControl, "Motor", STACK_SIZE, (void *) 1, 1, NULL);
	xTaskCreate(tLED, "ConsumerLED", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
