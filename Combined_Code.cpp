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

xQueueHandle queueMotor;
xQueueHandle queueLED;
xQueueHandle queueAud;
SemaphoreHandle_t semaphore = NULL;
char blueToothVal;
char lastValue;
int prodIndex = 0;
int consIndex = 0 ;
//int arrayBuffer[bufferSize];
//int currentState = 0; //A global var to indicate if bot is stationary(0) or moving(1) or end(2) mainly for audio purposes

void tSerial(void *p) {
	for(;;){
		//semaphore = xSemaphoreCreateMutex();
		//if(semaphore != NULL && xSemaphoreTake(semaphore, (TickType_t) 10) == pdTRUE){
			//if(Serial.available()) {
				//	char blueToothVal = Serial.read();
					//delay(100);
					xQueueSendToBack(queueMotor, (void *) &blueToothVal, (TickType_t) 10);
					xQueueSendToBack(queueLED, (void *) &blueToothVal, (TickType_t) 10);
					xQueueSendToBack(queueAud, (void *) &blueToothVal, (TickType_t) 10);
					//xSemaphoreGive(semaphore);
					vTaskDelay(1);
			//}
	}

}

void moveForward(){
	analogWrite(BIN_1, 240);
	analogWrite(AIN_2, 240);
	analogWrite(BIN_3, 240);
	analogWrite(AIN_4, 240);
	delay(100);
}

void moveLeft(){
	analogWrite(BIN_1, MAX_PWM_VOLTAGE);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, MAX_PWM_VOLTAGE);
	analogWrite(AIN_4, 0);
	delay(100);
}

void moveRight(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, MAX_PWM_VOLTAGE);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, MAX_PWM_VOLTAGE);
	delay(100);
}

void stop(){
	analogWrite(BIN_1, 0);
	analogWrite(AIN_2, 0);
	analogWrite(BIN_3, 0);
	analogWrite(AIN_4, 0);
	delay(100);
}

void tMotorControl(void *p) {
	for(;;){
		//Read from circular buffer
		//int value = arrayBuffer[consIndex];
		int value;
		if(semaphore != NULL && xSemaphoreTake(semaphore, (TickType_t) 10) == pdTRUE){

			if(xQueueReceive(queueMotor, &value, (TickType_t) 10)){
				if(value == 'O')
					moveForward();
				if(value == 'Z')
					moveLeft();
				if(value == '[')
					moveRight();
				if(value == 'z')
					stop();
				xSemaphoreGive(semaphore);
			}
		}
	}
}


void connectionEstablishedVisual(){
	byte pattern = B11000000;
	digitalWrite(LATCH,LOW);
	shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,pattern);
	digitalWrite(LATCH,HIGH);
	digitalWrite(LATCH, LOW);
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

void ConnectionEstablished() {
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

void tLED(void *p) {
	//portTickType xNextTickTime;
	//xNextTickTime = xTaskGetTickCount();
	for(;;) {
		int valueRun;
		if(semaphore != NULL && xSemaphoreTake(semaphore, (TickType_t) 10) == pdTRUE){
			if(xQueueReceive(queueLED, &valueRun, (TickType_t) 10)){
				if(valueRun == 'O' || valueRun == 'P' || valueRun == 'Z' || valueRun == '[') {
					movingModeVisual();
				}
				else {
					stopModeVisual();
				}
				xSemaphoreGive(semaphore);
			}
		}
	}
}

void tAudio(void *p){
	//portTickType xNextTickTime;
	//xNextTickTime = xTaskGetTickCount();
	for(;;) {
		int valueRun;
		if(xQueueReceive(queueAud, &valueRun, (TickType_t) 10)) {
			if(valueRun == 'z') {
				EndTune();
			}
			else {
				BabyShark();
			}
		}
	}
}

void setup(){
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(CLOCK_SHIFTREG,OUTPUT);
	pinMode(LATCH,OUTPUT);
	pinMode(DATA,OUTPUT);
	Serial.begin(9600);
	queueMotor = xQueueCreate(STACK_SIZE, sizeof(int));
	queueLED = xQueueCreate(STACK_SIZE, sizeof(int));
	queueAud = xQueueCreate(STACK_SIZE, sizeof(int));
}

void loop() {
	//char blueToothVal;
	if(Serial.available()) {
				blueToothVal = Serial.read();
				Serial.println(F("tSerial here"));

	}
	xTaskCreate(tSerial, "Serial", STACK_SIZE, (void * ) blueToothVal, 2, NULL);
	xTaskCreate(tMotorControl, "ConsumerMotor", STACK_SIZE, (void *) 1, 1, NULL);
	xTaskCreate(tAudio, "Audio", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tLED, "ConsumerLED", STACK_SIZE, (void * ) 1, 1, NULL);


//delay(100);
			vTaskStartScheduler();

}
