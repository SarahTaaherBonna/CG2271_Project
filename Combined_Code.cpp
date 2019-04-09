#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

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

int currentState = 0; //A global var to indicate if bot is stationary(0) or moving(1) or end(2) mainly for audio purposes
xQueueHandle queue;
SemaphoreHandle_t semaphore = NULL;
int index = 0;
//byte moving[index];

extern HardwareSerial Serial;

char blueToothVal;
char lastValue;

SemaphoreHandle_t semaphore = NULL;

//unsigned long lastTime = 0;
//unsigned long currentTime;

//#define bufferSize 4
//int arrayBuffer[bufferSize];
int prodIndex = 0;
int consIndex = 0 ;

void tSerial(void *p) {
	for(;;){
				
		if(Serial.available()) {
			if( xSemaphoreTake( semaphore, (TickType_t) portMAX_DELAY) == pdTRUE ) {
					blueToothVal = Serial.read();
					xQueueSendToBack(queue, (void *) &toRead, (TickType_t) 10);
					vTaskDelay(1);
				}
			}
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
				int value = arrayBuffer[consIndex];
				xQueueReceive(queue, &valuePrint, (TickType_t) 10);
				if(value == 'O')
					moveForward();
				if(value == 'Z')
					moveLeft();
				if(value == '[')
					moveRight();
				if(value == 'z')
					stop();
		}
	}


void connectionEstablishedVisual(){
	byte pattern = B11000000;
	digitalWrite(LATCH,LOW);
    shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,moving[index]);
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
	portTickType xNextTickTime;
	xNextTickTime = xTaskGetTickCount();
	for(;;) {
		int valueRun;
		xQueueReceive(queue, &valueRun, (TickType_t) 10);
		if(valueRun == 'O' || valueRun == 'P' || valueRun == 'Z' || valueRun == '[') {
			movingModeVisual();
		}
		else {
			stopModeVisual();
		}
		//vTaskDelayUntil(&xNextTickTime, 100); //1000 is arbitrary value, need to find actual value - continuously occuring so no delays here
	}
}

void tAudio(void *p){
	portTickType xNextTickTime;
	xNextTickTime = xTaskGetTickCount();
	for(;;) {
		int valueRun;
		xQueueReceive(queue, &valueRun, (TickType_t) 10);
		if(valueRun == 'z') {
			EndTune();
		}
		else
			BabyShark();
		//Serial.println(valuePrint);
		//vTaskDelayUntil(&xNextTickTime, 100); //1000 is arbitrary value, need to find actual value - no delays for audio or we are screwed
	}
}

void setup(){
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CLOCK_SHIFTREG,OUTPUT);
  pinMode(LATCH,OUTPUT);
  pinMode(DATA,OUTPUT);
  Serial.begin(9600);
  queue = xQueueCreate(STACK_SIZE, sizeof(int));
  semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(sempahore);
}

void loop() {
	xTaskCreate(tSerial, "Producer", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tMotorControl, "ConsumerMotor", STACK_SIZE, (void *) 1, 1, NULL);
	xTaskCreate(tAudio, "Audio", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(tLED, "ConsumerLED", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}

