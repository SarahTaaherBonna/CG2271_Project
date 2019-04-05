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


extern HardwareSerial Serial;

char blueToothVal;
char lastValue;

//SemaphoreHandle_t semaphore = NULL;
//SemaphoreHandle_t semaphoreFull = NULL;
//SemaphoreHandle_t semaphoreEmpty = NULL;
//SemaphoreHandle_t semaphoreMutex = NULL;


//unsigned long lastTime = 0;
//unsigned long currentTime;

//#define bufferSize 4
//int arrayBuffer[bufferSize];
//int prodIndex = 0;
//int consIndex = 0 ;

//static signed portBASE_TYPE xHigherPriorityTaskWoken;


//void INT0_ISR() {
//	xHigherPriorityTaskWoken = pdFALSE;
//	currentTime = millis();
//	if (currentTime - lastTime > 300) {
//		lastTime = currentTime;
//		/* Unblock the task by releasing the semaphore. */
//		xSemaphoreGiveFromISR( semaphore, &xHigherPriorityTaskWoken );
//	}
//	taskYIELD();
//}
//
//void producer(void *p) {
//	for(;;){
//		if( xSemaphoreTake( semaphore, (TickType_t) portMAX_DELAY) == pdTRUE ) {
//			if(xSemaphoreTake(semaphoreFull, (TickType_t) portMAX_DELAY) == pdTRUE){
//				if(xSemaphoreTake(semaphoreMutex, (TickType_t) portMAX_DELAY) == pdTRUE)
//				{
//					arrayBuffer[prodIndex] = analogRead(POTENTIOMETER);
//					prodIndex = (prodIndex + 1)%4;
//					xSemaphoreGive(semaphoreMutex);
//					xSemaphoreGive(semaphoreEmpty);
//				}
//
//			}
//		}
//
//	}
//}
//
//void consumer(void *p) {
//	for(;;){
//		if(xSemaphoreTake(semaphoreEmpty, (TickType_t) portMAX_DELAY) == pdTRUE)
//		{
//			if(xSemaphoreTake(semaphoreMutex, (TickType_t) portMAX_DELAY) == pdTRUE){
//				//Read from circular buffer
//				int valuePrint = arrayBuffer[consIndex];
//				consIndex = (consIndex + 1)%4;
//				Serial.println(valuePrint);
//
//				xSemaphoreGive(semaphoreMutex);
//				xSemaphoreGive(semaphoreFull);
//			}
//
//		}
//		vTaskDelay(TASK_PERIOD);
//	}
//}

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
  while(1)
  {
    count += 100;
     digitalWrite(LATCH,LOW);   
    shiftOut(DATA,CLOCK_SHIFTREG,MSBFIRST,moving[index]);  
    digitalWrite(LATCH,HIGH);   
    index++;   
    if(index >= 14)     
      index=0;
    if(count % 500 == 0)
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

void tSerial() {
  if(Serial.available()) {
    blueToothVal = Serial.read();
  }

  if(blueToothVal == 'n') {
//    digitalWrite(13, HIGH);
//    if(lastValue != 'n') {
//      Serial.println(F("LED is on."));
//    }
//    lastValue = blueToothVal;
  }
  else if(blueToothVal == 'f') {
//    digitalWrite(13, LOW);
//    if(lastValue != 'f') {
//      Serial.println(F("LED is off."));
//    }
//    lastValue = blueToothVal;
  }
  else if(blueToothVal == 'O') {
      //Forwards
  }
  else if(blueToothVal == 'P') {
      //go backwards
  }
  else if(blueToothVal == '-') {
      //turn 45 degrees Left
  }
  else if(blueToothVal == '.') {
      //turn 45 degrees Right
  }
  else if(blueToothVal == 'Z') {
      //Turn 90 degrees Left
  }
  else if(blueToothVal == '[') {
      //turn 90 degrees Right
  }
  else if(blueToothVal == 'z') {
      //Stop
  }

  delay(1000);
}


void tLED(void *p){
	
	
}

void tAudio(void *p){
	
}

void setup(){
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CLOCK_SHIFTREG,OUTPUT);   
  pinMode(LATCH,OUTPUT);   
  pinMode(DATA,OUTPUT);
  Serial.begin(9600);
//	Serial.begin(115200);
//	attachInterrupt(0, INT0_ISR, RISING);
//	semaphore = xSemaphoreCreateBinary();
//	semaphoreFull = xSemaphoreCreateCounting(4, 4);
//	semaphoreEmpty = xSemaphoreCreateCounting(4,0);
//	semaphoreMutex = xSemaphoreCreateMutex();

}

void loop() {
	xTaskCreate(producer, "Producer", STACK_SIZE, (void * ) 1, 1, NULL);
	xTaskCreate(consumer, "Consumer", STACK_SIZE, (void * ) 1, 1, NULL);
	vTaskStartScheduler();
}
