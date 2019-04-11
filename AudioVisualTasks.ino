#define REARLED_PIN 2
#define BUZZER_PIN 8
#define CLOCK_SHIFTREG 13
#define LATCH 4
#define DATA 7
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CLOCK_SHIFTREG,OUTPUT);   
  pinMode(LATCH,OUTPUT);   
  pinMode(DATA,OUTPUT);
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


void loop(){


}
