extern HardwareSerial Serial;

char blueToothVal;
char lastValue;

int BIN_1 = 3;
int AIN_2 = 5;
int BIN_3 = 10;
int AIN_4 = 11;
int MAX_PWM_VOLTAGE = 240;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_3, OUTPUT);
  pinMode(AIN_4, OUTPUT);
}

void loop() {
  if(Serial.available()) {
    blueToothVal = Serial.read();
  }

  if(blueToothVal == 'n') {
    digitalWrite(13, HIGH);
    if(lastValue != 'n') {
      Serial.println(F("LED is on."));
    }
    lastValue = blueToothVal;
  }
  else if(blueToothVal == 'f') {
    digitalWrite(13, LOW);
    if(lastValue != 'f') {
      Serial.println(F("LED is off."));
    }
    lastValue = blueToothVal;
  }
  if(blueToothVal == 'O') {
      //Forwards
      analogWrite(BIN_1, 240);
      analogWrite(AIN_2, 240);
      analogWrite(BIN_3, 240);
      analogWrite(AIN_4, 240);
      //delay(2000);
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
      analogWrite(BIN_1, MAX_PWM_VOLTAGE);
      analogWrite(AIN_2, 0);
      analogWrite(BIN_3, MAX_PWM_VOLTAGE);
      analogWrite(AIN_4, 0);
      //delay(2000);
    }
    else if(blueToothVal == '[') {
      //turn 90 degrees Right
      analogWrite(BIN_1, 0);
      analogWrite(AIN_2, MAX_PWM_VOLTAGE);
      analogWrite(BIN_3, 0);
      analogWrite(AIN_4, MAX_PWM_VOLTAGE);
      delay(2000);
    }
    else if(blueToothVal == 'z') {
      //Stop
      analogWrite(BIN_1, 0);
      analogWrite(AIN_2, 0);
      analogWrite(BIN_3, 0);
      analogWrite(AIN_4, 0);
      //delay(2000);
    }
  delay(1000);
}
