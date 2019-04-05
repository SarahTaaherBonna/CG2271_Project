extern HardwareSerial Serial;

char blueToothVal;
char lastValue;

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
void setup() {
  Serial.begin(9600);
  //pinMode(13, OUTPUT);
}

void loop() {

}
