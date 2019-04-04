char blueToothVal;
char lastValue;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
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
  delay(1000);
}
