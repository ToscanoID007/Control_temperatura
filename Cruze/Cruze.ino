const int pinCruce = 2;

void setup() {
  Serial.begin(115200); 
  
pinMode(pinCruce, INPUT_PULLUP);
}

void loop() {
  int estadoPin = digitalRead(pinCruce);
  Serial.println(estadoPin);
  delay(1); 
}