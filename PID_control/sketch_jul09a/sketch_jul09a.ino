void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Forward!");

  digitalWrite(12, HIGH);
  
  analogWrite(3, 250);
  delay(50);

  Serial.println("Reverse!");
  digitalWrite(12, LOW);

  
  analogWrite(3, 250);
  delay(50);
    
}
