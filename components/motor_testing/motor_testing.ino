const int dt = 1000;

void setup() {
  // put your setup code here, to run once:
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(13, OUTPUT); //Initiates Motor Channel B pin
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(12, HIGH);
  digitalWrite(13, LOW);

  analogWrite(11, 100);

 /**
  for(int i=0; i<50; i+=2) {
    //analogWrite(3, 255);
    //delay(25);
    analogWrite(3, i);
    analogWrite(11, i);
    Serial.println(i);
    delay(250);
  }
 **/
  
  delay(500);
  //digitalWrite(12, LOW);
  
  delay(10);

}
