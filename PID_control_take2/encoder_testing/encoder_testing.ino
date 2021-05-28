//The sample code for driving one way motor encoder
const byte encoder0pinA = 4;//A pin -> the interrupt pin 0
const byte encoder0pinB = 5;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction

int pos = 0;

void setup()
{
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module
}

void loop()
{
  readEncoder(); //TODO: rename
//  delay(10);
  Serial.println(pos);
}

void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode(encoder0pinB,INPUT);
}

void readEncoder()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
    // maybe just subtract by more than 1? do some empirical testing
    if(Direction){
      pos++;
    } else {
      pos--;
    }
  }
  encoder0PinALast = Lstate;

  if(!Direction)  duration++;
  else  duration--;
}
