//The sample code for driving one way motor encoder
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration = 0;//the number of the pulses
boolean Direction;//the rotation direction

int wheel_pos = 0; // distance travelled by wheel (in encoder ticks)
// TODO: is int big enough? may need long


void setup()
{
  Serial.begin(57600);//Initialize the serial port
  EncoderInit();//Initialize the module
}

void loop()
{
//  Serial.print("Pulse:");
//  Serial.println(duration);
  Serial.print("Pos:");
  Serial.println(wheel_pos);
  duration = 0;
}

void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(0, readEncoder, CHANGE);
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
  }
  encoder0PinALast = Lstate;

  if(!Direction)  wheel_pos++;
  else  wheel_pos--;
}
