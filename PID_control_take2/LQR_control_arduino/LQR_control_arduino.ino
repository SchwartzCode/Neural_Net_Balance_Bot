// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu; //initialize IMU object
// bounds for PWM values - used to clip input to analogWrite()
#define maxPWM  255
#define minPWM  0

#define dt  0.0033 //found empirically, probably a bit off but close enough

// constant used to blend accelerometer & gyroscope angle estimates
//(1-alpha)*angle_gyro
#define alpha  0.05 

// constants for calculating distance travelled from encoder ticks
#define TICKS_PER_REV  1920
#define WHEEL_RADIUS   0.033  // [m]
#define WHEEL_CIRCUMF  0.2073 // [m]

byte newEncoderData = 0;

// initialize variables
float angle = 0.0;

// encoder variables
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration = 0;//the number of the pulses
boolean Direction;//the rotation direction

long wheel_pos = 0; // distance travelled by wheel (in encoder ticks)
long wheel_pos_last = 0; // last position of wheels (for calculating d term)
long desired_pos = TICKS_PER_REV * 0.5 / WHEEL_CIRCUMF; //position in ticks to drive robot to (pos initially in m)

void setup(void) {

  // open Serial interface
  Serial.begin(115200);

  EncoderInit();
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // IMU library tries to connect and read data
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  
  // set IMU parameters
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // options:  +- 2, 4, 8, 16 [G]
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // options:  250, 500, 1000, 2000 [DEG]
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // options:  260, 184, 94, 44, 21, 10, 5 [Hz]
  Serial.println("");
  delay(100);

  // toggle pins connected to motor controller to output
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

/*
 * Initialize encoder pins
 */
void EncoderInit()
{
  Direction = true;//default -> Forward
  pinMode(encoder0pinB,INPUT);
  attachInterrupt(0, readEncoder, CHANGE);
}

/*
 * Interrupt routine for reading encoder data
 */
void readEncoder()
{
//  newEncoderData = PIND;
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

/*
 *  Read accelerometer and gyroscope data from IMU
 */
void readSensors(double *gyro_x){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // correct for IMU offsets (determined experimentally)
  double x_accel = a.acceleration.x - 0.68;
  double y_accel = a.acceleration.y + 0.35;
  double z_accel = a.acceleration.z + 1.78;
  *gyro_x = g.gyro.x + 0.05;

  // calculate accelerometer & gyroscope angle estimates
  double angle_accel = atan(y_accel / z_accel);
  double angle_gyro = angle + *gyro_x * dt;

  // blend two estimates together
  angle = (1 - alpha) * (angle_gyro) + alpha*angle_accel;

}

void loop() {

  double angle = 0;
  double gyro_x = 0;

  readSensors(&gyro_x);

  // force positive since analogWrite only takes values in range (0,255)
  outPWM = abs(LQR_PWM);

    // clip PWM value at limits of range (0 and 255)
  if (outPWM > maxPWM) {
    outPWM = maxPWM;
  } else if (outPWM < minPWM) {
     outPWM = minPWM;
  } else {
    // no clipping needed, must cast to int though (analogWrite() takes ints)
    outPWM = (int) abs(LQR_PWM);
  }

   if(LQR_PWM > 0) {
      // we want wheels to move backward
      digitalWrite(12, LOW);
      digitalWrite(13, HIGH);
    } else {
      // we want wheels to move forward
      digitalWrite(12, HIGH);
      digitalWrite(13, LOW);
    }
    // write PWM duty cycle to motors
    analogWrite(3, outPWM);
    analogWrite(11, outPWM);

}
