// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu; //initialize IMU object
// bounds for PWM values - used to clip input to analogWrite()
#define maxPWM  255
#define minPWM  0

// constant used to blend accelerometer & gyroscope angle estimates
//(1-alpha)*angle_gyro
#define alpha  0.05 

// constants for calculating distance travelled from encoder ticks
#define TICKS_PER_REV  1920
#define WHEEL_RADIUS   0.033  // [m]
#define WHEEL_CIRCUMF  0.2073 // [m]

// time step (updated every loop)
float dt = 0.0033; 
unsigned long last_time;

byte newEncoderData = 0;

// initialize IMU variables
float angle = 0.0;
double theta_dot = 0.0;

// encoder variables
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration = 0;//the number of the pulses
boolean Direction;//the rotation direction

long wheel_pos = 0; // distance travelled by wheel (in encoder ticks)
long wheel_pos_last = 0; // last position of wheels (for calculating x velocity)
long desired_pos = TICKS_PER_REV * 0.5 / WHEEL_CIRCUMF; //position in ticks to drive robot to (pos initially in m)

// LQR variables
float state[4] = {0.0, 0.0, 0.0, 0.0};
float desired_state[4] = {0.0, 0.0, 0.0, 0.0};
float LQR_gains[4] = {-0.00316228, -0.05138403, -0.57430165, -0.03902428};  // obtained from python control.LQR function

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

  // intitialize last time so first dt calculation isn't crazy
  last_time = millis();
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
void readSensors(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // correct for IMU offsets (determined experimentally)
  double x_accel = a.acceleration.x - 0.68;
  double y_accel = a.acceleration.y + 0.35;
  double z_accel = a.acceleration.z + 1.78;
  theta_dot = g.gyro.x + 0.05;

  // calculate accelerometer & gyroscope angle estimates
  double angle_accel = atan(y_accel / z_accel);
  double angle_gyro = angle + theta_dot * dt;

  // blend two estimates together
  angle = (1 - alpha) * (angle_gyro) + alpha*angle_accel;
}

/*
 *   Update state from IMU and encoder readings
 *        TODO: maybe add some sort of Luenberg observer here? since
 *              estimate of state can be gotten from last state + last output
 *    State vector: [x, x_dot, theta, theta_dot]
 */
void update_state(){
  // update time step based on how long last loop took
  unsigned long curr_time = millis();
  dt = (curr_time - last_time) / 1000;
  last_time = curr_time;

  // update state values
  state[0] = WHEEL_CIRCUMF * wheel_pos / TICKS_PER_REV;
  state[1] = WHEEL_CIRCUMF * (wheel_pos - wheel_pos_last) / (dt * TICKS_PER_REV);
  wheel_pos_last = wheel_pos;  // store for next loop
  state[2] = angle;
  state[3] = theta_dot;
}

/*
 *   Calculate input to the system based on current state and LQR gains K
 */
float calculate_LQR_PWM(){
  float LQR_PWM = 0;

  for(int i=0; i<4; i++){
    LQR_PWM -= LQR_gains[i] * (state[i] - desired_state[i]);
  }

  return LQR_PWM;
}

void loop() {

  // update IMU readings
  readSensors();

  // update state estimate from sensors
  update_state();

  float LQR_PWM = calculate_LQR_PWM();
  // force positive since analogWrite only takes values in range (0,255)
  int outPWM = abs(LQR_PWM);

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
