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

// initialize variables
float angle = 0.0;
float I_error = 0.0;
float y_vel = 0.0;
float y_pos = 0.0;

  
void setup(void) {

  // open Serial interface
  Serial.begin(115200);
  
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

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  // what information we'll be printing out (for training neural network)
//  Serial.println("y_accel | z_accel | gyro_x | I_error | PID_PWM");

  // toggle pins connected to motor controller to output
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

/*
 *  Read accelerometer and gyroscope data from IMU
 */
void readSensors(double *gyro_x){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // correct for IMU offsets (determined experimentally)
  double x_accel = a.acceleration.x - 0.78;
  double y_accel = a.acceleration.y + 0.425;
  double z_accel = a.acceleration.z + 1.78;
  *gyro_x = g.gyro.x + 0.05;

  // calculate accelerometer & gyroscope angle estimates
  double angle_accel = atan(y_accel / z_accel);
  double angle_gyro = angle + *gyro_x * dt;

  // blend two estimates together
  angle = (1 - alpha) * (angle_gyro) + alpha*angle_accel;

  // update y velocity using acceleration
  y_vel += y_accel*dt;
  // update y_position using y_vel
  y_pos += y_vel*dt;
}

/*
 *  Calculate PWM duty cycle to apply to motore given estimated angular position & velocity
 */
float attitude_PID(float *PID_PWM, int *outPWM, const float gyro_x){
  // determine error terms using IMU readings
  float D_error = -gyro_x;
  float error = -angle;
  I_error += error*dt;

  // determine P, I, and D terms using errors and gains
  float P = -2500 * error;
  float I = -1250 * I_error;
  float D = -200 * D_error;

  // sum P I D terms to get desired PWM output to motors
  *PID_PWM = P + I + D; //apply positional PID here too

  // force positive since analogWrite only takes values in range (0,255)
  *outPWM = abs(*PID_PWM);

    // clip PWM value at limits of range (0 and 255)
  if (*outPWM > maxPWM) {
    *outPWM = maxPWM;
  } else if (*outPWM < minPWM) {
     *outPWM = minPWM;
  } else {
    // no clipping needed, must cast to int though (analogWrite() takes ints)
    *outPWM = (int) abs(*PID_PWM);
  }

  return D_error;
}

void loop() {

  double angle = 0;
  double gyro_x = 0;
  readSensors(&gyro_x);

  float PID_PWM = 0.0;
  int outPWM = 0;
  float D_error = attitude_PID(&PID_PWM, &outPWM, gyro_x);

   if(PID_PWM > 0) {
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

  // reset I term when angular vel is low to prevent windup
  //    note: didn't include error because it's alright if angle is a bit off (esp. when trying to move to different translational position)
  if (abs(D_error) < 0.1) {
    I_error = 0;
  }

  
  Serial.println(y_vel);
  Serial.println(y_pos);
  Serial.println();

}
