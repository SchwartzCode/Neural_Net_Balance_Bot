// Basic demo for accelerometer readings from Adafruit MPU6050
#include <MatrixMath.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu; //initialize IMU object
// bounds for PWM values - used to clip input to analogWrite()
uint8_t maxPWM = 255;
uint8_t minPWM = 0;

// used to measure time at beginning and end of each loop
unsigned long currTime = 0;
unsigned long lastTime = 0;
const double dt = 0.0033;

// constant used to blend accelerometer & gyroscope angle estimates
//(1-alpha)*angle_gyro
float alpha = 0.05; 

// initialize variables
float angle = 0.0;
float lastAngle = 0.0;
float I_error = 0.0;

// not used currently but will maybe eventually move gains up here
//float Kp = 75;
//float Ki = 0.0;
//float Kd = 0.0;          

// initialize PID terms 
float P = 0.0;
float I = 0.0;
float D = 0.0;

// initialize matrices for synapse weights and biases
const mtx_type ang_b0[6][1] = { {0.001547},
                                {0.320026},
                                {0.340354},
                                {0.010771},
                                {1.562025},
                                {2.029194}};
                                
const mtx_type ang_w0[6][4] = {{0.000343, -0.070401, -1.655140, -0.002654},
                              {-0.009620, -0.214173, 0.294059, 0.002841},
                              {0.026603, -0.230408, -0.318019, 0.000271},
                              {-0.002329, -0.061315, 1.544243, 0.002200},
                              {-0.814444, 0.020287, -0.337145, 0.006185},
                              {-0.225695, 0.027229, 0.540696, 0.012758}};

const mtx_type ang_b1[1] = {0.029889};

const mtx_type ang_w1[6] = {0.701578, 0.119854, -0.109671, -0.754005, -1.482640, 1.128075};

const mtx_type controller_b0[6][1] = {{0.001547},
                                      {0.320026},
                                      {0.340354},
                                      {0.010771},
                                      {1.562025},
                                      {2.029194}};

const mtx_type controller_w0[6][4] = {{0.000343, -0.070401, -1.655140, -0.002654},
                                      {-0.009620, -0.214173, 0.294059, 0.002841},
                                      {0.026603, -0.230408, -0.318019, 0.000271},
                                      {-0.002329, -0.061315, 1.544243, 0.002200},
                                      {-0.814444, 0.020287, -0.337145, 0.006185},
                                      {-0.225695, 0.027229, 0.540696, 0.012758}};

const mtx_type controller_b1[1] = {0.029889};

const mtx_type controller_w1[6] = {0.701578, 0.119854, -0.109671, -0.754005, -1.482640, 1.128075};

  
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
      Serial.println("ruh roh loopie");
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
  Serial.println("y_accel | z_accel | gyro_x | I_error | PID_PWM");

  // toggle pins connected to motor controller to output
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}



void loop() {
  //get time when loop starts
  currTime = millis(); 
  //calculate time since last loop started
  unsigned long elapsedTime = currTime - lastTime; 
  Serial.println(elapsedTime);

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // correct for IMU offsets (determined experimentally)
  double z_accel = a.acceleration.z + 1.6;
  double y_accel = a.acceleration.y + 0.5;
  double x_accel = a.acceleration.x - 0.67;
  double gyro_x = g.gyro.x + 0.05;

  // calculate accelerometer & gyroscope angle estimates
  double angle_accel = atan(y_accel / z_accel);
  double angle_gyro = angle + gyro_x * dt;

  // blend two estimates together
  angle = (1 - alpha) * (angle_gyro) + alpha*angle_accel;

  // determine error terms using IMU readings
  float D_error = -gyro_x;
  float error = -angle;
  
  
  I_error += error*elapsedTime;

  // sum P I D terms to get desired PWM output to motors
  double PID_PWM = feedForward(y_accel, x_accel, gyro_x, I_error);

  // force positive since analogWrite only takes values in range (0,255)
  int outPWM = abs(PID_PWM);

  Serial.println(PID_PWM);

  // clip PWM value at limits of range (0 and 255)
  if (outPWM > maxPWM) {
    outPWM = maxPWM;
  } else if (outPWM < minPWM) {
     outPWM = minPWM;
  } else {
    // no clipping needed, must cast to int though (analogWrite() takes ints)
    outPWM = (int) abs(PID_PWM);
  }

  
  if ((abs(D_error) + abs(error)) > 0.1) {
     if(PID_PWM > 0) {
        // we want wheels to move backward
        digitalWrite(12, LOW);
        digitalWrite(13, HIGH);
      } else {
        // we want wheels to move forward
        digitalWrite(12, HIGH);
        digitalWrite(13, LOW);
      }
    // errors are too large, send current to motors to correct!
    analogWrite(3, outPWM);
    analogWrite(11, outPWM);
  } else {
    // errors are small enough; no need to drive motors
    analogWrite(3, 0);
    analogWrite(11, 0);
    // reset I term to prevent integral windup
    I_error = 0;
  }

  // lastTime is now the time when this pass through loop started
  lastTime = currTime; 
}


/*
 *  UTILITY FUNCTIONS
 */

double feedForward(float in1, float in2, float in3, float in4) {

//  mtx_type input[4] = {in1,in2,in3,in4};
//
////  Matrix.Print((mtx_type*)input, 1, 4, "input");
//
//  mtx_type tmp1[4];
//  Matrix.Multiply((mtx_type*)input, (mtx_type*)hidden1, 1, 4, 4, (mtx_type*)tmp1);
//
//   
//  // apply activation function
//  for(int i=0;i<4;i++){
//    prelu(tmp1[i]);
//  }
//
//  mtx_type tmp2[4];
//  Matrix.Multiply((mtx_type*)tmp1, (mtx_type*)hidden2, 1, 4, 4, (mtx_type*)tmp2);
//  // apply activation func
//  for(int i=0; i<4;i++){
//    prelu(tmp2[i]);
//  }
//
//  mtx_type tmp3[4];
//  Matrix.Multiply((mtx_type*)tmp2, (mtx_type*)hidden3, 1, 4, 4, (mtx_type*)tmp3);
//
//  //apply activation func
//  for(int i=0;i<4;i++){
//    prelu(tmp3[i]);
//  }
//  
//  mtx_type tmp_output[1];
//  Matrix.Multiply((mtx_type*)tmp3, (mtx_type*)output, 1, 4, 1, (mtx_type*)tmp_output);
//
//  double NN_PWM = 6.5*tmp_output[0];

//  return NN_PWM;
  return 0.0;
}

// better activation function
void prelu(double &input) {
  if(input < 0.0){
    input = 0.25*input;
  }
}

// not so good activation function
void relu6(double &input) {
  if(input < 0.0){
    input = 0.0;
  } else if(input > 6.0){
    input = 6.0;
  }
}
