// Basic demo for accelerometer readings from Adafruit MPU6050

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

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
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

  // correct for IMU offsets (determined experimentally)
  double z_accel = 0.0; //a.acceleration.z + 1.6;
  double y_accel = 0.0; //a.acceleration.y + 0.5;
  double x_accel = 0.0; //a.acceleration.x - 0.67;
  double gyro_x = 0.0; //g.gyro.x + 0.05;

  for(int i=0; i<100; i++){
      
    /* Get new sensor events witthe readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // update sensor readings
    x_accel += a.acceleration.x - 0.68;
    y_accel += a.acceleration.y + 0.35;
    z_accel += a.acceleration.z +1.78;
    gyro_x += g.gyro.x + 0.05;
  }

  z_accel /= 100;
  y_accel /= 100;
  x_accel /= 100;
  gyro_x /= 100;
  
  // print values that will be used to train Neural Network
  Serial.println(x_accel);
  Serial.println(y_accel);
  Serial.println(z_accel);
  Serial.println(gyro_x);
  Serial.println("");
}
