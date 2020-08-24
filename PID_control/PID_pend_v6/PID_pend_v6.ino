// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
uint8_t maxPWM = 250;
uint8_t minPWM = 50;

int counter = 0;
unsigned long currTime = 0, lastTime = 0;
float alpha = 0.75;
const double dt = 0.0033;

float angle = 0.0;
float lastAngle = 0.0;
float lastError = 0.0;
float cumError = 0.0;

float Kp = 75;
float Ki = 0.0;
float Kd = 0.0;          

float P = 0.0;
float I = 0.0;
float D = 0.0;
float I2 = 0.0;



  
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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

  currTime = millis();
  lastTime = millis();
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  currTime = millis();
  unsigned long timeDiff = currTime - lastTime;
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  double z_accel = a.acceleration.z + 2.3;
  double y_accel = a.acceleration.y + 0.37;
  double x_accel = a.acceleration.x - 0.67;

  double angle_accel = atan(z_accel / y_accel);
  double angle_gyro = g.gyro.x * dt;

  angle = (1 - alpha) * (angle + angle_gyro) + alpha*angle_accel;
  //Serial.println(360*angle/M_PI);

  

  float error = -0.05 - g.gyro.x;
  float angle_error = -0.02 - angle;
  
  
  float P = -700.0 * angle_error; //600 * error;

  cumError += angle_error * timeDiff;
  float I = 0.0 * cumError; //-300.0 * angle_error;

  D = 0.0 *(error - lastError);

  float accel_err = -2.6 - a.acceleration.z;
  

  lastError = angle_error;

  float PID_PWM = P + I + D;


  int outPWM = abs(PID_PWM);

  //int mappedPWM = map(outPWM, 0, 500, 30, 250);

  if (outPWM > maxPWM) {
    outPWM = maxPWM;
  } else if (outPWM < minPWM) {
     outPWM = minPWM;
  } else {
    outPWM = (int) abs(PID_PWM);
  }

  Serial.println(angle);
 /*
  Serial.print(P);
  Serial.print(" - ");
  Serial.print(I);
  Serial.print(" - ");
  Serial.print(D);
  Serial.print(" - ");
  Serial.print(D_2);
  Serial.print(" = ");
  Serial.println(outPWM);
  */
  
//(abs(angle_error) + abs(error)) > 0.1 && 
if ((abs(angle_error) + abs(error)) > 0.05 && abs(angle_error) < 1) {
   if(PID_PWM > 0) {
      //Serial.println("BACKWARD!!");
      digitalWrite(12, LOW);
      digitalWrite(13, HIGH);
    } else {
      //Serial.println("FORWARD!");
      digitalWrite(12, HIGH);
      digitalWrite(13, LOW);
    }
  analogWrite(3, outPWM);
  analogWrite(11, outPWM);
} else {
  analogWrite(3, 0);
  analogWrite(11, 0);
}

  

  lastTime = currTime;
}
