// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MatrixMath.h>
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
int last_output = 0;

// encoder variables
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 4;//B pin -> the digital pin 3
byte encoder0PinALast;
int duration = 0;//the number of the pulses
boolean Direction;//the rotation direction

long wheel_pos = 0; // distance travelled by wheel (in encoder ticks)
long wheel_pos_last = 0; // last position of wheels (for calculating d term)
long desired_pos = TICKS_PER_REV * 0.0 / WHEEL_CIRCUMF; //position in ticks to drive robot to (pos initially in m)

// init neural net weights
// initialize matrices for synapse weights and biases
const mtx_type ang_b0[6] = {0.001547, 0.320026, 0.340354, 0.010771, 1.562025, 2.029194};
                                
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
double  estimateAngle(){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // correct for IMU offsets (determined experimentally)
  double y_accel = a.acceleration.y + 0.41;
  double z_accel = a.acceleration.z +  1.76;
  double gyro_x = g.gyro.x + 0.05;

  mtx_type angle_inputs[4] = {angle, y_accel, z_accel, gyro_x};

  double new_angle_est = angle_feed_forward(angle_inputs);
  angle = new_angle_est;

  return gyro_x;
}

double angle_feed_forward(mtx_type inputs[4]){
  mtx_type tmp1[6];
  Matrix.Multiply((mtx_type*)ang_w0, (mtx_type*)inputs, 6, 4, 1, (mtx_type*)tmp1);

  for(int i=0; i<sizeof(tmp1)/sizeof(tmp1[0]); i++){
    tmp1[i] = relu(tmp1[i] + ang_b0[i]);
  }

  mtx_type tmp2[1];
  Matrix.Multiply((mtx_type*)ang_w1, (mtx_type*)tmp1, 1, 6, 1, (mtx_type*)tmp2);

  return tmp2[0] + ang_b1[0];
}

double get_controller_PWM_output(double gyro_x){
  mtx_type inputs[4] = {angle, gyro_x, wheel_pos, wheel_pos_last};

  mtx_type tmp1[6];
  Matrix.Multiply((mtx_type*)controller_w0, (mtx_type*)inputs, 6, 4, 1, (mtx_type*)tmp1);

  for(int i=0; i<sizeof(tmp1)/sizeof(tmp1[0]); i++){
    tmp1[i] = relu(tmp1[i] + controller_b0[i][0]);
  }

  mtx_type tmp2[1];
  Matrix.Multiply((mtx_type*)controller_w1, (mtx_type*)tmp1, 1, 6, 1, (mtx_type*)tmp2);

  wheel_pos_last = wheel_pos;

  return tmp2[0] + controller_b1[0];
}

double relu(double input) {
  double output = 0.0;
  if(input < 0.0){
    output = 0.0;
  } else {
    output = input;
  }

  return output;
}

// get sign of a number
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void loop() {

  double gyro_x = estimateAngle();

  double NN_PWM = 250*(get_controller_PWM_output(gyro_x) + 3.27);

  int outPWM = (int) abs(NN_PWM);

  Serial.println(NN_PWM);

    // clip PWM value at limits of range (0 and 255)
  if (outPWM > maxPWM) {
    outPWM = maxPWM;
  } else if (outPWM < minPWM) {
     outPWM = minPWM;
  } else {
    // no clipping needed, must cast to int though (analogWrite() takes ints)
    outPWM = (int) abs(NN_PWM);
  }

   if(NN_PWM > 0) {
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
  
  last_output = outPWM;
}
