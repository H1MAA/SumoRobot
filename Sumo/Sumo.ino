#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <VL53L0X.h>

#define front_leftIR 8 
#define front_rightIR 12 
#define leftIR A1 
#define rightIR 11
#define backIR A7


#define left_Forward 10
#define left_Backward 9
#define right_Forward 6
#define right_Backward 5


// ================================================================================================
// ===                                    IMU VARIABLES                                         ===
// ================================================================================================
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t fifoBuffer[64];
MPU6050 mpu;

// ================================================================================================
// ===                              FUNCTIONS DEFINITION                                        ===
// ================================================================================================
void updateIR(void);
void go_forward(void);
void go_backward(void);
void go_right(void);
void go_left(void);
void turn_left(void);
void turn_right(void);
void stop_motor(void);


VL53L0X tof_sensor;
char ir_data[5] = {0,0,0,0,0}; //front_left, front_right, left, right, back

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  init_imu();

  // INIT TIME OF FLIGHT
  tof_sensor.setTimeout(500);
   if (!tof_sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  tof_sensor.startContinuous();

  // SET MOTORS INITIAL STATE TO LOW
  analogWrite(left_Forward,0);
  analogWrite(right_Forward,0);
  analogWrite(left_Backward,0);
  analogWrite(left_Backward,0);
  
  
// ==============================================================================================
// ===                            PINMODES CONFIGURATION                                      ===
// ==============================================================================================

  pinMode(front_leftIR,INPUT_PULLUP); //pullup to work without IR sensors
  pinMode(front_rightIR,INPUT_PULLUP);
  pinMode(leftIR,INPUT_PULLUP);
  pinMode(rightIR,INPUT_PULLUP);
  pinMode(backIR,INPUT);
  pinMode(left_Forward,OUTPUT);
  pinMode(left_Backward,OUTPUT);
  pinMode(right_Forward,OUTPUT);
  pinMode(right_Backward,OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  updateIR();
  read_imu();
  int readingTOF = tof_sensor.readRangeContinuousMillimeters();
  
  if(readingTOF < 1500) //smash forward
  {
    go_forward();
    Serial.println("forward");
  }
  
  else
  {
    if (!ir_data[0]) //FRONT LEFT IR DETECTS OBJECT
    {
      go_left();
    }
    if (!ir_data[1]) //FRONT RIGHT IR DETECTS OBJECT
    {
      go_right();     
    }
    if (!ir_data[2]) //LEFT IR DETECTS OBJECT
    {
      turn_left();
    }
    if (!ir_data[3])  //RIGHT IR DETECTS OBJECT
    {
      turn_right();
    }
    if (!ir_data[4]) // BACK IR DETECTS OBJECT
    {
      //idk what we do here
    }
  
  }
}

void updateIR(void){

  ir_data[0] = digitalRead(front_leftIR);
  ir_data[1] = digitalRead(front_rightIR);
  ir_data[2] = digitalRead(leftIR);
  ir_data[3] = digitalRead(rightIR);
  ir_data[4] = digitalRead(backIR);
}
// ==================================================================================================
// ===                                  MOTOR CONTROL FUNCTIONS                                   ===
// ==================================================================================================

void go_forward(void){
  // Set motors forward    
  analogWrite(left_Forward,255);
  analogWrite(right_Forward,255);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
  Serial.println("Forward");
}

void go_backward(void){
  // Set motors forward    
  analogWrite(left_Forward,0);
  analogWrite(right_Forward,0);
  analogWrite(left_Backward,255);
  analogWrite(right_Backward,255);
  Serial.println("Forward");
}
void go_left(void){
  // Set motors forward    
  analogWrite(left_Forward,180);
  analogWrite(right_Forward,255);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
  Serial.println("Forward");
}
void go_right(void){
  // Set motors forward    
  analogWrite(left_Forward,255);
  analogWrite(right_Forward,180);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
  Serial.println("Forward");
}
void turn_left(void){
  // Set motors forward    
  analogWrite(left_Forward,0);
  analogWrite(right_Forward,255);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
  Serial.println("Forward");
}
void turn_right(void){
  // Set motors forward    
  analogWrite(left_Forward,255);
  analogWrite(right_Forward,0);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
  Serial.println("Forward");
}
void stop_motor(void){
  analogWrite(left_Forward,0);
  analogWrite(right_Forward,0);
  analogWrite(left_Backward,0);
  analogWrite(right_Backward,0);
}


// ================================================================================================
// ===                                 IMU FUNCTIONS                                           ====
// ================================================================================================
void init_imu()
{
  mpu.initialize();
  mpu.dmpInitialize();     // load and configure the DMP
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);  
}

void read_imu()
{
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))  // Get the Latest packet
  { 
    mpu.dmpGetQuaternion(&q, fifoBuffer);    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
  }
}
// ================================================================================================
// ===                                       SEARCH ALGORITHM                                  ====
// ================================================================================================

void search(void) //modify search algorithm if needed
{
  analogWrite(left_Forward,0);
  analogWrite(right_Forward,255);
  analogWrite(left_Backward,255);
  analogWrite(right_Backward,0);
  Serial.println("0 255 255 0"); //frontleft backleft frontright backright  
}
