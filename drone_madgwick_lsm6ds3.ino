// Black -> rear
// Red -> front
// Receiver -> left
// screen -L /dev/ttyUSB0 38400;  mv screenlog.0 Documents/drone-datalog.csv

/*

##### IMPORTANT FOR MPU6050 WITH DMP ######
TO THIS WORK PROPERLY YOU NEED TO MATCH THE DMP FREQUENCY TO THE OUR PID CALCULATION FREQUENCY (250HZ)
BY DEFAULT THE DMP IS 100Hz. TO FIX THAT YOU NEED MAKE THE FOLLOWING CHANGE ON THE MPU6050_6Axis_MotionApps20.c library:
setRate(4) -> setRate(3)
#define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01 -> #define MPU6050_DMP_FIFO_RATE_DIVISOR 0x00
###########################################

######## IMPORTANT FOR LSM6DS3 ############
TO THIS WORK PROPERLY YOU NEED DISABLE THE wire.begin() AND COMMENT OUT THE ERROR CHECK
ReturnError = IMU_HW_ERROR; -> // returnError = IMU_HW_ERROR;
Wire.begin(); -> // Wire.begin();
###########################################
*/

#define MPU6050_BOARD 0
#define LSM6DS3_BOARD 1

#define IMU_BOARD LSM6DS3_BOARD
// #define IMU_BOARD MPU6050_BOARD


// #include <Servo.h>
#include <SAMD_PWM.h>
#include <ServoInput.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MadgwickAHRS.h>
#if IMU_BOARD == MPU6050_BOARD
  #include "I2Cdev.h"
  #include "MPU6050.h"
  MPU6050 mpu;
#elif IMU_BOARD == LSM6DS3_BOARD
  #include "SparkFunLSM6DS3.h"
  LSM6DS3 myIMU(I2C_MODE, 0x6A);
#endif

/*
Todo list:
- Tune PID parameters
- Add battery voltage reading and compensation
- Add yaw control
- Check if there is realiable (async) way to write to the flight data to sdcard
- Check if doable to send telemetry over wifi
- Add barometer to control altitude
- How to improve the yaw control
- Add GPS control
- Add colision sensors
- Calculate the lift force
- Calculate the battery duration estimation

Evaluate list:
- Replace DMP with madgwick
- Replace MPU with arduino internal IMU
- Replace MPU6050 with ICM-20948
*/

// #define DEBUG_MODE true
// #define PRINT_PARAMETERS true


#define GYRO_RATE 2000

#if GYRO_RATE == 500
  #define GYRO_DIVISOR 65.5
#elif GYRO_RATE == 2000
  #define GYRO_DIVISOR 16.4
#endif


#define MPU6050_ADDRESS 0x68
#define INTERRUPT_PIN 2 //MPU6050 Interrupt pin

#define pinESC1 5
#define pinESC2 6
#define pinESC3 9
#define pinESC4 10
#define pinButton 3
#define pinBattery A2

#define testSpeed 0

#define PID_ANGLE_AMP 3
#define PID_MIN_SPEED_THRESHOLD 1100
#define PID_TAKEOFF_THRESHOLD 1200
#define MAX_RATE_SETPOINT_DPS 164

#define SPEED_LIMIT_RAW 1800
#define SPEED_MAX_OUTPUT 1900
#define SPEED_MIN_OUTPUT 1010

#define LEVEL_FILTER_ALPHA 0.05
#define LEVEL_GAIN_PITCH 3.0
#define LEVEL_GAIN_ROLL  3.0

#define PID_P_GAIN_PITCH 1.3 //0.8
#define PID_I_GAIN_PITCH 0.04 //0.004
#define PID_D_GAIN_PITCH 18//15 8 12

#define PID_P_GAIN_ROLL 1.3 //0.8
#define PID_I_GAIN_ROLL 0.04 //0.004 // 0.002 was ok , por regra de 3 deveria se 0.012 <- testar
#define PID_D_GAIN_ROLL 18//15 8 12

#define PID_P_GAIN_YAW 3 // 1
#define PID_I_GAIN_YAW 0.01 //0.02//0.002//0.02
#define PID_D_GAIN_YAW 0


#define PID_I_MAX 350 //150
#define PID_I_YAW_MAX 100 //150
#define PID_YAW_MAX 250 //150

#define REFERENCE_VOLTAGE  3.3    // Nano 33 IoT logic level
#define BATTERY_R1  10000.0
#define BATTERY_R2  2000.0

Madgwick filter;

// Servo esc1;
// Servo esc2;
// Servo esc3;
// Servo esc4;
SAMD_PWM* esc1;
SAMD_PWM* esc2;
SAMD_PWM* esc3;
SAMD_PWM* esc4;

const int SDA_PIN = A4; 
const int SCL_PIN = A5;

int engineSpeed = 1000;
int engineSpeed_raw = 1000;
int previous_engineSpeed_raw = 1000;

int batteryADCRaw = 0;
float batteryADC = 0;
float batteryVin = 0;
float batteryVinRaw = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float gx_dps, gy_dps, gz_dps, ax_gf, ay_gf, az_gf;

int16_t last_ax_raw, last_ay_raw, last_az_raw;
int16_t frozen_counter = 0;

// Angle variables
float angle_pitch, angle_roll, angle_yaw;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];        // [yaw, pitch, roll]
float gyro_filtered[3];  //
float gyro_offset[3];
int16_t gyro_internal_offset[3];
float accel_offset[3];

struct log {
  float gyro_pitch;
  float gyro_roll;
  float gyro_yaw;
  float angle_pitch;
  float angle_roll;
  float angle_yaw;
  int rc_engine_speed;
  float rc_pitch;
  float rc_roll;
  float rc_yaw;
  float error_pitch;
  float error_roll;
  float error_yaw;
  float i_pitch;
  float i_roll;
  float i_yaw;
  int leftRear;
  int rightRear;
  int leftFront;
  int rightFront;
  float batteryVin;
} flightLog;

// Timers
unsigned long loop_timer;
unsigned long log_timer;
unsigned long lastGoodPacket = 0;

// Status
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// Control
float pitchAngle = 0, rollAngle=0, yawAngle=0;
float pitch_level_adjust = 0, roll_level_adjust = 0, yaw_level_adjust = 0;
long pitchAngle_raw = 0, rollAngle_raw = 0, yawAngle_raw = 0 ;
long previous_pitchAngle_raw = 0, previous_rollAngle_raw = 0, previous_yawAngle_raw = 0 ;
ServoInputPin<3> rf_throttle;
ServoInputPin<A1> rf_pitch;
ServoInputPin<A7> rf_roll;
ServoInputPin<11> rf_yaw;


// PID
float pid_p_roll, pid_i_roll, pid_d_roll, pid_roll_output, roll_error, roll_error_previous;
float pid_p_yaw, pid_i_yaw, pid_d_yaw, pid_yaw_output, yaw_error, yaw_error_previous;
float pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_pitch_output, pitch_error, pitch_error_previous, pitch_gyro_desired, roll_gyro_desired, yaw_gyro_desired;
float pid_max = 400; //400;

String buf;

File fdr_file;


void setup() {

  // The first thing to do is attach to the esc's
  // esc1.attach(pinESC1,1000,2000);
  // esc2.attach(pinESC2,1000,2000);
  // esc3.attach(pinESC3,1000,2000);
  // esc4.attach(pinESC4,1000,2000);
  esc1 = new SAMD_PWM(pinESC1, 250.0f, 25.0f);
  esc2 = new SAMD_PWM(pinESC2, 250.0f, 25.0f);
  esc3 = new SAMD_PWM(pinESC3, 250.0f, 25.0f);
  esc4 = new SAMD_PWM(pinESC4, 250.0f, 25.0f);

  // And then set it the esc to 0 to arm the engines
  // esc1.write(1000);
  // esc2.write(1000);
  // esc3.write(1000);
  // esc4.write(1000);
  esc1->setPWM(pinESC1,250.0f,1000/40.0f);
  esc2->setPWM(pinESC2,250.0f,1000/40.0f);
  esc3->setPWM(pinESC3,250.0f,1000/40.0f);
  esc4->setPWM(pinESC4,250.0f,1000/40.0f);

  // Attach the servo inputs
  rf_throttle.attach();
  rf_pitch.attach();
  rf_roll.attach();
  rf_yaw.attach();

  // Start serial communication
  Serial.begin(230400);//115200
  // delay(8000);
  Serial.println("Starting setup");

  // Battery stuff
  analogReadResolution(12);
  batteryADC = (float)analogRead(pinBattery);
 

  // Start I2C communication
  Wire.begin();
  Wire.setClock(400000);
  // Wire.setClock(1000);
  // Wire.setWireTimeout(3000, true); //Only available on AVR based arduino

  // Setup IMU Board
  //delay(5);
  Serial.println("Starting setup");
  setupBoard();
  delay(5);

  pinMode(INTERRUPT_PIN, INPUT);

  // enable Arduino interrupt detection
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println(F(")..."));
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  
  delay(5);
  
  //Get gyro offset
  int samples_count=2000;
  float samples[3] = {0,0,0};
  for(int i=0; i < samples_count; i++){
      int16_t gx_raw, gy_raw, gz_raw;
      getRotation(&gx_raw, &gy_raw, &gz_raw); // direct register read, independent of the DMP FIFO
        samples[0] += gx_raw;
        samples[1] += gy_raw;
        samples[2] += gz_raw;
        delay(3);
  }

  gyro_offset[0] = samples[0] / samples_count;
  gyro_offset[1] = samples[1] / samples_count;
  gyro_offset[2] = samples[2] / samples_count;

  gyro_filtered[0]=0;
  gyro_filtered[1]=0;
  gyro_filtered[2]=0;


  // Setup madgwick filter
  filter.begin(250);
  
  Serial.print("Starting loop\n");
  // Serial.print("Pitch Gyro, Roll Gyro, Yaw Gyro, Pitch Angle, Roll Angle, Yaw angle, Desired Speed, Pitch Error, Roll Error, Yaw Error, Speed Rear Left, Speed Rear Right,	Speed Front Left, Speed Front Right, i_pitch, i_roll, i_yaw");
  Serial.print("\n");

  //Setup SD card
  // if(!SD.begin(4)){
  //   Serial.println(F("SD initialization failed!"));
  //   while(1); // voltar
  // }else{}
  // Serial.println("SD initialization done.");

  //Open flight record file
  // fdr_file = SD.open("fdr3.csv",FILE_WRITE);
  // fdr_file.println("gyro_pitch,gyro_roll,gyro_yaw,angle_pitch,angle_roll,angle_yaw,throttle,stick_pitch,stick_roll,error_pitch,error_roll,error_yaw,leftRear,rightRear,leftFront,rightFront,pid_i_pitch,pid_i_roll,pid_i_yaw");
  // fdr_file.flush();    
  // pinMode(A7, INPUT_PULLUP);
  // pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  buf="";
  // digitalWrite(LED_BUILTIN, HIGH);
  
  int16_t gx_raw, gy_raw, gz_raw;
  int16_t ax_raw, ay_raw, az_raw;
  
  // mpu.dmpGetQuaternion(&q, fifoBuffer);
  // mpu.dmpGetGravity(&gravity, &q);
  // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);      
  getMotion(&ax_raw, &ay_raw, &az_raw,&gx_raw, &gy_raw, &gz_raw);

  if (ax_raw == last_ax_raw && ay_raw == last_ay_raw && az_raw == last_az_raw) {
    frozen_counter++;
  } else {
    frozen_counter = 0; // Reset counter if data changes
  }

  gx_raw-=gyro_offset[0];
  gy_raw-=gyro_offset[1];
  gz_raw-=gyro_offset[2]; 

  gx_dps = gx_raw / GYRO_DIVISOR; // trustworthy, real deg/s
  gy_dps = gy_raw / GYRO_DIVISOR;
  gz_dps = gz_raw / GYRO_DIVISOR;
  
  #if IMU_BOARD == LSM6DS3_BOARD
  ax_raw -= accel_offset[0];
  ay_raw -= accel_offset[1];
  az_raw -= accel_offset[2];
  #endif

  ax_gf = ax_raw / 4096.0; // Convert to g-force
  ay_gf = ay_raw / 4096.0;
  az_gf = az_raw / 4096.0;


  #if IMU_BOARD == MPU6050_BOARD
    if(frozen_counter >=3){ // Check if still needed, and how to detect the issue
      resetWire();
    }else{
      filter.updateIMU(gx_dps,gy_dps,gz_dps,ax_gf,ay_gf,az_gf);
    }
    ypr[0] = filter.getYaw();
    ypr[1] = filter.getPitch();
    ypr[2] = filter.getRoll();
    gy_dps *=-1;
  #elif IMU_BOARD == LSM6DS3_BOARD
    filter.updateIMU(gx_dps,gy_dps,gz_dps,ax_gf,ay_gf,az_gf);
    ypr[0] = filter.getYaw() * -1;
    ypr[1] = filter.getPitch();
    ypr[2] = filter.getRoll() * -1;
    gx_dps *=-1;
  #endif


  // gy_dps *=-1;
  // gx_dps *=-1;

  if( ! (abs(gx_dps) > 400 || abs(gy_dps) > 400 || abs(gz_dps) > 400)  ){
    gyro_filtered[0]= ( (0.7 * gyro_filtered[0])  + (0.3 * gx_dps) );
    gyro_filtered[1]= ( (0.7 * gyro_filtered[1])  + (0.3 * gy_dps) );
    gyro_filtered[2]= ( (0.7 * gyro_filtered[2])  + (0.3 * gz_dps) );

  }
  lastGoodPacket = millis();

  

  // if (millis() - lastGoodPacket > 100) {  // way more than your ~5ms expected interval
  //   #ifdef DEBUG_MODE
  //     Serial.println("Reseting FIFO, more 100ms since last packet");
  //   #endif
  //   mpu.resetFIFO();
  //   lastGoodPacket = millis(); // avoid spamming resets
  // }



  last_ax_raw = ax_raw;
  last_ay_raw = ay_raw;
  last_az_raw = az_raw;

  #ifdef PRINT_PARAMETERS
    flightLog.gyro_pitch = gyro_filtered[1];
    flightLog.gyro_roll = gyro_filtered[0];
    flightLog.gyro_yaw = gyro_filtered[2];

    flightLog.angle_pitch = ypr[1];
    flightLog.angle_roll = ypr[0];
    flightLog.angle_yaw = ypr[2];

    // buf += String(gyro_filtered[1] );
    // buf += F(",");

    // buf += String(gyro_filtered[0] );
    // buf += F(",");

    // buf += String(gyro_filtered[2] );
    // buf += F(",");

    // buf += String(ypr[1]);
    // buf += F(",");

    // buf += String(ypr[2]);
    // buf += F(",");
    
    // buf += String(ypr[0]);
    // buf += F(",");
  #endif

  engineSpeed_raw = rf_throttle.getPulse();
  if(abs(engineSpeed_raw - previous_engineSpeed_raw) > 500){
    engineSpeed_raw= previous_engineSpeed_raw;
  }

  engineSpeed = (0.8 * engineSpeed) +  (0.2 * engineSpeed_raw);
  pitchAngle_raw = rf_pitch.mapDeadzone(-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS, 0.1) *-1;
  rollAngle_raw = rf_roll.mapDeadzone(-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS, 0.1);
  yawAngle_raw = rf_yaw.mapDeadzone(-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS, 0.1);

  if(abs(pitchAngle_raw - previous_pitchAngle_raw) > 60){
    pitchAngle_raw = previous_pitchAngle_raw;
  }

  if(abs(rollAngle_raw - previous_rollAngle_raw) > 60){
    rollAngle_raw = previous_rollAngle_raw;
  }

  if(abs(yawAngle_raw - previous_yawAngle_raw) > 60){
    yawAngle_raw = previous_yawAngle_raw;
  }

  pitchAngle = (0.7 * pitchAngle) + (0.3 * pitchAngle_raw);
  rollAngle = (0.7 * rollAngle) + (0.3 * rollAngle_raw);
  yawAngle = (0.7 * yawAngle) + (0.3 * yawAngle_raw);

  previous_pitchAngle_raw = pitchAngle_raw;
  previous_rollAngle_raw = rollAngle_raw;
  previous_yawAngle_raw = yawAngle_raw;
  previous_engineSpeed_raw = engineSpeed_raw;

  readBatteryVoltage();
  
  if(engineSpeed<1040){
    disableEngines();
  }else{
    // if(!fdr_file){
    //   fdr_file = SD.open("fdr3.csv",FILE_WRITE);
    // }
    enableEngines();
  }
  #ifdef PRINT_PARAMETERS
    flightLog.rc_engine_speed = engineSpeed;
    flightLog.rc_pitch = pitchAngle;
    flightLog.rc_roll = rollAngle;
    flightLog.rc_yaw = yawAngle;

    flightLog.batteryVin = batteryVin;

    // buf += String(engineSpeed);
    // buf += F(",");

    // buf += String(pitchAngle);
    // buf += F(",");

    // buf += String(rollAngle);
    // buf += F(",");
  #endif

  setAllEnginesSpeed(engineSpeed);
  #ifdef PRINT_PARAMETERS
    // Serial.print(buf);
    if(micros() - log_timer > 100000 ){
      char buffer[100];
      sprintf(buffer,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f", flightLog.gyro_pitch,
                                                flightLog.gyro_roll,
                                                flightLog.gyro_yaw,
                                                flightLog.angle_pitch,
                                                flightLog.angle_roll,
                                                flightLog.angle_yaw,
                                                flightLog.rc_engine_speed,
                                                flightLog.rc_pitch,
                                                flightLog.rc_roll,
                                                flightLog.rc_yaw,
                                                flightLog.batteryVin);
      Serial.println(buffer);
      log_timer = micros();
    }
    
  #endif

  while(micros() - loop_timer < 4000 );                                      //We wait until 4000us are passed. // VOLTAR
  #ifdef DEBUG_MODE
    Serial.print("\nLoop Timer:");
    Serial.print(micros() - loop_timer);
    Serial.print("\n");
  #endif
  loop_timer = micros();
  

  // digitalWrite(LED_BUILTIN, LOW);
  
  // comment out writing to file, as it could impact on the loop frquency
  // fdr_file.println(buf);

}

void disableEngines(){
  // esc1.writeMicroseconds(1000);
  // esc2.writeMicroseconds(1000);
  // esc3.writeMicroseconds(1000);
  // esc4.writeMicroseconds(1000);
  esc1->setPWM(pinESC1,250.0f,1000/40.0f);
  esc2->setPWM(pinESC2,250.0f,1000/40.0f);
  esc3->setPWM(pinESC3,250.0f,1000/40.0f);
  esc4->setPWM(pinESC4,250.0f,1000/40.0f);
  
  //fdr_file.flush();
  
  // if(fdr_file){
  //   fdr_file.flush();
  //   fdr_file.close();
  // }        
}

void enableEngines(){
 
  // if(!esc1.attached()){
  //   esc1.attach(pinESC1,1000,2000);
  // }

  // if(!esc2.attached()){
  //   esc2.attach(pinESC2,1000,2000);
  // }

  // if(!esc3.attached()){
  //   esc3.attach(pinESC3,1000,2000);
  // }

  // if(!esc4.attached()){
  //   esc4.attach(pinESC4,1000,2000);
  // }  
}


void setAllEnginesSpeed(int speed){

  int leftRear,rightRear,leftFront,rightFront; 

  if(speed > SPEED_LIMIT_RAW){
    speed = SPEED_LIMIT_RAW;
  }

  // pitch_level_adjust = ((1.0 - LEVEL_FILTER_ALPHA) * pitch_level_adjust) + (LEVEL_FILTER_ALPHA * ypr[1] * LEVEL_GAIN_PITCH);
  // roll_level_adjust  = ((1.0 - LEVEL_FILTER_ALPHA) * roll_level_adjust)  + (LEVEL_FILTER_ALPHA * ypr[2] * LEVEL_GAIN_ROLL);

  // pitch_gyro_desired = constrain(pitchAngle - pitch_level_adjust, -MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
  // roll_gyro_desired  = constrain(rollAngle  - roll_level_adjust,  -MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
  
  pitch_gyro_desired = constrain(( pitchAngle - ((ypr[1] * 5.0 ))),-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
  roll_gyro_desired =  constrain(( rollAngle - ((ypr[2] * 5.0 ))),-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);

  // pitch_gyro_desired = pitchAngle ;
  // roll_gyro_desired =  rollAngle ;

  yaw_gyro_desired = yawAngle;

  pitch_error = pitch_gyro_desired - gyro_filtered[1];
  roll_error  = roll_gyro_desired - gyro_filtered[0];
  yaw_error = yaw_gyro_desired - gyro_filtered[2];

  // if (pitch_error >= -0.2 && pitch_error <=0.2){
  //   pitch_error=0;
  // }

  // if (roll_error >= -0.2 && roll_error <=0.2){
  //   roll_error=0;
  // }

  // if( yaw_error >= 1 && yaw_error <= 1  ){
  //     yaw_error=0;
  // }

  #ifdef PRINT_PARAMETERS
    flightLog.error_pitch = pitch_error;
    flightLog.error_roll = roll_error;
    flightLog.error_yaw = yaw_error;
    // buf += String(pitch_error);
    // buf += F(",");

    // buf += String(roll_error,3);
    // buf += F(",");    

    // buf += String(yaw_error,3);
    // buf += F(",");
  #endif

  if(speed >= PID_MIN_SPEED_THRESHOLD){
    
    // Calculate  pid_pitch_output 
    pid_p_pitch  = pitch_error * PID_P_GAIN_PITCH;
    if( speed > PID_TAKEOFF_THRESHOLD){
      pid_i_pitch  += pitch_error * PID_I_GAIN_PITCH;
      if (pid_i_pitch > PID_I_MAX) {
          pid_i_pitch = PID_I_MAX;      
      }else if(pid_i_pitch < PID_I_MAX * -1){
          pid_i_pitch = PID_I_MAX * -1;
      }
    }else{
      pid_i_pitch = 0;
    }
    pid_d_pitch  = (pitch_error - pitch_error_previous) * PID_D_GAIN_PITCH;

    pid_pitch_output = pid_p_pitch + pid_i_pitch + pid_d_pitch;
    if (pid_pitch_output > pid_max) {
        pid_pitch_output = pid_max;      
    }else if(pid_pitch_output < pid_max * -1){
        pid_pitch_output = pid_max * -1;    
    }
    pitch_error_previous = pitch_error;

    // Calculate  pid_roll_output  
    pid_p_roll = roll_error * PID_P_GAIN_ROLL;
    if( speed > PID_TAKEOFF_THRESHOLD){
      pid_i_roll += roll_error * PID_I_GAIN_ROLL;
      if (pid_i_roll > PID_I_MAX) {
          pid_i_roll = PID_I_MAX;      
      }else if(pid_i_roll < PID_I_MAX * -1){
          pid_i_roll = PID_I_MAX * -1;
      }
    }else{
      pid_i_roll = 0;
    }       
    pid_d_roll = (roll_error - roll_error_previous) * PID_D_GAIN_ROLL;

    pid_roll_output = pid_p_roll + pid_i_roll + pid_d_roll;

    if (pid_roll_output > pid_max) {
        pid_roll_output = pid_max;      
    }else if(pid_roll_output < pid_max * -1){
        pid_roll_output = pid_max * -1;    
    }

    roll_error_previous = roll_error;

    // Calculate  pid_yaw_output  
    pid_p_yaw = yaw_error * PID_P_GAIN_YAW;
    if( speed > PID_TAKEOFF_THRESHOLD){
      pid_i_yaw += yaw_error * PID_I_GAIN_YAW; 
      if (pid_i_yaw > PID_I_YAW_MAX) {
          pid_i_yaw = PID_I_YAW_MAX;      
      }else if(pid_i_yaw < PID_I_YAW_MAX * -1){
          pid_i_yaw = PID_I_YAW_MAX * -1;
      }
    }else{
      pid_i_yaw = 0;
    }
    pid_d_yaw = (yaw_error - yaw_error_previous) * PID_D_GAIN_YAW;

    pid_yaw_output = pid_p_yaw + pid_i_yaw + pid_d_yaw;
    if (pid_yaw_output > PID_YAW_MAX) {
        pid_yaw_output = PID_YAW_MAX;      
    }else if(pid_yaw_output < PID_YAW_MAX * -1){
        pid_yaw_output = PID_YAW_MAX * -1;    
    }
    yaw_error_previous = yaw_error;
  }

  leftRear = speed +   ((-pid_pitch_output + pid_roll_output - pid_yaw_output));
  rightRear = speed +  ((-pid_pitch_output - pid_roll_output + pid_yaw_output));
  leftFront= speed +   ((pid_pitch_output + pid_roll_output + pid_yaw_output));
  rightFront = speed + ((pid_pitch_output - pid_roll_output - pid_yaw_output));

  //Battery drop compensation
  if (batteryVin < 12.4f && batteryVin > 8.0f) {
    float compensation = (12.4f - batteryVin) / 35.0f;
    leftRear   += leftRear * compensation;
    rightRear  += rightRear * compensation;
    leftFront  += leftFront * compensation;
    rightFront += rightFront * compensation;
  }

  if(leftRear > SPEED_MAX_OUTPUT){
    leftRear = SPEED_MAX_OUTPUT;
  }
  if(rightRear > SPEED_MAX_OUTPUT){
    rightRear = SPEED_MAX_OUTPUT;
  }
  if(leftFront > SPEED_MAX_OUTPUT){
    leftFront = SPEED_MAX_OUTPUT;
  }
  if(rightFront > SPEED_MAX_OUTPUT){
    rightFront = SPEED_MAX_OUTPUT;
  }

  if(leftRear < SPEED_MIN_OUTPUT){
    leftRear = SPEED_MIN_OUTPUT;
  }
  if(rightRear < SPEED_MIN_OUTPUT){
    rightRear = SPEED_MIN_OUTPUT;
  }
  if(leftFront < SPEED_MIN_OUTPUT){
    leftFront = SPEED_MIN_OUTPUT;
  }
  if(rightFront < SPEED_MIN_OUTPUT){
    rightFront = SPEED_MIN_OUTPUT;
  }

  if(speed < PID_MIN_SPEED_THRESHOLD){
    leftRear    = speed;
    rightRear   = speed;
    leftFront   = speed;
    rightFront  = speed;
    pid_i_pitch = 0; //REMOVE IF WRONG
    pid_i_roll = 0;
    pid_i_yaw = 0;
    pitch_error_previous = 0;
    roll_error_previous = 0;
    yaw_error_previous = 0;
  }

  //Rear engines
  // esc1.writeMicroseconds(leftRear);//Left 
  // esc2.writeMicroseconds(rightRear);//Right
  esc1->setPWM(pinESC1,250.0f,leftRear/40.0f);
  esc2->setPWM(pinESC2,250.0f,rightRear/40.0f);


  //Front engines
  // esc3.writeMicroseconds(leftFront);//Left  
  // esc4.writeMicroseconds(rightFront);//Right
  esc3->setPWM(pinESC3,250.0f,leftFront/40.0f);
  esc4->setPWM(pinESC4,250.0f,rightFront/40.0f);

  #ifdef PRINT_PARAMETERS
    
    flightLog.i_pitch = pid_i_pitch;
    flightLog.i_roll = pid_i_roll;
    flightLog.i_yaw = pid_i_yaw;

    flightLog.leftRear = leftRear;
    flightLog.rightRear = rightRear;
    flightLog.leftFront = leftFront;
    flightLog.rightFront = rightFront;
    
    // buf += String(leftRear);
    // buf += F(",");

    // buf += String(rightRear);
    // buf += F(",");

    // buf += String(leftFront);
    // buf += F(",");
      
    // buf += String(rightFront);
    // buf += F(",");  

    // buf += String(pid_i_pitch);
    // buf += F(",");
    
    // buf += String(pid_i_roll);
    // buf += F(",");  
      
    // buf += String(pid_i_yaw);
  #endif
   
}

void dmpDataReady() {
    mpuInterrupt = true;
}

void resetWire(){
  #ifdef DEBUG_MODE
    Serial.println("DMP disabled/stuck, reseting WIRE communication");
  #endif
  Wire.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);

  // If SDA is held low by a slave, pump the clock line to force it to release SDA
  for (int i = 0; i < 9; i++) {
    if (digitalRead(SDA_PIN) == HIGH) {
      break; // Slave released the line, we can stop
    }
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }

  // Send a STOP condition manually
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); // SDA goes high while SCL is high = STOP
  delayMicroseconds(5);

  Wire.begin();
  Wire.setClock(400000);
}


void setupBoard()
{
  Serial.println("Setup IMU board.");
  #if IMU_BOARD == MPU6050_BOARD
    mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    #ifdef GYRO_RATE == 2000
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    #elif GYRO_RATE == 500
      mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    #endif
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setRate(3);
    mpu.setSleepEnabled(false);

    // Calibrate MPU6050
  
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(15);
    // mpu.CalibrateGyro(15);

    // 13:27:08.558 -> ....................	XAccel			YAccel				ZAccel			XGyro			YGyro			ZGyro
    // 13:27:43.732 ->  [-2101,-2100] --> [-35,9]	[-1933,-1932] --> [-2,2]	[1124,1125] --> [16262,16395]	[-56,-55] --> [0,2]	[108,109] --> [-3,2]	[64,65] --> [-8,2]
    // 13:27:45.346 ->  [-2101,-2100] --> [-49,9]	[-1933,-1932] --> [-3,2]	[1124,1125] --> [16191,16395]	[-56,-55] --> [0,2]	[108,109] --> [-4,2]	[64,65] --> [-11,2]
    // 13:28:15.699 -> -------------- done --------------

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-56); //-55
    mpu.setYGyroOffset(108); //112
    mpu.setZGyroOffset(64); //63
    mpu.setXAccelOffset(-2101); // -2101
    mpu.setYAccelOffset(-1933); // -1933
    mpu.setZAccelOffset(1124); // 1124
  #elif IMU_BOARD == LSM6DS3_BOARD
    myIMU.settings.accelEnabled = 1;
    myIMU.settings.accelRange = 8;          // Keep your 8g Range
    myIMU.settings.accelSampleRate = 416;   // Keep your 260Hz ODR
    myIMU.settings.accelBandWidth = 50;

    myIMU.settings.gyroEnabled = 1;
    myIMU.settings.gyroRange = GYRO_RATE;        // <-- Sets Gyroscope to +/- 2000 dps range
    myIMU.settings.gyroSampleRate = 416;    // Keep your 260Hz ODR
    myIMU.settings.gyroBandWidth = 50;

    accel_offset[0] = 55;
    accel_offset[1] = 74; 
    accel_offset[2] = 34;
    // gyro_offset = {-3, 1, 2};
    
    // if (0 != 0) {

    // Make a SensorSettings object to remember what you wanted to set everyhting to
    SensorSettings settingsIWanted;
    
    int test = myIMU.begin(&settingsIWanted);
    if (test != 0) {
      Serial.println("Device error or not connected.");
      Serial.println(test);
      while (1); 
    }
    // compareSettings(settingsIWanted);
    // while (1);


  #endif



}

// void compareSettings(SensorSettings desiredSettings){
//   if(myIMU.settings.accelBandWidth != desiredSettings.accelBandWidth )    { Serial.println("'accelBandWidth' was changed!"); }
//   if(myIMU.settings.accelRange != desiredSettings.accelRange )            { Serial.println("'accelRange' was changed!"); }
//   if(myIMU.settings.accelSampleRate != desiredSettings.accelSampleRate )  { Serial.println("'accelSampleRate' was changed!"); }
//   if(myIMU.settings.gyroRange != desiredSettings.gyroRange )              { Serial.println("'gyroRange' was changed!"); }
//   if(myIMU.settings.gyroSampleRate != desiredSettings.gyroSampleRate )    { Serial.println("'gyroSampleRate' was changed!"); }
//   Serial.println("Device config ok.");
// }

void getRotation(int16_t* x, int16_t* y, int16_t* z){
  #if IMU_BOARD == MPU6050_BOARD
    mpu.getRotation(x, y, z);
  #elif IMU_BOARD == LSM6DS3_BOARD
    uint8_t buffer[6];
    // Start at register 0x22 (OUTX_L_G) and read 6 sequential bytes
    myIMU.readRegisterRegion(buffer, 0x22, 6);
    
    // Combine the Low and High bytes using bitwise math (Little Endian format)
    // Gyro data comes first in the memory map
    *x = ((int16_t)buffer[0] | int16_t(buffer[1] << 8));
    *y = (int16_t)buffer[2] | int16_t(buffer[3] << 8);
    *z = (int16_t)buffer[4] | int16_t(buffer[5] << 8);
  #endif

}

void getMotion(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
  #if IMU_BOARD == MPU6050_BOARD
    mpu.getMotion6(ax, ay, az,gx, gy, gz);
  #elif IMU_BOARD == LSM6DS3_BOARD
    uint8_t buffer[12];
    // Start at register 0x22 (OUTX_L_G) and read 12 sequential bytes
    myIMU.readRegisterRegion(buffer, 0x22, 12);
    
    // Combine the Low and High bytes using bitwise math (Little Endian format)
    // Gyro data comes first in the memory map
    *gx = ((int16_t)buffer[0] | int16_t(buffer[1] << 8)); 
    *gy = ((int16_t)buffer[2] | int16_t(buffer[3] << 8));
    *gz = ((int16_t)buffer[4] | int16_t(buffer[5] << 8)); 
    
    // Accelerometer data immediately follows
    *ax = ((int16_t)buffer[6] | int16_t(buffer[7] << 8));
    *ay = ((int16_t)buffer[8] | int16_t(buffer[9] << 8));
    *az = ((int16_t)buffer[10] | int16_t(buffer[11] << 8));
  #endif
}

void readBatteryVoltage(){
  // Read analog value and smooth
  batteryADCRaw = analogRead(pinBattery);
  batteryADC = (batteryADC * 0.92) + ((float)batteryADCRaw * 0.08);
  
  // Convert it analog PIN volts
  batteryVinRaw = (batteryADC / 4095.0) * REFERENCE_VOLTAGE;

  // Calculate the battery volts
  batteryVin = batteryVinRaw * ( BATTERY_R1 + BATTERY_R2 ) / BATTERY_R2;

}
