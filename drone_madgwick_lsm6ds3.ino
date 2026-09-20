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
#include <DFRobot_BMP3XX.h>

bool readBarometer(bool force = false);
/*
Todo list:
- Tune PID parameters
- Detect RC signal lost (DONE)
- Add battery voltage reading and compensation (DONE)
- Add yaw control (DONE)
- ON/OFF switch from RC (DONE)
- Check if there is realiable (async) way to write to the flight data to sdcard
- Check if doable to send telemetry over wifi
- Add barometer to control altitude
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
#define PRINT_PARAMETERS true




#define GYRO_RATE 2000

#if GYRO_RATE == 500
  #define GYRO_DIVISOR 65.5
#elif GYRO_RATE == 2000
  #define GYRO_DIVISOR 16.4
#endif


#define MPU6050_ADDRESS 0x68
#define BARO_INT_PIN 2 //MPU6050 Interrupt pin

#define pinESC1 5
#define pinESC2 6
#define pinESC3 9
#define pinESC4 10
#define pinButton 3
#define pinBattery A2
#define pinBatteryLED A3

#define testSpeed 0

#define PID_ANGLE_AMP 3
#define PID_MIN_SPEED_THRESHOLD 1100
#define PID_TAKEOFF_THRESHOLD 1100
#define MAX_RATE_SETPOINT_DPS 164

#define SPEED_LIMIT_RAW 1800
#define SPEED_MAX_OUTPUT 1900
#define SPEED_MIN_OUTPUT 1010

#define LEVEL_FILTER_ALPHA 0.05
#define LEVEL_GAIN_PITCH 3.0
#define LEVEL_GAIN_ROLL  3.0

#define PID_P_GAIN_PITCH 1.1 //1.3
#define PID_I_GAIN_PITCH 0.04 //0.004
#define PID_D_GAIN_PITCH 10//16

#define PID_P_GAIN_ROLL 1.1 //0.8
#define PID_I_GAIN_ROLL 0.04 //0.004 // 0.002 was ok , por regra de 3 deveria se 0.012 <- testar
#define PID_D_GAIN_ROLL 10//15 8 12

#define PID_P_GAIN_YAW 4 // 1
#define PID_I_GAIN_YAW 0.01 //0.02//0.002//0.02
#define PID_D_GAIN_YAW 0

#define PID_P_GAIN_ALT 50.0  
#define PID_I_GAIN_ALT 0.2   
#define PID_D_GAIN_ALT 0.1


#define PID_I_MAX 400 //150
#define PID_I_YAW_MAX 150 //150
#define PID_YAW_MAX 250 //150

#define REFERENCE_VOLTAGE  3.3    // Nano 33 IoT logic level
#define BATTERY_R1  10000.0
#define BATTERY_R2  2000.0

#define RC_MISSING_THRESHOLD 50

#define STATE_OFF 0
#define STATE_STARTING 1
#define STATE_ON 2

#define SEALEVELPRESSURE_HPA 1013.25f
// #define SEALEVELPRESSURE_PA 101325.0f

DFRobot_BMP388_I2C bmp(&Wire, bmp.eSDOGND);

Madgwick filter;

SAMD_PWM* esc1;
SAMD_PWM* esc2;
SAMD_PWM* esc3;
SAMD_PWM* esc4;

const int SDA_PIN = A4; 
const int SCL_PIN = A5;

int rc_missing_count = 0;
int stick_arm_count = 0;
int toogleSwitchRaw = 1000;
int engineSpeed = 1000;
int engineSpeed_raw = 1000;
int previous_engineSpeed_raw = 1000;
int master_state = STATE_OFF;
bool alt_hold_mode = true, previous_alt_hold_mode = true;
bool gyroCalibrationDone = false;

bool toogle_on = false;
bool toogle_altitude_hold = false;

float current_altitude = 0, previous_altitude = 0, filtered_altitude = 0, filtered_a_up_g = 0, raw_pressure =0, home_altitude=0;
float current_vertical_speed = 0, target_vertical_speed = 0;
float altitude_setpoint = 0, altitude_error=0;
unsigned long last_baro_read = 0;
volatile bool baroDataReady;
bool baroDataReadDone = false;

// Complementary filter state: fuses integrated accel with the noisy barometer
float vvel_estimate = 0;   // fused vertical velocity, m/s (+ = up), replaces raw baro derivative
float alt_estimate  = 0;   // fused altitude, m
#define ALT_FUSION_KP 0.3f  // 0-1, how hard each new baro sample pulls the altitude estimate
#define ALT_FUSION_KV 0.005f  // how hard each new baro sample pulls the velocity estimate - tune on the bench
// #define ALT_FUSION_KV 0.03f 
float alt_vel_error = 0, alt_vel_error_previous = 0;
float pid_p_alt = 0, pid_i_alt = 1100, pid_d_alt = 0;

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
  bool rcLost;
  int master_state;
  float altitude;
  float ground_altitude;
  float vertical_speed;
  float raw_pressure;
  int toogleSwitch;
  float rc_vspeed_raw;
  float a_net;
  float alt_estimate;
  float altitude_setpoint;
  float rc_vspeed_filtered;
} flightLog;

// Timers
unsigned long loop_timer;
unsigned long log_timer;
volatile unsigned long baro_isr_micros;
unsigned long lastGoodPacket = 0;

// Status
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// Control
float pitchAngle = 0, rollAngle=0, yawAngle=0, altitudeStick=0;
float pitch_level_adjust = 0, roll_level_adjust = 0, yaw_level_adjust = 0;
long pitchAngle_raw = 0, rollAngle_raw = 0, yawAngle_raw = 0 ;
float rc_vspeed_raw=0, previous_rc_vspeed_raw=0,rc_vspeed_filtered=0;
long previous_pitchAngle_raw = 0, previous_rollAngle_raw = 0, previous_yawAngle_raw = 0 ;
ServoInputPin<3> rf_throttle;
ServoInputPin<A1> rf_pitch;
ServoInputPin<A7> rf_roll;
ServoInputPin<11> rf_yaw;
ServoInputPin<13> rf_switch;


// PID
float pid_p_roll, pid_i_roll, pid_d_roll, pid_roll_output, roll_error, roll_error_previous;
float pid_p_yaw, pid_i_yaw, pid_d_yaw, pid_yaw_output, yaw_error, yaw_error_previous;
float pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_pitch_output, pitch_error, pitch_error_previous, pitch_gyro_desired, roll_gyro_desired, yaw_gyro_desired;
float pid_max = 400; //400;

String buf;

File fdr_file;


void setup() {

  // The first thing to do is attach to the esc's
  esc1 = new SAMD_PWM(pinESC1, 250.0f, 25.0f);
  esc2 = new SAMD_PWM(pinESC2, 250.0f, 25.0f);
  esc3 = new SAMD_PWM(pinESC3, 250.0f, 25.0f);
  esc4 = new SAMD_PWM(pinESC4, 250.0f, 25.0f);

  // And then set it the esc to 1000 to arm the engines
  esc1->setPWM(pinESC1,250.0f,1000/40.0f);
  esc2->setPWM(pinESC2,250.0f,1000/40.0f);
  esc3->setPWM(pinESC3,250.0f,1000/40.0f);
  esc4->setPWM(pinESC4,250.0f,1000/40.0f);

  // Attach the servo inputs
  rf_throttle.attach();
  rf_pitch.attach();
  rf_roll.attach();
  rf_yaw.attach();
  rf_switch.attach();

  // Start serial communication
  Serial.begin(230400);//115200
  // delay(8000);
  Serial.println("Starting setup");

  // Battery stuff
  analogReadResolution(12);
  batteryADC = (float)analogRead(pinBattery);
  pinMode(pinBatteryLED, OUTPUT);
  

  // Start I2C communication
  Wire.begin();
  Wire.setClock(400000);
  // Wire.setClock(1000);
  // Wire.setWireTimeout(3000, true); //Only available on AVR based arduino

  // Setup IMU Board
  //delay(5);
  Serial.println("Starting setup");
  setupBoard();
  setupBarometerBoard();
  delay(5);

  pinMode(BARO_INT_PIN, INPUT);

  // enable Arduino interrupt detection
  Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
  Serial.print(digitalPinToInterrupt(BARO_INT_PIN));
  Serial.println(F(")..."));
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  attachInterrupt(
        digitalPinToInterrupt(BARO_INT_PIN),
        baroISR,
        RISING
    );
  
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

}

void loop() {
  buf="";
  

  int16_t gx_raw, gy_raw, gz_raw;
  int16_t ax_raw, ay_raw, az_raw;

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

  if( ! (abs(gx_dps) > 400 || abs(gy_dps) > 400 || abs(gz_dps) > 400)  ){
    gyro_filtered[0]= ( (0.7 * gyro_filtered[0])  + (0.3 * gx_dps) );
    gyro_filtered[1]= ( (0.7 * gyro_filtered[1])  + (0.3 * gy_dps) );
    gyro_filtered[2]= ( (0.7 * gyro_filtered[2])  + (0.3 * gz_dps) );

  }
  lastGoodPacket = millis();

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
  #endif

  if(!isRCSignalLost()){

    toogleSwitchRaw = rf_switch.getPulse();
    flightLog.toogleSwitch = toogleSwitchRaw;
    readToogles();
    alt_hold_mode=toogle_altitude_hold;
    
    if(alt_hold_mode && !previous_alt_hold_mode){
      pid_i_alt = constrain(rf_throttle.getPulse(), 1300, 1700);
      last_baro_read = baro_isr_micros;
      vvel_estimate = 0;
      readBarometer(true);
      readBarometer(true);
      rc_vspeed_raw = 0;
      previous_rc_vspeed_raw = 0;
      rc_vspeed_filtered=0;
      altitude_setpoint = alt_estimate;
      
    }
    if(!alt_hold_mode && previous_alt_hold_mode){
      engineSpeed = pid_i_alt;
      previous_engineSpeed_raw = engineSpeed;
      altitude_setpoint = alt_estimate;
    }

    previous_alt_hold_mode=alt_hold_mode;
    if(alt_hold_mode){
      if(baroDataReadDone){
        baroDataReadDone=false;
        flightLog.altitude = filtered_altitude;
        flightLog.alt_estimate = alt_estimate;
        flightLog.vertical_speed = current_vertical_speed;
        if(alt_hold_mode && master_state == STATE_ON){
          rc_vspeed_raw = ((float)rf_throttle.mapDeadzone(-2000,2000, 0.2))/ 1000.0;
          if(abs(rc_vspeed_raw - previous_rc_vspeed_raw) > 1){
            rc_vspeed_raw=previous_rc_vspeed_raw;
          }
          rc_vspeed_filtered = (rc_vspeed_filtered*0.8) + (rc_vspeed_raw*0.2);
          flightLog.rc_vspeed_raw = rc_vspeed_raw;
          flightLog.rc_vspeed_filtered = rc_vspeed_filtered;
          
          if(rc_vspeed_raw == 0 && previous_rc_vspeed_raw !=0 ){
            altitude_setpoint = alt_estimate;
          }
          if(rc_vspeed_raw == 0){
            // altitude_error = alt_estimate -  altitude_setpoint;
            // alt_vel_error = current_vertical_speed - (altitude_error * 2);
            altitude_error = altitude_setpoint - alt_estimate;
            alt_vel_error = altitude_error;
            // alt_vel_error  = (altitude_error * 2) - current_vertical_speed;
            //Add a dead band here
          }else{      
            alt_vel_error = rc_vspeed_filtered - current_vertical_speed;
          }
          flightLog.altitude_setpoint = altitude_setpoint;

          // if(abs(alt_vel_error) < 0.3){
          //   alt_vel_error=0;
          // }
          
          pid_p_alt = alt_vel_error * PID_P_GAIN_ALT;
          pid_i_alt += alt_vel_error * PID_I_GAIN_ALT;
          pid_i_alt = constrain(pid_i_alt, 1400, 1700);
          pid_d_alt = (alt_vel_error - alt_vel_error_previous) * PID_D_GAIN_ALT;
          alt_vel_error_previous = alt_vel_error;

          engineSpeed = pid_i_alt + pid_p_alt + pid_d_alt;
          engineSpeed = constrain(engineSpeed, 1000, 1700);

          // if(filtered_altitude - home_altitude > 2){
          //   engineSpeed-=50;
          // }

          // altitudeStick = 
          previous_rc_vspeed_raw = rc_vspeed_raw;
        }
      }
      
    }else{
      engineSpeed_raw = rf_throttle.getPulse();
      if(abs(engineSpeed_raw - previous_engineSpeed_raw) > 500){
        engineSpeed_raw= previous_engineSpeed_raw;
      }
      engineSpeed = (0.8 * engineSpeed) +  (0.2 * engineSpeed_raw);
    }
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
    flightLog.rcLost=false;
  }else{
    flightLog.rcLost=true;


    pitchAngle = 0;
    rollAngle = 0;
    yawAngle = 0;

    pitchAngle_raw=0;
    rollAngle_raw=0;
    yawAngle_raw=0;
    rc_vspeed_raw=0;
  
    previous_pitchAngle_raw = 0;
    previous_rollAngle_raw = 0;
    previous_yawAngle_raw = 0;

    pid_i_alt = 1000;
    // last_baro_read =0; 

    // If lost RC in-flight start emergency land
    if(engineSpeed >= 1039) {
      if(rc_missing_count % 2 == 0){
        engineSpeed-=2;
        engineSpeed_raw=engineSpeed;
        previous_engineSpeed_raw=engineSpeed_raw;
      }
    }
  }

  readBatteryVoltage();

  //Master Switch control
  if(master_state==STATE_OFF && engineSpeed<1040 && (MAX_RATE_SETPOINT_DPS - yawAngle ) <= 20 ){
    stick_arm_count++;
    if(stick_arm_count>100){
      master_state=STATE_STARTING;
      stick_arm_count=0;
    }
  }else if(master_state==STATE_STARTING ){
    if(!gyroCalibrationDone)
      calibrateOffset();

    readBarometer(true);
    home_altitude = current_altitude;
    pid_i_pitch = 0;
    pid_i_roll = 0;
    pid_i_yaw = 0;
    pid_i_alt = 1000;
    alt_vel_error = 0;
    alt_vel_error_previous = 0;
    pitchAngle = 0; rollAngle = 0; yawAngle = 0;
    pitchAngle_raw = 0; rollAngle_raw = 0; yawAngle_raw = 0;
    previous_pitchAngle_raw = 0; previous_rollAngle_raw = 0; previous_yawAngle_raw = 0; 
    rc_missing_count = 0;
    
    master_state=STATE_ON;
  }else if(master_state==STATE_ON && engineSpeed<1040 && (MAX_RATE_SETPOINT_DPS +yawAngle ) <= 20 ){
    stick_arm_count++;
    if(stick_arm_count>100){
      pid_i_alt = 1000;
      master_state=STATE_OFF;
      stick_arm_count=0;
    }
  }else{
    stick_arm_count = 0;
  }


  if( engineSpeed<1040 || master_state==STATE_OFF || master_state==STATE_STARTING ){
    disableEngines();
  }else{
    setAllEnginesSpeed(engineSpeed);
  }

  if(alt_hold_mode){
    readBarometer();
  }

  #ifdef PRINT_PARAMETERS
    flightLog.rc_engine_speed = engineSpeed;
    flightLog.rc_pitch = pitchAngle;
    flightLog.rc_roll = rollAngle;
    flightLog.rc_yaw = yawAngle;
    flightLog.batteryVin = batteryVin;
    flightLog.master_state = master_state;
    flightLog.ground_altitude = filtered_altitude - home_altitude;
  #endif

  #ifdef PRINT_PARAMETERS
    // Serial.print(buf);
    // if(micros() - log_timer > 100000 ){
      if(true ){
      char buffer[250];
      // sprintf(buffer,"%d,%f,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%d,%f,%f,%f,%d,%f,%", flightLog.master_state,
      //                                           flightLog.gyro_pitch,
      //                                           flightLog.gyro_roll,
      //                                           flightLog.gyro_yaw,
      //                                           flightLog.angle_pitch,
      //                                           flightLog.angle_roll,
      //                                           flightLog.angle_yaw,
      //                                           flightLog.rc_engine_speed,
      //                                           flightLog.rc_pitch,
      //                                           flightLog.rc_roll,
      //                                           flightLog.rc_yaw,
      //                                           flightLog.batteryVin,
      //                                           flightLog.rcLost,
      //                                           flightLog.altitude,
      //                                           flightLog.vertical_speed,
      //                                           flightLog.ground_altitude,
      //                                           flightLog.toogleSwitch,
      //                                           flightLog.rc_vspeed_raw,
      //                                           flightLog.a_net);
      sprintf(buffer,"%f,%f,%f,%f,%f,%f,%d,%f,%f", 
                                                flightLog.altitude,
                                                flightLog.alt_estimate,
                                                flightLog.vertical_speed,
                                                flightLog.raw_pressure,
                                                flightLog.a_net,
                                                flightLog.altitude_setpoint,
                                                flightLog.rc_engine_speed,
                                                flightLog.rc_vspeed_raw,
                                                flightLog.rc_vspeed_filtered);
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
  
  // comment out writing to file, as it could impact on the loop frquency
  // fdr_file.println(buf);

}

void disableEngines(){
  esc1->setPWM(pinESC1,250.0f,1000/40.0f);
  esc2->setPWM(pinESC2,250.0f,1000/40.0f);
  esc3->setPWM(pinESC3,250.0f,1000/40.0f);
  esc4->setPWM(pinESC4,250.0f,1000/40.0f);
}

void enableEngines(){

}


void setAllEnginesSpeed(int speed){

  int leftRear,rightRear,leftFront,rightFront; 

  if(speed > SPEED_LIMIT_RAW){
    speed = SPEED_LIMIT_RAW;
  }
  
  pitch_gyro_desired = constrain(( pitchAngle - ((ypr[1] * 5.0 ))),-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);
  roll_gyro_desired =  constrain(( rollAngle - ((ypr[2] * 5.0 ))),-MAX_RATE_SETPOINT_DPS, MAX_RATE_SETPOINT_DPS);

  yaw_gyro_desired = yawAngle;

  pitch_error = pitch_gyro_desired - gyro_filtered[1];
  roll_error  = roll_gyro_desired - gyro_filtered[0];
  yaw_error = yaw_gyro_desired - gyro_filtered[2];

  #ifdef PRINT_PARAMETERS
    flightLog.error_pitch = pitch_error;
    flightLog.error_roll = roll_error;
    flightLog.error_yaw = yaw_error;
  #endif

  if(speed >= PID_MIN_SPEED_THRESHOLD){

    int throttle_headroom = speed - PID_MIN_SPEED_THRESHOLD;

    if (throttle_headroom < 0) {
        throttle_headroom = 0;
    }

    if (throttle_headroom < 400 ){
      pid_max = throttle_headroom;
    }else
    {
      pid_max = 400;
    }
    
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
    if (pid_yaw_output > pid_max) {
        pid_yaw_output = pid_max;      
    }else if(pid_yaw_output < pid_max * -1){
        pid_yaw_output = pid_max * -1;    
    }
    yaw_error_previous = yaw_error;
  }

  leftRear = speed +   ((-pid_pitch_output + pid_roll_output - pid_yaw_output));
  rightRear = speed +  ((-pid_pitch_output - pid_roll_output + pid_yaw_output));
  leftFront= speed +   ((pid_pitch_output + pid_roll_output + pid_yaw_output));
  rightFront = speed + ((pid_pitch_output - pid_roll_output - pid_yaw_output));

  // Battery drop compensation
  if (batteryVin < 12.4f && batteryVin > 8.0f) {
    float compensation = (12.4f - batteryVin) / 35.0f;
    leftRear   += leftRear * compensation;
    rightRear  += rightRear * compensation;
    leftFront  += leftFront * compensation;
    rightFront += rightFront * compensation;
  }

  if(batteryVin < 10.5f ){
    digitalWrite(pinBatteryLED, HIGH);
  }else{
    digitalWrite(pinBatteryLED, LOW);
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
  esc1->setPWM(pinESC1,250.0f,leftRear/40.0f);
  esc2->setPWM(pinESC2,250.0f,rightRear/40.0f);

  //Front engines
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

void setupBarometerBoard(){
  //temp disable
  // return;
  if (bmp.begin()!=ERR_OK) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while(1);
  } else {
    Serial.println("BMP3XX Barometer initialized successfully!");
    bool ok = bmp.setSamplingMode(bmp.eNormalPrecision2);

    if(ok){
      Serial.println("BMP Samp Config OK");
    }else{
      Serial.println("BMP Samp Config Failed");
      while(1);
    }
    Serial.println("BMP Samp period");
    Serial.println(bmp.getSamplingPeriodUS());

    bmp.setINTMode(
        bmp.eINTPinPP |
        bmp.eINTPinActiveLevelHigh |
        // bmp.eINTLatchEN |
        bmp.eINTLatchDIS |
        bmp.eIntFWtmDis |
        bmp.eINTFFullDIS |
        bmp.eINTInitialLevelLOW |
        bmp.eINTDataDrdyEN
    );
    // bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    // bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    // bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
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

    // Make a SensorSettings object to remember what you wanted to set everyhting to
    SensorSettings settingsIWanted;
    
    int test = myIMU.begin(&settingsIWanted);
    if (test != 0) {
      Serial.println("Device error or not connected.");
      Serial.println(test);
      while (1); 
    }

  #endif

}

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
  batteryADC = (batteryADC * 0.999f) + ((float)batteryADCRaw * 0.001f);
  
  // Convert it analog PIN volts
  batteryVinRaw = (batteryADC / 4095.0) * REFERENCE_VOLTAGE;

  // Calculate the battery volts
  batteryVin = batteryVinRaw * ( BATTERY_R1 + BATTERY_R2 ) / BATTERY_R2;

}

void calibrateOffset(){
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

  gyroCalibrationDone=true;
}

bool isRCSignalLost(){
  if(!rf_throttle.available()){
    rc_missing_count++;
  }else{
    rc_missing_count=0;
  }
    
  if(rc_missing_count > RC_MISSING_THRESHOLD){
    return true;
  }
  return false;
}

bool readBarometer(bool force){
  //temp disable
  // return false;
  if(baroDataReady || force){
    //  Serial.println("Barometer Ready");
    baroDataReady = false;
    unsigned long current_time = baro_isr_micros;
    if (last_baro_read == 0) {
      last_baro_read = current_time;
      filtered_altitude = bmp.readAltitudeM();
      previous_altitude = filtered_altitude;
      alt_estimate = filtered_altitude;
      vvel_estimate = 0;
      return false;
    }
    float dt_baro = (current_time - last_baro_read) / 1000000.0f;
    last_baro_read = current_time;
    if (dt_baro <= 0.001f) return false;
    
    current_altitude = bmp.readAltitudeM(); // This to heavy for Arduino NANO IOT
    #ifdef PRINT_PARAMETERS
      raw_pressure = bmp.readPressPa() / 100;
      flightLog.raw_pressure= raw_pressure;
    #endif
    // float raw_pressure = 0;
    // current_altitude = (SEALEVELPRESSURE_PA - raw_pressure) / 8.3f;
    // current_altitude = 0;

    filtered_altitude =  (filtered_altitude * 0.85) + (current_altitude * 0.15);
    // filtered_altitude =  (filtered_altitude * 0.7) + (current_altitude * 0.3);

    // float baro_velocity = (filtered_altitude - previous_altitude) / dt_baro;
    previous_altitude = filtered_altitude;

        // ---- Predict: integrate tilt-compensated vertical acceleration over dt_baro ----
    // Rotates the body-frame accelerometer reading into the earth vertical axis using
    // the current Madgwick pitch/roll, so the drone's own tilt isn't mistaken for climb/descent.
    float pitch_rad = radians(ypr[1]);
    float roll_rad  = radians(ypr[2])*-1;
    float a_up_g =  ax_gf * (-sin(pitch_rad))
                  + ay_gf * ( sin(roll_rad) * cos(pitch_rad))
                  + az_gf * ( cos(roll_rad) * cos(pitch_rad));
    // NOTE: verify this sign on the bench - az_gf should read ~+1.0g sitting level and still.
    // If a_up_g reads ~-1.0g level instead, flip all three signs above.
    // Checked it is around 1
    filtered_a_up_g = (filtered_a_up_g * 0.7) + (a_up_g * 0.3);
    float a_net = (filtered_a_up_g - 1.0f) * 9.81f;  // strip gravity -> net vertical accel, m/s^2
    #ifdef PRINT_PARAMETERS
      flightLog.a_net = a_net;
      flightLog.raw_pressure= raw_pressure;
    #endif

    vvel_estimate += a_net * dt_baro;
    alt_estimate  += vvel_estimate * dt_baro;

    // ---- Correct: nudge the smooth-but-drifting accel estimate toward the noisy-but-
    // ---- unbiased barometer reading. This is a multiply by a fixed gain, NOT a divide
    // ---- by dt, which is what turned every bit of baro noise into m/s of fake velocity. ----
    float alt_error = filtered_altitude - alt_estimate;
    alt_estimate  += ALT_FUSION_KP * alt_error;
    // vvel_estimate += ALT_FUSION_KV * alt_error;
    vvel_estimate += (ALT_FUSION_KV * alt_error)  / dt_baro;

    current_vertical_speed = vvel_estimate;
    // current_vertical_speed = (current_vertical_speed * 0.85) + (baro_velocity * 0.15);
    // current_vertical_speed = baro_velocity;
    baroDataReadDone=true;
    return true;
  }
  return false;
}

void baroISR()
{
    //  Serial.println("Barometer Ready");
    baro_isr_micros = micros();
    baroDataReady = true;
}

void readToogles(){

  if (toogleSwitchRaw > 900 && toogleSwitchRaw < 1200) {
  toogle_altitude_hold = false; toogle_on  = false; // Both UP
  } 
  else if (toogleSwitchRaw >= 1200 && toogleSwitchRaw < 1500) {
    toogle_altitude_hold = true;  toogle_on  = false; // SwA DOWN, SwB UP
  } 
  else if (toogleSwitchRaw >= 1500 && toogleSwitchRaw < 1800) {
    toogle_altitude_hold = false; toogle_on  = true;  // SwA UP, SwB DOWN
  } 
  else if (toogleSwitchRaw >= 1800) {
    toogle_altitude_hold = true;  toogle_on  = true;  // Both DOWN
  }else{
    toogle_altitude_hold = false; toogle_on  = false;
  }

}