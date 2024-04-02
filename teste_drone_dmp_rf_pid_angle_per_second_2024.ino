// Black -> rear
// Red -> front
// Receiver -> left
// screen -L /dev/ttyUSB0 38400;  mv screenlog.0 Documents/drone-datalog.csv

#include <PinChangeInterrupt.h>
#include <Servo.h>
#include <ServoInput.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


#define MPU6050_ADDRESS 0x68
#define INTERRUPT_PIN 2 //MPU6050 Interrupt pin

#define pinESC1 5
#define pinESC2 6
#define pinESC3 9
#define pinESC4 10
#define pinButton 3
#define pinPot A2

#define testSpeed 0

// Estabilization
//#define SPEED_CORRECTION_MULTIPLIER 1.5 //First working flight was with 1.5
//#define SPEED_CORRECTION_MULTIPLIER 0.4 //0.4 //First working flight was with 1.5

#define PID_P_GAIN_PITCH 0.8//1 //1.8//0.8
#define PID_I_GAIN_PITCH 0.04//0.03//0.02//0.004 //0.002
#define PID_D_GAIN_PITCH 12 //15//12.6 //6//8 //5

#define PID_P_GAIN_ROLL 0.8//1 //1.8//0.8
#define PID_I_GAIN_ROLL 0.04//0.03//0.02 //0.004 // 0.002 was ok , por regra de 3 deveria se 0.012 <- testar
#define PID_D_GAIN_ROLL 12 //15//12.6//6//8 //5

#define PID_P_GAIN_YAW 3 //4 // 2//0.8//0.4 //4
#define PID_I_GAIN_YAW 0.02//0.002//0.02
#define PID_D_GAIN_YAW 0


// #define SPEED_CORRECTION_MULTIPLIER 1.5 //First working flight was with 1.5

// #define PID_I_GAIN_PITCH 0.04
// #define PID_D_GAIN_PITCH 24
// #define PID_I_GAIN_ROLL 0.04
// #define PID_D_GAIN_ROLL 24


// #define INCREASE 43
// #define DECREASE 45
#define INCREASE 119
#define DECREASE 115
#define SHUTDOWN 48
#define ON_OFF 32

MPU6050 mpu;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int engineSpeed = testSpeed;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;

// Var text messages
// char result[50];

// Angle variables
float angle_pitch, angle_roll, angle_yaw;
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t gyro[3];        // [yaw, pitch, roll]
int16_t gyro_filtered[3];  //
float gyro_offset[3];
VectorFloat gravity;    // [x, y, z]            gravity vector

// Timers
unsigned long debug_delay,loop_timer;

// Status
int buttonState ;
bool setupDone=false, engineOn=false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// Control
int incomingByte = 0;
int pitchAngle = 0, rollAngle=0;
ServoInputPin<3> rf_throttle;
ServoInputPin<A1> rf_pitch;
// ServoInputPin<12> rf_roll;
ServoInputPin<A2> rf_roll;
// PID
float pid_p_roll, pid_i_roll, pid_d_roll, pid_roll_output, roll_error, roll_error_previous;
float pid_p_yaw, pid_i_yaw, pid_d_yaw, pid_yaw_output, yaw_error, yaw_error_previous;
float pid_p_pitch, pid_i_pitch, pid_d_pitch, pid_pitch_output, pitch_error, pitch_error_previous, pitch_gyro_desired, roll_gyro_desired, yaw_gyro_desired;
float pid_max = 200; //400;

String buf;

File fdr_file;


void setup() {

  // The first thing to do is attach to the esc's
  esc1.attach(pinESC1,1000,2000);
  esc2.attach(pinESC2,1000,2000);
  esc3.attach(pinESC3,1000,2000);
  esc4.attach(pinESC4,1000,2000);

  // And then set it the esc to 0 to arm the engines
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  esc4.write(0);

  // Start serial communication, we use it to communicate to the HC-05 bluetooth module
  Serial.begin(115200);
  //Serial.begin(2400);

  // REENABLE Serial.println("ESC attached");

  // Start I2C communication
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true); 

  // Setup MPU6050
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // REENBLE Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(41); //3
  mpu.setYGyroOffset(39); //20
  mpu.setZGyroOffset(-8); //-15
  mpu.setXAccelOffset(-4023);
  mpu.setYAccelOffset(-1014);
  mpu.setZAccelOffset(977); // 730 //1688 factory default for my test chip
  

  // Calibrate MPU6050
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      // REENABLE mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      // REENABLE Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      // REENABLE Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      // REENABLE Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      // REENABLE Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      // REENABLE Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }  
  
  //Get gyro offset
  int samples_count=2000;
  int16_t samples[3] = {0,0,0};
  for(int i=0; i < samples_count; i++){
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) && dmpReady) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
          
        mpu.dmpGetGyro(gyro, fifoBuffer);
        
        // gyro[0]*=-1; // AQUI TESTANDO INVERTER
        samples[0] += gyro[0];
        samples[1] += gyro[1];
        samples[2] += gyro[2];
        delay(3);
    }    
  }

  gyro_offset[0] = samples[0] / samples_count;
  gyro_offset[1] = samples[1] / samples_count;
  gyro_offset[2] = samples[2] / samples_count;

  setupDone=true;
  // REENABLE Serial.print("Starting\n");
  // Serial.print("Pitch Gyro, Roll Gyro, Yaw Gyro, Pitch Angle, Roll Angle, Yaw angle, Desired Speed, Pitch Error, Roll Error, Yaw Error, Speed Rear Left, Speed Rear Right,	Speed Front Left, Speed Front Right, i_pitch, i_roll, i_yaw");
  Serial.print("\n");

  //Setup SD card
  if(!SD.begin(4)){
    // Serial.println("SD initialization failed!");
    while(1);
  }else{}
  // Serial.println("SD initialization done.");

  //Open flight record file
  fdr_file = SD.open("fdr.csv",FILE_WRITE);
  
}

void loop() {
  
  if (!dmpReady) return;  // If the MPU6050 DMP is not ready do not turn the engine on
  buf="";
  
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);      
      // ypr[1]*=-1; // AQUI TESTANDO INVERTER
      // ypr[2]*=-1; // AQUI TESTANDO INVERTER
      
      
      mpu.dmpGetGyro(gyro, fifoBuffer);
      gyro[1]*=-1; // AQUI TESTANDO INVERTER
      // gyro[0]*=-1; // AQUI TESTANDO INVERTER

      if( ! (abs(gyro[0]) > 180 || abs(gyro[1]) > 180 || abs(gyro[2]) > 180)  ){

      

        gyro[0]-=gyro_offset[0];
        gyro[1]-=gyro_offset[1];
        gyro[2]-=gyro_offset[2];
        
        gyro_filtered[0]= (int16_t) ( (0.8 * gyro_filtered[0])  + (0.2 * gyro[0]) );
        gyro_filtered[1]= (int16_t) ( (0.8 * gyro_filtered[1])  + (0.2 * gyro[1]) );
        gyro_filtered[2]= (int16_t) ( (0.8 * gyro_filtered[2])  + (0.2 * gyro[2]) );

      }
      // Serial.print("ypr\t");
      // Serial.print(ypr[0] * 180/M_PI);
      // Serial.print("\t");

      // Serial.println(ypr[2] * 180/M_PI);          
  }
  // Serial.print(ypr[1] * 180/M_PI);
  // Serial.print(",");

  // buf += F("GyroPitch:");
  buf += String(gyro_filtered[1] );
  buf += F(",");

  // buf += F("GyroRoll:");
  buf += String(gyro_filtered[0] );
  buf += F(",");

  // buf += F("GyroYaw:");
  buf += String(gyro_filtered[2] );
  buf += F(",");

  // buf += F("Pitch:");
  buf += String(ypr[1] * 180/M_PI,3);
  buf += F(",");

  // Serial.print(ypr[2] * 180/M_PI);
  // Serial.print(",");
  // buf += F("Roll:");
  buf += String(ypr[2] * 180/M_PI,3);
  buf += F(",");
  
  // Serial.print(ypr[0] * 180/M_PI);
  // Serial.print(",");
  // buf += F("Yaw:");
  buf += String(ypr[0] * 180/M_PI,3);
  buf += F(",");
  
    //int potValue = analogRead(pinPot);
    //engineSpeed = map(potValue,0,1023,0,50);    
    //engineSpeed = 25;

  // engineSpeed = map(rf_throttle.getAngle(),2,178,0,180);
  engineSpeed = (0.8 * engineSpeed) +  (0.2 * map(rf_throttle.getAngle(),2,178,0,180));
  // pitchAngle = map(rf_pitch.getAngle(),90,180,0,20);    
  pitchAngle = (0.8 * pitchAngle) + (-0.2 * map(rf_pitch.getAngle(),90,180,0,90)) ;
  // pitchAngle*=-1;      
  rollAngle = (0.8 * rollAngle) + (0.2 * map(rf_roll.getAngle(),90,180,0,90));

  
  // boolean testservo;
  // testservo = rf_throttle.available();
  // buf += F("######");
  // buf += String(testservo);
  // buf += F("######");          
  if(engineSpeed<3){
    disableEngines();
  }else{
    enableEngines();
  }
   
  if(engineSpeed<0){
    engineSpeed=0;
  }
    
  // sprintf(result, "%d", engineSpeed);
  // //Serial.print("Engine Speed:");
  // Serial.print(result);
  // Serial.print(",");
  // buf += F("EngineSpeed:");
  buf += String(engineSpeed);
  buf += F(",");
         
  // sprintf(result, "%d", pitchAngle);
  // //Serial.print("Desired pitchAngle:");
  // Serial.print(result);
  // Serial.print(",");
  // buf += F("JoystickPitch:");
  buf += String(pitchAngle);
  buf += F(",");

  buf += String(rollAngle);
  buf += F(",");


  setAllEnginesSpeed(engineSpeed);
    
  
  
  while(micros() - loop_timer < 4000 );                                      //We wait until 4000us are passed.
  loop_timer = micros();
  // fdr_file.println(buf);
  Serial.print(buf);
  Serial.print("\n");
}

void disableEngines(){
  esc1.detach();
  esc2.detach();
  esc3.detach();
  esc4.detach();  
}

void enableEngines(){
  if(!esc1.attached()){
    esc1.attach(pinESC1,1000,2000);
  }

  if(!esc2.attached()){
    esc2.attach(pinESC2,1000,2000);
  }

  if(!esc3.attached()){
    esc3.attach(pinESC3,1000,2000);
  }

  if(!esc4.attached()){
    esc4.attach(pinESC4,1000,2000);
  }  
}


void setAllEnginesSpeed(int speed){

  int leftRear,rightRear,leftFront,rightFront; 
  //Serial.print("\tPITCH: :\t");print(ypr[1]);

  if(speed > 90){
    speed = 90;
  }

  

  // pitch_gyro_desired = ( pitchAngle - (int)(ypr[1] * 180/M_PI)  ) * 5;
  
  // pitch_gyro_desired = ( pitchAngle - ((int)(ypr[1] * 180/M_PI)*3) );
  pitch_gyro_desired = ( pitchAngle - ((ypr[1] * 180/M_PI)*3) );
  
  // pitch_gyro_desired = ( 0 - (int)(ypr[1] * 180/M_PI)  ) * 5;
  // roll_gyro_desired =  ( pitchAngle - (int)(ypr[2] * 180/M_PI)  ) * 5;
  
  // roll_gyro_desired =  ( 0 - ((int)(ypr[2] * 180/M_PI)*3) );
  roll_gyro_desired =  ( rollAngle - ((ypr[2] * 180/M_PI)*3) );
  
  // roll_gyro_desired =  ( 0 - (int)(ypr[2] * 180/M_PI)  ) * 5;
  
  // min 5 degres dead band
  // if((int)(ypr[1] * 180/M_PI) > -5 && (int)(ypr[1] * 180/M_PI) <5 ){
  //   pitch_gyro_desired = 0;
  // }

  // yaw_gyro_desired = ( 0 - (int)(ypr[0] * 180/M_PI) ) * 1;
  yaw_gyro_desired = 0;

  pitch_error = pitch_gyro_desired - gyro_filtered[1];
  roll_error  = roll_gyro_desired - gyro_filtered[0];
  yaw_error = yaw_gyro_desired - gyro_filtered[2];
  
  //  Try a deadband
  // if( pitch_error >= -1 && pitch_error <= 1  ){
  //     pitch_error=0;
  // }
  // if( roll_error >= -1 && roll_error <= 1  ){
  //     roll_error=0;
  // }
  if( yaw_error >= -1 && yaw_error <= 1  ){
      yaw_error=0;
  }
  
  // sprintf(result, "%d", pitch_error);
  // Serial.print(result);
  // Serial.print(",");
  //buf += String(pitch_error,3);
  // buf += F("PitchError:");
  buf += String(pitch_error);
  buf += F(",");

  // sprintf(result, "%d", roll_error);
  // Serial.print(result);
  // Serial.print(",");
  // buf += F("RollError:");
  buf += String(roll_error,3);
  buf += F(",");    

  // sprintf(result, "%d", yaw_error);
  // Serial.print(result);
  // Serial.print(",");
  // buf += F("YawError:");
  buf += String(yaw_error,3);
  buf += F(",");
          
  if(speed >= 15){
    //pitch_error = abs(pitch_error);
    // Calculate  pid_pitch_output 
    pid_p_pitch  = pitch_error * PID_P_GAIN_PITCH;
    pid_i_pitch  += pitch_error * PID_I_GAIN_PITCH;
    if (pid_i_pitch > pid_max) {
        pid_i_pitch = pid_max;      
    }else if(pid_i_pitch < pid_max * -1){
        pid_i_pitch = pid_max * -1;
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
    pid_i_roll += roll_error * PID_I_GAIN_ROLL;
    if (pid_i_roll > pid_max) {
        pid_i_roll = pid_max;      
    }else if(pid_i_roll < pid_max * -1){
        pid_i_roll = pid_max * -1;
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
    pid_i_yaw += yaw_error * PID_I_GAIN_YAW; 
    if (pid_i_yaw > pid_max) {
        pid_i_yaw = pid_max;      
    }else if(pid_i_yaw < pid_max * -1){
        pid_i_yaw = pid_max * -1;
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




  // leftRear = speed - pid_pitch_output + pid_roll_output - pid_yaw_output;
  // rightRear = speed - pid_pitch_output - pid_roll_output + pid_yaw_output;
  // leftFront= speed + pid_pitch_output + pid_roll_output + pid_yaw_output;
  // rightFront = speed + pid_pitch_output - pid_roll_output - pid_yaw_output;
  
  leftRear = speed +   ((-pid_pitch_output + pid_roll_output - pid_yaw_output)*0.18);
  rightRear = speed +  ((-pid_pitch_output - pid_roll_output + pid_yaw_output)*0.18);
  leftFront= speed +   ((pid_pitch_output + pid_roll_output + pid_yaw_output)*0.18);
  rightFront = speed + ((pid_pitch_output - pid_roll_output - pid_yaw_output)*0.18);


  if(leftRear > 110){
    leftRear = 110;
  }
  if(rightRear > 110){
    rightRear = 110;
  }
  if(leftFront > 110){
    leftFront = 110;
  }
  if(rightFront > 110){
    rightFront = 110;
  }

  if(leftRear < 10){
    leftRear = 10;
  }
  if(rightRear < 10){
    rightRear = 10;
  }
  if(leftFront < 10){
    leftFront = 10;
  }
  if(rightFront < 10){
    rightFront = 10;
  }

  
  if(speed < 15){
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
  // esc1.write(0);//Left
  esc1.write(leftRear);//Left
  //Serial.print("Left Right:");
  // Serial.print(leftRear);
  // Serial.print(",");
  // buf += F("LeftRear:");
  buf += String(leftRear);
  buf += F(",");
  
  
  // esc2.write(0);//Right
  esc2.write(rightRear);//Right
  //Serial.print("\tRear Right:");
  // Serial.print(rightRear);
  // Serial.print(",");
  // buf += F("RightRear:");
  buf += String(rightRear);
  buf += F(",");
  
  //Front engines
  
  // esc3.write(0);//Left
  esc3.write(leftFront);//Left
  //Serial.print("\tFront Left:");
  // Serial.print(leftFront);
  // Serial.print(",");
  // buf += F("LeftFront:");
  buf += String(leftFront);
  buf += F(",");
  
  // esc4.write(0);//Right
  esc4.write(rightFront);//Right
  //Serial.print("\tFront Right:");
  // Serial.print(rightFront);
  //Serial.print(",");
  // buf += F("RightFront:");
  buf += String(rightFront);
  buf += F(",");  

  // buf += F("pid_i_pitch:");
  buf += String(pid_i_pitch);
  buf += F(",");

  // buf += F("pid_i_roll:");
  buf += String(pid_i_roll);
  buf += F(",");  
  
  // buf += F("pid_i_yaw:");
  buf += String(pid_i_yaw);
  // buf += F(",");  
}


void dmpDataReady() {
    mpuInterrupt = true;
}

