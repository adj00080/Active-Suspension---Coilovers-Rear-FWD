/*
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;               //Change to the name of any supported IMU!

//accelerometer variables
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;
//one shots
int passflag1 = 0;
int passflag2 = 0;
int passflag3 = 0;
int passflag4 = 0;
int passflag5 = 0;
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
int MOTOR_STEPS = 200;
// Microstepping mode. If you hardwired it to save pins, set to the same value here.
int MICROSTEPS = 8;
////stepper driver pins
//stepper1
const int DIR1 = 55;
const int STEP1 = 54;
const int ENABLE1 = 38;
//stepper2
const int DIR2 = 61;
const int STEP2 = 60;
const int ENABLE2 = 56;
//define variables for positioning
//////////////xxxxxxxxxxxxx////////////////////
int home=0; //
int position; 
int move_position;
int target_position = 0; 
int inverse;
//////////////yyyyyyyyyyyyy////////////////////
int homey=0; //
int positiony; 
int move_positiony ;
int target_positiony = 0; 
//addittional IO
int enable_sw_pin= 3;
bool enable_sw;
int pushbutton1_pin = 2;
bool pushbutton1 = 0;
//DEFINE MOTOR STEPS WITH MICRO STEPS ADDED
int motor_steps_total = MOTOR_STEPS*MICROSTEPS;
 
///////////////////////////////////////////////////////////////////
void setup() {

    //define buttons and switches as input //accelerometer
    Wire.begin();
    Wire.setClock(400000); //400khz clock
    Serial.begin(115200);
    int err = IMU.init(calib, IMU_ADDRESS);
    while (!Serial) {;}
    //IO
    pinMode(enable_sw_pin, INPUT);
    pinMode(pushbutton1_pin, INPUT);
    //motor1
    pinMode(DIR1, OUTPUT);
    pinMode(STEP1, OUTPUT);
    pinMode(ENABLE1, OUTPUT);
    digitalWrite(ENABLE1, LOW);
    //motor2
    pinMode(DIR2, OUTPUT);
    pinMode(STEP2, OUTPUT);
    pinMode(ENABLE2, OUTPUT);
    digitalWrite(ENABLE2, LOW);
}
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void accelerometer_home(){
  delay(1000);
  Serial.println("Keep IMU level.");
  delay(1000);
  IMU.calibrateAccelGyro(&calib);
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
  Serial.print("accelerometer_home done");
  Serial.print("\t");
}
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void stepper_zero(){
passflag2 = 0;
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, LOW);
while (passflag2+1 <= (motor_steps_total*4)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag2 ++;
}
position =0;
positiony =0;
delay(100);
Serial.print("stepper_zero done");
Serial.print("\t");
}
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void home_steppers(){
home = motor_steps_total*1; //change this value for starting point stiffness (max 4 turns. 6 clicks/turn)
passflag5 =0;
digitalWrite(DIR1, HIGH);
digitalWrite(DIR2, HIGH);
while (passflag5+1 <= home){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag5 ++;
}
position = home;
positiony = home;
delay(100);
Serial.print("home_steppers done");
Serial.print("\t");
}
//////////////////////////////////////////////////////////////////////
/////////////////////////X Axis///////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//1600 per rotation = home
//[0]    - {-2} home position - motor_steps_total*1
//[640]  - {-1} home position - motor_steps_total*.6
//[1600] - {0}  home position 
//[2560] - {+1} home position + motor_steps_total*.6
//[3200] - {+2} home position + motor_steps_total*1
//
void positioningX(){
///////////////////////Position -2 ////////////////////////////////////
if (accelData.accelX <= -1 & accelData.accelX >= -2){
target_position = home - motor_steps_total*1;
move_position = target_position-position;
if (move_position <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, HIGH);
}
else {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);}
passflag3 = 0;
while (passflag3+1 <= abs(move_position)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag3 ++;
}
position = target_position;
passflag3 =0;
}
///////////////////////Position -1 ////////////////////////////////////
if (accelData.accelX <=-0.5 & accelData.accelX >=-1){
target_position = home - motor_steps_total*.6;///move 4 clicks (4/6=.667)
move_position = target_position-position;
if (move_position <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, HIGH);
}
else {
digitalWrite(DIR1, HIGH);
digitalWrite(DIR2, LOW);}
passflag3 = 0;
while (passflag3+1 <= abs(move_position)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag3 ++;
}
position = target_position;
}
/////////////////////////Position 0 ////////////////////////////
if (accelData.accelX <= .5 & accelData.accelX >= -0.5){
target_position = home;
move_position = target_position-position;
if (move_position <=0 ){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, HIGH);
}
else {
//(move_position > 0){
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);}
passflag3 = 0;
while (passflag3+1 <= abs(move_position)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag3 ++;
}
position = target_position;
inverse = 0;
}
///////////////////////Position 1 ////////////////////////////////////
if (accelData.accelX <= 1 & accelData.accelX >= .5){
target_position = home + motor_steps_total*.6;///move 4 clicks (4/6=.667)
move_position = target_position-position;
if (move_position <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, HIGH);
}
else {
digitalWrite(DIR1, HIGH);
digitalWrite(DIR2, LOW);}
passflag3 = 0;
while (passflag3+1 <= abs(move_position)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag3 ++;
}
position = target_position;
}
///////////////////////Position 2 ////////////////////////////////////
if (accelData.accelX <= 2 & accelData.accelX >= 1){
target_position = home + motor_steps_total*1;
move_position = target_position-position;
if (move_position <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, HIGH);
}
else {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);}
passflag3 = 0;
while (passflag3+1 <= abs(move_position)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag3 ++;
}
position = target_position;
passflag3 =0;
}
////////////////////////////////////////////////////////////////////////////////
}
//////////////////////////////////////////////////////////////////////
/////////////////////////Y axis///////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void positioningY(){
/////////////////////////Position 0 ////////////////////////////
if ((accelData.accelY <= .3 & accelData.accelY >= -0.3)){
target_positiony = home;
move_positiony = target_positiony-positiony;
if (move_positiony <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, LOW);
}
else if (move_positiony>0){
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);}
passflag4 = 0;
while (passflag4+1 <= abs(move_positiony)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag4 ++;
}
positiony = target_positiony;
}
///////////////////////Position 1 ////////////////////////////////////
if ((accelData.accelY <= 1 & accelData.accelY >= .3) || (accelData.accelY <=-0.3 & accelData.accelY >=-1)){
target_positiony = home + motor_steps_total*0.667;
move_positiony = target_positiony-positiony;
if (move_positiony <=0 ){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, LOW);
}
else if (move_positiony>0){
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);}
passflag4 = 0;
while (passflag4+1 <= abs(move_positiony)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag4 ++;
}
positiony = target_positiony;
}
///////////////////////Position 2 ////////////////////////////////////
if ((accelData.accelY <= 2 & accelData.accelY >= 1) || (accelData.accelY <=-1 & accelData.accelY >=-2)){
target_positiony = home + motor_steps_total*1;
move_positiony = target_positiony-positiony;
if (move_positiony <=0){
digitalWrite(DIR1, LOW);
digitalWrite(DIR2, LOW);
}
else if (move_positiony>0){
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);}
passflag4 = 0;
while (passflag4+1 <= abs(move_positiony)){
digitalWrite(STEP1, HIGH);
digitalWrite(STEP2, HIGH);
digitalWrite(STEP1, LOW);
digitalWrite(STEP2, LOW);
delayMicroseconds(175);
passflag4 ++;
}
positiony = target_positiony;
passflag3 =0;
}
////////////////////////////////////////////////////////////////////////////////
}

//////////////////////////////////////////////////////////////////////
void accelerometer_values(){
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
}
//////////////////////////////////////////////////////////////////////
void enable(){
  enable_sw = digitalRead(enable_sw_pin);
  if (enable_sw ==0){
    digitalWrite(ENABLE1, HIGH);
    digitalWrite(ENABLE2, HIGH);
    }
  else digitalWrite(ENABLE1, LOW); 
}
//////////////////////////////////////////////////////////////////////
void re_initialize(){
pushbutton1 = digitalRead(pushbutton1_pin);
if (pushbutton1 == HIGH){
  passflag1=0;
}
}
//////////////////////////////////////////////////////////////////////
void Diagnostics(){
  //Serial.print(enable_sw);
  Serial.print("\t");
  //Serial.print(enable_sw_pin);
 

  //if (IMU.hasTemperature()) {
	 // Serial.print("\t");
	  //Serial.println(IMU.getTemp());
  //}
   Serial.print("X Axis:");
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print("Y Axis:");
  Serial.print(accelData.accelY);
  //Serial.print("\t");
  //Serial.print(accelData.accelZ);
 // Serial.print("\t");
 // Serial.print(gyroData.gyroX);
  //Serial.print("\t");
  //Serial.print(gyroData.gyroY);
  //Serial.print("\t");
  //Serial.print(gyroData.gyroZ);
 // Serial.print("\t");
  //Serial.print("\n");
  //delay(50);
   Serial.print("\n");
}
//////////////////////////////////////////////////////////////////////

void loop() {

    if (passflag1 == 0){
      enable();
      accelerometer_home();
      stepper_zero();
      home_steppers();
      passflag1 ++;
      delay(500);
    }
    
    else if (passflag1==1){
      enable();
      //re_initialize();
      accelerometer_values();
      positioningX();
      positioningY();
      Diagnostics();
    }
    
  


}


  


