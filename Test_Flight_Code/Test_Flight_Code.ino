/*
 *      Setup Diagram (top down):
 * 
 *          Servo 9          Servo 6
 *     
 *         
 *                    Gyro
 *     
 *     
 *          Servo 10         Servo 5
 * 
 * 
 *      Pitch (tilt along this axis:     <------>)
 *      
 *      Yaw   (along this axis:)
 *                    /\
 *                    ||
 *                    ||
 *                    ||
 *                    ||
 *                    ||
 *                    \/
 *      Roll  (⑨)
 *
 */

#include <Wire.h>
#include <Servo.h>

//SD Card
#include <SPI.h>
#include <SD.h>
#define chipSelect 4

//Gyro libary: https://github.com/jarzebski/Arduino-MPU6050
#include <MPU6050.h>
MPU6050 mpu;
#define timeStep 0.01
double pitch = 0,    yaw = 0,    roll = 0,    Para = 0;
float Accel;


//Altimeter libary from sparkfun: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include "SparkFunBME280.h"
BME280 Altimeter;
int Altitude;

//Servos
Servo BRServo;        //to pin 5
Servo TRServo;        //to pin 6
Servo TLServo;        //to pin 9
Servo BLServo;        //to pin 10
Servo Parachute;      //to pin 3

int BRServoAng, TLServoAng, BLServoAng, TRServoAng;

//Konstants for tuning
#define PFactor 1
#define YFactor 1
#define RFactor 1
#define MinAngle 45
#define MaxAngle 135

digittlaRead(pinsense,(;

//Flight variables
#define ParachutePin 2        //Connect to parachute detector D2
int Launchtime = 0;           //Time between turning on and launch
double FlightTime = 0;           //Time since launch
bool Launched = false;        //Is the rocket launched?
bool Attached = true;         //Is the rocket's first parachute still attached (not deployed)?

                          //------------------------//
void setup() {            //------SETUP  BEGIN------//
                          //------------------------//
  //Connect servos
  Serial.begin(115200);
  BRServo.attach(5);
  TRServo.attach(6);
  TLServo.attach(9);
  BLServo.attach(10);
  Parachute.attach(3);

  Parachute.write(170);  //Sets parachute servo to LOCKED position

  //Gyro+Accel start with settings: 250DPS (degrees/second), 16G (maximum 16Gs, expecting 10.7G MAX)
        while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_16G)) {
          Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
          delay(500);
        }
        mpu.calibrateGyro();
        mpu.setThreshold(1);

  //Start Altimeter
        Wire.begin();
        Altimeter.setI2CAddress(0x76);
        if (Altimeter.beginI2C() == false) Serial.println("Sensor B connect failed");

  BRServo.write(90);  //Set servos straight up
  TLServo.write(90);
  BLServo.write(90);
  TRServo.write(90); 
  //delay(3000); 
                          //------------------------//
}                         //-------SETUP  END-------//
                          //------------------------//
                          
                          //------------------------//
void loop() {             //-------LOOP BEGIN-------//
                          //------------------------//
  //Acceleration
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();
  Accel = normAccel.ZAxis;  //Variable to Z axis acceleration

  //Angles
  Vector norm = mpu.readNormalizeGyro();
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.ZAxis * timeStep;
  yaw = yaw + norm.XAxis * timeStep;

  //Altitude
  Altitude = Altimeter.readFloatPressure();
  
    //READOUTS
  Serial.print("Pitch : ");
  Serial.print(pitch);
  Serial.print("   Yaw : ");
  Serial.print(yaw);
  Serial.print("   Roll : ");
  Serial.print(roll);  
  Serial.print("   Acceleration : ");
  Serial.print(Accel);
  Serial.print("   Pressure : ");
  Serial.print(Altitude);
  Serial.print("   Launched : ");
  Serial.print(Launched);
  Serial.print("   Attached : ");
  Serial.print(Attached);
  Serial.print("   Launchtime : ");
  Serial.print(Launchtime);
  Serial.print("   Flighttime : ");
  Serial.println(FlightTime);
  
  

  if (Accel > 15 && Launched == false) {  //Find launch time (once rocket feels 2.5G at about 0.1s)
    Launched = true;                     //Only happens once since bool Launched only false once
    Launchtime = millis();
  }

  if(Launched == true){
    FlightTime = millis() - Launchtime;   //Find time from blast off (ONLY if launched)
  }

  if(digitalRead(ParachutePin) == HIGH) { //Pin normally connected to ground, once parachute deploys, goes floating
    Attached = false;
  }

  if (Attached == true) {     //Fin correction active when parachute not attached
    BRServoAng = 90 + pitch*PFactor + yaw*YFactor + roll*RFactor;
    TLServoAng = 90 - pitch*PFactor - yaw*YFactor + roll*RFactor;
    BLServoAng = 90 - pitch*PFactor + yaw*YFactor + roll*RFactor;
    TRServoAng = 90 + pitch*PFactor - yaw*YFactor + roll*RFactor;   
     
    BRServo.write  (constrain(BRServoAng, MinAngle, MaxAngle));
    TLServo.write  (constrain(TLServoAng, MinAngle, MaxAngle));
    BLServo.write  (constrain(BLServoAng, MinAngle, MaxAngle));
    TRServo.write  (constrain(TRServoAng, MinAngle, MaxAngle));   //90 is straight up  
  } 
  else
  {
    BRServo.write(5); TRServo.write(5); TLServo.write(5); BLServo.write(5); //Try to slow down rocket
  }

  if (FlightTime > 25000){
    Parachute.write(10);      //Move parachute servo after 25 seconds (after launch) to deploy secondary parachutes
    //Serial.print("Secondary Parachute deployed");
  }    


  //Datalogging
  if(FlightTime < 60000){
  File dataFile = SD.open("DATALOG.TXT", FILE_WRITE);
  dataFile.print(FlightTime);                                       //Time,
  dataFile.print(",");          dataFile.print(Altitude);           //Raw pressure data,
  dataFile.print(",");          dataFile.print(normAccel.ZAxis);    //Acceleration,
  dataFile.print(",");          dataFile.print(pitch);              //Pitch,
  dataFile.print(",");          dataFile.print(roll);               //Roll,
  dataFile.print(",");          dataFile.println(yaw);              //Yaw
  dataFile.close();
  }
  
                          //------------------------//
}                         //--------LOOP END--------//
                          //------------------------//
