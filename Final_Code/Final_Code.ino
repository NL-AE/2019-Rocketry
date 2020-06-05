#include <Wire.h>
#include <Servo.h>

//SD Card
#include <SPI.h>
#include <SD.h>
#define chipSelect 4

//Gyro
#include <MPU6050.h>
MPU6050 mpu;
#define timeStep 0.01
double pitch = 0,    yaw = 0,    roll = 0;
double PitchOut = 0, YawOut = 0, RollOut = 0, Para = 0;  //

//Altimeter
#include "SparkFunBME280.h"
BME280 Altimeter;
int Altitude;
int Apogee;
#define V1 8.75  //Speed with one parachute
#define V2 4.75  //Speed with all deployed

Servo North;        //to pin 5
Servo East;         //to pin 6
Servo South;        //to pin 9
Servo West;         //to pin 10
Servo Parachute;    //to pin 3

#define RFactor 5
#define PFactor 10
#define YFactor 10

int LaunchTime, Time;
bool Launched = false;
float Accel;

#define ParachutePin 2        //Connect to parachute detector D8
bool Attached = true;

#define MinimumAlt 50

void setup() {
  Serial.begin(9600);
  North.attach(5);
  East.attach(6);
  South.attach(9);
  West.attach(10);
  Parachute.attach(3);

  Parachute.write(170);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  Wire.begin();
  Altimeter.setI2CAddress(0x76);
  if(Altimeter.beginI2C() == false) Serial.println("Sensor B connect failed");
}

void loop() {

  if(Launched == false){
    //READ ACCELERATION
  }  
  
  if(Accel>30 && Launched == false){
    Launched == true;
    Launchtime = millis();
  }

  if(digitalRead(ParachutePin) == HIGH){ Attached = false; }    //Parachute deployment check

  Altitude = 13504.985*(1-pow((Altimeter.readFloatPressure()/101325.), 0.190289));  

    if(Attached == true){
      Vector norm = mpu.readNormalizeGyro();
        pitch = pitch + norm.YAxis * timeStep;
        roll = roll + norm.ZAxis * timeStep;
        yaw = yaw + norm.XAxis * timeStep;
      
      North.write  (90 + pitch*PFactor - yaw*YFactor + roll*RFactor);
      South.write  (90 - pitch*PFactor + yaw*YFactor + roll*RFactor);
    
      West.write   (90 + pitch*PFactor - yaw*YFactor + roll*RFactor);  
      East.write   (90 - pitch*PFactor + yaw*YFactor + roll*RFactor);
    } else {
      North.write(5);East.write(5);South.write(5);West.write(5);

    if(Altitude <= MinimumAlt){Parachute.write(10)}                        //If minimum altitude is reached
    
    if(Time >= (Apogee-((36.5)*V2)*1000/(V1-V2)) ){Parachute.write(10)}    //Calculated time    
    }

  if(Altitude > Apogee){Apogee = Altitude;}  //Apogee
  Time = millis() - Launchtime;              //Time from launch


  if(Launched == true){
    File dataFile = SD.open("datalog.txt", FILE_WRITE);   
    dataFile.print(Time);
    dataFile.print(",");
    dataFile.print(Altimeter.readFloatPressure());  //Raw pressure data
    dataFile.print(",");
    dataFile.print(Altitude);   //Calculated pressure
    dataFile.close();    
  }

  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print("   Roll = ");
  Serial.print(roll);  
  Serial.print("   Yaw = ");
  Serial.println(yaw);
  
}
