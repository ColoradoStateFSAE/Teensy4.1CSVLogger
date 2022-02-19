// SensorReader v0.3 for Colorado State University formula SAE team car
//  Written by Aidan Farley
//  For use with CSVLogger v0.8

#include <Arduino.h>
#include <MPU9250_WE.h>
#include <Wire.h>

#define BUFFER_LENGTH 64 // set I2C buffer length to 64 bits
#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR); // Object declaration for IMU

int ledPin = 13;
int recording = 0;
String data = "";
String data2 = "";

void setup()
{
  Serial.begin(115200);

  pinMode(13, OUTPUT);

  randomSeed(analogRead(0));

  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);

  Wire.begin(); // join i2c bus (address optional for master)

  if (Serial)
  {
    // check for IMU
    if (!myMPU9250.init())
    {
      Serial.println("MPU9250 does not respond");
    }
    else
    {
      Serial.println("MPU9250 is connected");
    }
  }

  if (Serial)
  {
    Serial.println("Position your MPU9250 flat and don't move it");
    Serial.println("Calibrating...");
  }
  digitalWrite(ledPin, HIGH); // turn on indicator led
  delay(1000);
  myMPU9250.autoOffsets();   // calibrate IMU
  digitalWrite(ledPin, LOW); // turn off indicator led

  if (Serial)
  {
    Serial.println("Done!");
  }

  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_4G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ);
  myMPU9250.setMagOpMode(AK8963_PWR_DOWN); // Magnetometer off
  delay(200);

  // Average 4, 8, 16 and 32 readings, smooths sensor input and jitter
  // Set to 1 for no averaging
  // Reccomended setting is 1 for fast logging rate (0.04s or under)
  analogReadAveraging(1);

  // ready blink
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(ledPin, LOW);
    delay(80);
    digitalWrite(ledPin, HIGH);
    delay(80);
    digitalWrite(ledPin, LOW);
  }
  delay(100);
}

void loop()
{
  // this is how you access the variables. [name of the group].[variable name]
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  double resultantG = myMPU9250.getResultantG(gValue);

  if (recording == 1)
  {
    data = "";
    data2 = "";                 // clear data string
    digitalWrite(ledPin, HIGH); // turn on indicator led

    // combines data into single string
    data += gValue.x;
    data += ",";
    data += gValue.y;
    data += ",";
    data += gValue.z;
    data += ",";
    data += resultantG;
    data += ",";
    data += gyr.x;
    data += ",";
    data += gyr.y;
    data += ",";
    data += gyr.z;

    Wire.beginTransmission(9); // start transmitting to I2C device #9
    Wire.write(data.c_str());  // sends data string
    Wire.endTransmission();    // stop transmitting

    Wire.requestFrom(0x9, 1); // request a byte as character from I2C address 0x9
    recording = Wire.read();  // receive a byte as character
    Serial.print(recording);
    Serial.print(',');
    Serial.print(data);
    Serial.println(data2);
  }
  else
  {
    digitalWrite(ledPin, LOW); // turn off indicator led
    Wire.requestFrom(0x9, 1);  // request a byte as character from I2C address 0x9
    recording = Wire.read();   // receive a byte as character
  }
}
