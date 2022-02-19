//SensorReader v0.2 for Colorado State University formula SAE team car
// Written by Aidan Farley
// For use with CSVLogger v0.7

#include <EasyTransfer.h>
#include <MPU9250_WE.h>
#include <Wire.h>

#define MPU9250_ADDR 0x68

//create object
EasyTransfer ETin, ETout;
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

int ledPin = 13;

struct SEND_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float gValuex;
  float gValuey;
  float gValuez;
  float resultantG;
  float gyrx;
  float gyry;
  float gyrz;
};

struct RECEIVE_DATA_STRUCTURE {
  int recording;
};

//give a name to the group of data
SEND_DATA_STRUCTURE sensordata;
RECEIVE_DATA_STRUCTURE logging;

void setup() {
  Serial1.begin(250000);
  Serial.begin(115200);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(logging), &Serial1);
  ETout.begin(details(sensordata), &Serial1);

  pinMode(13, OUTPUT);

  randomSeed(analogRead(0));

  digitalWrite(ledPin, HIGH);
  delay(200);
  digitalWrite(ledPin, LOW);

  Wire.begin();

  if (Serial) {
    if (!myMPU9250.init()) {
      Serial.println("MPU9250 does not respond");
    }
    else {
      Serial.println("MPU9250 is connected");
    }
  }

  if (Serial) {
    Serial.println("Position your MPU9250 flat and don't move it");
    Serial.println("Calibrating...");
  }
  digitalWrite(ledPin, HIGH); // calibration blink
  delay(1000);
  myMPU9250.autoOffsets();
  digitalWrite(ledPin, LOW);
  if (Serial) {
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

  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, LOW);
    delay(80);
    digitalWrite(ledPin, HIGH);
    delay(80);
    digitalWrite(ledPin, LOW);
  }
  delay(100);
}

void loop() {
  //this is how you access the variables. [name of the group].[variable name]
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
  sensordata.resultantG = myMPU9250.getResultantG(gValue);
  sensordata.gValuex = gValue.x;
  sensordata.gValuey = gValue.y;
  sensordata.gValuez = gValue.z;
  sensordata.gyrx = gyr.x;
  sensordata.gyry = gyr.y;
  sensordata.gyrz = gyr.z;

  //send the data
  if (ETin.receiveData()) {
    digitalWrite(ledPin, HIGH);
    delay(1);
    ETout.sendData();
    Serial.print(logging.recording);
    Serial.print(',');
    Serial.print(gValue.x);
    Serial.print(',');
    Serial.print(gValue.y);
    Serial.print(',');
    Serial.print(gValue.z);
    Serial.print(',');
    Serial.print(sensordata.resultantG);
    Serial.print(',');
    Serial.print(gyr.x);
    Serial.print(',');
    Serial.print(gyr.y);
    Serial.print(',');
    Serial.println(gyr.z);
  }
  else {
    Serial1.flush();
    digitalWrite(ledPin, LOW);
    }
}
