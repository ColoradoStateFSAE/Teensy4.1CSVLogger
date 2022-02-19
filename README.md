# CSVLogger
A dual Teensy 4.1 based sensor logger for use with the Colorado State University Formula SAE team car.
Writes .csv logs to microsd card.
Currently only logging Gyroscope and Accelerometer data.
Uses I2C to communicate data between a dedicated sensor teensy and a dedicated logging teensy for better preformance.

Remember teensy 4.1 is only 3.3v tolerant on it's pins.
Provided image shows current wiring:

![image](https://user-images.githubusercontent.com/45497901/154790496-8538843e-05eb-4b07-854a-a8a9ebed1b7c.png)
