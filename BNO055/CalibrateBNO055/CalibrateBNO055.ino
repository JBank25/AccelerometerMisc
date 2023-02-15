/*
Recall the nonzero readings we were getting when the IMU was sitting still, as well as the slightly
off readings of gravity in this state. We would like a way to make our readings MORE accurate. Grabbing 
data without calibration is a contributing factor in this!

0 - completely uncalibrated 3 - completely calibrated.

Gyro is the most easy to calibrate, all we need to to is let it stay still.
Swinging around the magnetometer allows it to calibrate.
Moving and holding is good for the calibration of the accelerometer. Try moving it 360 degrees across
x, y, and z pausing every 45 degrees, worked well for me.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 500 //sample every 100ms, change as needed

Adafruit_BNO055 myIMU = Adafruit_BNO055();  //creating our imu object

void setup() 
{
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  Serial.println("Setting up now");
  uint8_t temp = myIMU.getTemp();
  Serial.println(temp);
  myIMU.setExtCrystalUse(true); //use the crystal on the board 
}

void loop() 
{
  uint8_t system_calibration, gyro_calibration, accel_calibration, mag_calibration = 0;
  myIMU.getCalibration(&system_calibration, &gyro_calibration, &accel_calibration, &mag_calibration);

  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);  

  Serial.println("Accelerometer:");
  Serial.print("x:");
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print("y:");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print("z:");
  Serial.print(acc.z());
  Serial.println();

  Serial.println("Gyroscope:");
  Serial.print("x:");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print("y:");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print("z:");
  Serial.print(gyro.z());
  Serial.println();

  Serial.println("Magnetometer:");
  Serial.print("x:");
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print("y:");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.print("z:");
  Serial.print(mag.z());
  Serial.println();

  Serial.println("Calibrations:");
  Serial.print("Accelerometer Calibration: ");
  Serial.print(accel_calibration);
  Serial.print(",");
  Serial.print("Gyro Calibration: ");
  Serial.print(gyro_calibration);
  Serial.print(",");
  Serial.print("Magnetometer Calibration: ");
  Serial.print(mag_calibration);
  Serial.print(",");
  Serial.print("System Calibration: ");
  Serial.print(system_calibration);
  Serial.println();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}