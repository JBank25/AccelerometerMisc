/*
  Where we are currently at here, we can determine tilt in the x and y directions but ONLY for a static
  platform. If our system has vibration, say swaying back and forth quickly we will read tilt when there is
  NONE. What can we do to prevent this?
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS 500 //sample every 100ms, change as needed
#define GRAVITY 9.8
#define RADIANS_TO_DEGREES(deg) (deg/2/3.141592654*360)
Adafruit_BNO055 myIMU = Adafruit_BNO055();  //creating our imu object

float theta;  //tilt about x axis
float phi;    //tilt about y axis

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

float getTiltAboutX(double xAccel, double zAccel)
{
  return RADIANS_TO_DEGREES(atan2(xAccel/GRAVITY, zAccel/GRAVITY));
}
float getTiltAboutY(double yAccel, double zAccel)
{
  return RADIANS_TO_DEGREES(atan2(yAccel/GRAVITY, zAccel/GRAVITY));
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
  Serial.print(acc.x()/GRAVITY);
  Serial.print(",");
  Serial.print("y:");
  Serial.print(acc.y()/GRAVITY);
  Serial.print(",");
  Serial.print("z:");
  Serial.print(acc.z()/GRAVITY);
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

  theta = getTiltAboutX(acc.x(), acc.z());
  Serial.print("Tilt About X Degrees:");
  Serial.print(theta);
  Serial.print(" ");
  phi = getTiltAboutY(acc.y(), acc.z());
  Serial.print("Tilt About Y Degrees:");
  Serial.print(phi);
  Serial.println();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}