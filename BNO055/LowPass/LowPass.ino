/*
  Low pass -> pass the low frequency data. 
  This will allow us to filter out the high frequency data that corrupts our
  tilt calculations

  By providing only a small scaling weight to the new measured tilt angles we can
  nudge our calculations in the direction that they are moving in without fully going there.
  For example if we scale by 0.05*measured then we will be moving very slowly in the direction
  of the measured angle. If we scale by .5 we would be moving very quickly.


*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS 500 //sample every 100ms, change as needed
#define GRAVITY 9.8
#define RADIANS_TO_DEGREES(deg) (deg/2/3.141592654*360)
#define OLD_FILTER_WEIGHT 0.6
#define MEASURED_TILT_WEIGHT (1 - OLD_FILTER_WEIGHT)
Adafruit_BNO055 myIMU = Adafruit_BNO055();  //creating our imu object

float theta;  //tilt about x axis
float phi;    //tilt about y axis
float thetaMeasured;
float thetaFilterOld = 0;
float thetaFilterNew;
float phiMeasured;
float phiFilterOld = 0;
float phiFilterNew;

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

  thetaMeasured = getTiltAboutX(acc.x(), acc.z());
  Serial.print("Tilt About X Degrees Measured:");
  Serial.print(thetaMeasured);
  Serial.print(" ");
  phiMeasured = getTiltAboutY(acc.y(), acc.z());
  Serial.print("Tilt About Y Degrees Measured:");
  Serial.print(phiMeasured);
  Serial.println();

  phiFilterNew = OLD_FILTER_WEIGHT*phiFilterOld + MEASURED_TILT_WEIGHT*phiMeasured;
  thetaFilterNew = OLD_FILTER_WEIGHT*thetaFilterOld + MEASURED_TILT_WEIGHT*thetaMeasured;
  Serial.print("Theta Filtered New: ");
  Serial.print(thetaFilterNew);
  Serial.print(" ");
  Serial.print("Phi Filtered New: ");
  Serial.print(phiFilterNew);
  Serial.println();
  phiFilterOld = phiFilterNew;
  thetaFilterOld = thetaFilterNew;
  delay(BNO055_SAMPLERATE_DELAY_MS);
}