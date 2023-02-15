/*
  The low pass filter approach to filtering out vibration worked, but it made our 
  sensor quite sluggish. Using oNLY an accelerometer this is about as good as we can make
  it. In this sketch we will turn on the gyro to measure angular velocity around our
  x, y, and z axis. 

  Look at the behavior of the gyro over time after moving about an axis. Notice the accumulation of
  error, the drift that occurs when using this sensor.

  We need a quick sensor and one that is not subject to accumulation of error. Right now we have an 
  accelerometer which is not subject to the accumulation of error, but it is quite slow. We also
  have a gyro here that is very fast but accumulates error and drifts away from anything meaningful
  over time.

  How can we get the best of both sensors?
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

//thetaGyr0 = thetaGyro,old + dt*angular rate
float thetaGyro = 0;
float phiGyro = 0;
float dt;
unsigned long millisOld;  

void setup() 
{
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  Serial.println("Setting up now");
  uint8_t temp = myIMU.getTemp();
  Serial.println(temp);
  myIMU.setExtCrystalUse(true); //use the crystal on the board 
  millisOld = millis();
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

  Serial.println("Gyroscope:");
  Serial.print("x:");
  Serial.print(gyro.x()/GRAVITY);
  Serial.print(",");
  Serial.print("y:");
  Serial.print(gyro.y()/GRAVITY);
  Serial.print(",");
  Serial.print("z:");
  Serial.print(gyro.z()/GRAVITY);
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

  dt = (millis() - millisOld) / 1000.;  //dt in seconds
  millisOld = millis();
  thetaGyro = thetaGyro + gyro.x() * dt;
  phiGyro = phiGyro + gyro.y() * dt;
  Serial.println("Gyro:");
  Serial.print("Tilt about x:");
  Serial.print(thetaGyro);
  Serial.print("Tilt about y:");
  Serial.print(phiGyro);
  Serial.println();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}