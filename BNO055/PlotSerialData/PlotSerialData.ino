/*
The built in serial plotter is good, but it limits our ability to control the visualization.
Thw y-axis for example changes continuously in a non-ideal way. 

Luckily, we can download software for this
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
  delay(BNO055_SAMPLERATE_DELAY_MS);
}