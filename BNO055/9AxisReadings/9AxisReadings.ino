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
  Adafruit_BNO055 myIMU = Adafruit_BNO055();  //creating our imu object
  //imu::Vector<3> acc -> create an object of type imu::Vector<3> named 'acc'
  //The myIMU.getVector() method is used to retrieve the latest acceleration vector from the BNO055 sensor. It takes a single argument, Adafruit_BNO055::VECTOR_ACCELEROMETER, which specifies which type of vector to retrieve
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