/*
If we are to move our system in x, y, or z directions we will see an INCREASE in the magnitude
for our accelerometer readings. Look at the serial plotter to visualize this a bit better. Sample 
Rate (BNO055_SAMPLERATE_DELAY_MS) can be decrease  (meaning sampling faster) to better visualize this

We can also see some noise corrupting our non-zero measurements we would ideally expect.

The z acclerometer reading is what is measuring gravity for us. Accelerometer measures
the force on a suspended mass.


*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS 500 //sample every x mseconds, change as needed

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

  delay(BNO055_SAMPLERATE_DELAY_MS);
}