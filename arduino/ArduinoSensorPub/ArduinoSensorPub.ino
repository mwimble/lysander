/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include "lysander/ArduinoSensors.h"
#include <i2c_t3.h>
#include <VL53L0X.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_t3.h>
#include <utility/imumaths.h>

#define SENSOR_COUNT 4
#define SENSOR_LOOP_COUNT 4

int XSHUT[SENSOR_COUNT] = {4, 5, 6, 7};
int SENSOR_I2C_ADDRESS[SENSOR_COUNT] = {61, 62, 63, 64};
VL53L0X sensor[SENSOR_COUNT];

Adafruit_BNO055 bno = Adafruit_BNO055(WIRE1_BUS, -1, BNO055_ADDRESS_A, I2C_MASTER, I2C_PINS_37_38, I2C_PULLUP_EXT, I2C_RATE_100, I2C_OP_MODE_ISR);

ros::NodeHandle  nh;

lysander::ArduinoSensors arduinoSensors;
ros::Publisher pub("arduino_sensors", &arduinoSensors);

void setup() { 
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  bno.begin();
  delay(1000);


  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    pinMode(XSHUT[i], OUTPUT);
    digitalWrite(XSHUT[i], LOW);
  }
  
  
  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    digitalWrite(XSHUT[i], HIGH);
    delay(10);
    sensor[i].setAddress(SENSOR_I2C_ADDRESS[i]);
    delay(10);
    sensor[i].init();
    sensor[i].setTimeout(200);
    sensor[i].startContinuous(0);
    sensor[i].setMeasurementTimingBudget(20000);
  }
}

void loop()
{
//  delay(200);
  for (int i = 0; i < SENSOR_LOOP_COUNT; i++) {
    arduinoSensors.frontLeftMm = sensor[0].readRangeContinuousMillimeters();
    arduinoSensors.frontRightMm = sensor[1].readRangeContinuousMillimeters();
    arduinoSensors.backLeftMm = sensor[2].readRangeContinuousMillimeters();
    arduinoSensors.backRightMm = sensor[3].readRangeContinuousMillimeters();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    arduinoSensors.euler_x = euler.x();
    arduinoSensors.euler_y = euler.y();
    arduinoSensors.euler_z = euler.z();
  }
  
  pub.publish(&arduinoSensors);
  nh.spinOnce();
}


