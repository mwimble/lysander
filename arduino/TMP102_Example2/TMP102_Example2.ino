#include <Wire.h> // Used to establied serial communication on the I2C bus
#include "SparkFunTMP102.h" // Used to send and recieve specific information from our sensor

// Connections
// VCC = 3.3V
// GND = GND
// SDA = A4
// SCL = A5

TMP102 leftMotor(0x48);
TMP102 rightMotor(0x49);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Start serial communication at 9600 baud
  leftMotor.begin();
  rightMotor.begin();

  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  leftMotor.setConversionRate(2);
  rightMotor.setConversionRate(2);

  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  leftMotor.setExtendedMode(0);
  rightMotor.setExtendedMode(0);
}

void loop() {
  rightMotor.setLowTempF(25);
  Serial.print("LowF left: ");
  
  Serial.print(leftMotor.readLowTempF());
  Serial.print(", LowF right: ");
  Serial.print(rightMotor.readLowTempF());
  // put your main code here, to run repeatedly:
  float leftTemp = leftMotor.readTempF();
  float rightTemp = rightMotor.readTempF();
  Serial.print(", Left F: ");
  Serial.print(leftTemp);
  Serial.print(", Right F: ");
  Serial.print(rightTemp);

  leftTemp = leftMotor.readTempC();
  rightTemp = rightMotor.readTempC();
  Serial.print(", Left C: ");
  Serial.print(leftTemp);
  Serial.print(", Right C: ");
  Serial.print(rightTemp);

  Serial.println();
  delay(1000);
}
