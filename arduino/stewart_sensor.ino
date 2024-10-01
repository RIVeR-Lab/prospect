#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define WIRE Wire
#define TCAADDR 0x70
//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
SFEVL53L1X distanceSensor3;

// Helper function for changing TCA output channel
void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  WIRE.beginTransmission(TCAADDR);
  WIRE.write(1 << channel);
  WIRE.endTransmission();  
}

void setup() {
  Serial.begin(9600);
  
  while (!Serial) {
     delay(10);
  }
  Serial.println("\nI2C Scanner");
  
  WIRE.begin();
  tcaselect(1);
  if (distanceSensor1.init() == false)
    Serial.println("Sensor #1 online!");
   tcaselect(6);
  if (distanceSensor2.init() == false)
    Serial.println("Sensor #2 online!"); 
  tcaselect(7);
  if (distanceSensor3.init() == false)
    Serial.println("Sensor #3 online!"); 
  pinMode(A0, OUTPUT);    // sets the digital pin 13 as output
  digitalWrite(A0, HIGH); // sets the digital pin 13 on
}


void find_devices() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void readSerial() {
  String str = Serial.readString();
  // Check string is non-empty
  if (str.length() < 3){
    return;
  }
  // Set light value
  if (str.indexOf("light_power") > -1) {
    // Set light equal to the power value
    int offset = 12; // Number of characters until useful data begins
    String command = str.substring(offset);
    command.trim();
    if (command.equals("ON")) {
      digitalWrite(A0, LOW); 
    } else if (command.equals("OFF")) {
      digitalWrite(A0, HIGH); 
    }
    return;
  }
//  // Set integration time
//  if (str.indexOf("integration_time") > -1) {
//    // Extract the integration time to use
//    // Set light equal to the power value
//    int offset = 16; // Number of characters until useful data begins
//    String command = str.substring(offset);
//    command.trim();
//    // Convert string to integer value
//    int newTime = command.toInt();
//    delayTime = newTime;
//  }
}

void loop() {
//  Serial.println("Checking index 0");
  tcaselect(1);
  distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
  int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor1.stopRanging();
//  Serial.println("Checking index 1");
  tcaselect(6);
  distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor2.stopRanging();
//  Serial.println("Checking index 2");
  tcaselect(7);
  distanceSensor3.startRanging(); //Write configuration bytes to initiate measurement
  int distance3 = distanceSensor3.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor3.stopRanging();
  Serial.print("Distance 1 (mm): ");
  Serial.print(distance1);
  Serial.print(", Distance 2 (mm): ");
  Serial.print(distance2);
  Serial.print(", Distance 3 (mm): ");
  Serial.print(distance3);
  Serial.println();
//  digitalWrite(A0, HIGH); // sets the digital pin 13 on
  //delay(100);          // wait 10 milliseconds for the next scan
//  digitalWrite(A0, LOW); // sets the digital pin 13 on
  //delay(100);          // wait 10 milliseconds for the next scan
  delay(10);
  readSerial();

}
