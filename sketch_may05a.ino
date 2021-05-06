#include "Nano33BleHID.h"
#include <Arduino_LSM9DS1.h>
#include "MadgwickAHRS.h"
#define M_PI 3.141592653589793238462643

// initialize a Madgwick filter:
Madgwick filter;
// sensor's sample rate is fixed at 104 Hz:
const float sensorRate = 104.00;

float yaw, pitch, roll;
float vertZero, horzZero;
float vertValue, horzValue;
float prev_horzValue, prev_vertValue;
const float sensitivity =0.001;

// Alias to Nano33BleHID<HIDGamepadService>
Nano33BleMouse mouse("Ble Mouse");

void setup() {
  Serial.begin(9600);
  // attempt to start the IMU:
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }
  // start the filter to run at the sample rate:
  filter.begin(sensorRate);
  
    // initialize the ble HID.
    mouse.initialize();

    // Launch the eventqueue thread.
    MbedBleHID_RunEventThread();
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;
    vertZero = 0;
    horzZero = 0;
    prev_horzValue = 0;
    prev_vertValue = 0;
}

void loop() {
  // values for acceleration and rotation:
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;

  // values for orientation:
  float roll, pitch, heading;
  // check if the IMU is ready to read:
  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable()) {
    // read accelerometer &and gyrometer:
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readGyroscope(xGyro, yGyro, zGyro);

    // update the filter, which computes orientation:
    filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);

    // print the yaw, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
      } 
    vertValue = yaw - vertZero; 
    horzValue = roll - horzZero; 
    vertZero = yaw; 
    horzZero = roll;  
    horzValue = atan2(yAcc, zAcc) * 180/M_PI;
    vertValue = atan2(xAcc, sqrt(yGyro*yGyro + zGyro*zGyro)) * 180/M_PI;
    horzValue = (horzValue+prev_horzValue)/2;
    vertValue = (vertValue+prev_vertValue)/2;
    // Retrieve the HID service handle.
    auto *hid = mouse.hid();

    // Update internal values.
//    float theta = PI * (roll / pitch);
    hid->motion(horzValue * sensitivity, vertValue * sensitivity);
    prev_horzValue = horzValue;
    prev_vertValue = vertValue;
    
    // Send them !
    hid->SendReport();
}
