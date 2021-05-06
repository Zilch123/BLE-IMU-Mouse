#include "Nano33BleHID.h"
#include <Arduino_LSM9DS1.h>
#include "MadgwickAHRS.h"
#include "SensorFusion.h" 
#define M_PI 3.141592653589793238462643

float xAcc, yAcc, zAcc;
float yaw, pitch, roll;
float vertValue, horzValue;
const float sensitivity =0.001;
const float miu = 0.001;
int sign = 1;

Nano33BleMouse mouse("Ble Mouse");

void setup() {
  Serial.begin(115200);
  
  // attempt to start the IMU:
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    // stop here if you can't access the IMU:
    while (true);
  }

    // initialize the ble HID.
    mouse.initialize();
    
    // Launch the eventqueue thread.
    MbedBleHID_RunEventThread();

}

void loop() {
  // values for acceleration and rotation:
  if (IMU.accelerationAvailable() &&
      IMU.gyroscopeAvailable()) {
    // read accelerometer &and gyrometer:
    IMU.readAcceleration(xAcc, yAcc, zAcc);}

    if(zAcc>0){sign  = 1;}
    else{sign  = -1;}
    roll  = atan2( yAcc,sign* sqrt(zAcc*zAcc+ miu*xAcc*xAcc)) * 180.0/M_PI;
    pitch  = atan2(-xAcc, sqrt(yAcc*yAcc + zAcc*zAcc)) * 180.0/M_PI;
    
    horzValue = map(roll, 30.000, -30.000, -0.006, 0.001);
    vertValue = map(pitch, 30.000, -30.000, -0.006, 0.001);

    Serial.print("Pitch: "+String(pitch));
    Serial.print("      Roll:"+String(roll));
    Serial.print("      horzValue: "+String(horzValue));
    Serial.println("      vertValue:"+String(vertValue));
    
    // Retrieve the HID service handle.
    auto *hid = mouse.hid();

    // Update internal values.
    hid->motion(roll *sensitivity , pitch *sensitivity);
    // Send them !
    hid->SendReport();
}
