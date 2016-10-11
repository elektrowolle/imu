//
// Created by Benjamin Skirlo on 29.09.16.
//
#define DEVICE_I2C 1
#include "mbed.h"
#include "bno055/IMU.h"
#include "bno055/BNO055_driver/bno055.h"



I2C i2c(I2C_SDA, I2C_SCL);

unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 40;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream
s32 bno055_data_readout_template(void);
template <typename T>
void print(T v){
  printf(v);
}

template <>
void print<unsigned char>(unsigned char v){
  printf("%i", v);
}

template <>
void print<long unsigned int>(long unsigned int v){
  printf("%i", v);
}

template <>
void print<float>(float v){
  printf("%ul", v);
}

void println(){
  printf("\n\r");
}

template <typename T>
void println(T v){
  print(v);
  println();
}


void delay(int ms){
  Thread::wait(ms);
}

int main(){

  NAxisMotion motion(i2c);

  println("\t==== INIT Sensor ====");
  motion.initSensor(0x28 << 1);
  println("\t==== Set PowerMode ====");
  motion.setPowerMode(BNO055_POWER_MODE_NORMAL);
  println("\t==== Set Operational Mode ====");
  motion.setOperationMode(BNO055_OPERATION_MODE_NDOF);
  println("\t==== Set Update Mode ====");
  motion.setUpdateMode(NAxisMotion::MANUAL);
//  motion.accelInterrupts(1,1,1);
//  motion.enableAnyMotion(BNO055_ACCEL_RANGE_4G, BNO055_ACCEL_BW_1000HZ);
//  println("\t==== Write AccelConfig ====");
  motion.writeAccelConfig(0x03,0x07,0x00);
//  println("\t==== Update AccelConfig ====");
  motion.updateAccelConfig();

  println();
  println("Default accelerometer configuration settings...");
  print("Range: ");
  println(motion.readAccelRange());
  print("Bandwidth: ");
  println(motion.readAccelBandwidth());
  print("Power Mode: ");
  println(motion.readAccelPowerMode());
  println("Streaming in ...");	//Countdown
  print("3...");
  delay(1000);	//Wait for a second
  print("2...");
  delay(1000);	//Wait for a second
  println("1...");
  delay(1000);	//Wait for a second

  while(true) {
      motion.updateAccel();        //Update the Accelerometer data
//      motion.updateLinearAccel();  //Update the Linear Acceleration data
//      motion.updateGravAccel();    //Update the Gravity Acceleration data
//      motion.updateCalibStatus();  //Update the Calibration Status
      updateSensorData = false;

      print("aX: ");
      print(motion.readAccelX()); //Accelerometer X-Axis data
      print("m/s2 ");

//      print(" aY: ");
//      print(motion.readAccelY());  //Accelerometer Y-Axis data
//      print("m/s2 ");
//
//      print(" aZ: ");
//      print(motion.readAccelZ());  //Accelerometer Z-Axis data
//      print("m/s2 ");
//
//      print("      lX: ");
//      print(motion.readLinearAccelX()); //Linear Acceleration X-Axis data
//      print("m/s2 ");
//
//      print(" lY: ");
//      print(motion.readLinearAccelY());  //Linear Acceleration Y-Axis data
//      print("m/s2 ");
//
//      print(" lZ: ");
//      print(motion.readLinearAccelZ());  //Linear Acceleration Z-Axis data
//      print("m/s2 ");
//
//      print("      gX: ");
//      print(motion.readGravAccelX()); //Gravity Acceleration X-Axis data
//      print("m/s2 ");
//
//      print(" gY: ");
//      print(motion.readGravAccelY());  //Gravity Acceleration Y-Axis data
//      print("m/s2 ");
//
//      print(" gZ: ");
//      print(motion.readGravAccelZ());  //Gravity Acceleration Z-Axis data
//      print("m/s2 ");

      print("      C: ");
      print(motion.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

      println();

      Thread::wait(100);
    }

  return 0;
}
