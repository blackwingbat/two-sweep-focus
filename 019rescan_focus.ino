#include <Dynamixel2Arduino.h>
#include <stdio.h>
#include <string.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 
#define lmt_swt 16

const uint8_t DXL_ID1 = 1, DXL_ID2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID1);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID1);
  dxl.setOperatingMode(DXL_ID1, OP_POSITION);
  dxl.torqueOn(DXL_ID1);

  // Get DYNAMIXEL information
  dxl.ping(DXL_ID2);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID2);
  dxl.setOperatingMode(DXL_ID2, OP_POSITION);
  dxl.torqueOn(DXL_ID2);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID1, 30);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID2, 30);

  delay(3000);
  Serial.setTimeout(10);

  while(digitalRead(lmt_swt)==1){
  dxl.setGoalVelocity(DXL_ID1, 190);
  delay(50);
  }//home_pos
  dxl.setGoalVelocity(DXL_ID1, 0);
  delay(200);   
  
  dxl.setGoalVelocity(DXL_ID1, 1194);
  delay(1300);//center
  dxl.setGoalVelocity(DXL_ID1, 1024);
  delay(200); 
}

void awal(){
  int x;
  while(digitalRead(lmt_swt)==1){
    dxl.setGoalVelocity(DXL_ID1, 190);
    delay(50);   
  }
  dxl.setGoalVelocity(DXL_ID1, 0);
  delay(500);
}

void tengah(){
  int x;
  while(digitalRead(lmt_swt)==1){
    dxl.setGoalVelocity(DXL_ID1, 190);
    delay(50);   
  }
  dxl.setGoalVelocity(DXL_ID1, 0);
  delay(500);
  dxl.setGoalVelocity(DXL_ID1, 1194);
  delay(1300);//center
  dxl.setGoalVelocity(DXL_ID1, 1024);
  delay(200);
}

void gerak(){
    dxl.setGoalVelocity(DXL_ID1, 1100);
    delay(90);
    dxl.setGoalVelocity(DXL_ID1, 1050);
    delay(10);
    dxl.setGoalVelocity(DXL_ID1, 1024);
    delay(2230);
}

void loop() {
  while(Serial.available()){
    String str = Serial.readString();
    str.trim();
    String motor = str.substring(0, 1);
    int imotor = motor.toInt();
    String sudut = str.substring(2, 4);
    int isudut = sudut.toInt();
    float imili = dxl.getPresentPosition(DXL_ID2);

    if(imotor==1){
      if(isudut==0){
        awal();
      }
      else if(isudut==1){
        tengah();
      }
      else if(isudut==2){
        gerak();
      }
    }
    
    if(imotor==2){
      dxl.setGoalPosition(DXL_ID2, isudut, UNIT_DEGREE);
    }

    if(imotor==3){
      if(isudut==0)
        dxl.setGoalPosition(DXL_ID2, imili);
      else if(isudut==1)
        dxl.setGoalPosition(DXL_ID2, imili+0.3);
    }


  }
}
