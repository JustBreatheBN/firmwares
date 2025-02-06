//---------- I2C --------------------------------------------------------------------------------
#include<Wire.h>

//---------- MPU6050 Measurement & Filtering Range ----------------------------------------------
#define AFS_SEL 2  // Accelerometer Configuration Settings   AFS_SEL=2, Full Scale Range = +/- 8 [g]
#define DLPF_SEL  0  // DLPF Configuration Settings  Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 

//---------- Variables for gravity --------------------------------------------------------------
const int MPU_ADDR=0x69;  // I2C address of the MPU-6050
int AcX,AcY,AcZ;  // Accelerometer values
long Cal_AcX, Cal_AcY, Cal_AcZ; // Calibration values
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Min_GAcX=0, Max_GAcX=0, PtoP_GAcX, Min_GAcY=0, Max_GAcY=0, PtoP_GAcY, Min_GAcZ=0, Max_GAcZ=0, PtoP_GAcZ; // Finding Min, Max & Peak to Peak of gravity value
float Min = 0, Max = 0; // Initial value of Min, Max
int cnt; // Count of calibration process
float Grvt_unit; // Gravity value unit
long period, prev_time; // Period of calculation

float vibrationThreshold = 1.0; // Adjust this value according to the sensitivity required for vibration detection

void setup(){
  Wire.begin(1,2);
 
  init_MPU6050();
  
  Serial.begin(115200);

  Gravity_Range_Option();
  
  Calib_MPU6050(); // Calculating calibration value
}

void loop(){
  ReadDate_MPU6050();
  Calc_Grvt();
  Display_Grvt();
//  CheckVibration();
  delay(1000);
}

//void CheckVibration() {
//  // Check if any of the gravity values (GAcX, GAcY, GAcZ) exceeds the vibration threshold
//  if (abs(GAcX) > vibrationThreshold || abs(GAcY) > vibrationThreshold || abs(GAcZ) > vibrationThreshold) {
//    Serial.println("Vibration Detected!");
//    // You can add further actions here, like triggering an alarm, sending an alert, etc.
//  }
//}

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);

  //MPU6050 Accelerometer Configuration Setting
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);  // Accelerometer Configuration register
  if(AFS_SEL == 0) Wire.write(0x00);     // AFS_SEL=0, Full Scale Range = +/- 2 [g]
  else if(AFS_SEL == 1) Wire.write(0x08);     // AFS_SEL=1, Full Scale Range = +/- 4 [g]
  else if(AFS_SEL == 2) Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  else  Wire.write(0x18);     // AFS_SEL=3, Full Scale Range = +/- 10 [g]
  Wire.endTransmission(true);

  //MPU6050 DLPF(Digital Low Pass Filter)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);  // DLPF_CFG register
  if(DLPF_SEL == 0) Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 
  else if(DLPF_SEL == 1)  Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz 
  else if(DLPF_SEL == 2)  Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz 
  else if(DLPF_SEL == 3)  Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz 
  else if(DLPF_SEL == 4)  Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz 
  else if(DLPF_SEL == 5)  Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz 
  else  Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz 
  Wire.endTransmission(true);
}

void Gravity_Range_Option(){
  switch(AFS_SEL) { // Selecting Gravity unit value
    case 0:
      Grvt_unit = 16384;
      break;
    case 1:
      Grvt_unit = 8192;
      break;
    case 2:
      Grvt_unit = 4096;
      break;
    case 3:
      Grvt_unit = 3276.8;
      break;
  }
}  

void Calib_MPU6050() {  
  for(int i = 0 ; i < 500 ; i++) { // Summing Iteration for finding calibration value
    if(i % 200 == 0) {  // Display progress every 200 cycles
      cnt++;
      if(cnt == 1)  { // Characters to display first
        Serial.print("Calculating .");
      }
      else  { // Display progress by point
        Serial.print(".");
      }      
    }    
    
    ReadDate_MPU6050(); // Read Accelerometer data
    
    delay(10);

    // Sum data
    Cal_AcX += AcX;
    Cal_AcY += AcY;
    Cal_AcZ += AcZ;
  }

  // Average Data
  Cal_AcX /= 2000;
  Cal_AcY /= 2000;
  Cal_AcZ /= 2000;

  // Serial Print
  Serial.println("");
  Serial.println("End of Calculation");
  Serial.print("Cal_AcX = "); Serial.print(Cal_AcX);
  Serial.print(" | Cal_AcY = "); Serial.print(Cal_AcY);
  Serial.print(" | Cal_AcZ = "); Serial.println(Cal_AcZ);

  delay(2000);
}

void ReadDate_MPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers
  Wire.requestFrom(MPU_ADDR,6,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void Calc_Grvt() {
  AcX = (AcX - Cal_AcX);  // Calibrated Accelerometer value
  AcY = (AcY - Cal_AcY);  // Calibrated Accelerometer value
  AcZ = (AcZ - Cal_AcZ);  // Calibrated Accelerometer value
  
  GAcX = AcX / Grvt_unit; // Converting the Calibrated value to Gravity value
  GAcY = AcY / Grvt_unit; // Converting the Calibrated value to Gravity value
  GAcZ = AcZ / Grvt_unit; // Converting the Calibrated value to Gravity value

  //---------- Calculating Min, Max & Peak to Peak of Gravity --------------------------------------

  Min_GAcX = min(Min_GAcX, GAcX);
  Max_GAcX = max(Max_GAcX, GAcX);
  PtoP_GAcX = Max_GAcX - Min_GAcX;

  Min_GAcY = min(Min_GAcY, GAcY);
  Max_GAcY = max(Max_GAcY, GAcY);
  PtoP_GAcY = Max_GAcY - Min_GAcY;

  Min_GAcZ = min(Min_GAcZ, GAcZ);
  Max_GAcZ = max(Max_GAcZ, GAcZ);
  PtoP_GAcZ = Max_GAcZ - Min_GAcZ;
}  

void Display_Grvt() {
  //---------- Serial print ----------------------------------------------------------------------
  Serial.print("AcX= " + String(AcX));
  Serial.print(" |AcY= " + String(AcY));
  Serial.println(" |AcZ= " + String(AcZ));
}
