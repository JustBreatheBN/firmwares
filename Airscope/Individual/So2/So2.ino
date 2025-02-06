/*!
  * @file  readGasConcentration.ino
  * @brief Obtain the corresponding gas concentration in the current environment and output the concentration value
  * @n Experiment method: Connect the sensor communication pin to the main control and burn codes into it. 
  * @n Communication mode selection, dial switch SEL:0: IIC, 1: UART
    @n i2c address selection, the default i2c address is 0x74, A1 and A0 are combined into 4 types of IIC addresses
                | A1 | A0 |
                | 0  | 0  |    0x74
                | 0  | 1  |    0x75
                | 1  | 0  |    0x76
                | 1  | 1  |    0x77   default i2c address  
  * @n Experimental phenomenon: You can see the corresponding gas concentration value of the environment at this time by printing on the serial port
  */
#include "DFRobot_MultiGasSensor.h"

//Enabled by default, use IIC communication at this time. Use UART communication when disabled
#define I2C_COMMUNICATION

#ifdef I2C_COMMUNICATION
#define I2C_ADDRESS 0x74
DFRobot_GAS_I2C gas(&Wire, I2C_ADDRESS);
#endif

void setup() {

  Serial.begin(115200);

  while(!gas.begin())
  {
    Serial.println("NO Deivces !");
    delay(1000);
  }

   

  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);
}

void loop() {
  Serial.print("Ambient ");
  Serial.print(gas.queryGasType());
  Serial.print(" concentration is: ");
  Serial.print(gas.readGasConcentrationPPM());
  delay(1000);
}
