#include <SimpleFOC.h>

// instantiate driver
// instantiate sensor
// instantiate current sense
// instantiate motor

void setup() {  

  // init joint sensor
  // link the motor to the sensor

  // power supply voltage
  driver.voltage_power_supply = 12;
  
  // driver init
  // link the motor to the driver
  driver.init();

  // current sense init
  // link the motor to the current sense

  // set control loop type to be used
  // motor init

  // align encoder and start FOC

}

void loop() {

  // FOC algorithm function

  // velocity control loop function
  // setting the target velocity or 2rad/s

  // monitoring function outputting motor variables to the serial terminal 

  // user communication

}
