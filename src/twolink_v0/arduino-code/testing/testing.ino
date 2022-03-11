#include <SimpleFOC.h>

// motor instance
BLDCMotor motor = BLDCMotor(11);
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 7);



// Magnetic sensor instance
MagneticSensorSPI AS5x4x = MagneticSensorSPI(10, 14, 0x3FFF);



float target_angle_rad = 0;
float target_angle_deg = 0;
float target_angle_initial;
float max_angle, min_angle;
float actual_angle_deg;



void setup() {
  // initialize magnetic sensor hardware
  AS5x4x.init();
  // link the motor to the sensor
  motor.linkSensor(&AS5x4x);
  // driver config
  driver.init();
  motor.linkDriver(&driver);
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;  
  // controller configuration
  // default parameters in defaults.h
  // controller configuration based on the control type
  // velocity PID controller parameters
  // default P=0.5 I = 10 D =0
  // PID parameters without print commands
  // motor.PID_velocity.P = 8.0;
  // motor.PID_velocity.I = 15;
  // motor.PID_velocity.D = 0.001;
  // PID parameters with print commands
  motor.PID_velocity.P = 1;
  motor.PID_velocity.I = 0.05;
  motor.PID_velocity.D = 0.001;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.05;
  // angle P controller - default P=20
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  // default 20
  motor.velocity_limit = 9;
  // default voltage_power_supply
  motor.voltage_limit = 10;
  // use monitoring with serial
  Serial.begin(9600);
  // comment out if not needed
  motor.useMonitoring(Serial);
  // initialize motor
  motor.init();
  // align encoder and start FOC
  //motor.initFOC(2.82,CW);
  motor.initFOC();
  Serial.println("Motor ready.");
  _delay(1000);
  // set intial angles variables
  target_angle_initial = 0;
  min_angle = -90;
  max_angle = 90;
}




void loop() {
  if (target_angle_deg <= min_angle)
  {
    target_angle_deg = min_angle;
  }
  if (target_angle_deg >= max_angle)
  {
    target_angle_deg = max_angle;
  }
  target_angle_rad = (target_angle_initial + target_angle_deg) * 0.01745;
  motor.loopFOC();
  motor.move(target_angle_rad);
  // receive the used commands from serial
  serialReceiveUserCommand();
  actual_angle_deg = -1*(AS5x4x.getAngle()*57.30);
  Serial.println(actual_angle_deg );
}
  
  
  
  
void serialReceiveUserCommand() 
{
  // a string to hold incoming data
  static String received_chars;
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    
    // end of user input
    if (inChar == '\n') 
    {
      // change the motor target
      target_angle_deg = received_chars.toFloat();
      // reset the command buffer
      received_chars = "";
    }
  }
}
