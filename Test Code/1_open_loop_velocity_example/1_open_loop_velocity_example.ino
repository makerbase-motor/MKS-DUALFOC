// MKS DUAL FOC open-loop speed control routine.Test library: SimpleFOC 2.1.1 Test hardware: MKS DUAL FOC V3.1
// Enter "T+number" in the serial port to set the speed of the two motors. For example, set the motor to rotate at 10rad/s, input "T10", and the motor will rotate at 5rad/s by default when it is powered on
// When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7), and set it to your own number of pole pairs
// The default power supply voltage set by the program is 12V, please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
  
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1  = BLDCDriver3PWM(26,27,14,12);

//Target variable
float target_velocity = 5;

//Serial command setting
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 3;   // [V]
  motor.velocity_limit = 40; // [rad/s]
  
  driver1.voltage_power_supply = 12;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 3;   // [V]
  motor1.velocity_limit = 40; // [rad/s]

 
  //Open loop control mode setting
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor.init();
  motor1.init();


  //Add T command
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);
  motor1.move(target_velocity);

  //User newsletter
  command.run();
}
