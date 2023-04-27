/*
MKS DUAL FOC closed-loop position control routine.Test library: SimpleFOC 2.1.1 Test hardware: MKS DUAL FOC V3.1
Input in the serial port window: T+position, you can make the two motors rotate in closed loop
For example, let the two motors rotate 180°, then input the radian system: T3.14
When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7), and set it to your own number of pole pairs
The default power supply voltage set by the program is 12V, please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The motor targeted by the default PID is the 2804 gimbal motor. To use your own motor, you need to modify the PID parameters to achieve better results.
*/

#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor parameters
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26,27,14,12);

//Command settings
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  I2Cone.begin(19,18, 400000); 
  I2Ctwo.begin(23,5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //Connect the motor object with the sensor object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply voltage setting [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver1.voltage_power_supply = 12;
  driver1.init();
  //Connect the motor and driver objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  //Speed PI loop setting
  motor.PID_velocity.P = 0.1;
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor1.PID_velocity.I = 1;
  //Angle P ring setting
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //Max motor limit motor
  motor.voltage_limit = 1;
  motor1.voltage_limit = 1;
  
  //Speed low-pass filter time constant
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //Set a maximum speed limit
  motor.velocity_limit = 20;
  motor1.velocity_limit = 20;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //Initialize the motor
  motor.init();
  motor1.init();
  //Initialize FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  Serial.print(sensor.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);
  
  command.run();
}
