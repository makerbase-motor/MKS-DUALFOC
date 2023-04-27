/**
MKS DUAL FOC closed-loop position-torque mutual control routine Test library: SimpleFOC 2.1.1 Test hardware: MKS DUAL FOC V3.1
Turn one of the motors after power on, and the other motor will follow, and the two motors maintain mutual control of position and torque at all times
When using your own motor, please remember to modify the default number of pole pairs, that is, the value in BLDCMotor(7), and set it to your own number of pole pairs
The default power supply voltage set by the program is 12V, please remember to modify the values in voltage_power_supply and voltage_limit variables if you use other voltages for power supply
The default PID target motor is 2804 PTZ motor, and the encoder used is AS5600. To use your own motor, you need to modify the PID parameters to achieve better results.
 */
 
#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);


void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();

  driver1.voltage_power_supply = 12;
  driver1.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::torque;

  // maximal voltage to be set to the motor
  motor.voltage_limit = 12;
  motor1.voltage_limit = 12;
  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  
  //Initialize the motor
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move( 5*(motor1.shaft_angle - motor.shaft_angle));
  motor1.move( 5*(motor.shaft_angle - motor1.shaft_angle));
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}
