#include <SimpleFOC.h>

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(11, 5.55f, 100);
BLDCDriver3PWM driver = BLDCDriver3PWM(D1, D2, D3, D0);

Commander commander = Commander(Serial);

void onMotor(char* cmd) {
  commander.motor(&motor, cmd);
}

void setup() {
  Serial.begin(115200);

  as5600.init();
  Serial.println("AS5600 ready");

  driver.voltage_power_supply = 15;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);

  motor.linkSensor(&as5600);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.voltage_limit = 15.0;
  motor.velocity_limit = 100.0;

  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 10.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.limit = motor.voltage_limit;

  motor.LPF_velocity.Tf = 0.02;

  motor.P_angle.P = 10.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;

  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 0.009203f;

  motor.init();
  motor.initFOC();
  Serial.println("FOC ready (position control)");

  Serial.print("Direction: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial.print("zero_electric_angle: ");
  Serial.println(motor.zero_electric_angle, 6);

  commander.add('M', onMotor);
}

void loop() {
  motor.loopFOC();
  motor.move(2.0f);

  commander.run();
}
