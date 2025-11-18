#include <SimpleFOC.h>

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

void setup() {
  Serial.begin(115200);
  as5600.init();

  Serial.println("AS5600 ready");
}

void loop() {
  as5600.update();
  Serial.print(as5600.getAngle(), 6);
  Serial.print("\t");
  Serial.println(as5600.getVelocity(), 6);
}