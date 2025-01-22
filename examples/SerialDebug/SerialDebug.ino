#include "Arduino_ScienceKitCarrier.h"

ScienceKitCarrier science_kit;

void setup() {
  Serial.begin(115200);
  science_kit.begin(NO_AUXILIARY_THREADS);
}

void loop() {
  science_kit.update(ROUND_ROBIN_ENABLED);
  Serial.print(science_kit.getAccelerationX());
  Serial.print("\t");
  Serial.print(science_kit.getAccelerationY());
  Serial.print("\t ");
  Serial.print(science_kit.getAccelerationZ());
  Serial.print("\t| ");
  Serial.print(science_kit.getAngularVelocityX());
  Serial.print("\t");
  Serial.print(science_kit.getAngularVelocityY());
  Serial.print("\t");
  Serial.print(science_kit.getAngularVelocityZ());
  Serial.print("\t| ");
  Serial.print(science_kit.getMagneticFieldX());
  Serial.print("\t");
  Serial.print(science_kit.getMagneticFieldY());
  Serial.print("\t");
  Serial.print(science_kit.getMagneticFieldZ());
  Serial.print("\t|| ");
  Serial.print(science_kit.getProximity());
  Serial.print("\t");
  Serial.print(science_kit.getRed());
  Serial.print("\t");
  Serial.print(science_kit.getGreen());
  Serial.print("\t");
  Serial.print(science_kit.getBlue());
  Serial.print("\t|| ");
  Serial.print(science_kit.getPhase1());
  Serial.print("\t");
  Serial.print(science_kit.getFrequency1());
  Serial.print("\t");
  Serial.print(science_kit.getRange1());
  Serial.print("\t| ");
  Serial.print(science_kit.getPhase2());
  Serial.print("\t");
  Serial.print(science_kit.getFrequency2());
  Serial.print("\t");
  Serial.print(science_kit.getRange2());
  Serial.print("\t|| ");
  Serial.print(science_kit.getInputA());
  Serial.print("\t");
  Serial.print(science_kit.getInputB());
  Serial.print("\t|| ");
  Serial.print(science_kit.getVoltage());
  Serial.print("\t");
  Serial.print(science_kit.getCurrent());
  Serial.print("\t| ");
  Serial.print(science_kit.getResistance());
  Serial.print("\t|| ");
  Serial.print("\n");
  delay(10);
}
