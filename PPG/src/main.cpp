#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>

Adafruit_LSM6DSO32 imu;
bool imuReady = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting I2C: SDA=GPIO8, SCL=GPIO9");
  if (!Wire.begin(8, 9)) {
    Serial.println("Failed to start I2C bus.");
    return;
  }
  Wire.setClock(100000);

  uint8_t deviceCount = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C device responded at 0x%02X\n", address);
      ++deviceCount;
    }
  }
  if (deviceCount == 0) {
    Serial.println("No I2C devices responded. Check power, common GND, SDA and SCL.");
  }

  for (uint8_t address : {0x6A, 0x6B}) {
    if (imu.begin_I2C(address, &Wire)) {
      imuReady = true;
      Serial.printf("LSM6DSO32 initialized at 0x%02X!\n", address);
      break;
    }
  }
  if (!imuReady) {
    Serial.println("LSM6DSO32 not detected at 0x6A or 0x6B. Check wiring and sensor model, then reset.");
  }
}

void loop() {
  if (!imuReady) {
    Serial.println("IMU unavailable. Check startup diagnostics; reset to scan again.");
    delay(3000);
    return;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  imu.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" Z: ");
  Serial.println(accel.acceleration.z);

  Serial.print(" | G: ");
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);

  Serial.println();

  delay(100);
}
