#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include "MAX30105.h"

Adafruit_LSM6DSO32 imu;
MAX30105 ppg;

bool imuReady = false;
bool ppgReady = false;
bool debug = true;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  if (debug)
  {
    Serial.println("Starting I2C: SDA=GPIO8, SCL=GPIO9");
  }

  if (!Wire.begin(8, 9))
  {
    Serial.println("Failed to start I2C bus.");
    return;
  }

  Wire.setClock(100000);

  // -----------------------------
  // I2C scanner
  // -----------------------------
  if (debug)
  {
    uint8_t deviceCount = 0;

    for (uint8_t address = 1; address < 127; ++address)
    {
      Wire.beginTransmission(address);

      if (Wire.endTransmission() == 0)
      {
        Serial.printf("I2C device responded at 0x%02X\n", address);
        ++deviceCount;
      }
    }

    if (deviceCount == 0)
    {
      Serial.println("No I2C devices responded.");
    }
  }

  // -----------------------------
  // Initialize IMU
  // -----------------------------
  for (uint8_t address : {0x6A, 0x6B})
  {
    if (imu.begin_I2C(address, &Wire))
    {
      imuReady = true;

      if (debug)
      {
        Serial.printf(
            "LSM6DSO32 initialized at 0x%02X!\n",
            address);
      }

      break;
    }
  }

  if (!imuReady)
  {
    Serial.println(
        "LSM6DSO32 not detected at 0x6A or 0x6B.");
  }

  // -----------------------------
  // Initialize MAX30101
  // -----------------------------
  if (ppg.begin(Wire, I2C_SPEED_STANDARD))
  {
    ppgReady = true;

    if (debug)
    {
      Serial.println("MAX30101 initialized!");
    }

    byte ledBrightness = 30;
    byte sampleAverage = 4;
    byte ledMode = 3;

    int sampleRate = 100;
    int pulseWidth = 411;
    int adcRange = 4096;

    ppg.setup(
        ledBrightness,
        sampleAverage,
        ledMode,
        sampleRate,
        pulseWidth,
        adcRange);
  }
  else
  {
    Serial.println("MAX30101 not detected.");
  }

  if (debug)
  {
    Serial.println("Setup complete.");
  }
}

void loop()
{
  // -----------------------------
  // IMU
  // -----------------------------
  if (imuReady)
  {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    imu.getEvent(&accel, &gyro, &temp);

    if (debug)
    {
      Serial.print("Accel: ");
      Serial.print(accel.acceleration.x);
      Serial.print(", ");
      Serial.print(accel.acceleration.y);
      Serial.print(", ");
      Serial.print(accel.acceleration.z);

      Serial.print(" | Gyro: ");
      Serial.print(gyro.gyro.x);
      Serial.print(", ");
      Serial.print(gyro.gyro.y);
      Serial.print(", ");
      Serial.print(gyro.gyro.z);
    }
  }

  // -----------------------------
  // PPG
  // -----------------------------
  if (ppgReady)
  {
    long red = ppg.getRed();
    long ir = ppg.getIR();
    long green = ppg.getGreen();

    if (debug)
    {
      Serial.print(" | PPG Red: ");
      Serial.print(red);

      Serial.print(" IR: ");
      Serial.print(ir);

      Serial.print(" Green: ");
      Serial.print(green);
    }
  }

  if (debug)
  {
    Serial.println();
  }

  delay(20);
}
