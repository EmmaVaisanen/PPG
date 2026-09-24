#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include "MAX30105.h"

// --------------------------------------------------
// Configuration
// --------------------------------------------------

constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;
constexpr uint32_t I2C_FREQUENCY = 100000;

constexpr bool DEBUG = true;

constexpr byte PPG_LED_BRIGHTNESS = 30;
constexpr byte PPG_SAMPLE_AVERAGE = 4;
constexpr byte PPG_LED_MODE = 3;
constexpr int PPG_SAMPLE_RATE = 100;
constexpr int PPG_PULSE_WIDTH = 411;
constexpr int PPG_ADC_RANGE = 4096;

// --------------------------------------------------
// Sensors
// --------------------------------------------------

Adafruit_LSM6DSO32 imu;
MAX30105 ppg;

bool imuReady = false;
bool ppgReady = false;

// --------------------------------------------------
// Measurement structure
// --------------------------------------------------

struct SensorData
{
    uint32_t timeMs;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    long red;
    long ir;
    long green;
};

// --------------------------------------------------
// Function declarations
// --------------------------------------------------

void scanI2C();
bool initializeIMU();
bool initializePPG();

SensorData readSensors();

void printDebug(const SensorData &data);
void printCSV(const SensorData &data);

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (DEBUG)
    {
        Serial.println("Starting system...");
        Serial.println("I2C: SDA=GPIO8, SCL=GPIO9");
    }

    if (!Wire.begin(SDA_PIN, SCL_PIN))
    {
        Serial.println("Failed to start I2C bus.");
        return;
    }

    Wire.setClock(I2C_FREQUENCY);

    if (DEBUG)
    {
        scanI2C();
    }

    imuReady = initializeIMU();
    ppgReady = initializePPG();

    if (DEBUG)
    {
        Serial.println();
        Serial.println("System status:");
        Serial.printf("IMU: %s\n", imuReady ? "READY" : "NOT FOUND");
        Serial.printf("PPG: %s\n", ppgReady ? "READY" : "NOT FOUND");
        Serial.println("Setup complete.");
    }
    else
    {
        // Header for Python / CSV
        Serial.println(
            "time_ms,red,ir,green,ax,ay,az,gx,gy,gz"
        );
    }
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
    SensorData data = readSensors();

    if (DEBUG)
    {
        printDebug(data);
    }
    else
    {
        printCSV(data);
    }

    delay(20);
}

// --------------------------------------------------
// I2C scanner
// --------------------------------------------------

void scanI2C()
{
    Serial.println("Scanning I2C bus...");

    uint8_t deviceCount = 0;

    for (uint8_t address = 1; address < 127; ++address)
    {
        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0)
        {
            Serial.printf(
                "I2C device found at 0x%02X\n",
                address
            );

            ++deviceCount;
        }
    }

    if (deviceCount == 0)
    {
        Serial.println("No I2C devices found.");
    }
}

// --------------------------------------------------
// IMU initialization
// --------------------------------------------------

bool initializeIMU()
{
    for (uint8_t address : {0x6A, 0x6B})
    {
        if (imu.begin_I2C(address, &Wire))
        {
            if (DEBUG)
            {
                Serial.printf(
                    "LSM6DSO32 initialized at 0x%02X\n",
                    address
                );
            }

            return true;
        }
    }

    Serial.println(
        "LSM6DSO32 not detected at 0x6A or 0x6B."
    );

    return false;
}

// --------------------------------------------------
// PPG initialization
// --------------------------------------------------

bool initializePPG()
{
    if (!ppg.begin(Wire, I2C_SPEED_STANDARD))
    {
        Serial.println("MAX30101 not detected.");
        return false;
    }

    ppg.setup(
        PPG_LED_BRIGHTNESS,
        PPG_SAMPLE_AVERAGE,
        PPG_LED_MODE,
        PPG_SAMPLE_RATE,
        PPG_PULSE_WIDTH,
        PPG_ADC_RANGE
    );

    if (DEBUG)
    {
        Serial.println("MAX30101 initialized.");
    }

    return true;
}

// --------------------------------------------------
// Read sensors
// --------------------------------------------------

SensorData readSensors()
{
    SensorData data{};

    data.timeMs = millis();

    // IMU
    if (imuReady)
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        imu.getEvent(&accel, &gyro, &temp);

        data.ax = accel.acceleration.x;
        data.ay = accel.acceleration.y;
        data.az = accel.acceleration.z;

        data.gx = gyro.gyro.x;
        data.gy = gyro.gyro.y;
        data.gz = gyro.gyro.z;
    }

    // PPG
    if (ppgReady)
    {
        data.red = ppg.getRed();
        data.ir = ppg.getIR();
        data.green = ppg.getGreen();
    }

    return data;
}

// --------------------------------------------------
// Human-readable debug output
// --------------------------------------------------

void printDebug(const SensorData &data)
{
    Serial.printf(
        "t=%lu | "
        "Accel: %.2f, %.2f, %.2f | "
        "Gyro: %.2f, %.2f, %.2f | "
        "PPG: R=%ld IR=%ld G=%ld\n",

        data.timeMs,

        data.ax,
        data.ay,
        data.az,

        data.gx,
        data.gy,
        data.gz,

        data.red,
        data.ir,
        data.green
    );
}

// --------------------------------------------------
// Machine-readable output for Python
// --------------------------------------------------

void printCSV(const SensorData &data)
{
    Serial.printf(
        "%lu,%ld,%ld,%ld,"
        "%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f\n",

        data.timeMs,

        data.red,
        data.ir,
        data.green,

        data.ax,
        data.ay,
        data.az,

        data.gx,
        data.gy,
        data.gz
    );
}