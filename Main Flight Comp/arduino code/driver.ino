/*
  GPT GENERATED - WILL EDIT LATER WHEN WE'VE GOT A GOOD IDEA ON
  WHAT WE'RE TESTING ON THE BREADBOARD

  ESP32 “constant polling” example for:
    - Adafruit BMP581 (I2C)
    - Adafruit BNO080/BNO085 (I2C via Adafruit_BNO08x)
    - Adafruit Ultimate GPS (UART)

  Libraries to install (Arduino Library Manager):
    - Adafruit BMP581
    - Adafruit BNO08x
    - Adafruit GPS Library
    - Adafruit Unified Sensor (often pulled in automatically)

  Wiring (typical):
    I2C:
      ESP32 SDA -> BMP581 SDA + BNO08x SDA
      ESP32 SCL -> BMP581 SCL + BNO08x SCL
      3V3/GND shared

    GPS (UART):
      GPS TX -> ESP32 RX (GPS_RX_PIN)
      GPS RX -> ESP32 TX (GPS_TX_PIN)
      3V3 + GND
*/

#include <Wire.h>
#include <Adafruit_BMP581.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>

// ---------- Pins / Buses ----------
static const int I2C_SDA_PIN = 21; // change if you’re using different ESP32 pins
static const int I2C_SCL_PIN = 22;

static const int GPS_RX_PIN = 16; // ESP32 receives on this pin (connect to GPS TX)
static const int GPS_TX_PIN = 17; // ESP32 transmits on this pin (connect to GPS RX)

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t GPS_BAUD = 9600;

// ---------- I2C Addresses (adjust if you changed ADR/SDO jumpers) ----------
static const uint8_t BMP581_ADDR = 0x47; // common default on Adafruit BMP581
static const uint8_t BNO08X_ADDR = 0x4B; // common default for BNO08x over I2C (sometimes 0x4A)

// ---------- Devices ----------
Adafruit_BMP581 bmp;
Adafruit_BNO08x bno;
sh2_SensorValue_t bnoValue;

HardwareSerial GPSSerial(1);
Adafruit_GPS GPS(&GPSSerial);

// ---------- Latest values (so you can use them elsewhere) ----------
struct
{
    float tempC = NAN;
    float pressPa = NAN;
    float altM = NAN;
} baro;

struct
{
    // Rotation Vector quaternion
    float qw = NAN, qx = NAN, qy = NAN, qz = NAN;

    // Calibrated gyro (rad/s)
    float gx = NAN, gy = NAN, gz = NAN;

    // Linear accel (m/s^2) or accel (m/s^2) depending on what you enable
    float ax = NAN, ay = NAN, az = NAN;
} imu;

struct
{
    bool hasFix = false;
    float lat = NAN, lon = NAN;
    float speedKnots = NAN;
    float angleDeg = NAN;
    float altitudeM = NAN;
    uint8_t sats = 0;
} gps;

// ---------- Timing ----------
uint32_t lastBaroMs = 0;
uint32_t lastPrintMs = 0;

static const uint32_t BARO_PERIOD_MS = 100;  // 10 Hz BMP581 reads
static const uint32_t PRINT_PERIOD_MS = 200; // 5 Hz printing

// If you want altitude from pressure, pick a reference sea-level pressure.
// Better: update this with a local QNH or calibrate at known altitude.
static const float SEA_LEVEL_HPA = 1013.25f;

// ---------- Helpers ----------
static float pressureToAltitudeM(float pressurePa, float seaLevelHpa)
{
    // Barometric formula (approx, ISA)
    float pressure_hPa = pressurePa / 100.0f;
    return 44330.0f * (1.0f - powf(pressure_hPa / seaLevelHpa, 0.1903f));
}

void setupBMP581()
{
    if (!bmp.begin_I2C(BMP581_ADDR, &Wire))
    {
        Serial.println("ERROR: BMP581 not found on I2C. Check wiring/address.");
        while (1)
            delay(10);
    }

    // Many Adafruit sensor libs work fine without extra config.
    // If your BMP581 lib exposes oversampling/ODR, you can set it here.
    Serial.println("BMP581 OK");
}

void setupBNO08x()
{
    if (!bno.begin_I2C(BNO08X_ADDR, &Wire))
    {
        Serial.println("ERROR: BNO08x not found on I2C. Check wiring/address.");
        while (1)
            delay(10);
    }

    // Enable whichever reports you want.
    // Interval is in microseconds in Adafruit_BNO08x.
    const uint32_t imuIntervalUs = 10000; // 100 Hz (10,000 us)

    // Rotation vector (good “orientation” quaternion)
    if (!bno.enableReport(SH2_ROTATION_VECTOR, imuIntervalUs))
    {
        Serial.println("WARN: couldn't enable ROTATION_VECTOR");
    }

    // Calibrated gyro
    if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, imuIntervalUs))
    {
        Serial.println("WARN: couldn't enable GYROSCOPE_CALIBRATED");
    }

    // Linear acceleration (gravity removed). Alternatively use SH2_ACCELEROMETER.
    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, imuIntervalUs))
    {
        Serial.println("WARN: couldn't enable LINEAR_ACCELERATION");
    }

    Serial.println("BNO08x OK");
}

void setupGPS()
{
    GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    GPS.begin(GPS_BAUD);

    // Output sentences: RMC + GGA is a common minimal set (time/date, fix, sats, altitude, etc.)
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

    // Update rate: 1 Hz is default-ish; 5 Hz is common if you want “faster” GPS.
    // NOTE: Higher rates cost more power and some modules have limits.
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);

    // Some modules also support higher baud; keep 9600 until everything works.
    Serial.println("GPS OK (listening on Serial1)");
}

void pollGPS()
{
    // Keep reading bytes as often as possible
    char c = GPS.read();

    // Optional: echo raw NMEA to Serial for debugging
    // if (c) Serial.write(c);

    // Parse when a full sentence is received
    if (GPS.newNMEAreceived())
    {
        if (!GPS.parse(GPS.lastNMEA()))
        {
            // parse failed; just keep going
            return;
        }

        // Update snapshot
        gps.hasFix = GPS.fix;
        gps.sats = GPS.satellites;

        if (GPS.fix)
        {
            // GPS.latitude / longitude are in ddmm.mmmm format + hemisphere;
            // Adafruit_GPS also gives decimal degrees via GPS.latitudeDegrees / longitudeDegrees
            gps.lat = GPS.latitudeDegrees;
            gps.lon = GPS.longitudeDegrees;
            gps.speedKnots = GPS.speed;
            gps.angleDeg = GPS.angle;
            gps.altitudeM = GPS.altitude;
        }
    }
}

void pollBMP581(uint32_t nowMs)
{
    if (nowMs - lastBaroMs < BARO_PERIOD_MS)
        return;
    lastBaroMs = nowMs;

    // The BMP581 library typically provides these:
    baro.tempC = bmp.readTemperature();
    baro.pressPa = bmp.readPressure();

    if (!isnan(baro.pressPa))
    {
        baro.altM = pressureToAltitudeM(baro.pressPa, SEA_LEVEL_HPA);
    }
}

void pollBNO08x()
{
    // Drain all available sensor events each loop
    while (bno.getSensorEvent(&bnoValue))
    {
        switch (bnoValue.sensorId)
        {
        case SH2_ROTATION_VECTOR:
        {
            imu.qw = bnoValue.un.rotationVector.real;
            imu.qx = bnoValue.un.rotationVector.i;
            imu.qy = bnoValue.un.rotationVector.j;
            imu.qz = bnoValue.un.rotationVector.k;
        }
        break;

        case SH2_GYROSCOPE_CALIBRATED:
        {
            imu.gx = bnoValue.un.gyroscope.x;
            imu.gy = bnoValue.un.gyroscope.y;
            imu.gz = bnoValue.un.gyroscope.z;
        }
        break;

        case SH2_LINEAR_ACCELERATION:
        {
            imu.ax = bnoValue.un.linearAcceleration.x;
            imu.ay = bnoValue.un.linearAcceleration.y;
            imu.az = bnoValue.un.linearAcceleration.z;
        }
        break;

        default:
            // ignore other reports you didn’t enable
            break;
        }
    }
}

void printStatus(uint32_t nowMs)
{
    if (nowMs - lastPrintMs < PRINT_PERIOD_MS)
        return;
    lastPrintMs = nowMs;

    Serial.println("-----");
    Serial.printf("BMP581: T=%.2f C  P=%.2f Pa  Alt=%.2f m\n",
                  baro.tempC, baro.pressPa, baro.altM);

    Serial.printf("BNO08x: q=[%.4f %.4f %.4f %.4f]  gyro=[%.4f %.4f %.4f]  linAcc=[%.3f %.3f %.3f]\n",
                  imu.qw, imu.qx, imu.qy, imu.qz,
                  imu.gx, imu.gy, imu.gz,
                  imu.ax, imu.ay, imu.az);

    if (gps.hasFix)
    {
        Serial.printf("GPS: FIX sats=%u lat=%.6f lon=%.6f alt=%.2f m spd=%.2f kn heading=%.2f deg\n",
                      gps.sats, gps.lat, gps.lon, gps.altitudeM, gps.speedKnots, gps.angleDeg);
    }
    else
    {
        Serial.printf("GPS: NO FIX sats=%u\n", gps.sats);
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(200);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // fast I2C

    Serial.println("\nStarting sensors...");

    setupBMP581();
    setupBNO08x();
    setupGPS();

    Serial.println("All initialized.");
}

void loop()
{
    const uint32_t nowMs = millis();

    // “Constant polling”:
    pollGPS();         // call as fast as possible
    pollBNO08x();      // drain all IMU events
    pollBMP581(nowMs); // periodic baro read

    printStatus(nowMs);
}
