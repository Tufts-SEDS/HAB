#include <atomic>

enum class SensorType : uint8_t
{
    Temperature,
    BMP,
    GPS,
    IMU,
    Accelerometer
};

enum class SensorStatus : uint8_t
{
    Ok,
    Err,
    ErrBadHealth,
    ErrInit,
    ErrRead,
    ErrCalib,
    ErrTask
};

struct sensor_data_snapshot
{
    double baro_altitude;
    float baro_temp;
    float baro_pressure;

    double gps_lat;
    double gps_lon;
    double gps_alt;
    float gps_speed;
    float gps_cog;
    float gps_mag_vari;
    int gps_num_sats;
    int gps_fix_status;
    int gps_year;
    int gps_month;
    int gps_day;
    int gps_hour;
    int gps_minute;
    int gps_second;
    bool gps_fix_valid;

    float imu_accel_x;
    float imu_accel_y;
    float imu_accel_z;
    float imu_gyro_x;
    float imu_gyro_y;
    float imu_gyro_z;
    float imu_mag_x;
    float imu_mag_y;
    float imu_mag_z;

    float hg_accel_x;
    float hg_accel_y;
    float hg_accel_z;

    float temp_temp_c;
    unsigned long timestamp;
};

struct EkfSnapshot
{
    float yaw, pitch, roll, vz, altitude, az;
};

struct CompleteSnapshot
{
    sensor_data_snapshot sensors;
    EkfSnapshot ekf;
};

struct sensor_value
{
    sensor_type type;
    union
    {
        struct
        {
            double altitude;
            float temp, pressure;
        } bmp;
        struct
        {
            double lat, lon, alt;
            float speed, cog, mag_vari;
            int num_sats, fix_status;
            int year, month, day, hour, minute, second;
            bool fix_valid;
        } gps;
        struct
        {
            float accel[3], gyro[3], mag[3];
            float temp;
        } imu;
        struct
        {
            float accel[3];
        } accelerometerHG;
        struct
        {
            float temp_c;
        } temp;
    } data;
};