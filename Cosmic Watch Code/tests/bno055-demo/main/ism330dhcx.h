#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#define ISM330DHCX_H_I2C_ADDRESS 0x29
#define ISM330DHCX_H_WHO_AM_I_VAL 0x6B

typedef enum
{
    ACCEL_FS_2G = 0,  // Accelerometer full scale range is +/- 2g
    ACCEL_FS_4G = 1,  // Accelerometer full scale range is +/- 4g
    ACCEL_FS_8G = 2,  // Accelerometer full scale range is +/- 8g
    ACCEL_FS_16G = 3, // Accelerometer full scale range is +/- 16g
} accel_fs;

typedef enum
{
    GYRO_FS_250DPS = 0,  // Gyroscope full scale range is +/- 250 degree per sencond
    GYRO_FS_500DPS = 1,  // Gyroscope full scale range is +/- 500 degree per sencond
    GYRO_FS_1000DPS = 2, // Gyroscope full scale range is +/- 1000 degree per sencond
    GYRO_FS_2000DPS = 3, // Gyroscope full scale range is +/- 2000 degree per sencond
} gyro_fs;

struct ISM330DHCXConfig
{
    accel_fs accel_range;
    gyro_fs gyro_range;
    i2c_port_num_t port;
    i2c_addr_bit_len_t addr_len;

    bool enable_accel_dlpf;
    bool enable_gyro_dlpf;
    uint32_t scl_clk_speed;
    uint16_t ism330dhcx_address;
};

typedef struct
{
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} ism330dhcx_raw_gyro_value_t;

typedef struct
{
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
} ism330dhcx_raw_accel_value_t;

class ISM330DHCX
{
public:
    ISM330DHCX(ISM330DHCXConfig &config);
    ~ISM330DHCX();

    bool configure();
    ism330dhcx_raw_accel_value_t getAccel();
    ism330dhcx_raw_gyro_value_t getGyro();
    // void wakeup();
    // void sleep();
    // void reset();

private:
    // void setGyroFS(bno055_gyro_fs_t gyro_fs);
    // void setGyroSensitivity();
    // void getRawGyro();

    // void setAccelFS(bno055_accel_fs_t accel_fs);
    // void setAccelSensitivity();
    // void getRawAccel();

    // void enableDLPF(bool enable);
    // void setAccelDLPF(bno055_dlpf_t dlpf_accel);
    // void setGyroDLPF(bno055_dlpf_t dlpf_gyro);

    ism330dhcx_raw_accel_value_t curr_raw_accel_vals;
    ism330dhcx_raw_gyro_value_t curr_raw_gyro_vals;

    float gyro_sensitivity;
    float accel_sensitivity;

    i2c_addr_bit_len_t ism330dhcx_addr_len;
    uint16_t ism330dhcx_addr;
    i2c_master_dev_handle_t ism330dhcx_dev_handle;
};

#endif