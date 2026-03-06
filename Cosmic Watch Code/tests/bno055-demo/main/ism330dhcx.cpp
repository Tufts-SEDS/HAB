#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"

#include "ism330dhcx.h"

// Config registers
#define REG_WHO_AM_I (0x0F)
#define REG_CTRL1_XL (0x10) // accelerometer control register 1
#define REG_CTRL2_G (0x11)  // gyroscope control register 2
#define REG_CTRL3_C (0x12)

// Gyro registers
#define REG_OUTX_L_G (0x22)
#define REG_OUTX_H_G (0x23)
#define REG_OUTY_L_G (0x24)
#define REG_OUTY_H_G (0x25)
#define REG_OUTZ_L_G (0x26)
#define REG_OUTZ_H_G (0x27)

// Accel registers
#define REG_OUTX_L_A (0x28)
#define REG_OUTX_H_A (0x29)
#define REG_OUTY_L_A (0x2A)
#define REG_OUTY_H_A (0x2B)
#define REG_OUTZ_L_A (0x2C)
#define REG_OUTZ_H_A (0x2D)

// Modes
#define MODE_NDOF (0x0C)

// Preset register values
#define VAL_CTRL1_XL (0x80)
#define VAL_CTRL2_G (0x88)
#define VAL_CTRL3_C_SW_RESET (0x03)

#define DELAY_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#define MASTER_TRANSMIT_TIMEOUT (50)

void ism330dhcx_write(i2c_master_dev_handle_t sensor,
                      uint8_t const *data_buf, const uint8_t data_len)
{
    ESP_ERROR_CHECK(i2c_master_transmit(sensor, data_buf, data_len, MASTER_TRANSMIT_TIMEOUT));
}

void ism330dhcx_read(i2c_master_dev_handle_t sensor, const uint8_t reg_start_addr, uint8_t *rx, uint8_t rx_size)
{
    const uint8_t tx[] = {reg_start_addr};

    ESP_ERROR_CHECK(i2c_master_transmit_receive(sensor, tx, sizeof(tx), rx, rx_size, MASTER_TRANSMIT_TIMEOUT));
}

ISM330DHCX::ISM330DHCX(ISM330DHCXConfig &config)
    : ism330dhcx_addr_len(config.addr_len), ism330dhcx_addr(config.address), ism330dhcx_dev_handle(nullptr)
{
    // making the i2c device controlled by master
    i2c_device_config_t ism330dhcx_cfg = {
        .dev_addr_length = config.addr_len,
        .device_address = config.address,
        .scl_speed_hz = config.scl_clk_speed};

    // grab the i2c bus given port and connect that shii
    i2c_master_bus_handle_t bus_handle;
    // TODO: add logging to the statements below
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(config.port, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &ism330dhcx_cfg, &ism330dhcx_dev_handle));
}

ISM330DHCX::~ISM330DHCX()
{
    i2c_master_bus_rm_device(ism330dhcx_dev_handle);
}

bool ISM330DHCX::configure()
{
    DELAY_MS(650);
    uint8_t tmp[1] = {0};
    ism330dhcx_read(ism330dhcx_dev_handle, 0x00, tmp, sizeof(tmp));
    if (tmp[0] != ISM330DHCX_H_WHO_AM_I_VAL)
        return false;

    return true;
}

ism330dhcx_raw_gyro_value_t ISM330DHCX::getGyro()
{
    uint8_t data_rd[6] = {0};
    ism330dhcx_read(ism330dhcx_dev_handle, REG_OUTX_L_G, data_rd, sizeof(data_rd));

    ism330dhcx_raw_gyro_value_t out;

    out.raw_gyro_x = (int16_t)((data_rd[1] << 8) + (data_rd[0]));
    out.raw_gyro_y = (int16_t)((data_rd[3] << 8) + (data_rd[2]));
    out.raw_gyro_z = (int16_t)((data_rd[5] << 8) + (data_rd[6]));

    return out;
}

ism330dhcx_raw_accel_value_t ISM330DHCX::getAccel()
{
    uint8_t data_rd[6] = {0};
    ism330dhcx_read(ism330dhcx_dev_handle, REG_OUTX_L_A, data_rd, sizeof(data_rd));

    ism330dhcx_raw_accel_value_t out;

    out.raw_gyro_x = (int16_t)((data_rd[1] << 8) + (data_rd[0]));
    out.raw_gyro_y = (int16_t)((data_rd[3] << 8) + (data_rd[2]));
    out.raw_gyro_z = (int16_t)((data_rd[5] << 8) + (data_rd[6]));

    return out;
}