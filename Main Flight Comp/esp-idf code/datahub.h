#pragma once

#include "sensor_types.h"
#include "atomic_tuple.h"
#include "sensor_manager.h"
#include "fault_manager.h"

class Datahub
{
public:
    Datahub();
    ~Datahub();

private:
    // this is just gonna be a shit ton of atomics tbh
    // we have two copies of each variable bc of threading, we have one active
    //      atomic and one shadow atomic at any given time. while one is updating,
    //      the other will be the "front facing" data field that other modules
    //      would pull data from

    AtomicTuple<double> baro_altitude;
    AtomicTuple<float> baro_temp;
    AtomicTuple<float> baro_pressure;

    AtomicTuple<double> gps_lat{0.0};
    AtomicTuple<double> gps_lon{0.0};
    AtomicTuple<double> gps_alt{0.0};
    AtomicTuple<float> gps_speed;
    AtomicTuple<float> gps_cog;
    AtomicTuple<float> gps_mag_vari;
    AtomicTuple<int> gps_num_sats{0};
    AtomicTuple<int> gps_fix_status{0};
    AtomicTuple<int> gps_year{0};
    AtomicTuple<int> gps_month{0};
    AtomicTuple<int> gps_day{0};
    AtomicTuple<int> gps_hour{0};
    AtomicTuple<int> gps_minute{0};
    AtomicTuple<int> gps_second{0};
    AtomicTuple<bool> gps_fix_valid{false};

    AtomicTuple<float> imu_accel_x;
    AtomicTuple<float> imu_accel_y;
    AtomicTuple<float> imu_accel_z;
    AtomicTuple<float> imu_gyro_x;
    AtomicTuple<float> imu_gyro_y;
    AtomicTuple<float> imu_gyro_z;
    AtomicTuple<float> imu_mag_x;
    AtomicTuple<float> imu_mag_y;
    AtomicTuple<float> imu_mag_z;
    AtomicTuple<float> imu_temp;

    AtomicTuple<float> hg_accel_x;
    AtomicTuple<float> hg_accel_y;
    AtomicTuple<float> hg_accel_z;

    AtomicTuple<unsigned long> timestamp;
};