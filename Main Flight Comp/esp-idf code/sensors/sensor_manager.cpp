#include "sensor_manager.h"

SensorManager::SensorManager(SensorRegistry sensor_registry)
{

    sensor_registry_ = sensor_registry;
    num_sensors_ = sensor_registry.getNumSensors();
}

~SensorManager::SensorManager()
{
    // TODO:
    // should prob call some shit from the registry to remove the sensors
    //      from the i2c/spi bus or something like that
    // should also kill the dev handle
}

Event<CompleteSnapshot> SensorManager::pollSensors()
{
    SensorDataSnapshot snapshot;
    loadSnapshot(&snapshort);

    CompleteSnapshot completeSnapshot;
    completeSnapshot.data = snapshot;

    FaultEvent fault{0U, 0U, 0U, 0U, false};
    Event<CompleteSnapshot> evt{fault, completeSnapshot};
    return evt;
    // loop through array of the sensors and call their get data function?
    // they'll be on different threads so we gotta use mutexs
}

/* Helper Function that goes from DataHub Object -> AtomicTuple -> Load */
void SensorManager::loadSnapshot(SensorDataSnapshot *snapshot)
{
    snapshot->baro_altitude = sensor_data_.baro_altitude.load();
    snapshot->baro_temp = sensor_data_.baro_temp.load();
    snapshot->baro_pressure = sensor_data_.baro_temp.load();

    snapshot->gps_lat = sensor_data_.gps_lat.load();
    snapshot->gps_lon = sensor_data_.gps_lon.load();
    snapshot->gps_alt = sensor_data_.gps_alt.load();
    snapshot->gps_speed = sensor_data_.gps_speed.load();
    snapshot->gps_cog = sensor_data_.gps_cog.load();
    snapshot->gps_mag_vari = sensor_data_.gps_mag_vari.load();
    snapshot->gps_num_sats = sensor_data_.gps_num_sats.load();
    snapshot->gps_fix_status = sensor_data_.gps_fix_status.load();
    snapshot->gps_year = sensor_data_.gps_year.load();
    snapshot->gps_month = sensor_data_.gps_month.load();
    snapshot->gps_day = sensor_data_.gps_day.load();
    snapshot->gps_hour = sensor_data_.gps_hour.load();
    snapshot->gps_minute = sensor_data_.gps_minute.load();
    snapshot->gps_second = sensor_data_.gps_second.load();
    snapshot->gps_fix_valid = sensor_data_.gps_fix_valid.load();

    snapshot->imu_accel_x = sensor_data_.imu_accel_x.load();
    snapshot->imu_accel_y = sensor_data_.imu_accel_y.load();
    snapshot->imu_accel_z = sensor_data_.imu_accel_z.load();
    snapshot->imu_gyro_x = sensor_data_.imu_gyro_x.load();
    snapshot->imu_gyro_y = sensor_data_.imu_gyro_y.load();
    snapshot->imu_gyro_z = sensor_data_.imu_gyro_z.load();
    snapshot->imu_mag_x = sensor_data_.imu_mag_x.load();
    snapshot->imu_mag_y = sensor_data_.imu_mag_y.load();
    snapshot->imu_mag_z = sensor_data_.imu_mag_z.load();
    snapshot->imu_temp = sensor_data_.imu_temp.load();

    snapshot->hg_accel_x = sensor_data_.hg_accel_x.load();
    snapshot->hg_accel_y = sensor_data_.hg_accel_y.load();
    snapshot->hg_accel_z = sensor_data_.hg_accel_z.load();

    snapshot->timestamp = sensor_data_.timestamp.load();
}