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

Result<CompleteSnapshot> SensorManager::pollSensors()
{
    // loop through array of the sensors and call their get data function?
    // they'll be on different threads so we gotta use mutexs
}