#include "event_manager.h"

EventManager::EventManager()
{
    num_sensors_ = num_sensors;
    sensor_registry_ = SensorRegistry();
}

~EventManager::EventManager()
{
    // TODO:
    // should prob call some shit from the registry to remove the sensors
    //      from the i2c/spi bus or something like that
    // should also kill the dev handle for each sensor
}