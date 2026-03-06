#include "sensor_registry.h"

uint8_t SensorRegistry::getNumSensors()
{
    return num_sensors_;
}

bool SensorRegistry::addSensor(VSensor *sensor)
{
    if (num_sensors_ >= MAX_SENSORS)
        return false;
    sensors_[num_sensors_++] = sensor;
    return true;
}