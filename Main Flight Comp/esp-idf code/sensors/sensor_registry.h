/*---------------------- Sensor Registry ---------------------*/
/*

    Defines a registry that consolidates the initialization of
    all the sensors

*/
/*------------------------------------------------------------*/
#include <vector>

#pragma once

#define MAX_SENSORS 8

#include <inttypes.h>
#include <string_view>
#include "sensor_types.h"
#include "virtual_sensor.h"
#include "event_def.h"

class SensorRegistry
{
public:
    SensorRegistry();
    ~SensorRegistry();

    bool addSensor(VSensor *sensor);
    void initializeSensors();
    uint8_t getNumSensors();

private:
    VSensor *sensors_[MAX_SENSORS];
    uint8_t num_sensors_ = 0;
    std::vector<SensorChecker> sensorCheckers_;
    void configure() = 0;
    void applyCorrections(sensor_value *data) = 0;
    Event event;
};
