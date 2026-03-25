#pragma once

#include "fault_types.h"
#include "datahub.h"
#include "event_def.h"
#include "sensor_types.h"
#include "sensor_registry.h"

class SensorManager
{
public:
    SensorManager(SensorRegistry sensor_registry);
    ~SensorManager();

    Event<CompleteSnapshot> pollSensors();

private:
    uint8_t num_sensors_;
    SensorRegistry sensor_registry_;
    Datahub sensor_data_;
};