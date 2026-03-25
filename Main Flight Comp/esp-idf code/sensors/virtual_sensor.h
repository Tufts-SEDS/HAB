/*---------------------- Virtual Sensor ----------------------*/
/*

    Defines a universal Sensor Virtual Class that each sensor
    polymorphically utilizes. This is because we want to be
    able to pass this class as a parameter to fault manager
    for error handling

*/
/*------------------------------------------------------------*/

#pragma once

#include <inttypes.h>
#include "sensor_types.h"

class VSensor
{
public:
    virtual ~VSensor() = default;
    virtual std::string_view name() const = 0;
    virtual uint32_t id() const = 0;
    virtual SensorType type() const = 0;

    virtual SensorStatus initialize() = 0;
    virtual SensorSample read() = 0;

private:
    virtual void configure() = 0;
    virtual void applyCorrections(sensor_value *data) = 0;
};
