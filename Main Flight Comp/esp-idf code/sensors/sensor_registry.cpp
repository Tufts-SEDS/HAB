#include "sensor_registry.h"
#include "sensor_types.h"

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

Event SensorRegistry::initializeSensors()
{
    /* Clear initially for sanity sake */
    sensorCheckers_.clear();

    for (int i = 0; i < num_sensors_; ++i)
    {
        /* Initialize hardware */
        SensorStatus initStat = sensors_[i]->initialize();

        /* Prepare Checker Object for this sensor */
        SensorChecker currentChecker;
        currentChecker.type = sensors_[i]->type();

        /* Initialize current sensor , check SensorStatus Ok's value*/
        if (initStat != SensorStatus::Ok)
        {
            currentChecker.status = SensorStatus::ErrInit;
            sensorCheckers_.push_back(currentChecker);
            continue;
        }

        /* Create Thread Instance */
        BaseType_t xReturned = xTaskCreate(
            VSensor::read,
            sensors_[i]->name(),
            2048,
            static_cast<void *>(sensors[i]), /* Pass specific Sensor to Thread */
            5,
            NULL);

        /* Pass whether Thread Worked / Failed into status */
        currentChecker.status = (xReturned == pdPASS) ? SensorStatus::Ok : SensorStatus::ErrInit;

        /* Add result to vector */
        sensorCheckers_.push_back(currentChecker);
    }

    return Event<SensorChecker>(sensorCheckers_);
}