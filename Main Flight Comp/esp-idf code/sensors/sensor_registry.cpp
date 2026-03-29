#include "fault_types.h"
#include "sensor_registry.h"
#include "sensor_types.h"

uint8_t SensorRegistry::getNumSensors()
{
    return num_sensors_;
}

Event<bool> SensorRegistry::addSensor(VSensor *sensor)
{
    if (num_sensors_ >= MAX_SENSORS)
    {
        FaultEvent fault{Status::OutOfRange, FaultClass::Configuration, Severity::Major, 0U, true};
        Event<bool> evt{fault, false};
        return evt;
    }
    sensors_[num_sensors_++] = sensor;
    FaultEvent fault{0U, 0U, 0U, 0U, false};
    Event<bool> evt{fault, true};
    return evt;
}

Event<std::vector<SensorChecker>> SensorRegistry::initializeSensors()
{
    /* Clear initially for sanity sake */
    sensorCheckers_.clear();

    for (int i = 0; i < num_sensors_; ++i)
    {
        /* Initialize hardware */
        SensorStatus initStat = sensors_[i]->initialize();

        /* Prepare Checker struct for this sensor */
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
            sensors_[i]->vreadTask, // TODO: this might be a bug
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

    FaultEvent fault{0U, 0U, 0U, 0U, false};
    Event<std::vector<SensorChecker>> evt{fault, sensorCheckers_};
    return evt;
}