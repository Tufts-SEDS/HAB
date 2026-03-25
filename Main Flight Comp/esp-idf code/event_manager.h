#pragma once

#include "sensor_manager.h"
#include "fault_manager.h"

class EventManager
{
public:
    EventManager();
    ~EventManager();

private:
    FaultManager fault_manager_;
};