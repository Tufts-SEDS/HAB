#pragma once

#include "fault_manager.h"

template <typename T>
struct Event
{
    FaultEvent faultevt;
    T value;
    bool is_valid_fault() const noexcept { return faultevt.is_valid; }
};
