#pragma once

#include "sensor_manager.h"
#include "fault_manager.h"

template <typename T>
struct Event
{
    FaultEvent st;
    T value;

    static Result Ok(const T &v) noexcept { return Result{Status::Ok, v}; }
    static Result Err(Status e) noexcept { return Result{e, T{}}; }
    bool is_ok() const noexcept { return st == Status::Ok; }
};

class EventManager
{
public:
private:
};