#pragma once

#include "fault_manager.h"

template <typename T>
struct Event
{
    FaultEvent faultevt;
    T value;
    static Result Ok(const T &v) noexcept { return Result{Status::Ok, v}; }
    static Result Err(Status e) noexcept { return Result{e, T{}}; }
    bool is_valid_fault() const noexcept { return faultevt.is_valid; }
};
