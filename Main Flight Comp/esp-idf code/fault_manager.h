#include <cstdint>
#include <cstddef>
#include <climits>

#include "esp_log.h"

/*---------------------- Status / Fault Types ----------------------*/

enum class Status : uint8_t
{
    Ok = 0,
    InvalidParam,
    OutOfRange, // shouldnt happen right?
    Timeout,
    DataBad,
    InternalError
};

enum class FaultClass : uint8_t
{
    None = 0,
    Sensor,
    Computation,
    Configuration,
    Detector,
    DataStorage
};

enum class Severity : uint8_t
{
    Info = 0,
    Minor,
    Major,
    Critical
};

struct FaultEvent
{
    FaultClass cls;
    Severity sev;
    Status st;
    uint16_t source_id; // module/sensor ID
};

/*---------------------- Fault Manager ----------------------*/
/*
 * Bounded logging: counters + last fault captured.
 * You can extend to a small ring buffer if you want traceability.
 */

class FaultManager
{
public:
    static void Report(const FaultEvent &evt) noexcept
    {
        last_ = evt;

        switch (evt.sev)
        {
        case Severity::Critical:
            ++critical_count_;
            EnterSafeState();
            break;
        case Severity::Major:
            ++major_count_;
            break;
        case Severity::Minor:
            ++minor_count_;
            break;
        case Severity::Info:
        default:
            break;
        }

        // debug log, rewrite to make write to local storage
        ESP_LOGW(TAG, "FAULT cls=%u sev=%u st=%u src=%u detail=%lu",
                 static_cast<unsigned>(evt.cls),
                 static_cast<unsigned>(evt.sev),
                 static_cast<unsigned>(evt.st),
                 static_cast<unsigned>(evt.source_id),
                 static_cast<unsigned long>(evt.detail));
    }

    static FaultEvent Last() noexcept { return last_; }
    static uint32_t CriticalCount() noexcept { return critical_count_; }
    static uint32_t MajorCount() noexcept { return major_count_; }
    static uint32_t MinorCount() noexcept { return minor_count_; }

private:
    static void EnterSafeState() noexcept
    {
        // Put the system into a safe configuration:
        // - raise a discrete or set a global "safe state" flag
        // - wtf else do i do here
        safe_state_ = true;
        ESP_LOGE(TAG, "ENTERING SAFE STATE");
    }

    static bool safe_state_ = false;

    static uint32_t critical_count_ = 0;
    static uint32_t major_count_ = 0;
    static uint32_t minor_count_ = 0;

    static FaultEvent last_{
        FaultClass::None, Severity::Info, Status::Ok, 0U, 0U};
};

/* Helper: report and return */
static inline Status ReportAndReturn(
    FaultClass cls, Severity sev, Status st, uint16_t src, uint32_t detail) noexcept
{
    FaultManager::Report(FaultEvent{cls, sev, st, src, detail});
    return st;
}

// lightweight "Expected" type
/*
 * READ THIS: AVOID EXCEPTIONS AT ALL COSTS; return either Status::Ok + value or error status
 * lowk might be dumb to use this as standin for std expected but id rather something smaller and not
 * from the std
 */

template <typename T>
struct Result
{
    Status st;
    T value;

    static Result Ok(const T &v) noexcept { return Result{Status::Ok, v}; }
    static Result Err(Status e) noexcept { return Result{e, T{}}; }
    bool is_ok() const noexcept { return st == Status::Ok; }
};

// had chet generate some patterns/use cases for this, this will prob be integrated into
// a larger event manager class that will be the thing to unpack the Result struct

struct SensorSample
{
    int32_t raw;
    bool valid;
};

static Result<SensorSample> SensorRead(uint16_t sensor_id) noexcept
{
    // Replace with real IO (I2C/SPI/ADC). Demonstrate the pattern only.

    bool timeout = false;
    bool data_bad = false;
    int32_t raw_value = 123;

    if (timeout)
    {
        (void)ReportAndReturn(FaultClass::Sensor, Severity::Major,
                              Status::Timeout, sensor_id, 0U);
        return Result<SensorSample>::Err(Status::Timeout);
    }

    if (data_bad)
    {
        (void)ReportAndReturn(FaultClass::Sensor, Severity::Major,
                              Status::DataBad, sensor_id, 0U);
        return Result<SensorSample>::Err(Status::DataBad);
    }

    SensorSample s{raw_value, true};
    return Result<SensorSample>::Ok(s);
}

/*---------------------- Computation with guards ----------------------*/

static Result<int32_t> ConvertToEngineeringUnits(const SensorSample &in) noexcept
{
    if (!in.valid)
    {
        (void)ReportAndReturn(FaultClass::Sensor, Severity::Major,
                              Status::DataBad, 0U, static_cast<uint32_t>(in.raw));
        return Result<int32_t>::Err(Status::DataBad);
    }

    // Plausibility checks (tune for your sensor)
    if (in.raw < -2000 || in.raw > 2000)
    {
        (void)ReportAndReturn(FaultClass::Sensor, Severity::Major,
                              Status::OutOfRange, 0U, static_cast<uint32_t>(in.raw));
        return Result<int32_t>::Err(Status::OutOfRange);
    }

    // Example scaling with overflow guard: eng = raw * 10
    constexpr int32_t scale = 10;

    if ((in.raw > 0 && in.raw > (INT32_MAX / scale)) ||
        (in.raw < 0 && in.raw < (INT32_MIN / scale)))
    {
        (void)ReportAndReturn(FaultClass::Computation, Severity::Critical,
                              Status::InternalError, 0U, static_cast<uint32_t>(in.raw));
        return Result<int32_t>::Err(Status::InternalError);
    }

    return Result<int32_t>::Ok(in.raw * scale);
}

/*---------------------- Application step w/ safe fallback ----------------------*/

struct AppState
{
    int32_t last_good_eng = 0;
    bool degraded_mode = false;
};

static Status AppStep(AppState *state) noexcept
{
    if (state == nullptr)
    {
        return ReportAndReturn(FaultClass::Configuration, Severity::Critical,
                               Status::InvalidParam, 0U, 0U);
    }

    // Boundary read
    auto s = SensorRead(/*sensor_id=*/1U);
    if (!s.is_ok())
    {
        // deterministic fallback
        state->degraded_mode = true;
        return s.st; // already reported
    }

    // Conversion
    auto eng = ConvertToEngineeringUnits(s.value);
    if (!eng.is_ok())
    {
        state->degraded_mode = true;
        return eng.st; // already reported
    }

    // Normal path
    state->last_good_eng = eng.value;
    state->degraded_mode = false;
    return Status::Ok;
}

/*---------------------- Example FreeRTOS task loop ----------------------*/
// In ESP-IDF you'd call this from a task function.
static void RunLoop()
{
    AppState st{};

    while (true)
    {
        (void)AppStep(&st);

        // vTaskDelay(pdMS_TO_TICKS(10)); // typical
        // kept out to keep snippet freestanding
    }
}