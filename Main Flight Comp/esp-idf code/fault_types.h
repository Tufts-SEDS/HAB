/*---------------------- Status / Fault Types ----------------------*/

// Defines what went wrong with the specific function being called
enum class Status : uint8_t
{
    Ok = 0,
    InvalidParam,
    OutOfRange, // shouldnt happen right?
    Timeout,
    DataBad,
    InternalError
};

// Defines where in our pipeline went wrong
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

// Consolidates all the errors with our system
struct FaultEvent
{
    Status status;
    FaultClass fclass;
    Severity severity;
    uint16_t source_id; // module/sensor ID
    bool is_valid;      // for the event manager to check if there's actually something wrong
};
