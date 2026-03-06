#include <inttypes.h>
#include <string_view>
#include "sensor_types.h"

class VSensor
{
public:
    virtual ~VSensor() = default;
    virtual std::string_view name() const = 0;
    virtual uint32_t id() const = 0;
    virtual SensorType type() const = 0;

    virtual sensor_status initialize() = 0;
    virtual sensor_reading read() = 0;

private:
    virtual void configure() = 0;
    virtual void applyCorrections(sensor_value *data) = 0;
};