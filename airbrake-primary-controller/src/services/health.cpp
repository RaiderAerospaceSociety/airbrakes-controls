// Simple health service placeholder with flags snapshot
#include <Arduino.h>
#include "services/health.h"
#include "sensor_bmp1.h"
#include "sensor_imu1.h"
#include "services/fusion.h"

// Optionally bring in thresholds later via dedicated config header
#include "config/fc_config.h"

namespace svc
{

    static HealthResiduals s_last = {};
    static HealthSnapshot s_hs = {};

    void health_init() {}
    void health_update() {}
    bool health_get(HealthResiduals &out)
    {
        out = s_last;
        return true;
    }
    bool health_get_flags(HealthSnapshot &out)
    {
        out = s_hs;
        return true;
    }

} // namespace svc
