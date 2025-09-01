#include "services/health.h"

namespace svc {

static HealthResiduals s_last = {};

void health_init() {}
void health_update() {}
bool health_get(HealthResiduals &out) { out = s_last; return true; }

} // namespace svc

