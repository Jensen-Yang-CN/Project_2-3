#ifndef SYSTEMLOGHELPERS_H
#define SYSTEMLOGHELPERS_H

#include "common_types_extended.h"
#include "bridge_common_types.h"

#include <QJsonArray>
#include <vector>

namespace system_log {

QJsonArray berthCornersJson(const usv::Berth &berth);

void logBerthResultIfChanged(const usv::BerthMeasureResult &res);

void logBridgeResultIfChanged(const usv::BridgeMeasureResult &result);

}  // namespace system_log

#endif  // SYSTEMLOGHELPERS_H
