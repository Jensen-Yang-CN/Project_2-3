#include "SlamMapBerthStore.h"

#include <cmath>
#include <iostream>

namespace {

bool close(double left, double right)
{
    return std::abs(left - right) < 1.0e-6;
}

} // namespace

int main()
{
    SlamMapBerthStore store;
    usv::BerthMeasureResult result;
    result.timestamp = 1.0;
    usv::Berth berth;
    berth.cx = 10.0;
    berth.cy = 20.0;
    berth.w = 8.0;
    berth.l = 18.0;
    berth.kind = usv::BerthKind::UShape;
    berth.opening_edge = 2;
    result.expanded_berths.push_back(berth);
    store.observe(result);

    result.timestamp = 2.0;
    result.expanded_berths[0].cx = 10.4;
    result.expanded_berths[0].cy = 20.2;
    store.observe(result);
    if (store.records().size() != 1
        || store.records().front().observation_count != 2
        || !close(store.records().front().x, 10.2)
        || !close(store.records().front().y, 20.1)) {
        std::cerr << "persistent berth merge failed\n";
        return 1;
    }

    result.timestamp = 3.0;
    result.expanded_berths[0].cx = 30.0;
    store.observe(result);
    if (store.records().size() != 2) {
        std::cerr << "distant berth separation failed\n";
        return 1;
    }

    // A modulo-180 angle normalization must move the opening edge with the
    // local axes.  135 degrees and -45 degrees describe the same rectangle,
    // but edge 1 becomes edge 3 after the 180-degree axis reversal.
    SlamMapBerthStore normalizedStore;
    usv::BerthMeasureResult normalizedResult;
    normalizedResult.timestamp = 4.0;
    usv::Berth normalizedBerth;
    normalizedBerth.cx = 100.0;
    normalizedBerth.cy = 200.0;
    normalizedBerth.w = 8.0;
    normalizedBerth.l = 18.0;
    normalizedBerth.angle = 135.0;
    normalizedBerth.kind = usv::BerthKind::UShape;
    normalizedBerth.opening_edge = 1;
    normalizedResult.expanded_berths.push_back(normalizedBerth);
    normalizedStore.observe(normalizedResult);
    if (normalizedStore.records().size() != 1
        || !close(normalizedStore.records().front().angle_deg, -45.0)
        || normalizedStore.records().front().opening_edge != 3) {
        std::cerr << "angle normalization must preserve the physical opening edge\n";
        return 1;
    }

    // A non-canonical archive record must be normalized with the same invariant
    // before it is rendered or sent.
    usv::SlamMapBerth legacyRecord;
    legacyRecord.id = 42;
    legacyRecord.timestamp = 5.0;
    legacyRecord.x = 5.0;
    legacyRecord.y = 6.0;
    legacyRecord.width = 8.0;
    legacyRecord.length = 18.0;
    legacyRecord.angle_deg = 135.0;
    legacyRecord.kind = static_cast<uint8_t>(usv::BerthKind::UShape);
    legacyRecord.opening_edge = 1;
    legacyRecord.observation_count = 1;
    legacyRecord.confidence = 1.0;
    normalizedStore.setRecords({legacyRecord});
    if (normalizedStore.records().size() != 1
        || !close(normalizedStore.records().front().angle_deg, -45.0)
        || normalizedStore.records().front().opening_edge != 3) {
        std::cerr << "archive berth normalization failed\n";
        return 1;
    }

    std::cout << "All persistent berth store tests passed\n";
    return 0;
}
