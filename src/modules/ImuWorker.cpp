#include "ImuWorker.h"
#include "NaviProtocol.h"
#include "SensorMetrics.h"
#include <QtEndian>
#include <QDebug>

// 必须有这个构造函数的实现
ImuWorker::ImuWorker(QObject *parent) : BaseWorker(parent)
{
    // 初始化代码（如果有）
}

// 如果你声明了析构函数，也必须实现它
ImuWorker::~ImuWorker()
{
}

namespace {

bool parseNavPayload(const QByteArray &payload, IMUParsedData &out) {
    if (payload.size() < 80) {
        return false;
    }

    const NaviRawPayload* raw = reinterpret_cast<const NaviRawPayload*>(payload.data());
    using namespace NaviFactors;

    const uint16_t sw = qFromBigEndian<uint16_t>(raw->stateWord);
    out.naviState  = (sw >> 12) & 0x0F;
    out.yawValid   = !((sw >> 11) & 0x01);
    out.pitchValid = !((sw >> 10) & 0x01);
    out.rollValid  = !((sw >> 9) & 0x01);

    out.yaw   = qFromBigEndian<uint32_t>(raw->yaw) * ANGLE;
    out.pitch = qFromBigEndian<int32_t>(raw->pitch) * ANGLE;
    out.roll  = qFromBigEndian<int32_t>(raw->roll) * ANGLE;

    out.longitude = qFromBigEndian<int32_t>(raw->longitude) * LL;
    out.latitude  = qFromBigEndian<int32_t>(raw->latitude) * LL;
    out.timestamp = qFromBigEndian<uint32_t>(raw->timestamp) * TIMESTAMP;

    out.vx = qFromBigEndian<int32_t>(raw->x_Rate) * METRIC;
    out.vy = qFromBigEndian<int32_t>(raw->y_Rate) * METRIC;
    out.vz = qFromBigEndian<int32_t>(raw->z_Rate) * METRIC;

    out.ax = qFromBigEndian<int32_t>(raw->x_RateAcc) * METRIC;
    out.ay = qFromBigEndian<int32_t>(raw->y_RateAcc) * METRIC;
    out.az = qFromBigEndian<int32_t>(raw->z_RateAcc) * METRIC;

    out.vEast  = qFromBigEndian<int32_t>(raw->eastRate) * METRIC;
    out.vNorth = qFromBigEndian<int32_t>(raw->northRate) * METRIC;
    out.vSky   = qFromBigEndian<int32_t>(raw->skyRate) * METRIC;
    return true;
}

} // namespace

void ImuWorker::processPacket(QByteArray payload) {
    IMUParsedData out;
    if (!parseNavPayload(payload, out)) {
        return;
    }
    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::Imu, out.timestamp);
    emit imuDataReady(out);
}

void ImuWorker::processNavPacket(QByteArray payload, double captureTimestamp) {
    processNavPacketWithGeneration(payload, captureTimestamp, 1);
}

void ImuWorker::processNavPacketWithGeneration(QByteArray payload,
                                               double captureTimestamp,
                                               quint64 timelineGeneration) {
    IMUParsedData out;
    if (!parseNavPayload(payload, out)) {
        return;
    }
    if (captureTimestamp > 0.0) {
        out.timestamp = captureTimestamp;
    }
    out.timelineGeneration = timelineGeneration;
    SensorMetrics::instance().recordFrame(SensorMetrics::Channel::Imu, out.timestamp);
    emit imuDataReady(out);
}
