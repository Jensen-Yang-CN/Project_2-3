#ifndef IMUWORKER_H
#define IMUWORKER_H

#include <QObject>
#include "BaseWorker.h"
#include "NaviProtocol.h"

class ImuWorker : public BaseWorker
{
    Q_OBJECT
public:
    explicit ImuWorker(QObject *parent = nullptr);
    ~ImuWorker();


signals:
     void imuDataReady(const IMUParsedData &data);

public slots:
    void processPacket(QByteArray payload) override;
    void processNavPacket(QByteArray payload, double captureTimestamp);
    void processNavPacketWithGeneration(QByteArray payload,
                                        double captureTimestamp,
                                        quint64 timelineGeneration);


};

#endif // IMUWORKER_H
