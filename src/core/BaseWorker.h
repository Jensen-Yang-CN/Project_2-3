#ifndef BASEWORKER_H
#define BASEWORKER_H

#include <QObject>
#include <QByteArray>
#include <QImage>
class BaseWorker : public QObject
{
    Q_OBJECT
public:
    explicit BaseWorker(QObject *parent = nullptr) : QObject(parent){}
    virtual ~BaseWorker(){}

signals:
    void finished();                        // 处理完成
    void errorOccurred(const QString &msg); // 错误信号
    void frameReady(const QImage &img);    // 每帧数据

public slots:
    virtual void processPacket(QByteArray payload) = 0;

};

#endif // BASEWORKER_H
