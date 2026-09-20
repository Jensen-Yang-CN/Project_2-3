#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include <QObject>
#include <QList>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "DeviceConfig.h" // 包含我们统一的 DeviceInfo
//using Device::DeviceType;
using namespace device;
class DeviceManager : public QObject {
    Q_OBJECT
public:
    // 单例模式，全局访问
    static DeviceManager& instance() {
        static DeviceManager _instance;
        return _instance;
    }

    // 核心业务：添加并保存
    bool addDevice(const DeviceInfo &info);
    bool saveAllConfigs();
    // 核心业务：从文件加载
    bool loadConfig(const QString &filePath = QString());
    /** 加载 devices.json；失败时使用内置默认设备表 */
    bool loadConfigOrDefaults(const QString &filePath = QString());

    DeviceInfo deviceByName(const QString &name) const;
    bool hasDevice(const QString &name) const;

    // 获取所有已注册设备，供 MainWindow 循环创建解析器使用
    QList<DeviceInfo> devices() const { return m_devices; }

private:
    explicit DeviceManager(QObject *parent = nullptr) : QObject(parent) {}
    QList<DeviceInfo> m_devices;
    QString m_configPath;
    // 内部转换工具
    QJsonObject deviceInfoToJson(const DeviceInfo &info);
    DeviceInfo jsonToDeviceInfo(const QJsonObject &obj);

};

#endif
