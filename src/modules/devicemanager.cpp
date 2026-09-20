#include "devicemanager.h"
#include "AppConfig.h"

#include <QDebug>
#include <QHostAddress>

namespace {

QList<DeviceInfo> builtinDefaultDevices()
{
    QList<DeviceInfo> list;

    auto add = [&](const QString &name, DeviceType type, const char *ip, int port, int extraPort = 0) {
        DeviceInfo info;
        info.name = name;
        info.type = type;
        info.ip = QString::fromLatin1(ip);
        info.port = port;
        info.extraPort = extraPort;
        list.append(info);
    };

    add(QStringLiteral("210"), DeviceType::Lidar_LS, "192.168.1.210", 2369);
    add(QStringLiteral("211"), DeviceType::Lidar_LS, "192.168.1.211", 2369);
    add(QStringLiteral("201"), DeviceType::Lidar_RS, "192.168.1.201", 6699, 7788);
    add(QStringLiteral("213"), DeviceType::Lidar_RS, "192.168.1.213", 2369, 2368);
    add(QStringLiteral("214"), DeviceType::Lidar_RS, "192.168.1.214", 2369, 2368);
    add(QStringLiteral("IMU"), DeviceType::IMU, "101.1.101.20", 4353);

    {
        DeviceInfo info;
        info.name = QStringLiteral("h265Right");
        info.type = DeviceType::Camera_H265;
        info.ip = QStringLiteral("192.168.58.223");
        info.port = 0;
        info.metadata.insert(QStringLiteral("dispatch_by_ip"), true);
        info.metadata.insert(QStringLiteral("parser_name"), QStringLiteral("h265Right"));
        info.metadata.insert(
            QStringLiteral("pcap_alias_ips"),
            QVariantList{QStringLiteral("192.168.1.220"),
                         QStringLiteral("192.168.58.60")});
        info.metadata.insert(QStringLiteral("rtsp_url"), QStringLiteral("rtsp://192.168.58.223/live.sdp"));
        info.metadata.insert(QStringLiteral("stream_name"), QStringLiteral("USV02-右固连相机"));
        list.append(info);
    }
    {
        DeviceInfo info;
        info.name = QStringLiteral("h265Left");
        info.type = DeviceType::Camera_H265;
        info.ip = QStringLiteral("192.168.58.222");
        info.port = 0;
        info.metadata.insert(QStringLiteral("dispatch_by_ip"), true);
        info.metadata.insert(QStringLiteral("parser_name"), QStringLiteral("h265Left"));
        info.metadata.insert(
            QStringLiteral("pcap_alias_ips"),
            QVariantList{QStringLiteral("192.168.1.223"),
                         QStringLiteral("192.168.58.61")});
        info.metadata.insert(QStringLiteral("rtsp_url"), QStringLiteral("rtsp://192.168.58.222/live.sdp"));
        info.metadata.insert(QStringLiteral("stream_name"), QStringLiteral("USV02-左固连相机"));
        list.append(info);
    }
    add(QStringLiteral("h264Left"), DeviceType::Camera_H264, "192.168.1.170", 20080);
    add(QStringLiteral("h264Right"), DeviceType::Camera_H264, "192.168.1.170", 20082);

    return list;
}

}  // namespace

bool DeviceManager::addDevice(const DeviceInfo &info) {
    if (m_devices.isEmpty()) {
        loadConfigOrDefaults();
    }
    m_devices.append(info);
    return saveAllConfigs();
}

bool DeviceManager::loadConfigOrDefaults(const QString &filePath)
{
    const QString path = AppConfig::resolvePath(filePath.isEmpty() ? AppConfig::instance().devicesFilePath() : filePath);
    m_configPath = path;

    if (loadConfig(path))
        return true;

    qWarning() << "[DeviceManager] 使用内置默认设备表:" << path;
    m_devices = builtinDefaultDevices();
    return false;
}

bool DeviceManager::loadConfig(const QString &filePath) {
    const QString path = AppConfig::resolvePath(filePath);
    m_configPath = path;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return false;

    const QByteArray data = file.readAll();
    const QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isArray())
        return false;

    const QJsonArray array = doc.array();
    m_devices.clear();
    for (int i = 0; i < array.size(); ++i) {
        m_devices.append(jsonToDeviceInfo(array[i].toObject()));
    }
    return true;
}

DeviceInfo DeviceManager::deviceByName(const QString &name) const
{
    for (const DeviceInfo &info : m_devices) {
        if (info.name == name)
            return info;
    }
    return DeviceInfo{};
}

bool DeviceManager::hasDevice(const QString &name) const
{
    return !deviceByName(name).name.isEmpty();
}

QJsonObject DeviceManager::deviceInfoToJson(const DeviceInfo &info) {
    QJsonObject obj;
    obj["name"] = info.name;
    obj["type"] = QString(device::toString(info.type));
    obj["ip"] = info.ip;
    obj["port"] = info.port;
    obj["extraPort"] = info.extraPort;
    obj["metadata"] = QJsonObject::fromVariantMap(info.metadata);
    return obj;
}

DeviceInfo DeviceManager::jsonToDeviceInfo(const QJsonObject &obj) {
    DeviceInfo info;
    info.name = obj["name"].toString();
    info.type = device::fromString(obj["type"].toString());
    info.ip = obj["ip"].toString();
    info.port = obj["port"].toInt();
    info.extraPort = obj["extraPort"].toInt();
    info.metadata = obj["metadata"].toObject().toVariantMap();
    return info;
}

bool DeviceManager::saveAllConfigs() {
    QJsonArray array;
    for (const auto& device : m_devices) {
        array.append(deviceInfoToJson(device));
    }

    const QString path = m_configPath.isEmpty()
        ? AppConfig::instance().devicesFilePath()
        : m_configPath;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return false;

    file.write(QJsonDocument(array).toJson(QJsonDocument::Indented));
    return true;
}
