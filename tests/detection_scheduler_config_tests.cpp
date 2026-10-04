#include "AppConfig.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <iostream>

namespace {

void check(bool condition, const char *message, int &failures)
{
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

bool writeConfig(const QString &path, const QByteArray &contents)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;
    return file.write(contents) == contents.size();
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    int failures = 0;
    QTemporaryDir directory;
    check(directory.isValid(), "temporary config directory should exist",
          failures);
    if (!directory.isValid())
        return 1;

    const QString path = directory.filePath(QStringLiteral("config.json"));
    check(writeConfig(path, R"JSON({
      "compute_node_scheduler": {
        "stable_berth_timestamps_to_stop_bridge": 12,
        "missing_berth_timestamps_to_start_bridge": 14
      }
    })JSON"), "scheduler config fixture should be written", failures);
    check(AppConfig::instance().load(path),
          "scheduler config should load", failures);
    check(AppConfig::instance().computeNodeScheduler()
                  .stable_berth_timestamps_to_stop_bridge == 12
              && AppConfig::instance().computeNodeScheduler()
                     .missing_berth_timestamps_to_start_bridge == 14,
          "both scheduling thresholds should load", failures);

    check(AppConfig::instance().network().delivery_remote_ip
                  == QStringLiteral("101.1.101.105"),
          "map delivery should have the requested default remote IP", failures);
    QString saveError;
    check(AppConfig::instance().saveDeliveryRemoteIp(
              QStringLiteral("192.168.10.25"), &saveError),
          "delivery remote IP should save to the active config file", failures);
    check(AppConfig::instance().load(path),
          "config should reload after saving the delivery IP", failures);
    check(AppConfig::instance().network().delivery_remote_ip
                  == QStringLiteral("192.168.10.25"),
          "saved delivery IP should be restored after reload", failures);
    QFile savedConfig(path);
    check(savedConfig.open(QIODevice::ReadOnly),
          "saved config should remain readable", failures);
    if (savedConfig.isOpen()) {
        const QJsonObject savedRoot =
            QJsonDocument::fromJson(savedConfig.readAll()).object();
        check(savedRoot.value(QStringLiteral("compute_node_scheduler"))
                      .toObject()
                      .value(QStringLiteral(
                          "stable_berth_timestamps_to_stop_bridge"))
                      .toInt() == 12,
              "saving the IP should preserve unrelated config values",
              failures);
    }

    check(writeConfig(path, R"JSON({
      "compute_node_scheduler": {
        "stable_berth_timestamps_to_stop_bridge": 0,
        "missing_berth_timestamps_to_start_bridge": 20001
      }
    })JSON"), "invalid scheduler config fixture should be written", failures);
    check(AppConfig::instance().load(path),
          "invalid scheduler config should still load", failures);
    check(AppConfig::instance().computeNodeScheduler()
                  .stable_berth_timestamps_to_stop_bridge == 1
              && AppConfig::instance().computeNodeScheduler()
                     .missing_berth_timestamps_to_start_bridge == 10000,
          "scheduling thresholds should clamp to the safe range", failures);

    if (failures != 0)
        return 1;
    std::cout << "All detection scheduler config tests passed\n";
    return 0;
}
