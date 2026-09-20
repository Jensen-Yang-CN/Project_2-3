#include "SystemFileLogger.h"

#include "AppConfig.h"
#include "SensorMetrics.h"

#include <QDate>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QStorageInfo>

#include <algorithm>
#include <cstdio>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

SystemFileLogger::~SystemFileLogger()
{
    shutdown();
}

SystemFileLogger &SystemFileLogger::instance()
{
    static SystemFileLogger inst;
    return inst;
}

QString SystemFileLogger::levelToString(SystemLogLevel level)
{
    switch (level) {
    case SystemLogLevel::Debug: return QStringLiteral("DEBUG");
    case SystemLogLevel::Info: return QStringLiteral("INFO");
    case SystemLogLevel::Warn: return QStringLiteral("WARN");
    case SystemLogLevel::Error: return QStringLiteral("ERROR");
    }
    return QStringLiteral("INFO");
}

quintptr SystemFileLogger::currentThreadId()
{
#ifdef _WIN32
    return static_cast<quintptr>(GetCurrentThreadId());
#else
    return static_cast<quintptr>(
        std::hash<std::thread::id>{}(std::this_thread::get_id()));
#endif
}

void SystemFileLogger::init(const SystemLogConfig &config)
{
    if (running_.load())
        return;

    config_ = config;
    enabled_.store(config_.enabled);
    if (!enabled_.load())
        return;

    log_dir_ = AppConfig::resolvePath(config_.dir);
    QDir().mkpath(log_dir_);

    running_.store(true);
    writer_thread_ = std::thread(&SystemFileLogger::writerLoop, this);
    monitor_thread_ = std::thread(&SystemFileLogger::monitorLoop, this);

    QJsonObject boot;
    boot.insert(QStringLiteral("log_dir"), log_dir_);
    boot.insert(QStringLiteral("retain_days"), config_.retain_days);
    boot.insert(QStringLiteral("min_free_gb"), config_.min_free_gb);
    logEvent(SystemLogLevel::Info, "SystemFileLogger", "logger_started", boot);

    if (config_.cleanup_on_startup) {
        const CleanupReport report = cleanupOldLogs(false);
        QJsonObject cleanup;
        cleanup.insert(QStringLiteral("reason"), report.reason);
        cleanup.insert(QStringLiteral("deleted_files"), report.deleted_files);
        cleanup.insert(QStringLiteral("freed_bytes"), static_cast<qint64>(report.freed_bytes));
        cleanup.insert(QStringLiteral("deleted_names"),
                       QJsonArray::fromStringList(report.deleted_names));
        logEvent(SystemLogLevel::Info, "SystemFileLogger", "log_cleanup", cleanup);
    }

    const DiskSnapshot disk = queryDiskSpace();
    QJsonObject diskObj;
    diskObj.insert(QStringLiteral("free_gb"), disk.free_gb);
    diskObj.insert(QStringLiteral("total_gb"), disk.total_gb);
    diskObj.insert(QStringLiteral("root"), disk.root_path);
    logEvent(SystemLogLevel::Info, "SystemFileLogger", "disk_space", diskObj);

    if (disk.free_gb > 0.0 && disk.free_gb < config_.min_free_gb) {
        const CleanupReport report = cleanupOldLogs(true);
        QJsonObject cleanup;
        cleanup.insert(QStringLiteral("reason"), report.reason);
        cleanup.insert(QStringLiteral("deleted_files"), report.deleted_files);
        cleanup.insert(QStringLiteral("freed_bytes"), static_cast<qint64>(report.freed_bytes));
        logEvent(SystemLogLevel::Warn, "SystemFileLogger", "log_cleanup", cleanup);
    }
}

void SystemFileLogger::shutdown()
{
    if (!running_.exchange(false))
        return;

    queue_cv_.notify_all();
    if (writer_thread_.joinable())
        writer_thread_.join();
    if (monitor_thread_.joinable())
        monitor_thread_.join();

    {
        std::lock_guard<std::mutex> lock(file_mutex_);
        closeLogFile();
    }

    enabled_.store(false);
}

void SystemFileLogger::logEvent(SystemLogLevel level,
                                const char *module,
                                const char *event,
                                const QJsonObject &data)
{
    if (!enabled_.load())
        return;

    QJsonObject root;
    root.insert(QStringLiteral("ts"),
                QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("level"), levelToString(level));
    root.insert(QStringLiteral("tid"), static_cast<qint64>(currentThreadId()));
    root.insert(QStringLiteral("module"), QString::fromUtf8(module));
    root.insert(QStringLiteral("event"), QString::fromUtf8(event));

    for (auto it = data.constBegin(); it != data.constEnd(); ++it) {
        if (!root.contains(it.key()))
            root.insert(it.key(), it.value());
    }

    const QByteArray lineBytes =
        QJsonDocument(root).toJson(QJsonDocument::Compact) + '\n';
    enqueueLine(lineBytes.toStdString());
}

void SystemFileLogger::logLifecycle(const char *module,
                                    const char *state,
                                    const QJsonObject &extra)
{
    QJsonObject data = extra;
    data.insert(QStringLiteral("state"), QString::fromUtf8(state));
    logEvent(SystemLogLevel::Info, module, "node_lifecycle", data);
}

void SystemFileLogger::enqueueLine(std::string line)
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        queue_.push(std::move(line));
    }
    queue_cv_.notify_one();
}

void SystemFileLogger::writerLoop()
{
    while (running_.load() || !queue_.empty()) {
        std::queue<std::string> batch;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait_for(lock, std::chrono::milliseconds(200), [this]() {
                return !queue_.empty() || !running_.load();
            });
            std::swap(batch, queue_);
        }

        if (batch.empty())
            continue;

        std::lock_guard<std::mutex> fileLock(file_mutex_);
        if (!openLogFileForToday())
            continue;

        while (!batch.empty()) {
            const std::string &line = batch.front();
            if (log_file_) {
                std::fwrite(line.data(), 1, line.size(), log_file_);
            }
            batch.pop();
        }
        if (log_file_)
            std::fflush(log_file_);
    }
}

void SystemFileLogger::monitorLoop()
{
    const int fpsInterval = std::max(1, config_.sensor_fps_interval_sec);
    const int diskInterval = std::max(60, config_.disk_check_interval_sec);

    int fpsCounter = 0;
    int diskCounter = 0;

    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!running_.load())
            break;

        ++fpsCounter;
        if (fpsCounter >= fpsInterval) {
            fpsCounter = 0;
            SensorMetrics::instance().flushSummary(static_cast<double>(fpsInterval));
        }

        ++diskCounter;
        if (diskCounter >= diskInterval) {
            diskCounter = 0;

            const DiskSnapshot disk = queryDiskSpace();
            QJsonObject diskObj;
            diskObj.insert(QStringLiteral("free_gb"), disk.free_gb);
            diskObj.insert(QStringLiteral("total_gb"), disk.total_gb);
            diskObj.insert(QStringLiteral("root"), disk.root_path);
            logEvent(SystemLogLevel::Info, "SystemFileLogger", "disk_space", diskObj);

            if (disk.free_gb > 0.0 && disk.free_gb < config_.min_free_gb) {
                const CleanupReport report = cleanupOldLogs(true);
                QJsonObject cleanup;
                cleanup.insert(QStringLiteral("reason"), report.reason);
                cleanup.insert(QStringLiteral("deleted_files"), report.deleted_files);
                cleanup.insert(QStringLiteral("freed_bytes"), static_cast<qint64>(report.freed_bytes));
                logEvent(SystemLogLevel::Warn, "SystemFileLogger", "log_cleanup", cleanup);
            }
        }
    }
}

bool SystemFileLogger::openLogFileForToday()
{
    const QString today = QDate::currentDate().toString(QStringLiteral("yyyy-MM-dd"));
    if (log_file_ && current_date_ == today)
        return true;

    closeLogFile();
    current_date_ = today;

    const QString path =
        log_dir_ + QLatin1Char('/') + QStringLiteral("system_") + today + QStringLiteral(".jsonl");
    log_file_ = std::fopen(path.toLocal8Bit().constData(), "ab");
    return log_file_ != nullptr;
}

void SystemFileLogger::closeLogFile()
{
    if (log_file_) {
        std::fflush(log_file_);
        std::fclose(log_file_);
        log_file_ = nullptr;
    }
}

SystemFileLogger::DiskSnapshot SystemFileLogger::queryDiskSpace() const
{
    DiskSnapshot snap;
    const QStorageInfo storage(log_dir_);
    if (!storage.isValid())
        return snap;

    snap.root_path = storage.rootPath();
    snap.total_gb = static_cast<double>(storage.bytesTotal()) / (1024.0 * 1024.0 * 1024.0);
    snap.free_gb = static_cast<double>(storage.bytesAvailable()) / (1024.0 * 1024.0 * 1024.0);
    return snap;
}

double SystemFileLogger::freeSpaceGbOnLogVolume() const
{
    return queryDiskSpace().free_gb;
}

QDate SystemFileLogger::parseLogFileDate(const QString &fileName) const
{
    if (!fileName.startsWith(QStringLiteral("system_"))
        || !fileName.endsWith(QStringLiteral(".jsonl"))) {
        return {};
    }
    const QString datePart = fileName.mid(7, 10);
    return QDate::fromString(datePart, QStringLiteral("yyyy-MM-dd"));
}

SystemFileLogger::CleanupReport SystemFileLogger::cleanupOldLogs(bool aggressive_for_space)
{
    CleanupReport report;
    report.reason = aggressive_for_space
        ? QStringLiteral("low_disk_space")
        : QStringLiteral("retain_days");

    if (log_dir_.isEmpty())
        return report;

    const QDate today = QDate::currentDate();
    const QDate retainCutoff = today.addDays(-std::max(1, config_.retain_days));

    QDir dir(log_dir_);
    const QFileInfoList files =
        dir.entryInfoList({QStringLiteral("system_*.jsonl")}, QDir::Files, QDir::Name);

    auto tryDelete = [&](const QFileInfo &fi) -> bool {
        const QString todayFile =
            QStringLiteral("system_") + today.toString(QStringLiteral("yyyy-MM-dd"))
            + QStringLiteral(".jsonl");
        if (fi.fileName() == todayFile)
            return false;

        const qint64 size = fi.size();
        if (QFile::remove(fi.absoluteFilePath())) {
            report.deleted_files += 1;
            report.freed_bytes += size;
            report.deleted_names.append(fi.fileName());
            return true;
        }
        return false;
    };

    for (const QFileInfo &fi : files) {
        const QDate fileDate = parseLogFileDate(fi.fileName());
        if (!fileDate.isValid())
            continue;
        if (fileDate < retainCutoff)
            tryDelete(fi);
    }

    if (aggressive_for_space) {
        while (freeSpaceGbOnLogVolume() < config_.min_free_gb) {
            const QFileInfoList remaining =
                dir.entryInfoList({QStringLiteral("system_*.jsonl")}, QDir::Files, QDir::Name);
            bool deletedAny = false;
            for (const QFileInfo &fi : remaining) {
                if (tryDelete(fi)) {
                    deletedAny = true;
                    break;
                }
            }
            if (!deletedAny)
                break;
        }
    }

    return report;
}
