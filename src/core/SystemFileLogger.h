#ifndef SYSTEMFILELOGGER_H
#define SYSTEMFILELOGGER_H

#include "SystemLogConfig.h"

#include <QJsonObject>
#include <QString>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

enum class SystemLogLevel {
    Debug,
    Info,
    Warn,
    Error
};

/** JSONL 系统日志：按日滚动文件、异步写盘、过期清理与磁盘空间监控 */
class SystemFileLogger {
public:
    struct DiskSnapshot {
        double free_gb = 0.0;
        double total_gb = 0.0;
        QString root_path;
    };

    struct CleanupReport {
        int deleted_files = 0;
        qint64 freed_bytes = 0;
        QStringList deleted_names;
        QString reason;
    };

    static SystemFileLogger &instance();

    void init(const SystemLogConfig &config);
    void shutdown();

    bool isEnabled() const { return enabled_; }

    void logEvent(SystemLogLevel level,
                  const char *module,
                  const char *event,
                  const QJsonObject &data = {});

    void logLifecycle(const char *module, const char *state, const QJsonObject &extra = {});

    DiskSnapshot queryDiskSpace() const;
    CleanupReport cleanupOldLogs(bool aggressive_for_space = false);

private:
    SystemFileLogger() = default;
    ~SystemFileLogger();

    SystemFileLogger(const SystemFileLogger &) = delete;
    SystemFileLogger &operator=(const SystemFileLogger &) = delete;

    static QString levelToString(SystemLogLevel level);
    static quintptr currentThreadId();

    void enqueueLine(std::string line);
    void writerLoop();
    void monitorLoop();
    bool openLogFileForToday();
    void closeLogFile();
    double freeSpaceGbOnLogVolume() const;
    QDate parseLogFileDate(const QString &fileName) const;

    SystemLogConfig config_;
    QString log_dir_;
    std::atomic<bool> enabled_{false};
    std::atomic<bool> running_{false};

    std::queue<std::string> queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread writer_thread_;
    std::thread monitor_thread_;

    std::mutex file_mutex_;
    QString current_date_;
    FILE *log_file_ = nullptr;
};

#endif  // SYSTEMFILELOGGER_H
