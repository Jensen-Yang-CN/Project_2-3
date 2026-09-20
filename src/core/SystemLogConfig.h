#ifndef SYSTEMLOGCONFIG_H
#define SYSTEMLOGCONFIG_H

#include <QString>

struct SystemLogConfig {
    bool enabled = true;
    /** 相对程序目录或绝对路径 */
    QString dir = QStringLiteral("logs");
    /** 超过该天数的 system_YYYY-MM-DD.jsonl 会被删除 */
    int retain_days = 14;
    /** 剩余空间低于此值（GB）时，从最早日志开始额外清理 */
    double min_free_gb = 5.0;
    /** 启动时执行一次过期日志清理 */
    bool cleanup_on_startup = true;
    int sensor_fps_interval_sec = 5;
    int disk_check_interval_sec = 600;
};

#endif  // SYSTEMLOGCONFIG_H
