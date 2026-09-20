#ifndef UDPDISPATCHER_H
#define UDPDISPATCHER_H

#include <QObject>
#include <QHostAddress>
#include <QHash>
#include <QByteArray>
#include <QElapsedTimer>
#include <QMutex>
#include <QSet>
#include <functional>
#include <atomic>

// 定义哈希键结构
struct RouteKey {
    QHostAddress ip;
    quint16 port;

    bool operator==(const RouteKey &other) const {
        return (port == other.port) && (ip == other.ip);
    }
};

// 哈希函数实现
inline uint qHash(const RouteKey &key, uint seed = 0) {
    return qHash(key.ip.toString(), seed) ^ qHash(key.port, seed);
}


// 时间线代际在分流时就固化，避免跨线程排队后把旧包误当成新包。
struct Rule {
    std::function<void(const QByteArray&, double, quint64)> callback;
    QObject* receiver;
};

class UdpDispatcher : public QObject {
    Q_OBJECT
public:
    explicit UdpDispatcher(QObject *parent = nullptr);
    virtual ~UdpDispatcher() = default;
    void setAcceptedTimelineGeneration(quint64 generation) {
        m_acceptedTimelineGeneration.store(generation, std::memory_order_release);
    }
    quint64 acceptedTimelineGeneration() const {
        return m_acceptedTimelineGeneration.load(std::memory_order_acquire);
    }

    /**
     * @brief 添加分流规则 (模板实现必须放在头文件)
     * @param T 目标类类型
     * @param method 成员函数指针，例如 &RtpParser::inputPacket
     */
    /**
         * @brief 重载1：支持 2 个参数的成员函数 (data, timestamp)
         */
        template <typename T>
        void addRule(const QHostAddress &ip, quint16 port, T* parser,
                     void (T::*method)(const QByteArray&, double)) {
            RouteKey key{ip, port};
            Rule rule;
            rule.receiver = parser;

            // 绑定带两个参数的函数
            rule.callback = [parser, method](const QByteArray &payload, double ts,
                                             quint64 /*generation*/) {
                (parser->*method)(payload, ts);
            };

            routeTable.insert(key, rule);

            connect(parser, &QObject::destroyed, this, [this, key](){
                routeTable.remove(key);
            });
        }

        /** 支持数据包、时间戳和时间线代际的成员函数。 */
        template <typename T>
        void addRule(const QHostAddress &ip, quint16 port, T* parser,
                     void (T::*method)(const QByteArray&, double, quint64)) {
            RouteKey key{ip, port};
            Rule rule;
            rule.receiver = parser;
            rule.callback = [parser, method](const QByteArray &payload, double ts,
                                             quint64 generation) {
                (parser->*method)(payload, ts, generation);
            };

            routeTable.insert(key, rule);
            connect(parser, &QObject::destroyed, this, [this, key]() {
                routeTable.remove(key);
            });
        }

        /**
         * @brief 重载2：兼容 1 个参数的旧成员函数 (data)
         */
        template <typename T>
        void addRule(const QHostAddress &ip, quint16 port, T* parser,
                     void (T::*method)(const QByteArray&)) {
            RouteKey key{ip, port};
            Rule rule;
            rule.receiver = parser;

            // 包装：忽略传入的 timestamp，只调用单参数函数
            rule.callback = [parser, method](const QByteArray &payload, double /*ts*/,
                                             quint64 /*generation*/) {
                (parser->*method)(payload);
            };

            routeTable.insert(key, rule);

            connect(parser, &QObject::destroyed, this, [this, key](){
                routeTable.remove(key);
            });
        }

        /**
         * @brief 仅按源 IP 分流（忽略端口），用于 RTSP/RTP 动态端口场景。
         *        精确 (IP,端口) 规则优先；未命中时再查 IP 规则。
         */
        template <typename T>
        void addRuleByIp(const QHostAddress &ip, T* parser,
                         void (T::*method)(const QByteArray&, double)) {
            Rule rule;
            rule.receiver = parser;
            rule.callback = [parser, method](const QByteArray &payload, double ts,
                                             quint64 /*generation*/) {
                (parser->*method)(payload, ts);
            };

            ipRouteTable.insert(ip, rule);

            connect(parser, &QObject::destroyed, this, [this, ip]() {
                ipRouteTable.remove(ip);
            });
        }

        template <typename T>
        void addRuleByIp(const QHostAddress &ip, T* parser,
                         void (T::*method)(const QByteArray&, double, quint64)) {
            Rule rule;
            rule.receiver = parser;
            rule.callback = [parser, method](const QByteArray &payload, double ts,
                                             quint64 generation) {
                (parser->*method)(payload, ts, generation);
            };

            ipRouteTable.insert(ip, rule);
            connect(parser, &QObject::destroyed, this, [this, ip]() {
                ipRouteTable.remove(ip);
            });
        }

public slots:
    //void dispatch(const QHostAddress &ip, quint16 port, const QByteArray &payload);
    void dispatch(const QHostAddress &ip, quint16 port, const QByteArray &payload, double timestamp);
    void dispatchWithGeneration(const QHostAddress &ip, quint16 port,
                                const QByteArray &payload, double timestamp,
                                quint64 timelineGeneration);

private:
    void dispatchAccepted(const QHostAddress &ip, quint16 port,
                          const QByteArray &payload, double timestamp,
                          quint64 timelineGeneration);
    void invokeRule(const Rule &rule, const QByteArray &payload, double timestamp,
                    quint64 timelineGeneration);
    void recordDispatch(bool matched, const QString &sourceKey, const QString &matchMode);
    void flushDispatchStats(bool force = false);
    static QString normalizedSourceKey(const QHostAddress &ip, quint16 port);

    QHash<RouteKey, Rule> routeTable;
    QHash<QHostAddress, Rule> ipRouteTable;

    mutable QMutex m_statsMutex;
    QElapsedTimer m_statsTimer;
    bool m_statsTimerStarted = false;
    quint64 m_matchedTotal = 0;
    quint64 m_unmatchedTotal = 0;
    QHash<QString, quint64> m_matchedBySource;
    QHash<QString, quint64> m_unmatchedBySource;
    QSet<QString> m_loggedMissKeys;
    std::atomic<quint64> m_acceptedTimelineGeneration{1};
};

#endif // UDPDISPATCHER_H
