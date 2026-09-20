#include "AppConfig.h"
#include "PcapPlaybackController.h"
#include "PcapPlaybackRequest.h"
#include "PcapPlaybackTime.h"
#include "PcapngReader.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QElapsedTimer>
#include <QFile>
#include <QHostAddress>
#include <QTemporaryDir>
#include <QTimeZone>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string &message)
{
    if (condition)
        return;
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
}

void appendLe16(QByteArray &out, quint16 value)
{
    out.append(char(value & 0xff));
    out.append(char((value >> 8) & 0xff));
}

void appendLe32(QByteArray &out, quint32 value)
{
    out.append(char(value & 0xff));
    out.append(char((value >> 8) & 0xff));
    out.append(char((value >> 16) & 0xff));
    out.append(char((value >> 24) & 0xff));
}

void appendBe16(QByteArray &out, quint16 value)
{
    out.append(char((value >> 8) & 0xff));
    out.append(char(value & 0xff));
}

QByteArray makeIpv4Frame(quint8 protocol, const QByteArray &payload)
{
    QByteArray frame(12, '\0');
    appendBe16(frame, 0x0800);

    constexpr int transportHeaderSize = 8;
    const quint16 ipTotalLength =
        quint16(20 + transportHeaderSize + payload.size());
    frame.append(char(0x45));
    frame.append(char(0));
    appendBe16(frame, ipTotalLength);
    appendBe16(frame, 0);
    appendBe16(frame, 0);
    frame.append(char(64));
    frame.append(char(protocol));
    appendBe16(frame, 0);
    frame.append(char(192)); frame.append(char(168));
    frame.append(char(1)); frame.append(char(210));
    frame.append(char(192)); frame.append(char(168));
    frame.append(char(1)); frame.append(char(177));

    appendBe16(frame, 2369);
    appendBe16(frame, 2369);
    appendBe16(frame, quint16(transportHeaderSize + payload.size()));
    appendBe16(frame, 0);
    frame.append(payload);
    return frame;
}

void appendPcapPacket(QByteArray &pcap,
                      quint32 seconds,
                      quint32 microseconds,
                      const QByteArray &frame)
{
    appendLe32(pcap, seconds);
    appendLe32(pcap, microseconds);
    appendLe32(pcap, quint32(frame.size()));
    appendLe32(pcap, quint32(frame.size()));
    pcap.append(frame);
}

QString writePcap(const QString &path,
                  const std::vector<std::pair<quint32, QByteArray>> &packets)
{
    QByteArray pcap;
    appendLe32(pcap, 0xa1b2c3d4);
    appendLe16(pcap, 2);
    appendLe16(pcap, 4);
    appendLe32(pcap, 0);
    appendLe32(pcap, 0);
    appendLe32(pcap, 65535);
    appendLe32(pcap, 1);
    for (const auto &packet : packets)
        appendPcapPacket(pcap, 100, packet.first, packet.second);

    QFile file(path);
    check(file.open(QIODevice::WriteOnly), "temporary PCAP must be writable");
    if (file.isOpen()) {
        check(file.write(pcap) == pcap.size(), "complete PCAP must be written");
        file.close();
    }
    return path;
}

QString writeTimedPcap(
    const QString &path,
    const std::vector<std::pair<double, QByteArray>> &packets)
{
    QByteArray pcap;
    appendLe32(pcap, 0xa1b2c3d4);
    appendLe16(pcap, 2);
    appendLe16(pcap, 4);
    appendLe32(pcap, 0);
    appendLe32(pcap, 0);
    appendLe32(pcap, 65535);
    appendLe32(pcap, 1);
    for (const auto &packet : packets) {
        const double wholeSeconds = std::floor(packet.first);
        const quint32 seconds = static_cast<quint32>(wholeSeconds);
        const quint32 microseconds = static_cast<quint32>(
            std::llround((packet.first - wholeSeconds) * 1000000.0));
        appendPcapPacket(pcap, seconds, microseconds, packet.second);
    }

    QFile file(path);
    check(file.open(QIODevice::WriteOnly), "timed PCAP must be writable");
    if (file.isOpen()) {
        check(file.write(pcap) == pcap.size(), "timed PCAP must be complete");
        file.close();
    }
    return path;
}

QString writeConfig(const QString &path, double defaultSpeed)
{
    const QByteArray json = QByteArray("{\n  \"pcap_playback\": {\n")
        + "    \"default_speed\": " + QByteArray::number(defaultSpeed)
        + "\n  }\n}\n";
    QFile file(path);
    check(file.open(QIODevice::WriteOnly), "temporary config must be writable");
    if (file.isOpen()) {
        check(file.write(json) == json.size(), "complete config must be written");
        file.close();
    }
    return path;
}

void testConfigParsesAndClampsDefaultSpeed()
{
    QTemporaryDir dir;
    check(dir.isValid(), "temporary config directory must be valid");

    check(AppConfig::instance().load(writeConfig(dir.filePath("high.json"), 9.0)),
          "high-speed config must load");
    check(std::abs(AppConfig::instance().pcapPlayback().default_speed - 4.0) < 1e-9,
          "default playback speed must clamp to 4x");

    check(AppConfig::instance().load(writeConfig(dir.filePath("low.json"), 0.2)),
          "low-speed config must load");
    check(std::abs(AppConfig::instance().pcapPlayback().default_speed - 0.5) < 1e-9,
          "default playback speed must clamp to 0.5x");
}

void testUsvLocalTimeAndUnixTimestampParsing()
{
    const QTimeZone shanghai("Asia/Shanghai");
    const auto local = pcap_playback_time::parseTimestamp(
        QStringLiteral("2026-06-10 14:28:34.145"), shanghai);
    check(local.ok, "USV local display time must parse");

    const QDateTime expected(
        QDate(2026, 6, 10), QTime(14, 28, 34, 145), shanghai);
    check(std::abs(local.unix_seconds
                   - expected.toMSecsSinceEpoch() / 1000.0) < 1e-6,
          "USV local time must map to the same Unix timeline");

    const auto raw = pcap_playback_time::parseTimestamp(
        QString::number(local.unix_seconds, 'f', 6), shanghai);
    check(raw.ok && std::abs(raw.unix_seconds - local.unix_seconds) < 1e-6,
          "legacy Unix timestamp input must remain supported");
    check(pcap_playback_time::formatLocal(local.unix_seconds, shanghai)
              == QStringLiteral("2026-06-10 14:28:34.145"),
          "Unix timestamp must format exactly like the USV panel time");
    check(pcap_playback_time::snapshotBaseName(local.unix_seconds, shanghai)
              == QStringLiteral("20260610_142834_145"),
          "snapshot filename must use a Windows-safe USV time");
    check(!pcap_playback_time::parseTimestamp(
               QStringLiteral("2026-02-30 12:00:00.000"), shanghai).ok,
          "invalid calendar date must be rejected");
    check(!pcap_playback_time::parseTimestamp(
               QStringLiteral("not-a-time"), shanghai).ok,
          "unrecognized timestamp text must be rejected");
}

qint64 measureInterval(double speed, double captureIntervalSec)
{
    PcapPlaybackController controller;
    controller.reset(speed);
    check(controller.waitForPacket(100.0), "first packet must be released immediately");
    QElapsedTimer timer;
    timer.start();
    check(controller.waitForPacket(100.0 + captureIntervalSec),
          "next packet must be released after its scheduled wait");
    return timer.elapsed();
}

void testPlaybackSpeedScalesCaptureIntervals()
{
    const qint64 at1x = measureInterval(1.0, 0.16);
    const qint64 at2x = measureInterval(2.0, 0.16);
    const qint64 at4x = measureInterval(4.0, 0.16);

    check(at1x >= 125 && at1x < 450, "1x must retain roughly 160 ms spacing");
    check(at2x >= 55 && at2x < 250, "2x must shorten 160 ms spacing to about 80 ms");
    check(at4x >= 25 && at4x < 180, "4x must shorten 160 ms spacing to about 40 ms");
    check(at1x > at2x && at2x > at4x,
          "higher playback speeds must release packets sooner");
}

void testHalfSpeedDoublesCaptureInterval()
{
    const qint64 atHalf = measureInterval(0.5, 0.16);
    check(atHalf >= 260 && atHalf < 650,
          "0.5x must replay 160 ms of capture time in about 320 ms");
}

void testHighRatePacketsUseOneContinuousPlaybackClock()
{
    PcapPlaybackController controller;
    controller.reset(1.0);

    QElapsedTimer timer;
    timer.start();
    constexpr int packetCount = 600;
    constexpr double packetIntervalSec = 0.00005; // 20,000 packets/second
    for (int i = 0; i < packetCount; ++i) {
        check(controller.waitForPacket(200.0 + i * packetIntervalSec),
              "high-rate packet must be released by the playback clock");
    }
    const qint64 elapsed = timer.elapsed();

    check(elapsed >= 15,
          "high-rate playback must still respect about 30 ms of capture time");
    check(elapsed < 180,
          "sub-millisecond packet intervals must not be rounded into one sleep per packet");
}

void testPauseBlocksAndResumeContinues()
{
    PcapPlaybackController controller;
    controller.reset(4.0);
    check(controller.waitForPacket(50.0), "first packet must be released before pause");
    controller.setPaused(true);

    std::atomic<bool> released{false};
    std::thread waiter([&]() {
        released = controller.waitForPacket(50.04);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    check(!released.load(), "paused playback must not release another packet");

    controller.setPaused(false);
    waiter.join();
    check(released.load(), "resume must release the pending packet");
}

void testStopInterruptsPauseAndTimedWait()
{
    PcapPlaybackController controller;
    controller.reset(1.0);
    check(controller.waitForPacket(10.0), "first packet must be released before stop");

    std::atomic<bool> result{true};
    std::thread waiter([&]() {
        result = controller.waitForPacket(12.0);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(35));
    QElapsedTimer stopTimer;
    stopTimer.start();
    controller.stop();
    waiter.join();

    check(!result.load(), "stop must cancel the pending packet");
    check(stopTimer.elapsed() < 150, "stop must wake a timed wait promptly");
}

void testReaderPreservesUdpPayloadsAndCaptureTimestampsAt4x()
{
    QTemporaryDir dir;
    const QString pcapPath = writePcap(
        dir.filePath("capture-time.pcap"),
        {
            {0, makeIpv4Frame(17, QByteArray("udp-a"))},
            {50000, makeIpv4Frame(6, QByteArray("not-udp"))},
            {100000, makeIpv4Frame(17, QByteArray("udp-b"))},
        });

    PcapPlaybackConfig config;
    config.default_speed = 4.0;
    PcapngReader reader;
    reader.setPlaybackConfig(config);

    std::vector<QByteArray> payloads;
    std::vector<double> timestamps;
    QObject::connect(&reader, &PcapngReader::udpPacket,
                     [&](const QHostAddress &, quint16,
                         const QByteArray &payload, double timestamp) {
        payloads.push_back(payload);
        timestamps.push_back(timestamp);
    });

    QElapsedTimer timer;
    timer.start();
    reader.startAnalysis({pcapPath});
    const qint64 elapsed = timer.elapsed();

    check(payloads == std::vector<QByteArray>({QByteArray("udp-a"), QByteArray("udp-b")}),
          "reader must not drop or alter valid UDP payloads at 4x");
    check(timestamps.size() == 2
              && std::abs(timestamps[0] - 100.0) < 1e-6
              && std::abs(timestamps[1] - 100.1) < 1e-6,
          "playback speed must not modify downstream capture timestamps");
    check(elapsed >= 15 && elapsed < 250,
          "100 ms of capture time must replay in roughly 25 ms at 4x");
}

void testReaderSeeksByRawTimestampWithPreroll()
{
    QTemporaryDir dir;
    const QString pcapPath = writeTimedPcap(
        dir.filePath("raw-timestamp-seek.pcap"),
        {
            {100.000, makeIpv4Frame(17, QByteArray("skip"))},
            {100.010, makeIpv4Frame(17, QByteArray("warm-a"))},
            {100.020, makeIpv4Frame(17, QByteArray("warm-b"))},
            {100.030, makeIpv4Frame(17, QByteArray("target"))},
            {100.040, makeIpv4Frame(17, QByteArray("after"))},
        });

    PcapngReader reader;
    PcapPlaybackRequest request;
    request.files = QStringList{pcapPath};
    request.playback_speed = 4.0;
    request.seek_enabled = true;
    request.target_timestamp_sec = 100.030;
    request.preroll_sec = 0.020;
    request.scan_range = true;
    request.timeline_generation = 77;

    double rangeFirst = 0.0;
    double rangeLast = 0.0;
    std::vector<bool> prerollStates;
    std::vector<QByteArray> payloads;
    std::vector<double> timestamps;
    std::vector<quint64> timelineGenerations;
    double completedRequest = 0.0;
    double completedActual = 0.0;
    QString error;

    QObject::connect(&reader, &PcapngReader::captureRangeReady,
                     [&](double first, double last) {
        rangeFirst = first;
        rangeLast = last;
    });
    QObject::connect(&reader, &PcapngReader::seekPrerollChanged,
                     [&](bool active, double) { prerollStates.push_back(active); });
    QObject::connect(&reader, &PcapngReader::udpPacket,
                     [&](const QHostAddress &, quint16,
                         const QByteArray &payload, double timestamp) {
        payloads.push_back(payload);
        timestamps.push_back(timestamp);
    });
    QObject::connect(&reader, &PcapngReader::udpPacketWithGeneration,
                     [&](const QHostAddress &, quint16,
                         const QByteArray &, double, quint64 generation) {
        timelineGenerations.push_back(generation);
    });
    QObject::connect(&reader, &PcapngReader::seekCompleted,
                     [&](double requested, double actual) {
        completedRequest = requested;
        completedActual = actual;
    });
    QObject::connect(&reader, &PcapngReader::playbackError,
                     [&](const QString &message) { error = message; });

    reader.startPlayback(request);

    check(error.isEmpty(), "in-range raw timestamp seek must not report an error");
    check(std::abs(rangeFirst - 100.0) < 1e-6
              && std::abs(rangeLast - 100.04) < 1e-6,
          "range scan must report the first and last raw timestamps");
    check(payloads == std::vector<QByteArray>({
              QByteArray("warm-a"), QByteArray("warm-b"),
              QByteArray("target"), QByteArray("after")}),
          "fast scan must suppress old packets while preroll and target packets remain complete");
    check(timestamps.size() == 4
              && std::abs(timestamps.front() - 100.01) < 1e-6
              && std::abs(timestamps[2] - 100.03) < 1e-6
              && std::abs(timestamps.back() - 100.04) < 1e-6,
          "seek must preserve every emitted PCAP raw timestamp");
    check(timelineGenerations == std::vector<quint64>({77, 77, 77, 77}),
          "every packet must retain the playback timeline generation");
    check(prerollStates == std::vector<bool>({true, false}),
          "seek must bracket warmup packets with preroll state signals");
    check(std::abs(completedRequest - 100.03) < 1e-6
              && std::abs(completedActual - 100.03) < 1e-6,
          "seek completion must report the requested and first matching packet timestamps");
}

void testReaderRejectsTimestampBeyondCaptureRange()
{
    QTemporaryDir dir;
    const QString pcapPath = writeTimedPcap(
        dir.filePath("out-of-range-seek.pcap"),
        {
            {20.0, makeIpv4Frame(17, QByteArray("first"))},
            {20.1, makeIpv4Frame(17, QByteArray("last"))},
        });

    PcapngReader reader;
    PcapPlaybackRequest request;
    request.files = QStringList{pcapPath};
    request.seek_enabled = true;
    request.target_timestamp_sec = 21.0;
    request.preroll_sec = 2.0;
    request.scan_range = true;

    int emitted = 0;
    QString error;
    QObject::connect(&reader, &PcapngReader::udpPacket,
                     [&](const QHostAddress &, quint16, const QByteArray &, double) {
        ++emitted;
    });
    QObject::connect(&reader, &PcapngReader::playbackError,
                     [&](const QString &message) { error = message; });

    reader.startPlayback(request);

    check(emitted == 0, "out-of-range seek must not emit UDP packets");
    check(!error.isEmpty(), "out-of-range seek must report a playback error");
}

void testReaderSupportsCaptureRangeEndpoints()
{
    QTemporaryDir dir;
    const QString pcapPath = writeTimedPcap(
        dir.filePath("seek-endpoints.pcap"),
        {
            {30.0, makeIpv4Frame(17, QByteArray("first"))},
            {30.1, makeIpv4Frame(17, QByteArray("middle"))},
            {30.2, makeIpv4Frame(17, QByteArray("last"))},
        });

    auto runSeek = [&](double target) {
        PcapngReader reader;
        PcapPlaybackRequest request;
        request.files = QStringList{pcapPath};
        request.playback_speed = 4.0;
        request.seek_enabled = true;
        request.target_timestamp_sec = target;
        request.preroll_sec = 0.05;
        request.scan_range = true;

        std::vector<QByteArray> payloads;
        double actual = 0.0;
        QString error;
        QObject::connect(&reader, &PcapngReader::udpPacket,
                         [&](const QHostAddress &, quint16,
                             const QByteArray &payload, double) {
            payloads.push_back(payload);
        });
        QObject::connect(&reader, &PcapngReader::seekCompleted,
                         [&](double, double firstMatching) {
            actual = firstMatching;
        });
        QObject::connect(&reader, &PcapngReader::playbackError,
                         [&](const QString &message) { error = message; });
        reader.startPlayback(request);
        return std::make_tuple(payloads, actual, error);
    };

    const auto first = runSeek(30.0);
    check(std::get<2>(first).isEmpty()
              && std::abs(std::get<1>(first) - 30.0) < 1e-6
              && std::get<0>(first).size() == 3
              && std::get<0>(first).front() == QByteArray("first"),
          "seek to the first capture timestamp must retain the first packet");

    const auto last = runSeek(30.2);
    check(std::get<2>(last).isEmpty()
              && std::abs(std::get<1>(last) - 30.2) < 1e-6
              && std::get<0>(last) == std::vector<QByteArray>({QByteArray("last")}),
          "seek to the last capture timestamp must emit the last packet then finish");
}

void testReaderSeeksAcrossMultipleFilesInSelectionOrder()
{
    QTemporaryDir dir;
    const QString firstPath = writeTimedPcap(
        dir.filePath("part-a.pcap"),
        {
            {40.0, makeIpv4Frame(17, QByteArray("old"))},
            {40.1, makeIpv4Frame(17, QByteArray("warm-a"))},
        });
    const QString secondPath = writeTimedPcap(
        dir.filePath("part-b.pcap"),
        {
            {40.2, makeIpv4Frame(17, QByteArray("warm-b"))},
            {40.3, makeIpv4Frame(17, QByteArray("target"))},
        });

    PcapngReader reader;
    PcapPlaybackRequest request;
    request.files = QStringList{firstPath, secondPath};
    request.playback_speed = 4.0;
    request.seek_enabled = true;
    request.target_timestamp_sec = 40.25;
    request.preroll_sec = 0.20;
    request.scan_range = true;

    std::vector<QByteArray> payloads;
    double actual = 0.0;
    QObject::connect(&reader, &PcapngReader::udpPacket,
                     [&](const QHostAddress &, quint16,
                         const QByteArray &payload, double) {
        payloads.push_back(payload);
    });
    QObject::connect(&reader, &PcapngReader::seekCompleted,
                     [&](double, double firstMatching) { actual = firstMatching; });
    reader.startPlayback(request);

    check(payloads == std::vector<QByteArray>({
              QByteArray("warm-a"), QByteArray("warm-b"), QByteArray("target")}),
          "multi-file seek must preroll across files without replaying fast-scan packets");
    check(std::abs(actual - 40.3) < 1e-6,
          "multi-file seek must choose the first packet not less than the target");
}

void testReaderReportsLastReleasedPacketTimestamp()
{
    QTemporaryDir dir;
    const QString pcapPath = writeTimedPcap(
        dir.filePath("last-released.pcap"),
        {
            {60.0, makeIpv4Frame(17, QByteArray("first"))},
            {60.125, makeIpv4Frame(17, QByteArray("last"))},
        });

    PcapngReader reader;
    PcapPlaybackRequest request;
    request.files = QStringList{pcapPath};
    request.playback_speed = 4.0;
    request.scan_range = false;
    reader.startPlayback(request);

    check(std::abs(reader.lastReleasedTimestamp() - 60.125) < 1e-6,
          "reader must expose the exact timestamp of the last released packet");
}

}  // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    testConfigParsesAndClampsDefaultSpeed();
    testUsvLocalTimeAndUnixTimestampParsing();
    testPlaybackSpeedScalesCaptureIntervals();
    testHalfSpeedDoublesCaptureInterval();
    testHighRatePacketsUseOneContinuousPlaybackClock();
    testPauseBlocksAndResumeContinues();
    testStopInterruptsPauseAndTimedWait();
    testReaderPreservesUdpPayloadsAndCaptureTimestampsAt4x();
    testReaderSeeksByRawTimestampWithPreroll();
    testReaderRejectsTimestampBeyondCaptureRange();
    testReaderSupportsCaptureRangeEndpoints();
    testReaderSeeksAcrossMultipleFilesInSelectionOrder();
    testReaderReportsLastReleasedPacketTimestamp();

    if (failures != 0) {
        std::cerr << failures << " PCAP playback test assertion(s) failed\n";
        return 1;
    }
    std::cout << "All PCAP playback tests passed\n";
    return 0;
}
