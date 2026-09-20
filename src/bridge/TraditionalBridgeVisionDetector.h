#pragma once

// 移植包内完整携带“跨海大桥检测”算法，通过相对路径包含算法源文件，
// 确保桥面、桥体、桥墩/桥塔、可靠性判断和时序平滑与当前最新版一致。
// main 被改名，因而不会成为当前程序的入口。
#define main traditional_bridge_detector_standalone_main
#include "traditional_bridge_algorithm.cpp"
#undef main

#include <opencv2/video/tracking.hpp>

namespace usv {

struct TraditionalBridgeVisionResult {
    bool detected = false;
    float confidence = 0.0f;
    bool hasDeckLine = false;
    cv::Point2f deckStart;
    cv::Point2f deckEnd;
    std::vector<cv::Point2f> pierCenters;
    cv::Mat overlay;
};

// A physical pier often produces two separated vertical candidates: its left
// and right edges.  The original tight clustering intentionally keeps those
// lines apart; pair only adjacent, similarly tall candidates so a wide pier is
// reported once without merging two distinct piers.
inline std::vector<PierCluster> mergePairedPierEdges(std::vector<PierCluster> piers,
                                                      int imageWidth, int imageHeight)
{
    if (piers.size() < 2)
        return piers;

    std::sort(piers.begin(), piers.end(), [](const PierCluster &a, const PierCluster &b) {
        return a.x < b.x;
    });

    const int maxEdgePairGap = std::max(10, imageWidth / 95);
    std::vector<PierCluster> merged;
    for (size_t i = 0; i < piers.size();) {
        const PierCluster &left = piers[i];
        if (i + 1 >= piers.size()) {
            merged.push_back(left);
            ++i;
            continue;
        }

        const PierCluster &right = piers[i + 1];
        const int leftHeight = std::max(1, left.y1 - left.y0);
        const int rightHeight = std::max(1, right.y1 - right.y0);
        const int verticalOverlap = std::min(left.y1, right.y1) - std::max(left.y0, right.y0);
        const int allowedEndDifference = std::max(imageHeight / 30,
                                                  std::min(leftHeight, rightHeight) / 3);
        const bool samePierEdges = right.x - left.x <= maxEdgePairGap
            && verticalOverlap >= std::min(leftHeight, rightHeight) * 7 / 10
            && std::abs(left.y0 - right.y0) <= allowedEndDifference
            && std::abs(left.y1 - right.y1) <= allowedEndDifference;
        if (!samePierEdges) {
            merged.push_back(left);
            ++i;
            continue;
        }

        PierCluster combined;
        combined.count = std::max(1, left.count + right.count);
        combined.x = (left.x * left.count + right.x * right.count) / combined.count;
        combined.y0 = std::min(left.y0, right.y0);
        combined.y1 = std::max(left.y1, right.y1);
        merged.push_back(combined);
        i += 2;
    }
    return merged;
}

class TraditionalBridgeVisionDetector {
public:
    void reset()
    {
        temporalState_ = TemporalState{};
        previousGray_.release();
        previousPierCenters_.clear();
    }

    TraditionalBridgeVisionResult detect(const cv::Mat &bgr)
    {
        TraditionalBridgeVisionResult result;
        if (bgr.empty() || bgr.type() != CV_8UC3)
            return result;

        // 与原项目 processImage 的处理顺序保持一致；每个相机实例各自保有
        // TemporalState，左右相机不会互相污染时序记忆。
        double scale = 1.0;
        cv::Mat work = resizeForWork(bgr, 1280, scale);
        cv::Mat gray;
        cv::cvtColor(work, gray, cv::COLOR_BGR2GRAY);
        const int waterlineY = estimateWaterlineY(work);
        cv::Mat edges = makeEdges(work, waterlineY);
        auto lines = detectLines(edges);
        auto deck = estimateDeck(lines, gray);
        BridgeBandModel band = deck.valid ? estimateBridgeBand(gray, deck) : BridgeBandModel{};
        auto piers = deck.valid ? clusterVerticals(lines, deck, band, work.cols, work.rows, waterlineY)
                                : std::vector<PierCluster>{};
        if (deck.valid) {
            auto columns = detectPierColumnsFromEdges(edges, deck, band, work.cols, work.rows, waterlineY);
            piers.insert(piers.end(), columns.begin(), columns.end());
            piers = mergePierClusters(piers, work.cols, work.rows);
            piers = mergePairedPierEdges(std::move(piers), work.cols, work.rows);
            piers = filterPierAppearance(work, gray, piers);

            // 水面反光会产生很多短小的“竖边”。保留的桥墩必须有 Hough 长竖线
            // 从桥体下缘连续连接到水线附近；普通桥墩要求两条边，桥塔允许一条。
            piers.erase(std::remove_if(piers.begin(), piers.end(), [&](const PierCluster &pier) {
                const int h = pier.y1 - pier.y0;
                const bool tower = h > work.rows * 0.20 ||
                                   (pier.y0 < work.rows * 0.30 && h > work.rows * 0.13);
                const double bridgeBottom = deckYAt(deck, pier.x) + band.bottomOffset;
                int supports = 0;
                for (const auto &info : lines) {
                    const double angle = std::abs(info.angleDeg);
                    if (!(angle > 78.0 && angle < 102.0) ||
                        info.length < work.rows * (tower ? 0.08 : 0.11))
                        continue;
                    const int x = (info.line[0] + info.line[2]) / 2;
                    const int y0 = std::min(info.line[1], info.line[3]);
                    const int y1 = std::max(info.line[1], info.line[3]);
                    if (std::abs(x - pier.x) > work.cols / 85 ||
                        y0 > bridgeBottom + work.rows / 35 ||
                        y1 < waterlineY - work.rows / 22)
                        continue;
                    ++supports;
                }
                return supports < (tower ? 1 : 2);
            }), piers.end());

            // 海天线在远景中也会生成横跨整幅画面的长水平线，并且常被桥塔
            // 的竖线“支持”。真实桥面通常相对水线有可见间隔或有明确斜率；
            // 若候选几乎水平、紧贴水线，且桥体带已落入水面，则统一拒绝。
            // 此规则对左右相机使用相同阈值，不依赖某一路的固定 ROI。
            const double centerY = deckCenterY(deck, work.cols);
            int supportCount = 0;
            for (const PierCluster &pier : piers) {
                const double deckBottom = deckYAt(deck, pier.x) + (band.valid ? band.bottomOffset : work.rows / 35);
                if (pier.y0 <= deckBottom + work.rows / 28 && pier.y1 >= waterlineY - work.rows / 22)
                    ++supportCount;
            }
            const int bandHeight = band.valid ? band.bottomOffset - band.topOffset : 0;
            const bool nearHorizontal = std::abs(deck.m) < 0.008;
            const bool nearWaterline = std::abs(centerY - waterlineY) < work.rows * 0.105;
            const bool lacksBridgeStructure = supportCount == 0 || bandHeight < work.rows / 48;
            const bool horizonLike = nearHorizontal && nearWaterline && lacksBridgeStructure;
            if (horizonLike) {
                deck = DeckModel{};
                band = BridgeBandModel{};
                piers.clear();
            } else if (band.valid) {
                // 桥体绘制不得吞入水面，避免即使有效桥面也画出一大片海面。
                const int maxBottom = waterlineY - static_cast<int>(std::round(centerY)) - work.rows / 90;
                band.bottomOffset = std::min(band.bottomOffset, maxBottom);
                if (band.bottomOffset <= band.topOffset + 3)
                    band = BridgeBandModel{};
            }
            if (!hasReliableBridgeEvidence(deck, band, piers, work.cols, work.rows, waterlineY)) {
                deck = DeckModel{};
                band = BridgeBandModel{};
                piers.clear();
            }
        }
        applyTemporalMemory(temporalState_, deck, band, piers, work.cols, work.rows);
        stabilizePiersWithOpticalFlow(gray, piers, work.cols);

        // Export the final temporally-smoothed centers so the bus metrics and
        // the boxes drawn on the overlay refer to the same physical piers.
        if (!piers.empty()) {
            const double inverseScale = scale > 0.0 ? 1.0 / scale : 1.0;
            result.pierCenters.reserve(piers.size());
            for (const PierCluster &pier : piers) {
                result.pierCenters.emplace_back(static_cast<float>(pier.x * inverseScale),
                                                static_cast<float>((pier.y0 + pier.y1) * 0.5 * inverseScale));
            }
        }

        // Export the final, temporally-smoothed bridge-deck segment in the
        // original camera coordinate system for YOLOE/geometry fusion.
        if (deck.valid) {
            const double inverseScale = scale > 0.0 ? 1.0 / scale : 1.0;
            result.hasDeckLine = true;
            result.deckStart = cv::Point2f(static_cast<float>(deck.x0 * inverseScale),
                                           static_cast<float>(deckYAt(deck, deck.x0) * inverseScale));
            result.deckEnd = cv::Point2f(static_cast<float>(deck.x1 * inverseScale),
                                         static_cast<float>(deckYAt(deck, deck.x1) * inverseScale));
        }

        cv::Mat annotated = work.clone();
        drawDetection(annotated, deck, band, piers);
        if (scale != 1.0)
            cv::resize(annotated, annotated, bgr.size(), 0, 0, cv::INTER_LINEAR);

        // UI 合成器接收 BGRA 图，使用原始图加标注作为该帧的完整视觉结果。
        cv::cvtColor(annotated, result.overlay, cv::COLOR_BGR2BGRA);
        result.detected = deck.valid;
        result.confidence = deck.valid ? 1.0f : 0.0f;
        return result;
    }

private:
    static std::vector<PierCluster> filterPierAppearance(const cv::Mat &bgr, const cv::Mat &gray,
                                                          const std::vector<PierCluster> &piers)
    {
        std::vector<PierCluster> accepted;
        cv::Mat hsv, gradX, edges;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        cv::Sobel(gray, gradX, CV_16S, 1, 0, 3);
        cv::Canny(gray, edges, 45, 130);
        for (const PierCluster &pier : piers) {
            const int half = std::max(5, bgr.cols / 180);
            const cv::Rect roi(std::max(0, pier.x - half), std::max(0, pier.y0),
                               std::min(bgr.cols, pier.x + half + 1) - std::max(0, pier.x - half),
                               std::min(bgr.rows, pier.y1 + 1) - std::max(0, pier.y0));
            if (roi.width < 3 || roi.height < bgr.rows / 35) continue;
            const double edgeDensity = static_cast<double>(cv::countNonZero(edges(roi))) / roi.area();
            cv::Mat absGrad; cv::convertScaleAbs(gradX(roi), absGrad);
            const double verticalEdgeStrength = cv::mean(absGrad)[0];
            const double saturation = cv::mean(hsv(roi))[1];
            // A real pier has persistent vertical contrast; weak, low-texture water glare is rejected.
            if (edgeDensity >= 0.018 || verticalEdgeStrength >= 14.0 || saturation >= 42.0)
                accepted.push_back(pier);
        }
        return accepted;
    }

    void stabilizePiersWithOpticalFlow(const cv::Mat &gray, std::vector<PierCluster> &piers, int width)
    {
        if (!previousGray_.empty() && !previousPierCenters_.empty() && !piers.empty()) {
            std::vector<cv::Point2f> tracked;
            std::vector<uchar> status; std::vector<float> error;
            cv::calcOpticalFlowPyrLK(previousGray_, gray, previousPierCenters_, tracked, status, error);
            const float matchGap = static_cast<float>(std::max(12, width / 35));
            for (size_t i = 0; i < tracked.size(); ++i) {
                if (!status[i]) continue;
                PierCluster *best = nullptr; float bestDx = matchGap;
                for (PierCluster &pier : piers) {
                    const float dx = std::abs(tracked[i].x - pier.x);
                    if (dx < bestDx) { bestDx = dx; best = &pier; }
                }
                if (best) best->x = static_cast<int>(std::round(best->x * 0.7 + tracked[i].x * 0.3));
            }
        }
        previousGray_ = gray.clone();
        previousPierCenters_.clear();
        for (const PierCluster &pier : piers)
            previousPierCenters_.emplace_back(static_cast<float>(pier.x), static_cast<float>((pier.y0 + pier.y1) * 0.5));
    }

    TemporalState temporalState_;
    cv::Mat previousGray_;
    std::vector<cv::Point2f> previousPierCenters_;
};

} // namespace usv
