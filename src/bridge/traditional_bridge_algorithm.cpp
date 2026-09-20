#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Options {
    fs::path inputDir;
    fs::path outputDir;
    std::string pattern = ".jpg";
    int maxWidth = 1280;
    int sampleCount = 0;
    unsigned int seed = std::random_device{}();
};

struct LineInfo {
    cv::Vec4i line;
    double angleDeg = 0.0;
    double length = 0.0;
};

struct DeckModel {
    bool valid = false;
    double m = 0.0;
    double b = 0.0;
    int x0 = 0;
    int x1 = 0;
};

struct PierCluster {
    int x = 0;
    int y0 = 0;
    int y1 = 0;
    int count = 0;
};

struct BridgeBandModel {
    bool valid = false;
    int topOffset = 0;
    int bottomOffset = 0;
};

struct TemporalState {
    bool hasDeck = false;
    DeckModel deck;
    BridgeBandModel band;
    std::vector<PierCluster> piers;
    int missedDeckFrames = 0;
    int missedPierFrames = 0;
};

std::vector<PierCluster> mergePierClusters(std::vector<PierCluster> clusters, int width, int height);

struct DeckCandidate {
    std::vector<cv::Point2d> points;
    double centerY = 0.0;
    double angle = 0.0;
    double supportLength = 0.0;
    int segmentCount = 0;
};

struct DarkBand {
    bool valid = false;
    int yTop = 0;
    int yBottom = 0;
    int yCenter = 0;
    double strength = 0.0;
};

void printUsage() {
    std::cout
        << "Usage:\n"
        << "  bridge_detector --input <image_dir> [--output <output_dir>] [--pattern .jpg] [--sample 100]\n\n"
        << "Example:\n"
        << "  bridge_detector --input \"F:\\\\bowei+qiao\\\\qiao\\\\qiao1\\\\left_192.168.1.223\" --sample 100\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto needValue = [&](const std::string& name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--input") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.inputDir = value;
        } else if (arg == "--output") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.outputDir = value;
        } else if (arg == "--pattern") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.pattern = value;
        } else if (arg == "--max-width") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.maxWidth = std::max(320, std::atoi(value));
        } else if (arg == "--sample") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.sampleCount = std::max(0, std::atoi(value));
        } else if (arg == "--seed") {
            const char* value = needValue(arg);
            if (!value) return false;
            opt.seed = static_cast<unsigned int>(std::strtoul(value, nullptr, 10));
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }

    if (opt.inputDir.empty()) {
        std::cerr << "--input is required.\n";
        return false;
    }
    if (opt.outputDir.empty()) {
        const std::string inputName = opt.inputDir.filename().string();
        opt.outputDir = opt.inputDir.parent_path() / ("bridge_detection_" + inputName);
    }
    return true;
}

std::string pathText(const fs::path& p) {
    return p.u8string();
}

double lineLength(const cv::Vec4i& l) {
    const double dx = static_cast<double>(l[2] - l[0]);
    const double dy = static_cast<double>(l[3] - l[1]);
    return std::sqrt(dx * dx + dy * dy);
}

double lineAngleDeg(const cv::Vec4i& l) {
    const double dx = static_cast<double>(l[2] - l[0]);
    const double dy = static_cast<double>(l[3] - l[1]);
    return std::atan2(dy, dx) * 180.0 / CV_PI;
}

cv::Mat resizeForWork(const cv::Mat& src, int maxWidth, double& scale) {
    scale = 1.0;
    if (src.cols <= maxWidth) {
        return src.clone();
    }
    scale = static_cast<double>(maxWidth) / static_cast<double>(src.cols);
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(), scale, scale, cv::INTER_AREA);
    return dst;
}

int estimateWaterlineY(const cv::Mat& image) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    const int height = image.rows;
    const int width = image.cols;
    const int yMin = static_cast<int>(height * 0.38);
    const int yMax = static_cast<int>(height * 0.68);
    const int x0 = static_cast<int>(width * 0.08);
    const int x1 = static_cast<int>(width * 0.82);

    std::vector<double> waterScore(height, 0.0);
    for (int y = yMin; y <= yMax; ++y) {
        double total = 0.0;
        int count = 0;
        for (int x = x0; x < x1; x += 5) {
            const cv::Vec3b bgr = image.at<cv::Vec3b>(y, x);
            const cv::Vec3b pix = hsv.at<cv::Vec3b>(y, x);
            const double saturation = pix[1];
            const double value = pix[2];
            const double greenBias = static_cast<double>(bgr[1]) - static_cast<double>(bgr[0]);
            const double darkness = 255.0 - value;
            total += saturation * 0.55 + greenBias * 0.45 + darkness * 0.12;
            ++count;
        }
        waterScore[y] = count > 0 ? total / count : 0.0;
    }

    std::vector<double> smooth = waterScore;
    const int radius = std::max(3, height / 160);
    for (int y = yMin; y <= yMax; ++y) {
        double total = 0.0;
        int count = 0;
        for (int yy = std::max(yMin, y - radius); yy <= std::min(yMax, y + radius); ++yy) {
            total += waterScore[yy];
            ++count;
        }
        smooth[y] = total / std::max(1, count);
    }

    int bestY = static_cast<int>(height * 0.52);
    double bestSeparation = -1e9;
    const int band = std::max(12, height / 35);
    for (int y = yMin + band; y <= yMax - band; ++y) {
        double upper = 0.0;
        double lower = 0.0;
        for (int yy = y - band; yy < y; ++yy) {
            upper += smooth[yy];
        }
        for (int yy = y; yy < y + band; ++yy) {
            lower += smooth[yy];
        }
        upper /= band;
        lower /= band;
        const double separation = lower - upper;
        if (separation > bestSeparation) {
            bestSeparation = separation;
            bestY = y;
        }
    }

    return std::clamp(bestY, static_cast<int>(height * 0.42), static_cast<int>(height * 0.64));
}

cv::Mat makeEdges(const cv::Mat& image, int waterlineY) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat enhanced;
    auto clahe = cv::createCLAHE(2.2, cv::Size(8, 8));
    clahe->apply(gray, enhanced);

    cv::Mat blur;
    cv::GaussianBlur(enhanced, blur, cv::Size(3, 3), 0.0);

    cv::Mat edges;
    cv::Canny(blur, edges, 45, 130, 3);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 2));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);

    if (waterlineY > 0) {
        const int maskStart = std::clamp(waterlineY + image.rows / 80, 0, image.rows - 1);
        cv::rectangle(edges, cv::Rect(0, maskStart, edges.cols, edges.rows - maskStart), cv::Scalar(0), cv::FILLED);
    }
    return edges;
}

std::vector<LineInfo> detectLines(const cv::Mat& edges) {
    std::vector<cv::Vec4i> raw;
    cv::HoughLinesP(edges, raw, 1.0, CV_PI / 180.0, 28, 18, 18);

    std::vector<LineInfo> lines;
    lines.reserve(raw.size());
    for (const auto& l : raw) {
        LineInfo info;
        info.line = l;
        info.angleDeg = lineAngleDeg(l);
        info.length = lineLength(l);
        lines.push_back(info);
    }
    return lines;
}

double meanDarknessOnLine(const cv::Mat& gray, double m, double b, int x0, int x1) {
    const int step = std::max(3, gray.cols / 180);
    double total = 0.0;
    int count = 0;
    for (int x = x0; x <= x1; x += step) {
        const int y = static_cast<int>(std::round(m * x + b));
        if (y < 2 || y >= gray.rows - 2) {
            continue;
        }
        uchar v0 = gray.at<uchar>(y, x);
        uchar v1 = gray.at<uchar>(std::min(gray.rows - 1, y + 2), x);
        total += 255.0 - std::min(v0, v1);
        ++count;
    }
    return count > 0 ? total / count : 0.0;
}

DarkBand estimateDarkBridgeBand(const cv::Mat& gray) {
    DarkBand band;
    const int width = gray.cols;
    const int height = gray.rows;
    const int yMin = static_cast<int>(height * 0.16);
    const int yMax = static_cast<int>(height * 0.50);
    const int xMargin = static_cast<int>(width * 0.04);

    std::vector<double> rowDarkness(height, 0.0);
    for (int y = yMin; y <= yMax; ++y) {
        double total = 0.0;
        int count = 0;
        for (int x = xMargin; x < width - xMargin; x += 4) {
            const uchar v = gray.at<uchar>(y, x);
            if (v < 145) {
                total += 145.0 - v;
            }
            ++count;
        }
        rowDarkness[y] = count > 0 ? total / count : 0.0;
    }

    std::vector<double> smooth = rowDarkness;
    for (int y = yMin + 2; y <= yMax - 2; ++y) {
        smooth[y] = (rowDarkness[y - 2] + rowDarkness[y - 1] + rowDarkness[y] +
                     rowDarkness[y + 1] + rowDarkness[y + 2]) / 5.0;
    }

    int bestY = yMin;
    double best = 0.0;
    for (int y = yMin; y <= yMax; ++y) {
        if (smooth[y] > best) {
            best = smooth[y];
            bestY = y;
        }
    }
    if (best < 5.0) {
        return band;
    }

    int top = bestY;
    int bottom = bestY;
    const double threshold = std::max(3.0, best * 0.36);
    while (top > yMin && smooth[top - 1] > threshold) {
        --top;
    }
    while (bottom < yMax && smooth[bottom + 1] > threshold) {
        ++bottom;
    }

    band.valid = true;
    band.yTop = top;
    band.yBottom = bottom;
    band.yCenter = (top + bottom) / 2;
    band.strength = best;
    return band;
}

int countNearbyVerticals(const std::vector<LineInfo>& lines, double m, double b, int x0, int x1, int width, int height) {
    int count = 0;
    for (const auto& info : lines) {
        const auto& l = info.line;
        const double angle = std::abs(info.angleDeg);
        const bool vertical = angle > 76.0 && angle < 104.0;
        if (!vertical || info.length < height * 0.035) {
            continue;
        }

        const int xMid = (l[0] + l[2]) / 2;
        const bool outsideExpandedSpan = xMid < x0 - width * 0.18 || xMid > x1 + width * 0.12;
        if (outsideExpandedSpan) {
            continue;
        }

        const int yTop = std::min(l[1], l[3]);
        const int yBottom = std::max(l[1], l[3]);
        const double deckY = m * xMid + b;
        const bool crossesDeck = yTop < deckY + height * 0.09 && yBottom > deckY - height * 0.10;
        const bool tallTowerNearDeck = info.length > height * 0.10 && yTop < height * 0.52 && yBottom > deckY - height * 0.12;
        if (crossesDeck || tallTowerNearDeck) {
            ++count;
        }
    }
    return count;
}

DeckModel fitDeckModel(const std::vector<cv::Point2d>& points, int width) {
    DeckModel model;
    if (points.size() < 4) {
        return model;
    }

    int minX = width;
    int maxX = 0;
    double sumX = 0.0;
    double sumY = 0.0;
    for (const auto& p : points) {
        sumX += p.x;
        sumY += p.y;
        minX = std::min(minX, static_cast<int>(p.x));
        maxX = std::max(maxX, static_cast<int>(p.x));
    }

    const double meanX = sumX / points.size();
    const double meanY = sumY / points.size();
    double sxx = 0.0;
    double sxy = 0.0;
    for (const auto& p : points) {
        sxx += (p.x - meanX) * (p.x - meanX);
        sxy += (p.x - meanX) * (p.y - meanY);
    }

    model.m = (sxx > 1e-6) ? (sxy / sxx) : 0.0;
    model.b = meanY - model.m * meanX;
    model.x0 = std::clamp(minX - width / 35, 0, width - 1);
    model.x1 = std::clamp(maxX + width / 35, 0, width - 1);
    model.valid = true;
    return model;
}

void extendDeckToVisibleFrame(DeckModel& model, int width, int height) {
    if (!model.valid) {
        return;
    }

    int left = model.x0;
    int right = model.x1;
    const int yMin = static_cast<int>(height * 0.08);
    const int yMax = static_cast<int>(height * 0.66);

    for (int x = 0; x < width; ++x) {
        const int y = static_cast<int>(std::round(model.m * x + model.b));
        if (y >= yMin && y <= yMax) {
            left = x;
            break;
        }
    }
    for (int x = width - 1; x >= 0; --x) {
        const int y = static_cast<int>(std::round(model.m * x + model.b));
        if (y >= yMin && y <= yMax) {
            right = x;
            break;
        }
    }

    if (right - left > model.x1 - model.x0) {
        model.x0 = left;
        model.x1 = right;
    }
}

DeckModel estimateDeck(const std::vector<LineInfo>& lines, const cv::Mat& gray) {
    const int width = gray.cols;
    const int height = gray.rows;
    std::vector<DeckCandidate> candidates;
    const DarkBand darkBand = estimateDarkBridgeBand(gray);

    const int yMin = static_cast<int>(height * 0.18);
    const int yMax = static_cast<int>(height * 0.57);

    for (const auto& info : lines) {
        const auto& l = info.line;
        const double angle = std::abs(info.angleDeg);
        const double normalizedAngle = std::min(angle, std::abs(180.0 - angle));
        const double yMid = (l[1] + l[3]) * 0.5;
        if (normalizedAngle > 12.0 || info.length < width * 0.020 || yMid < yMin || yMid > yMax) {
            continue;
        }

        const double dx = static_cast<double>(l[2] - l[0]);
        const double dy = static_cast<double>(l[3] - l[1]);
        if (std::abs(dx) < 1.0) {
            continue;
        }
        const double slope = dy / dx;
        const double xMid = (l[0] + l[2]) * 0.5;
        const double projectedCenterY = yMid - slope * (xMid - width * 0.5);
        if (projectedCenterY < yMin - height * 0.06 || projectedCenterY > yMax + height * 0.06) {
            continue;
        }

        auto sameBand = [&](const DeckCandidate& c) {
            return std::abs(c.centerY - projectedCenterY) <= height * 0.040 &&
                   std::abs(c.angle - info.angleDeg) <= 5.5;
        };
        auto it = std::find_if(candidates.begin(), candidates.end(), sameBand);
        if (it == candidates.end()) {
            DeckCandidate candidate;
            candidate.centerY = projectedCenterY;
            candidate.angle = info.angleDeg;
            candidate.points.emplace_back(l[0], l[1]);
            candidate.points.emplace_back(l[2], l[3]);
            candidate.supportLength = info.length;
            candidate.segmentCount = 1;
            candidates.push_back(candidate);
        } else {
            const double oldWeight = std::max(1.0, it->supportLength);
            it->points.emplace_back(l[0], l[1]);
            it->points.emplace_back(l[2], l[3]);
            it->centerY = (it->centerY * oldWeight + projectedCenterY * info.length) / (oldWeight + info.length);
            it->angle = (it->angle * oldWeight + info.angleDeg * info.length) / (oldWeight + info.length);
            it->supportLength += info.length;
            it->segmentCount += 1;
        }
    }

    DeckModel best;
    double bestScore = -1.0;
    double bestCandidateY = 0.0;
    double bestDarkness = 0.0;
    int bestVerticalCount = 0;
    bool bestTouchesDarkBand = false;
    for (const auto& candidate : candidates) {
        DeckModel model = fitDeckModel(candidate.points, width);
        if (!model.valid || model.x1 - model.x0 < width * 0.18) {
            continue;
        }

        const double darkness = meanDarknessOnLine(gray, model.m, model.b, model.x0, model.x1);
        const int verticalCount = countNearbyVerticals(lines, model.m, model.b, model.x0, model.x1, width, height);
        const double extentScore = static_cast<double>(model.x1 - model.x0) / width;
        const double bandDistance = darkBand.valid ? std::abs(candidate.centerY - darkBand.yCenter) : 0.0;
        const double belowBand = darkBand.valid ? std::max(0.0, candidate.centerY - (darkBand.yBottom + height * 0.045)) : 0.0;
        const bool touchesDarkBand = darkBand.valid &&
                                     candidate.centerY >= darkBand.yTop - height * 0.05 &&
                                     candidate.centerY <= darkBand.yBottom + height * 0.08;

        // The sea horizon is often long and horizontal, but it is usually pale and
        // lower in the image. Real bridge deck edges are darker and have nearby piers.
        const double lowerBandPenalty = std::max(0.0, (candidate.centerY - height * 0.47) / (height * 0.10)) * 100.0;
        const double darkBandPenalty = darkBand.valid ? (bandDistance * 2.2 + belowBand * 7.5) : 0.0;
        const double darkBandBonus = touchesDarkBand ? 260.0 : 0.0;
        const double score = candidate.supportLength * 0.55
                           + extentScore * 260.0
                           + darkness * 3.0
                           + verticalCount * 120.0
                           + candidate.segmentCount * 18.0
                           + darkBandBonus
                           - lowerBandPenalty
                           - darkBandPenalty;
        if (score > bestScore) {
            bestScore = score;
            best = model;
            bestCandidateY = candidate.centerY;
            bestDarkness = darkness;
            bestVerticalCount = verticalCount;
            bestTouchesDarkBand = touchesDarkBand;
        }
    }

    if (!best.valid) {
        return best;
    }

    const bool lowerWaterlineCandidate = bestCandidateY > height * 0.48;
    const bool weakBridgeEvidence = bestVerticalCount == 0 && !bestTouchesDarkBand;
    const bool weakVisualContrast = bestDarkness < 42.0 && (!darkBand.valid || darkBand.strength < 8.0);
    const bool unsupportedLowerCandidate = bestVerticalCount == 0 && bestCandidateY > height * 0.45 &&
                                           !(bestTouchesDarkBand && bestDarkness > 80.0);
    const bool unsupportedMidOrLowerCandidate = bestVerticalCount == 0 && bestCandidateY > height * 0.38 &&
                                                bestDarkness < 95.0;
    if (weakBridgeEvidence && (lowerWaterlineCandidate || weakVisualContrast)) {
        return DeckModel{};
    }
    if (unsupportedLowerCandidate || unsupportedMidOrLowerCandidate) {
        return DeckModel{};
    }

    extendDeckToVisibleFrame(best, width, height);
    return best;
}

double deckYAt(const DeckModel& deck, int x) {
    return deck.m * x + deck.b;
}

double deckCenterY(const DeckModel& deck, int width) {
    return deckYAt(deck, width / 2);
}

DeckModel smoothDeckModel(const DeckModel& current, const DeckModel& previous, int width, int height) {
    if (!current.valid || !previous.valid) {
        return current.valid ? current : previous;
    }

    const double centerDiff = std::abs(deckCenterY(current, width) - deckCenterY(previous, width));
    const double slopeDiff = std::abs(current.m - previous.m);
    if (centerDiff > height * 0.12 || slopeDiff > 0.10) {
        return current;
    }

    DeckModel smoothed = current;
    constexpr double alpha = 0.72;
    smoothed.m = previous.m * (1.0 - alpha) + current.m * alpha;
    smoothed.b = previous.b * (1.0 - alpha) + current.b * alpha;
    smoothed.x0 = static_cast<int>(std::round(previous.x0 * (1.0 - alpha) + current.x0 * alpha));
    smoothed.x1 = static_cast<int>(std::round(previous.x1 * (1.0 - alpha) + current.x1 * alpha));
    smoothed.x0 = std::clamp(smoothed.x0, 0, width - 1);
    smoothed.x1 = std::clamp(smoothed.x1, smoothed.x0 + 1, width - 1);
    smoothed.valid = true;
    return smoothed;
}

BridgeBandModel smoothBridgeBand(const BridgeBandModel& current, const BridgeBandModel& previous) {
    if (!current.valid || !previous.valid) {
        return current.valid ? current : previous;
    }

    BridgeBandModel smoothed = current;
    constexpr double alpha = 0.72;
    smoothed.topOffset = static_cast<int>(std::round(previous.topOffset * (1.0 - alpha) + current.topOffset * alpha));
    smoothed.bottomOffset = static_cast<int>(std::round(previous.bottomOffset * (1.0 - alpha) + current.bottomOffset * alpha));
    smoothed.bottomOffset = std::max(smoothed.bottomOffset, smoothed.topOffset + 8);
    smoothed.valid = true;
    return smoothed;
}

std::vector<PierCluster> smoothPierClusters(const std::vector<PierCluster>& current, const std::vector<PierCluster>& previous, int width) {
    if (current.empty() || previous.empty()) {
        return current;
    }

    std::vector<PierCluster> result = current;
    const int matchGap = std::max(10, width / 45);
    constexpr double alpha = 0.70;

    for (auto& pier : result) {
        auto best = previous.end();
        int bestDx = matchGap + 1;
        for (auto it = previous.begin(); it != previous.end(); ++it) {
            const int dx = std::abs(it->x - pier.x);
            if (dx < bestDx) {
                bestDx = dx;
                best = it;
            }
        }
        if (best != previous.end() && bestDx <= matchGap) {
            pier.x = static_cast<int>(std::round(best->x * (1.0 - alpha) + pier.x * alpha));
            pier.y0 = static_cast<int>(std::round(best->y0 * (1.0 - alpha) + pier.y0 * alpha));
            pier.y1 = static_cast<int>(std::round(best->y1 * (1.0 - alpha) + pier.y1 * alpha));
            pier.count = std::max(pier.count, best->count);
        }
    }
    return result;
}

void applyTemporalMemory(TemporalState& state, DeckModel& deck, BridgeBandModel& band, std::vector<PierCluster>& piers, int width, int height) {
    if (deck.valid) {
        if (state.hasDeck && state.missedDeckFrames <= 3) {
            deck = smoothDeckModel(deck, state.deck, width, height);
            band = smoothBridgeBand(band, state.band);
        }

        if (!piers.empty()) {
            if (state.hasDeck && state.missedPierFrames <= 5) {
                piers = smoothPierClusters(piers, state.piers, width);
                piers = mergePierClusters(piers, width, height);
            }
            state.missedPierFrames = 0;
        } else if (state.hasDeck && state.missedPierFrames < 3 &&
                   std::abs(deckCenterY(deck, width) - deckCenterY(state.deck, width)) < height * 0.06) {
            piers = state.piers;
            ++state.missedPierFrames;
        } else {
            ++state.missedPierFrames;
        }

        state.hasDeck = true;
        state.deck = deck;
        state.band = band;
        state.piers = piers;
        state.missedDeckFrames = 0;
        return;
    }

    state.hasDeck = false;
    state.piers.clear();
    state.missedDeckFrames = 0;
    state.missedPierFrames = 0;
}

bool hasReliableBridgeEvidence(const DeckModel& deck, const BridgeBandModel& band, const std::vector<PierCluster>& piers, int width, int height, int waterlineY) {
    if (!deck.valid || !band.valid) {
        return false;
    }

    if (!piers.empty()) {
        return true;
    }

    const int sampleStep = std::max(32, width / 12);
    int supportedSamples = 0;
    int totalSamples = 0;
    for (int x = deck.x0; x <= deck.x1; x += sampleStep) {
        const int deckY = static_cast<int>(std::round(deckYAt(deck, x)));
        const int bridgeBottom = deckY + band.bottomOffset;
        if (bridgeBottom < waterlineY - height / 45) {
            ++supportedSamples;
        }
        ++totalSamples;
    }

    const bool bridgeBodyClearlyAboveWater = supportedSamples >= std::max(2, totalSamples * 2 / 3);
    const double nearHorizontal = std::abs(deck.m) < 0.018;
    const double deckAtCenter = deckYAt(deck, width / 2);
    const bool closeToWaterline = std::abs(deckAtCenter - waterlineY) < height * 0.075;

    if (nearHorizontal && closeToWaterline && !bridgeBodyClearlyAboveWater) {
        return false;
    }

    return bridgeBodyClearlyAboveWater;
}

BridgeBandModel estimateBridgeBand(const cv::Mat& gray, const DeckModel& deck) {
    BridgeBandModel band;
    if (!deck.valid) {
        return band;
    }

    const int height = gray.rows;
    const int minOffset = -std::max(8, height / 26);
    const int maxOffset = std::max(12, height / 12);
    const int sampleStep = std::max(3, gray.cols / 220);
    std::vector<double> profile(maxOffset - minOffset + 1, 0.0);

    for (int off = minOffset; off <= maxOffset; ++off) {
        double total = 0.0;
        int count = 0;
        for (int x = deck.x0; x <= deck.x1; x += sampleStep) {
            const int y = static_cast<int>(std::round(deckYAt(deck, x))) + off;
            if (y < 1 || y >= height - 1) {
                continue;
            }
            const uchar v = gray.at<uchar>(y, x);
            if (v < 155) {
                total += 155.0 - v;
            }
            ++count;
        }
        profile[off - minOffset] = count > 0 ? total / count : 0.0;
    }

    std::vector<double> smooth = profile;
    for (int i = 2; i + 2 < static_cast<int>(profile.size()); ++i) {
        smooth[i] = (profile[i - 2] + profile[i - 1] + profile[i] + profile[i + 1] + profile[i + 2]) / 5.0;
    }

    int bestIdx = 0;
    double best = 0.0;
    for (int i = 0; i < static_cast<int>(smooth.size()); ++i) {
        if (smooth[i] > best) {
            best = smooth[i];
            bestIdx = i;
        }
    }

    if (best < 2.5) {
        band.valid = true;
        band.topOffset = -std::max(4, height / 160);
        band.bottomOffset = std::max(14, height / 35);
        return band;
    }

    const double threshold = std::max(1.6, best * 0.34);
    int topIdx = bestIdx;
    int bottomIdx = bestIdx;
    while (topIdx > 0 && smooth[topIdx - 1] > threshold) {
        --topIdx;
    }
    while (bottomIdx + 1 < static_cast<int>(smooth.size()) && smooth[bottomIdx + 1] > threshold) {
        ++bottomIdx;
    }

    band.valid = true;
    band.topOffset = std::clamp(topIdx + minOffset - height / 120, minOffset, maxOffset);
    band.bottomOffset = std::clamp(bottomIdx + minOffset + height / 90, band.topOffset + 8, maxOffset);
    return band;
}

std::vector<PierCluster> detectPierColumnsFromEdges(const cv::Mat& edges, const DeckModel& deck, const BridgeBandModel& band, int width, int height, int waterlineY) {
    std::vector<double> score(width, 0.0);
    const int xStart = std::clamp(deck.x0 - width / 35, 0, width - 1);
    const int xEnd = std::clamp(deck.x1 + width / 35, 0, width - 1);
    const int waterCap = std::clamp(waterlineY - height / 140, 0, height - 1);

    for (int x = xStart; x <= xEnd; ++x) {
        const int deckY = static_cast<int>(std::round(deckYAt(deck, x)));
        const int bridgeBottom = deckY + (band.valid ? band.bottomOffset : height / 35);
        const int y0 = std::clamp(bridgeBottom + height / 80, 0, height - 1);
        const int searchBottom = std::min(waterCap, std::min(bridgeBottom + height / 5, static_cast<int>(height * 0.74)));
        const int y1 = std::clamp(searchBottom, 0, height - 1);
        if (y1 <= y0 + height / 60) {
            continue;
        }

        int edgeCount = 0;
        int longestRun = 0;
        int run = 0;
        for (int y = y0; y <= y1; ++y) {
            bool hasEdge = false;
            for (int xx = std::max(0, x - 1); xx <= std::min(width - 1, x + 1); ++xx) {
                if (edges.at<uchar>(y, xx) > 0) {
                    hasEdge = true;
                    break;
                }
            }
            if (hasEdge) {
                ++edgeCount;
                ++run;
                longestRun = std::max(longestRun, run);
            } else {
                run = 0;
            }
        }
        const double span = static_cast<double>(y1 - y0 + 1);
        score[x] = edgeCount / span + longestRun / span * 0.7;
    }

    std::vector<double> smooth = score;
    const int radius = std::max(2, width / 360);
    for (int x = xStart; x <= xEnd; ++x) {
        double total = 0.0;
        int count = 0;
        for (int xx = std::max(xStart, x - radius); xx <= std::min(xEnd, x + radius); ++xx) {
            total += score[xx];
            ++count;
        }
        smooth[x] = total / std::max(1, count);
    }

    double mean = 0.0;
    int count = 0;
    for (int x = xStart; x <= xEnd; ++x) {
        mean += smooth[x];
        ++count;
    }
    mean /= std::max(1, count);

    double variance = 0.0;
    for (int x = xStart; x <= xEnd; ++x) {
        const double d = smooth[x] - mean;
        variance += d * d;
    }
    const double stdev = std::sqrt(variance / std::max(1, count));
    const double threshold = std::max(0.045, mean + stdev * 0.55);

    std::vector<PierCluster> columns;
    int start = -1;
    for (int x = xStart; x <= xEnd + 1; ++x) {
        const bool active = x <= xEnd && smooth[x] >= threshold;
        if (active && start < 0) {
            start = x;
        }
        if ((!active || x == xEnd + 1) && start >= 0) {
            const int end = x - 1;
            const int center = (start + end) / 2;
            const int deckY = static_cast<int>(std::round(deckYAt(deck, center)));
            const int bridgeBottom = deckY + (band.valid ? band.bottomOffset : height / 35);
            int top = height - 1;
            int bottom = 0;
            int edgePixels = 0;
            const int searchTop = std::clamp(bridgeBottom + height / 80, 0, height - 1);
            const int searchBottom = std::min(waterCap, std::min(bridgeBottom + height / 5, static_cast<int>(height * 0.74)));
            const int underBridgeSpan = std::max(1, searchBottom - searchTop);
            for (int xx = std::max(xStart, start - 1); xx <= std::min(xEnd, end + 1); ++xx) {
                for (int y = searchTop; y <= searchBottom; ++y) {
                    if (edges.at<uchar>(y, xx) > 0) {
                        top = std::min(top, y);
                        bottom = std::max(bottom, y);
                        ++edgePixels;
                    }
                }
            }
            const bool reachesWaterSide = bottom >= waterCap - height / 26;
            const bool crossesEnoughUnderBridge = (bottom - top) >= std::max(static_cast<int>(height * 0.045), underBridgeSpan * 45 / 100);
            if (edgePixels >= 3 && end - start >= 1 && crossesEnoughUnderBridge && reachesWaterSide) {
                PierCluster c;
                c.x = center;
                c.y0 = std::clamp(top - height / 140, 0, height - 1);
                c.y1 = std::clamp(bottom + height / 100, 0, height - 1);
                c.count = std::max(1, end - start + 1);
                columns.push_back(c);
            }
            start = -1;
        }
    }

    return columns;
}

std::vector<PierCluster> mergePierClusters(std::vector<PierCluster> clusters, int width, int height) {
    if (clusters.empty()) {
        return clusters;
    }

    std::sort(clusters.begin(), clusters.end(), [](const PierCluster& a, const PierCluster& b) {
        return a.x < b.x;
    });

    std::vector<PierCluster> merged;
    const int mergeGap = std::max(6, width / 130);
    for (const auto& p : clusters) {
        if (merged.empty() || std::abs(merged.back().x - p.x) > mergeGap) {
            merged.push_back(p);
            continue;
        }
        auto& c = merged.back();
        const int total = std::max(1, c.count + p.count);
        c.x = (c.x * c.count + p.x * p.count) / total;
        c.y0 = std::min(c.y0, p.y0);
        c.y1 = std::max(c.y1, p.y1);
        c.count = total;
    }

    merged.erase(std::remove_if(merged.begin(), merged.end(), [&](const PierCluster& c) {
        const int h = c.y1 - c.y0;
        const int minHeight = (c.y0 < height * 0.40) ? static_cast<int>(height * 0.075) : static_cast<int>(height * 0.030);
        return h < minHeight && c.count < 2;
    }), merged.end());

    return merged;
}

std::vector<PierCluster> clusterVerticals(const std::vector<LineInfo>& lines, const DeckModel& deck, const BridgeBandModel& band, int width, int height, int waterlineY) {
    std::vector<PierCluster> candidates;
    const int searchYMax = static_cast<int>(height * 0.74);

    for (const auto& info : lines) {
        const auto& l = info.line;
        const double angle = std::abs(info.angleDeg);
        const bool vertical = angle > 78.0 && angle < 102.0;
        const int xMid = (l[0] + l[2]) / 2;
        const int yTop = std::min(l[1], l[3]);
        const int yBottom = std::max(l[1], l[3]);
        const double deckY = deckYAt(deck, xMid);
        const int bridgeBottom = static_cast<int>(std::round(deckY)) + (band.valid ? band.bottomOffset : height / 35);
        const int waterCappedBottom = std::max(0, waterlineY - height / 120);

        if (!vertical || info.length < height * 0.025) {
            continue;
        }

        const bool likelyBoatForeground = (xMid > width * 0.80 && yTop > height * 0.52) || yTop > height * 0.68;
        if (likelyBoatForeground) {
            continue;
        }

        const bool insideDeckSpan = xMid >= deck.x0 - width * 0.06 && xMid <= deck.x1 + width * 0.06;
        const bool crossesDeck = yTop < deckY + height * 0.08 && yBottom > deckY - height * 0.08;
        const bool tallOuterTower = info.length > height * 0.10 &&
                                    yTop < height * 0.52 &&
                                    yBottom > deckY - height * 0.10 &&
                                    yBottom < height * 0.76;
        const int underBridgeSpan = std::max(1, waterCappedBottom - bridgeBottom);
        const bool reachesWaterSide = yBottom >= waterCappedBottom - height / 24;
        const bool extendsBelowBridgeBody = yBottom > bridgeBottom + std::max(static_cast<int>(height * 0.035), underBridgeSpan * 40 / 100);
        if ((!insideDeckSpan && !tallOuterTower) || (!crossesDeck && !tallOuterTower) || yTop > searchYMax) {
            continue;
        }
        if (!tallOuterTower && (!extendsBelowBridgeBody || !reachesWaterSide)) {
            continue;
        }

        const int structureCappedBottom = static_cast<int>(std::round(deckY + (tallOuterTower ? height * 0.14 : height * 0.10)));
        const int shadowCappedBottom = tallOuterTower ? std::min(structureCappedBottom, waterCappedBottom) : waterCappedBottom;
        const int cappedBottom = std::min(yBottom, shadowCappedBottom);
        const int cappedTop = tallOuterTower ? std::min(yTop, static_cast<int>(deckY)) : std::max(yTop, bridgeBottom - height / 120);
        if (cappedBottom - cappedTop < height * 0.035) {
            continue;
        }

        PierCluster p;
        p.x = xMid;
        p.y0 = cappedTop;
        p.y1 = cappedBottom;
        p.count = 1;
        candidates.push_back(p);
    }

    std::sort(candidates.begin(), candidates.end(), [](const PierCluster& a, const PierCluster& b) {
        return a.x < b.x;
    });

    std::vector<PierCluster> clusters;
    const int mergeGap = std::max(8, width / 90);
    for (const auto& p : candidates) {
        if (clusters.empty() || std::abs(clusters.back().x - p.x) > mergeGap) {
            clusters.push_back(p);
            continue;
        }
        auto& c = clusters.back();
        c.x = (c.x * c.count + p.x) / (c.count + 1);
        c.y0 = std::min(c.y0, p.y0);
        c.y1 = std::max(c.y1, p.y1);
        c.count += 1;
    }

    clusters.erase(std::remove_if(clusters.begin(), clusters.end(), [&](const PierCluster& c) {
        const int minHeight = (c.y0 < height * 0.40) ? static_cast<int>(height * 0.10) : static_cast<int>(height * 0.035);
        return c.y1 - c.y0 < minHeight && c.count < 2;
    }), clusters.end());

    return mergePierClusters(clusters, width, height);
}

void drawDetection(cv::Mat& image, const DeckModel& deck, const BridgeBandModel& band, const std::vector<PierCluster>& piers) {
    if (!deck.valid) {
        cv::putText(image, "bridge deck not found", {24, 42}, cv::FONT_HERSHEY_SIMPLEX, 0.9, {0, 0, 255}, 2, cv::LINE_AA);
        return;
    }

    const cv::Scalar deckColor(0, 220, 255);
    const cv::Scalar bodyColor(0, 165, 255);
    const cv::Scalar pierColor(40, 220, 40);
    const cv::Scalar towerColor(255, 220, 0);

    if (band.valid) {
        std::vector<cv::Point> poly = {
            {deck.x0, static_cast<int>(std::round(deckYAt(deck, deck.x0))) + band.topOffset},
            {deck.x1, static_cast<int>(std::round(deckYAt(deck, deck.x1))) + band.topOffset},
            {deck.x1, static_cast<int>(std::round(deckYAt(deck, deck.x1))) + band.bottomOffset},
            {deck.x0, static_cast<int>(std::round(deckYAt(deck, deck.x0))) + band.bottomOffset},
        };
        cv::Mat overlay = image.clone();
        cv::fillConvexPoly(overlay, poly, bodyColor, cv::LINE_AA);
        cv::addWeighted(overlay, 0.22, image, 0.78, 0.0, image);
        cv::polylines(image, poly, true, bodyColor, 2, cv::LINE_AA);
    }

    cv::Point p0(deck.x0, static_cast<int>(std::round(deckYAt(deck, deck.x0))));
    cv::Point p1(deck.x1, static_cast<int>(std::round(deckYAt(deck, deck.x1))));
    cv::line(image, p0, p1, deckColor, 4, cv::LINE_AA);
    cv::putText(image, "bridge body", p0 + cv::Point(8, -12), cv::FONT_HERSHEY_SIMPLEX, 0.65, deckColor, 2, cv::LINE_AA);

    for (const auto& pier : piers) {
        const int h = pier.y1 - pier.y0;
        const bool tower = h > image.rows * 0.20 || (pier.y0 < image.rows * 0.30 && h > image.rows * 0.13);
        const int halfWidth = tower ? 13 : std::max(5, image.cols / 260);
        const cv::Scalar color = tower ? towerColor : pierColor;
        cv::Rect box(
            std::clamp(pier.x - halfWidth, 0, image.cols - 1),
            std::clamp(pier.y0, 0, image.rows - 1),
            std::min(halfWidth * 2, image.cols - std::clamp(pier.x - halfWidth, 0, image.cols - 1)),
            std::min(std::max(1, h), image.rows - std::clamp(pier.y0, 0, image.rows - 1)));
        cv::rectangle(image, box, color, 2, cv::LINE_AA);
        cv::line(image, {pier.x, pier.y0}, {pier.x, pier.y1}, color, 2, cv::LINE_AA);
    }

    cv::putText(image, "orange: bridge body  green: pier  cyan: tower", {24, image.rows - 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65, pierColor, 2, cv::LINE_AA);
}

bool hasPattern(const fs::path& p, const std::string& pattern) {
    std::string ext = p.extension().string();
    std::string pat = pattern;
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::transform(pat.begin(), pat.end(), pat.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext == pat;
}

bool processImage(const fs::path& input, const fs::path& output, const Options& opt, TemporalState& temporalState) {
    cv::Mat original = cv::imread(pathText(input), cv::IMREAD_COLOR);
    if (original.empty()) {
        std::cerr << "Failed to read: " << pathText(input) << "\n";
        return false;
    }

    double scale = 1.0;
    cv::Mat work = resizeForWork(original, opt.maxWidth, scale);
    cv::Mat gray;
    cv::cvtColor(work, gray, cv::COLOR_BGR2GRAY);
    const int waterlineY = estimateWaterlineY(work);
    cv::Mat edges = makeEdges(work, waterlineY);
    auto lines = detectLines(edges);
    auto deck = estimateDeck(lines, gray);
    BridgeBandModel band = deck.valid ? estimateBridgeBand(gray, deck) : BridgeBandModel{};
    auto piers = deck.valid ? clusterVerticals(lines, deck, band, work.cols, work.rows, waterlineY) : std::vector<PierCluster>{};
    if (deck.valid) {
        auto columnPiers = detectPierColumnsFromEdges(edges, deck, band, work.cols, work.rows, waterlineY);
        piers.insert(piers.end(), columnPiers.begin(), columnPiers.end());
        piers = mergePierClusters(piers, work.cols, work.rows);
        if (!hasReliableBridgeEvidence(deck, band, piers, work.cols, work.rows, waterlineY)) {
            deck = DeckModel{};
            band = BridgeBandModel{};
            piers.clear();
        }
    }
    applyTemporalMemory(temporalState, deck, band, piers, work.cols, work.rows);

    cv::Mat annotated = work.clone();
    drawDetection(annotated, deck, band, piers);

    if (scale != 1.0) {
        cv::resize(annotated, annotated, original.size(), 0, 0, cv::INTER_LINEAR);
    }

    if (!cv::imwrite(pathText(output), annotated)) {
        std::cerr << "Failed to write: " << pathText(output) << "\n";
        return false;
    }

    std::cout << input.filename().string() << ": deck=" << (deck.valid ? "yes" : "no")
              << ", vertical_groups=" << piers.size() << "\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        printUsage();
        return 2;
    }

    if (!fs::exists(opt.inputDir) || !fs::is_directory(opt.inputDir)) {
        std::cerr << "Input directory does not exist: " << pathText(opt.inputDir) << "\n";
        return 2;
    }

    std::error_code ec;
    fs::create_directories(opt.outputDir, ec);
    if (ec) {
        std::cerr << "Cannot create output directory: " << pathText(opt.outputDir) << " (" << ec.message() << ")\n";
        return 2;
    }

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(opt.inputDir)) {
        if (entry.is_regular_file() && hasPattern(entry.path(), opt.pattern)) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    if (opt.sampleCount > 0 && static_cast<size_t>(opt.sampleCount) < files.size()) {
        std::mt19937 rng(opt.seed);
        std::shuffle(files.begin(), files.end(), rng);
        files.resize(static_cast<size_t>(opt.sampleCount));
        std::sort(files.begin(), files.end());
    }

    if (files.empty()) {
        std::cerr << "No images matching " << opt.pattern << " in " << pathText(opt.inputDir) << "\n";
        return 1;
    }

    int ok = 0;
    TemporalState temporalState;
    for (const auto& file : files) {
        const fs::path out = opt.outputDir / (file.stem().string() + "_bridge.jpg");
        if (processImage(file, out, opt, temporalState)) {
            ++ok;
        }
    }

    std::cout << "Processed " << ok << " / " << files.size() << " images. Output: " << pathText(opt.outputDir) << "\n";
    return ok == 0 ? 1 : 0;
}
