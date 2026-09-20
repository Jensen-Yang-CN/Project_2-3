#include "CalibrationConfig.h"
#include "TraditionalBridgeVisionDetector.h"
#include "common_types_extended.h"

#include <opencv2/core.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
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

void testBlankFrameContract()
{
    usv::TraditionalBridgeVisionDetector detector;
    const cv::Mat blank(720, 1280, CV_8UC3, cv::Scalar::all(0));
    const usv::TraditionalBridgeVisionResult result = detector.detect(blank);
    check(!result.overlay.empty(), "blank frame must still return an overlay");
    check(result.overlay.size() == blank.size(),
          "overlay must preserve the input image size");
    check(result.overlay.type() == CV_8UC4,
          "overlay must use BGRA pixels for the Qt UI");
    check(!result.detected, "a black frame must not be reported as a bridge");
}

void testPairedPierEdgesMerge()
{
    PierCluster left;
    left.x = 100;
    left.y0 = 100;
    left.y1 = 500;
    left.count = 2;
    PierCluster right = left;
    right.x = 108;
    const std::vector<PierCluster> merged =
        usv::mergePairedPierEdges({left, right}, 1920, 1080);
    check(merged.size() == 1,
          "two close vertical edges of one pier must merge into one pier");
}

void testDualCameraCalibrationContract()
{
    const BridgeCameraCalibration left =
        CalibrationConfig::instance().bridgeCamera();
    const BridgeCameraCalibration right =
        CalibrationConfig::instance().rightCamera();
    check(left.intrinsic_k[0] > 0.0,
          "left bridge camera must have a valid focal length");
    check(right.intrinsic_k[0] > 0.0,
          "right bridge camera must have a valid focal length");

    usv::TraditionalBridgeVisionBusResult busResult;
    busResult.camera_side = usv::CameraSide::Right;
    busResult.piers.push_back(usv::BridgePierObservation{});
    check(busResult.camera_side == usv::CameraSide::Right
              && busResult.piers.size() == 1,
          "traditional bridge bus result must preserve camera side and piers");
}

}  // namespace

int main()
{
    testBlankFrameContract();
    testPairedPierEdgesMerge();
    testDualCameraCalibrationContract();
    if (failures != 0) {
        std::cerr << failures << " test(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "bridge vision update tests passed\n";
    return EXIT_SUCCESS;
}
