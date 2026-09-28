#pragma once

#include "Export.h"
#include "Types.h"

#include <array>
#include <string>

namespace m2m {

enum class HandednessMap {
    Auto,
    Direct,
    Swapped
};

// MediaPipe Hands 与 XNNPACK 推理参数。
struct DetectorConfig {
    int modelComplexity = 0;
    int maximumHands = 2;
    int xnnpackThreads = 4;
    double minimumDetectionConfidence = 0.5;
    double minimumTrackingConfidence = 0.6;
};

// MANO 拟合、数值 Jacobian 并行度和提交频率。
struct ManoConfig {
    int iterations = 3;
    int jacobianWorkers = 1;
    double poseSmoothing = 0.7;
    double maximumFps = 60;
};

// One Euro 位置滤波参数。
struct PositionFilterConfig {
    double minimumCutoff = 3;
    double beta = 0.02;
    double derivativeCutoff = 2;
    double maximumSpeed = 1500;
    int medianWindow = 3;
};

struct TrackingConfig {
    int handednessConfirmationFrames = 10;
    HandednessMap handednessMap = HandednessMap::Auto;
    PositionFilterConfig positionFilter{};
};

struct DepthConfig {
    bool enabled = true;
    double referenceMillimeters = 200;
    double minimumMillimeters = 10;
    double maximumMillimeters = 600;
    int calibrationFrames = 30;
    double minimumCutoff = 1;
    double beta = 0.02;
    double maximumSpeed = 500;
    double motionGain = 1.5;
};

// 捏合、球体和深度按钮的场景参数，距离单位在字段名中明确标出。
struct InteractionConfig {
    bool enabled = true;
    double pinchEnterMeters = 0.045;
    double pinchExitMeters = 0.055;
    double pinchGrabToleranceMillimeters = 8;
    double pinchMissingTimeoutSeconds = 0.18;
    Vector3 ballCenterMillimeters{150, 0, 250};
    double ballRadiusMillimeters = 50;
    double ballMinimumScale = 0.35;
    double ballMaximumScale = 8;
    double ballScaleSensitivity = 0.8;
    Vector3 buttonCenterMillimeters{-150, 0, 350};
    double buttonWidthMillimeters = 90;
    double buttonHeightMillimeters = 60;
    double buttonTravelMillimeters = 20;
    double fingertipRadiusMillimeters = 10;
};

struct RuntimeConfig {
    bool mirrorInput = false;
    DetectorConfig detector{};
    ManoConfig mano{};
    TrackingConfig tracking{};
    DepthConfig depth{};
    InteractionConfig interaction{};
};

// 读取并校验项目 JSON profile 中与运行库有关的字段。
M2M_RUNTIME_API RuntimeConfig loadRuntimeConfig(const std::string& filePath);

}
