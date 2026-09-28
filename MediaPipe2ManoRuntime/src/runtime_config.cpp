#include <MediaPipe2Mano/RuntimeConfig.h>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <cmath>
#include <stdexcept>

namespace m2m {
namespace {

// 这些读取函数同时检查 JSON 类型，错误信息统一使用“组.字段”定位。
QJsonObject object(const QJsonObject& root, const char* key) {
    const QJsonValue value = root.value(key);
    if (!value.isObject())
        throw std::runtime_error(std::string("Missing/invalid object: ") + key);
    return value.toObject();
}

double number(const QJsonObject& root, const char* group, const char* key) {
    const QJsonValue value = object(root, group).value(key);
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error(std::string("Missing/invalid number: ") + group + "." + key);
    return value.toDouble();
}

int integer(const QJsonObject& root, const char* group, const char* key, int minimum, int maximum) {
    const double value = number(root, group, key);
    if (value != std::floor(value) || value < minimum || value > maximum)
        throw std::runtime_error(std::string("Integer out of range: ") + group + "." + key);
    return int(value);
}

bool boolean(const QJsonObject& root, const char* group, const char* key) {
    const QJsonValue value = object(root, group).value(key);
    if (!value.isBool())
        throw std::runtime_error(std::string("Missing/invalid boolean: ") + group + "." + key);
    return value.toBool();
}

std::string text(const QJsonObject& root, const char* group, const char* key) {
    const QJsonValue value = object(root, group).value(key);
    if (!value.isString())
        throw std::runtime_error(std::string("Missing/invalid string: ") + group + "." + key);
    return value.toString().toStdString();
}

Vector3 vector3(const QJsonObject& root, const char* group, const char* key) {
    const QJsonValue value = object(root, group).value(key);
    if (!value.isArray() || value.toArray().size() != 3)
        throw std::runtime_error(std::string(group) + "." + key + " must contain three numbers");
    Vector3 result;
    float* coordinates[3] = {&result.x, &result.y, &result.z};
    for (int index = 0; index < 3; ++index) {
        const auto coordinate = value.toArray().at(index);
        if (!coordinate.isDouble() || !std::isfinite(coordinate.toDouble()))
            throw std::runtime_error(std::string("Invalid coordinate: ") + group + "." + key);
        *coordinates[index] = float(coordinate.toDouble());
    }
    return result;
}

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

}

RuntimeConfig loadRuntimeConfig(const std::string& filePath) {
    // 先完成 JSON 语法检查，再按功能组填充强类型配置。
    QFile file(QString::fromStdString(filePath));
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot open runtime configuration: " + filePath);
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
        throw std::runtime_error("Invalid runtime configuration: " + parseError.errorString().toStdString());
    const QJsonObject root = document.object();

    RuntimeConfig config;

    // 输入镜像和 MediaPipe detector。
    config.mirrorInput = boolean(root, "camera", "mirror");
    config.detector.modelComplexity = integer(root, "detector", "model_complexity", 0, 1);
    config.detector.maximumHands = integer(root, "detector", "max_hands", 1, 2);
    config.detector.xnnpackThreads = integer(root, "detector", "xnnpack_threads", 1, 64);
    config.detector.minimumDetectionConfidence = number(root, "detector", "min_detection_confidence");
    config.detector.minimumTrackingConfidence = number(root, "detector", "min_tracking_confidence");
    require(config.detector.minimumDetectionConfidence >= 0 &&
                config.detector.minimumDetectionConfidence <= 1,
            "detector.min_detection_confidence must be in [0,1]");
    require(config.detector.minimumTrackingConfidence >= 0 && config.detector.minimumTrackingConfidence <= 1,
            "detector.min_tracking_confidence must be in [0,1]");

    // MANO 求解参数。
    config.mano.iterations = integer(root, "mano", "iterations", 0, 10000);
    config.mano.jacobianWorkers = integer(root, "mano", "jacobian_workers", 0, 45);
    config.mano.poseSmoothing = number(root, "mano", "pose_smoothing");
    config.mano.maximumFps = number(root, "mano", "max_fps");
    require(config.mano.poseSmoothing >= 0 && config.mano.poseSmoothing <= 1,
            "mano.pose_smoothing must be in [0,1]");
    require(config.mano.maximumFps > 0 && config.mano.maximumFps <= 1000, "mano.max_fps must be in (0,1000]");

    // 左右手身份确认和位置滤波。
    config.tracking.handednessConfirmationFrames =
        integer(root, "tracking", "handedness_confirm_frames", 1, 10000);
    const std::string mapping = text(root, "tracking", "handedness_map");
    if (mapping == "auto")
        config.tracking.handednessMap = HandednessMap::Auto;
    else if (mapping == "direct")
        config.tracking.handednessMap = HandednessMap::Direct;
    else if (mapping == "swapped")
        config.tracking.handednessMap = HandednessMap::Swapped;
    else
        throw std::runtime_error("tracking.handedness_map must be auto, direct or swapped");
    const QJsonObject filter = object(root, "tracking").value("position_filter").toObject();
    if (filter.isEmpty())
        throw std::runtime_error("Missing/invalid object: tracking.position_filter");
    auto filterNumber = [&](const char* key) {
        const QJsonValue value = filter.value(key);
        if (!value.isDouble() || !std::isfinite(value.toDouble()))
            throw std::runtime_error(std::string("Invalid tracking.position_filter.") + key);
        return value.toDouble();
    };
    config.tracking.positionFilter.minimumCutoff = filterNumber("min_cutoff");
    config.tracking.positionFilter.beta = filterNumber("beta");
    config.tracking.positionFilter.derivativeCutoff = filterNumber("derivative_cutoff");
    config.tracking.positionFilter.maximumSpeed = filterNumber("max_speed");
    const double window = filterNumber("median_window");
    require(window == std::floor(window) && window >= 1 && window <= 100000,
            "tracking.position_filter.median_window must be a positive integer");
    config.tracking.positionFilter.medianWindow = int(window);

    // 深度范围、校准帧数和深度滤波。
    config.depth.enabled = boolean(root, "depth_estimation", "enabled");
    config.depth.referenceMillimeters = number(root, "depth_estimation", "reference_depth");
    config.depth.minimumMillimeters = number(root, "depth_estimation", "minimum_depth");
    config.depth.maximumMillimeters = number(root, "depth_estimation", "maximum_depth");
    config.depth.calibrationFrames = integer(root, "depth_estimation", "calibration_frames", 1, 100000);
    config.depth.minimumCutoff = number(root, "depth_estimation", "min_cutoff");
    config.depth.beta = number(root, "depth_estimation", "beta");
    config.depth.maximumSpeed = number(root, "depth_estimation", "max_speed");
    config.depth.motionGain = number(root, "depth_estimation", "motion_gain");
    require(config.depth.minimumMillimeters >= 0 &&
                config.depth.minimumMillimeters < config.depth.maximumMillimeters &&
                config.depth.referenceMillimeters >= config.depth.minimumMillimeters &&
                config.depth.referenceMillimeters <= config.depth.maximumMillimeters,
            "Invalid depth range/reference");
    require(config.depth.minimumCutoff > 0 && config.depth.beta >= 0 && config.depth.motionGain > 0,
            "Invalid depth filter settings");

    // pinch、ball、button 三组同时存在时启用交互。
    config.interaction.enabled = root.contains("pinch") && root.contains("ball") && root.contains("button");
    if (config.interaction.enabled) {
        // 捏合迟滞和丢失超时。
        config.interaction.pinchEnterMeters = number(root, "pinch", "enter_distance");
        config.interaction.pinchExitMeters = number(root, "pinch", "exit_distance");
        config.interaction.pinchGrabToleranceMillimeters = number(root, "pinch", "grab_tolerance");
        config.interaction.pinchMissingTimeoutSeconds = number(root, "pinch", "missing_timeout");
        require(config.interaction.pinchEnterMeters > 0 &&
                    config.interaction.pinchEnterMeters < config.interaction.pinchExitMeters,
                "Require 0 < pinch.enter_distance < pinch.exit_distance");

        // 球体尺寸和缩放范围。
        config.interaction.ballCenterMillimeters = vector3(root, "ball", "center");
        config.interaction.ballRadiusMillimeters = number(root, "ball", "radius");
        config.interaction.ballMinimumScale = number(root, "ball", "minimum_scale");
        config.interaction.ballMaximumScale = number(root, "ball", "maximum_scale");
        config.interaction.ballScaleSensitivity = number(root, "ball", "scale_sensitivity");
        require(config.interaction.ballRadiusMillimeters > 0 && config.interaction.ballMinimumScale > 0 &&
                    config.interaction.ballMaximumScale >= config.interaction.ballMinimumScale &&
                    config.interaction.ballScaleSensitivity > 0,
                "Invalid ball settings");

        // 深度按钮的场景位置和碰撞尺寸。
        config.interaction.buttonCenterMillimeters = vector3(root, "button", "center");
        config.interaction.buttonWidthMillimeters = number(root, "button", "width");
        config.interaction.buttonHeightMillimeters = number(root, "button", "height");
        config.interaction.buttonTravelMillimeters = number(root, "button", "travel");
        config.interaction.fingertipRadiusMillimeters = number(root, "button", "tip_radius");
        require(config.interaction.buttonWidthMillimeters > 0 &&
                    config.interaction.buttonHeightMillimeters > 0 &&
                    config.interaction.buttonTravelMillimeters > 0 &&
                    config.interaction.fingertipRadiusMillimeters > 0,
                "Invalid button settings");
    }

    return config;
}

}
