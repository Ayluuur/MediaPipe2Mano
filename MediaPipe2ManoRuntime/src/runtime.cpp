#include <MediaPipe2Mano/Runtime.h>

#include "core.h"
#include "detector.h"

#include <QDir>
#include <QFileInfo>
#include <QString>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace m2m {
namespace {

// 公共配置使用带单位的字段名，内部算法沿用原有的紧凑配置结构。
DepthEstimatorConfig internalDepthConfig(const RuntimeConfig& config) {
    DepthEstimatorConfig result;
    result.enabled = config.depth.enabled;
    result.reference = config.depth.referenceMillimeters;
    result.minimum = config.depth.minimumMillimeters;
    result.maximum = config.depth.maximumMillimeters;
    result.calibrationFrames = config.depth.calibrationFrames;
    result.minCutoff = config.depth.minimumCutoff;
    result.beta = config.depth.beta;
    result.maxSpeed = config.depth.maximumSpeed;
    result.motionGain = config.depth.motionGain;
    return result;
}

PositionFilterOptions internalPositionConfig(const RuntimeConfig& config) {
    PositionFilterOptions result;
    result.minCutoff = config.tracking.positionFilter.minimumCutoff;
    result.beta = config.tracking.positionFilter.beta;
    result.derivativeCutoff = config.tracking.positionFilter.derivativeCutoff;
    result.maxSpeed = config.tracking.positionFilter.maximumSpeed;
    result.medianWindow = config.tracking.positionFilter.medianWindow;
    return result;
}

// 以下转换函数集中处理 Eigen 与公共标准类型之间的边界。
Vec3 eigenVector(const Vector3& value) {
    return {value.x, value.y, value.z};
}

Vector3 publicVector(const Eigen::Ref<const Eigen::Vector3d>& value) {
    return {float(value.x()), float(value.y()), float(value.z())};
}

Transform publicTransform(const Eigen::Matrix4d& matrix) {
    Transform result;
    for (int row = 0; row < 4; ++row)
        for (int column = 0; column < 4; ++column)
            result.values[4 * row + column] = float(matrix(row, column));
    return result;
}

Quaternion publicQuaternion(const Mat3& rotation) {
    const Eigen::Quaterniond quaternion(rotation);
    return {float(quaternion.x()), float(quaternion.y()), float(quaternion.z()), float(quaternion.w())};
}

template <std::size_t Count> void copyPoints(const Matrix& source, std::array<Vector3, Count>& destination) {
    if (source.rows() != int(Count) || source.cols() != 3)
        throw std::runtime_error("Unexpected landmark matrix dimensions");
    for (std::size_t index = 0; index < Count; ++index) {
        destination[index] = {float(source(int(index), 0)), float(source(int(index), 1)),
                              float(source(int(index), 2))};
    }
}

int bytesPerPixel(PixelFormat format) {
    return format == PixelFormat::Rgb24 || format == PixelFormat::Bgr24 ? 3 : 4;
}

// 所有输入格式在进入 MediaPipe 前转换为连续 RGB24；镜像也在这一步完成。
std::vector<std::uint8_t> normalizeRgb(const ImageView& image, bool mirror) {
    const int components = bytesPerPixel(image.format);
    const std::ptrdiff_t minimumStride = std::ptrdiff_t(image.width) * components;
    if (!image.data || image.width <= 0 || image.height <= 0 || image.strideBytes < minimumStride)
        throw std::invalid_argument("ImageView contains invalid data, dimensions or stride");
    std::vector<std::uint8_t> rgb(std::size_t(image.width) * image.height * 3);
    for (int y = 0; y < image.height; ++y) {
        const auto* sourceRow = image.data + std::ptrdiff_t(y) * image.strideBytes;
        auto* destinationRow = rgb.data() + std::size_t(y) * image.width * 3;
        for (int x = 0; x < image.width; ++x) {
            const int sourceX = mirror ? image.width - 1 - x : x;
            const auto* source = sourceRow + std::ptrdiff_t(sourceX) * components;
            auto* destination = destinationRow + 3 * x;
            const bool blueFirst = image.format == PixelFormat::Bgr24 || image.format == PixelFormat::Bgra32;
            destination[0] = source[blueFirst ? 2 : 0];
            destination[1] = source[1];
            destination[2] = source[blueFirst ? 0 : 2];
        }
    }
    return rgb;
}

}

class Runtime::Impl {
  public:
    Impl(std::string resourceRoot, RuntimeConfig runtimeConfig)
        : root(QDir::cleanPath(QString::fromStdString(std::move(resourceRoot))).toStdString()),
          configuration(std::move(runtimeConfig)),
          detector(QString::fromStdString(root), configuration.detector.modelComplexity,
                   configuration.detector.maximumHands, configuration.detector.xnnpackThreads,
                   configuration.detector.minimumDetectionConfidence,
                   configuration.detector.minimumTrackingConfidence),
          depthTracker(internalDepthConfig(configuration)),
          resolver(configuration.tracking.handednessConfirmationFrames),
          ball(configuration.interaction.ballRadiusMillimeters,
               eigenVector(configuration.interaction.ballCenterMillimeters)),
          controller(ball, configuration.interaction.ballMinimumScale,
                     configuration.interaction.ballMaximumScale,
                     configuration.interaction.ballScaleSensitivity) {
        validateResources();

        // 左右手拥有独立的跟踪、滤波和 IK 状态；三角面索引在初始化时缓存。
        const PositionFilterOptions position = internalPositionConfig(configuration);
        for (int side = 0; side < 2; ++side) {
            const std::string model =
                root + "/models/MANO_" + (side == 0 ? std::string("LEFT") : std::string("RIGHT")) + ".bin";
            states[side] = std::make_unique<HandState>(model, side, configuration.mano.iterations,
                                                       configuration.mano.poseSmoothing, position,
                                                       configuration.mirrorInput);
            states[side]->converter.setJacobianWorkers(configuration.mano.jacobianWorkers);
            const Matrix internalFaces = states[side]->converter.getFaces(true);
            faces[side].reserve(std::size_t(internalFaces.rows()) * 3);
            for (int row = 0; row < internalFaces.rows(); ++row)
                for (int column = 0; column < 3; ++column)
                    faces[side].push_back(std::uint32_t(internalFaces(row, column)));
            depths[side] = configuration.depth.referenceMillimeters;
            pinches[side].enter = configuration.interaction.pinchEnterMeters;
            pinches[side].exit = configuration.interaction.pinchExitMeters;
            pinches[side].missingTimeout = configuration.interaction.pinchMissingTimeoutSeconds;
        }

        createButton();
    }

    FrameResult process(const ImageView& image, double timestamp) {
        // 时间戳参与滤波和轨迹超时，必须保持单调。
        if (!std::isfinite(timestamp) || timestamp < 0)
            throw std::invalid_argument("timestampSeconds must be finite and nonnegative");
        if (timestamp < lastTimestamp)
            throw std::invalid_argument("timestampSeconds must be monotonic");
        lastTimestamp = timestamp;

        // 1. 格式转换、手部检测和左右手身份稳定。
        std::vector<std::uint8_t> rgb = normalizeRgb(image, configuration.mirrorInput);
        std::vector<Detection> raw =
            detector.process(rgb.data(), image.width, image.height, std::ptrdiff_t(image.width) * 3);
        applyHandednessMapping(raw);
        for (auto& state : states)
            state->collect();
        std::map<int, Detection> detections = resolver.resolve(raw, states, timestamp);

        // 2. 根据手掌尺度更新深度和场景平移。
        std::map<int, std::optional<double>> sceneScales;
        if (configuration.depth.enabled)
            updateDepth(detections, sceneScales, image, timestamp);

        // 3. 更新每只手的滤波状态，并计算捏合点。
        for (int side : resolver.resolvedOrder) {
            Detection& detection = detections.at(side);
            HandState& state = *states[side];
            Matrix filtered = state.update(detection, timestamp);
            if (configuration.depth.enabled) {
                state.updateScenePosition(
                    detection, depths[side], image.width, image.height, timestamp,
                    sceneScales[side].value_or(std::numeric_limits<double>::quiet_NaN()));
            }
            if (configuration.interaction.enabled)
                updatePinch(side, filtered, detection, timestamp);
        }
        deactivateMissingHand(raw, detections);

        // 4. 提交当前最新关键点；正在求解的手不会重复提交任务。
        for (int side = 0; side < 2; ++side) {
            HandState& state = *states[side];
            if (!detections.count(side))
                state.pending.reset();
            if (configuration.interaction.enabled)
                pinches[side].expire(timestamp);
            state.submit(timestamp, configuration.mano.maximumFps);
        }

        // 5. 更新场景交互，再组装公共返回结构。
        if (configuration.interaction.enabled)
            updateInteraction(detections);
        return makeResult(detections, timestamp);
    }

    void requestCalibration(double timestamp) {
        if (configuration.depth.enabled)
            depthTracker.requestCalibration(timestamp);
    }

    void reset() {
        // reset 恢复构造后的状态，配置、模型和 detector 实例继续复用。
        for (auto& state : states)
            state->deactivate();
        depthTracker = MultiHandDepthEstimator(internalDepthConfig(configuration));
        resolver.reset();
        depths.fill(configuration.depth.referenceMillimeters);
        pinches = {Pinch{}, Pinch{}};
        for (auto& pinch : pinches) {
            pinch.enter = configuration.interaction.pinchEnterMeters;
            pinch.exit = configuration.interaction.pinchExitMeters;
            pinch.missingTimeout = configuration.interaction.pinchMissingTimeoutSeconds;
        }
        ball = Ball(configuration.interaction.ballRadiusMillimeters,
                    eigenVector(configuration.interaction.ballCenterMillimeters));
        controller.reset();
        createButton();
        sequence = 1;
        lastTimestamp = -inf;
    }

    std::string root;
    RuntimeConfig configuration;

  private:
    // 构造阶段只检查运行库启动必需的 graph 和左右手模型。
    void validateResources() const {
        const QString path = QString::fromStdString(root);
        for (const char* relative :
             {"assets/hands_0_10_9.pbtxt", "models/MANO_LEFT.bin", "models/MANO_RIGHT.bin"}) {
            if (!QFileInfo::exists(path + "/" + relative))
                throw std::runtime_error(std::string("Missing runtime resource: ") + relative);
        }
    }

    // viewer profile 不包含交互字段，此时 button 保持为空。
    void createButton() {
        if (!configuration.interaction.enabled) {
            button.reset();
            return;
        }
        button = std::make_unique<DepthButton>(eigenVector(configuration.interaction.buttonCenterMillimeters),
                                               configuration.interaction.buttonWidthMillimeters,
                                               configuration.interaction.buttonHeightMillimeters,
                                               configuration.interaction.buttonTravelMillimeters,
                                               configuration.interaction.fingertipRadiusMillimeters);
    }

    // Auto 模式沿用输入镜像约定；Direct 和 Swapped 可显式覆盖。
    void applyHandednessMapping(std::vector<Detection>& detections) const {
        const bool swap =
            configuration.tracking.handednessMap == HandednessMap::Swapped ||
            (configuration.tracking.handednessMap == HandednessMap::Auto && !configuration.mirrorInput);
        if (swap)
            for (auto& detection : detections)
                detection.rawSide = 1 - detection.rawSide;
    }

    void updateDepth(const std::map<int, Detection>& detections,
                     std::map<int, std::optional<double>>& sceneScales, const ImageView& image,
                     double timestamp) {
        std::map<int, double> sizes;

        // palmScale 返回毫米/像素比例，其倒数用于多手深度估计器的尺度输入。
        for (const auto& [side, detection] : detections) {
            HandState& state = *states[side];
            sceneScales[side] = palmScale(detection.screen, detection.world, state.sceneReferenceKeypoints,
                                          image.width, image.height, &state.sceneSegmentCorrections);
            sizes[side] =
                sceneScales[side] ? 20.0 / *sceneScales[side] : std::numeric_limits<double>::quiet_NaN();
        }
        for (const auto& [side, depth] : depthTracker.update(sizes, timestamp))
            depths[side] = depth;
    }

    void updatePinch(int side, const Matrix& filtered, const Detection&, double timestamp) {
        HandState& state = *states[side];
        std::optional<Vec3> scenePoint;

        // 捏合控制点取拇指与食指指尖的中点。
        if (state.result && state.displayWrist) {
            const Matrix points = scenePoints(state.result->keypoints, *state.displayWrist, depths[side],
                                              state.sceneTranslation);
            scenePoint = ((points.row(16) + points.row(20)) * .5).transpose();
        }
        if (pinches[side].update(filtered, timestamp, ball,
                                 configuration.interaction.pinchGrabToleranceMillimeters, sequence,
                                 scenePoint))
            ++sequence;
    }

    void deactivateMissingHand(const std::vector<Detection>& raw,
                               const std::map<int, Detection>& detections) {
        // 单手帧中，另一只手超过轨迹时限后立即释放状态和交互控制权。
        if (raw.size() != 1 || detections.size() != 1)
            return;
        const int presentSide = detections.begin()->first;
        for (int side = 0; side < 2; ++side)
            if (side != presentSide) {
                const double presentTimestamp = states[presentSide]->lastSeen;
                if (presentTimestamp - states[side]->lastSeen > resolver.trackTimeout) {
                    states[side]->deactivate();
                    if (configuration.interaction.enabled)
                        pinches[side].release();
                }
            }
    }

    void updateInteraction(const std::map<int, Detection>& detections) {
        // 球体使用捏合中点，按钮使用 MANO 索引 16 的食指指尖。
        controller.update(pinches);
        std::map<int, Vec3> fingertips;
        for (const auto& [side, detection] : detections) {
            const HandState& state = *states[side];
            if (state.result && state.displayWrist) {
                fingertips[side] = scenePoints(state.result->keypoints, *state.displayWrist, depths[side],
                                               state.sceneTranslation)
                                       .row(16)
                                       .transpose();
            }
        }
        button->update(fingertips);
    }

    FrameResult makeResult(const std::map<int, Detection>& detections, double timestamp) const {
        // 帧级交互状态与校准状态每帧都返回。
        FrameResult frame;
        frame.timestampSeconds = timestamp;
        frame.calibrationStatus = depthTracker.calibrationStatus(timestamp);
        frame.ball.centerMillimeters = publicVector(ball.center);
        frame.ball.rotation = publicQuaternion(ball.rotation);
        frame.ball.scale = float(ball.scale);
        if (button) {
            frame.button.pressed = button->pressed;
            frame.button.pressCount = button->pressCount;
            for (int side : button->contacts)
                frame.button.contacts[side] = true;
        }

        frame.hands.reserve(detections.size());
        for (const auto& [side, detection] : detections) {
            // 检测结果无需等待 IK，可立即填入。
            HandResult hand;
            hand.side = side == 0 ? HandSide::Left : HandSide::Right;
            hand.detectionScore = float(detection.score);
            hand.stabilizedSide = detection.stabilized;
            hand.depthMillimeters = float(depths[side]);
            copyPoints(detection.screen, hand.screenLandmarks);
            copyPoints(detection.world, hand.worldLandmarksMeters);
            if (configuration.interaction.enabled) {
                hand.pinching = pinches[side].pinching;
                hand.controllingBall = pinches[side].controls;
                hand.pinchDistanceMeters = float(pinches[side].distance);
            }

            const HandState& state = *states[side];
            if (state.result && state.displayWrist) {
                // 网格和关键点共用同一场景变换，三角面使用初始化时的缓存。
                hand.meshReady = true;
                hand.ikSeconds = state.result->ikSeconds;
                const Matrix sceneVertices = scenePoints(state.result->vertices, *state.displayWrist,
                                                         depths[side], state.sceneTranslation);
                const Matrix sceneKeypoints = scenePoints(state.result->keypoints, *state.displayWrist,
                                                          depths[side], state.sceneTranslation);
                hand.mesh.vertices.reserve(std::size_t(sceneVertices.rows()));
                for (int row = 0; row < sceneVertices.rows(); ++row)
                    hand.mesh.vertices.push_back({float(sceneVertices(row, 0)), float(sceneVertices(row, 1)),
                                                  float(sceneVertices(row, 2))});
                hand.mesh.triangleIndices = faces[side];
                copyPoints(sceneKeypoints, hand.sceneKeypointsMillimeters);

                // 骨架位置直接取场景变换的平移列，索引与 jointNames 一致。
                if (const auto skeleton = state.getSkeleton("scene")) {
                    hand.skeleton.jointNames = skeleton->names;
                    hand.skeleton.parents = skeleton->parents;
                    hand.skeleton.fitErrorMillimeters = float(skeleton->fitErrorMm);
                    hand.skeleton.mirroredLocalAxes = skeleton->mirroredLocalAxes;
                    const auto local = skeleton->localTransforms();
                    for (int joint = 0; joint < 16; ++joint) {
                        const auto position = skeleton->transforms[joint].topRightCorner<3, 1>();
                        hand.skeleton.scenePositionsMillimeters[joint] = {
                            float(position.x()), float(position.y()), float(position.z())};
                        hand.skeleton.sceneTransforms[joint] = publicTransform(skeleton->transforms[joint]);
                        hand.skeleton.localTransforms[joint] = publicTransform(local[joint]);
                    }
                }
            }
            frame.hands.push_back(std::move(hand));
        }
        return frame;
    }

    Detector detector;
    std::array<std::unique_ptr<HandState>, 2> states;
    MultiHandDepthEstimator depthTracker;
    HandednessResolver resolver;
    std::array<double, 2> depths{};
    std::array<Pinch, 2> pinches;
    Ball ball;
    std::unique_ptr<DepthButton> button;
    BallController controller;
    std::array<std::vector<std::uint32_t>, 2> faces;
    int sequence = 1;
    double lastTimestamp = -inf;
};

// ---------- Runtime 公共 PImpl 转发 ----------

Runtime::Runtime(std::string resourceRoot, RuntimeConfig config)
    : impl_(new Impl(std::move(resourceRoot), std::move(config))) {}

Runtime::~Runtime() {
    delete impl_;
}
Runtime::Runtime(Runtime&& other) noexcept : impl_(std::exchange(other.impl_, nullptr)) {}
Runtime& Runtime::operator=(Runtime&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = std::exchange(other.impl_, nullptr);
    }
    return *this;
}

FrameResult Runtime::process(const ImageView& image, double timestampSeconds) {
    return impl_->process(image, timestampSeconds);
}

void Runtime::requestDepthCalibration(double timestampSeconds) {
    impl_->requestCalibration(timestampSeconds);
}

void Runtime::reset() {
    impl_->reset();
}

const RuntimeConfig& Runtime::config() const noexcept {
    return impl_->configuration;
}
const std::string& Runtime::resourceRoot() const noexcept {
    return impl_->root;
}

}
