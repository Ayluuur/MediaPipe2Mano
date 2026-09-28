#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace m2m {

enum class HandSide {
    Left = 0,
    Right = 1
};

enum class PixelFormat {
    Rgb24,
    Bgr24,
    Rgba32,
    Bgra32
};

// data 由调用方持有，在 process() 返回前保持有效。
struct ImageView {
    const std::uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    std::ptrdiff_t strideBytes = 0;
    PixelFormat format = PixelFormat::Rgb24;
};

struct Vector2 {
    float x = 0;
    float y = 0;
};

struct Vector3 {
    float x = 0;
    float y = 0;
    float z = 0;
};

struct Quaternion {
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 1;
};

// 行主序 4×4 矩阵；平移分量位于 values[3]、values[7]、values[11]。
struct Transform {
    std::array<float, 16> values{};
};

struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<std::uint32_t> triangleIndices;
};

// 16 关节骨架，同时提供场景变换和相对父关节的局部变换。
struct Skeleton {
    std::array<std::string, 16> jointNames{};
    std::array<int, 16> parents{};
    std::array<Vector3, 16> scenePositionsMillimeters{};
    std::array<Transform, 16> sceneTransforms{};
    std::array<Transform, 16> localTransforms{};
    float fitErrorMillimeters = 0;
    bool mirroredLocalAxes = false;
};

// meshReady 标记网格、场景关键点和骨架是否已经生成。
struct HandResult {
    HandSide side = HandSide::Left;
    float detectionScore = 0;
    bool stabilizedSide = false;
    bool meshReady = false;
    bool pinching = false;
    bool controllingBall = false;
    float pinchDistanceMeters = 0;
    float depthMillimeters = 0;
    double ikSeconds = 0;
    std::array<Vector3, 21> screenLandmarks{};
    std::array<Vector3, 21> worldLandmarksMeters{};
    std::array<Vector3, 21> sceneKeypointsMillimeters{};
    Mesh mesh;
    Skeleton skeleton;
};

struct BallState {
    Vector3 centerMillimeters{};
    Quaternion rotation{};
    float scale = 1;
};

struct ButtonState {
    bool pressed = false;
    int pressCount = 0;
    std::array<bool, 2> contacts{};
};

struct FrameResult {
    double timestampSeconds = 0;
    std::vector<HandResult> hands;
    BallState ball;
    ButtonState button;
    std::string calibrationStatus;
};

}
