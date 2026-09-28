# 公共 C++ API

公共 API 位于 `MediaPipe2ManoRuntime/include/MediaPipe2Mano/`，命名空间为 `m2m`。

## 调用流程

```cpp
#include <MediaPipe2Mano/Runtime.h>

auto config = m2m::loadRuntimeConfig("configs/interaction.json");
m2m::Runtime runtime(projectRoot, config);

m2m::ImageView image{
    pixels,
    width,
    height,
    strideBytes,
    m2m::PixelFormat::Bgr24
};

const double timestampSeconds = /* 单调时钟，单位为秒 */;
const m2m::FrameResult frame = runtime.process(image, timestampSeconds);

for (const auto& hand : frame.hands) {
    if (hand.meshReady) {
        // 使用 hand.mesh、hand.skeleton 和 hand.sceneKeypointsMillimeters。
    }
}
```

调用方持有 `ImageView::data`，其有效期覆盖本次 `process()` 调用。

## Runtime

### 构造

```cpp
Runtime(std::string resourceRoot, RuntimeConfig config);
```

`resourceRoot` 指向包含 `assets/` 和 `models/` 的目录。构造函数加载 MANO 模型、MediaPipe graph 和 detector DLL。

### 处理图像

```cpp
FrameResult process(const ImageView& image, double timestampSeconds);
```

输入约束：

- `width`、`height` 和 `strideBytes` 与像素缓冲区一致；
- 格式为 `Rgb24`、`Bgr24`、`Rgba32` 或 `Bgra32`；
- 时间戳有限、非负、单调不减；
- 一个外部线程顺序调用一个 Runtime 实例。

函数同步执行 detector，异步提交 MANO IK，并返回最近完成的 IK 结果。

### 状态控制

```cpp
void requestDepthCalibration(double timestampSeconds);
void reset();
const RuntimeConfig& config() const noexcept;
const std::string& resourceRoot() const noexcept;
```

| 函数 | 作用 |
| --- | --- |
| `requestDepthCalibration()` | 启动深度标定窗口 |
| `reset()` | 清除轨迹、滤波、IK 快照、标定和交互状态 |
| `config()` | 返回实例使用的配置 |
| `resourceRoot()` | 返回规范化后的资源根目录 |

## 配置

```cpp
RuntimeConfig loadRuntimeConfig(const std::string& filePath);
```

该函数读取并校验 JSON。返回的强类型配置可在构造 Runtime 前修改。

| 成员 | JSON 来源 | 用途 |
| --- | --- | --- |
| `mirrorInput` | `camera.mirror` | detector 前水平镜像 |
| `detector` | `detector` | 模型、手数、XNNPACK 和置信度 |
| `mano` | `mano` | IK 迭代、Jacobian worker、平滑和帧率 |
| `tracking` | `tracking` | 左右手稳定与位置滤波 |
| `depth` | `depth_estimation` | 深度估计、范围和标定 |
| `interaction` | `pinch/ball/button` | 捏合、球体和按钮 |


## FrameResult

```cpp
struct FrameResult {
    double timestampSeconds;
    std::vector<HandResult> hands;
    BallState ball;
    ButtonState button;
    std::string calibrationStatus;
};
```

`hands` 包含当前帧稳定跟踪到的手。`ball`、`button` 和 `calibrationStatus` 表示交互与标定状态。

### HandResult

| 字段 | 含义 |
| --- | --- |
| `side` | 稳定后的左/右手身份 |
| `detectionScore` | MediaPipe handedness 置信度 |
| `stabilizedSide` | 身份是否经过轨迹稳定处理 |
| `meshReady` | mesh、场景关键点和 skeleton 是否可用 |
| `screenLandmarks` | 21 个归一化屏幕点 |
| `worldLandmarksMeters` | 21 个 MediaPipe 世界点，米 |
| `sceneKeypointsMillimeters` | 21 个场景点，毫米 |
| `mesh` | MANO 顶点和三角形索引 |
| `skeleton` | 16 关节位置和变换 |
| `pinching` | 捏合状态 |
| `controllingBall` | 球体控制状态 |
| `pinchDistanceMeters` | 捏合距离，米 |
| `depthMillimeters` | 估计深度，毫米 |
| `ikSeconds` | 最近一次 IK 耗时，秒 |

### Mesh

```cpp
struct Mesh {
    std::vector<Vector3> vertices;
    std::vector<std::uint32_t> triangleIndices;
};
```

顶点使用毫米单位。

### Skeleton

Skeleton 包含 16 个 MANO 关节：

| 字段 | 含义 |
| --- | --- |
| `jointNames` | 关节名称 |
| `parents` | 父关节索引 |
| `scenePositionsMillimeters` | 场景位置，毫米 |
| `sceneTransforms` | 关节到场景空间的变换 |
| `localTransforms` | 关节到父关节空间的局部变换 |
| `fitErrorMillimeters` | IK 平均关键点拟合误差 |
| `mirroredLocalAxes` | 镜像对局部轴方向的影响标记 |

变换矩阵为行主序 4×4，平移位于 `values[3]`、`values[7]`、`values[11]`。四元数字段顺序为 `x, y, z, w`。
