# 开发说明

本文说明工程边界、运行流程、代码定位和验证方法。公共类型和函数见 [公共 C++ API](API.md)，构建与运行命令见 [项目 README](../README.md)。

## 工程结构

| 模块 | 职责 |
| --- | --- |
| `MediaPipe2ManoRuntime` | 接收图像并生成 `FrameResult` |
| `MediaPipe2ManoViewer` | 采集摄像头/视频、调度帧处理并显示 2D/3D 结果 |
| `ThirdParty/MediaPipe` | MediaPipe detector 的 C ABI 声明与桥接源码 |
| `assets` / `models` | MediaPipe graph、TFLite 文件和 MANO 二进制模型 |
| `configs` | Viewer 与 Runtime 的运行参数 |

Runtime 的公共头文件位于：

```text
MediaPipe2ManoRuntime/include/MediaPipe2Mano/
├─ Export.h
├─ Runtime.h
├─ RuntimeConfig.h
└─ Types.h
```

## 运行流程

`Runtime::process()` 依次执行：

1. 校验 `ImageView` 和时间戳，将输入转换为 RGB；
2. 调用 MediaPipe，生成最多两只手的 21 个关键点；
3. 稳定左右手身份并更新位置滤波；
4. 估计深度和场景坐标；
5. 收集已完成的 MANO IK，提交当前帧 IK；
6. 生成 mesh、16 关节 skeleton 和交互状态；
7. 返回 `FrameResult`。

MediaPipe detector 同步执行，MANO IK 在线程池中异步执行。

Viewer 在 `application.cpp` 中管理输入、帧调度、性能统计、二维标注和 Open3D 几何体。

## 代码定位

| 修改内容 | 主要文件 | 关联内容 |
| --- | --- | --- |
| 公共字段或函数 | `include/MediaPipe2Mano/*.h` | `runtime.cpp`、API 文档 |
| MediaPipe 输入输出 | `detector.*`、`ThirdParty/MediaPipe/bridge.*` | DLL C ABI、结构体布局 |
| 左右手稳定 | `HandednessResolver` | Runtime 与 Viewer 帧路径 |
| 滤波和深度 | `core.*` | 三个默认配置 |
| MANO IK | `ManoModel`、`KeypointsToMano` | 异步提交、Jacobian worker |
| 骨架 | `skeleton.cpp` | 单位、矩阵布局、父子关系 |
| 捏合、球体和按钮 | `Pinch`、`BallController`、`DepthButton` | Viewer 几何体、API 状态 |
| Viewer 界面与渲染 | `application.cpp`、`main.cpp` | 命令行、有限帧验证 |
| JSON 字段 | `runtime_config.cpp`、`application.cpp` | 默认 profile、API 文档 |

## 线程模型

- 一个外部线程顺序调用一个 `Runtime` 实例。
- 时间戳使用有限、非负、单调不减的秒值。
- 每只手维护一个 IK 执行器和一个待完成任务。
- `mano.jacobian_workers` 控制单次 IK 中数值 Jacobian 的并行度。
- `reset()` 清理轨迹、滤波、IK 快照、深度标定和交互状态。
- Runtime 析构过程等待已提交任务结束并释放 detector 与 MANO 模型。

## 坐标和单位

| 数据 | 坐标与单位 |
| --- | --- |
| `screenLandmarks` | 归一化屏幕坐标，`x/y` 通常为 `[0, 1]` |
| `worldLandmarksMeters` | MediaPipe 相机局部坐标，米 |
| `sceneKeypointsMillimeters` | Viewer 场景坐标，毫米 |
| mesh 顶点 | Viewer 场景坐标，毫米 |
| skeleton 位置 | Viewer 场景坐标，毫米 |
| `Transform::values` | 行主序 4×4，平移位于 `3/7/11` |
| `Quaternion` | `x, y, z, w` |

坐标或单位变更会同时影响 mesh、skeleton、交互碰撞和 Viewer 渲染。

## 构建配置

| 配置 | 用途 |
| --- | --- | 
| `Develop` | 关闭优化、生成 PDB、链接 Release 第三方 ABI |
| `Release` | 启用优化，用于性能测试和交付 |

当前 Open3D 仅提供 Release 开发包，因此开发调试使用 `Develop`。
