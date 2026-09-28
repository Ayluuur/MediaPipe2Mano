#pragma once

#include "Export.h"
#include "RuntimeConfig.h"
#include "Types.h"

#include <string>

namespace m2m {

class M2M_RUNTIME_API Runtime final {
  public:
    Runtime(std::string resourceRoot, RuntimeConfig config);
    ~Runtime();

    Runtime(Runtime&&) noexcept;
    Runtime& operator=(Runtime&&) noexcept;
    Runtime(const Runtime&) = delete;
    Runtime& operator=(const Runtime&) = delete;

    // MediaPipe 检测同步完成，MANO IK 提交到 Qt 管理的线程池异步执行。
    // 同一个实例由一个调用线程顺序使用，时间戳必须单调不减。
    FrameResult process(const ImageView& image, double timestampSeconds);

    void requestDepthCalibration(double timestampSeconds);
    void reset();
    const RuntimeConfig& config() const noexcept;
    const std::string& resourceRoot() const noexcept;

  private:
    class Impl;
    Impl* impl_ = nullptr;
};

}
