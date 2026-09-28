#pragma once
#include "core.h"
#include "bridge.h"
#include <QLibrary>
#include <cstddef>
#include <cstdint>
namespace m2m {
class M2M_RUNTIME_API Detector {
  public:
    Detector(const QString& root, int complexity, int maxHands, int xnnpackThreads, double detection,
             double tracking);
    ~Detector();
    std::vector<Detection> process(const std::uint8_t* rgb, int width, int height,
                                   std::ptrdiff_t strideBytes);

  private:
    QLibrary library;
    void* handle = nullptr;
    decltype(&m2m_process) processFn = nullptr;
    decltype(&m2m_destroy) destroyFn = nullptr;
    int64_t timestamp = 0;
};
}
