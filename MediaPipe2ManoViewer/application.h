#pragma once
#include "core.h"
#include <QJsonObject>
#include <QString>
namespace m2m {
QJsonObject loadConfig(const QString& root, const QString& mode, const QString& overridePath);
void validateConfig(const QJsonObject& config, bool interaction);
// runApplication 创建 OpenCV 摄像头窗口和 Open3D 三维窗口，Qt 事件循环负责调度。
int runApplication(const QString& root, const QJsonObject& config, bool interaction, int maximumFrames = 0,
                   bool synthetic = false, const QString& capturePath = {}, bool hidden = false,
                   const QString& videoPath = {}, double benchmarkSeconds = 0);
}
