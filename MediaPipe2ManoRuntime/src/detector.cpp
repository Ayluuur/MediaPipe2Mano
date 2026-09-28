#include "detector.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryFile>
#include <stdexcept>
namespace m2m {

Detector::Detector(const QString& root, int complexity, int maxHands, int xnnpackThreads, double detection,
                   double tracking) {
    // 优先加载调用程序目录中的 detector，开发目录下的 Bin 作为兼容路径。
    QString path = QCoreApplication::applicationDirPath() + "/mediapipe_hands.dll";
    if (!QFileInfo::exists(path))
        path = root + "/Bin/mediapipe_hands.dll";
    library.setFileName(path);
    if (!library.load())
        throw std::runtime_error(
            ("Native MediaPipe DLL is unavailable. Run tools/build_mediapipe.ps1.\n" + library.errorString())
                .toStdString());
    auto create = reinterpret_cast<decltype(&m2m_create)>(library.resolve("m2m_create"));
    processFn = reinterpret_cast<decltype(&m2m_process)>(library.resolve("m2m_process"));
    destroyFn = reinterpret_cast<decltype(&m2m_destroy)>(library.resolve("m2m_destroy"));
    if (!create || !processFn || !destroyFn)
        throw std::runtime_error("Invalid native MediaPipe DLL ABI");

    // 初始化前把 graph 中的模型路径展开为绝对路径，进程工作目录保持不变。
    QFile sourceGraph(root + "/assets/hands_0_10_9.pbtxt");
    if (!sourceGraph.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot open MediaPipe graph");
    QByteArray graphText = sourceGraph.readAll();
    for (const auto& model : {"mediapipe/modules/palm_detection/palm_detection_lite.tflite",
                              "mediapipe/modules/palm_detection/palm_detection_full.tflite",
                              "mediapipe/modules/hand_landmark/hand_landmark_lite.tflite",
                              "mediapipe/modules/hand_landmark/hand_landmark_full.tflite",
                              "mediapipe/modules/hand_landmark/handedness.txt"}) {
        const QByteArray relative(model);
        const QByteArray absolute = QDir::fromNativeSeparators(root + "/assets/" + model).toUtf8();
        if (!graphText.contains(relative) || !QFileInfo::exists(QString::fromUtf8(absolute)))
            throw std::runtime_error("Incomplete MediaPipe model resources");
        graphText.replace(relative, absolute);
    }
    QTemporaryFile expandedGraph(QDir::tempPath() + "/MediaPipe2Mano-XXXXXX.pbtxt");
    if (!expandedGraph.open() || expandedGraph.write(graphText) != graphText.size() || !expandedGraph.flush())
        throw std::runtime_error("Cannot prepare MediaPipe graph");
    char error[4096]{};
    QByteArray graph = QDir::toNativeSeparators(expandedGraph.fileName()).toUtf8();

    // DLL 在 m2m_create 返回前完成 graph 读取，临时文件随后可以释放。
    handle = create(graph.constData(), complexity, maxHands, xnnpackThreads, float(detection),
                    float(tracking), error, sizeof(error));
    if (!handle)
        throw std::runtime_error(error);
}
Detector::~Detector() {
    if (handle && destroyFn)
        destroyFn(handle);
}

std::vector<Detection> Detector::process(const std::uint8_t* rgb, int width, int height,
                                         std::ptrdiff_t strideBytes) {
    M2MHand hands[2]{};
    char error[4096]{};
    // 与 Python SolutionBase.process 保持一致：时间戳固定步进 33333 微秒，不跟随摄像头帧率。
    timestamp += 33333;
    int count =
        processFn(handle, rgb, width, height, int(strideBytes), timestamp, hands, 2, error, sizeof(error));
    if (count < 0)
        throw std::runtime_error(error);

    // C ABI 使用连续 float 数组，这里转换为核心算法使用的 21×3 矩阵。
    std::vector<Detection> out;
    for (int i = 0; i < count; ++i) {
        Detection d;
        d.rawSide = hands[i].side;
        d.score = hands[i].score;
        d.screen.resize(21, 3);
        d.world.resize(21, 3);
        for (int k = 0; k < 63; ++k) {
            d.screen.data()[k] = hands[i].screen[k];
            d.world.data()[k] = hands[i].world[k];
        }
        out.push_back(std::move(d));
    }
    return out;
}
}
