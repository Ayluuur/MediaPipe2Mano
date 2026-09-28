#pragma once

#include <MediaPipe2Mano/Runtime.h>

#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QImage>
#include <QMainWindow>
#include <QThreadPool>
#include <QTimer>

#include <open3d/Open3D.h>
#include <opencv2/videoio.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

class QLabel;
class QPushButton;

class MediaPipe2ManoViewer final : public QMainWindow {
  public:
    MediaPipe2ManoViewer(std::string resourceRoot, m2m::RuntimeConfig config, int cameraIndex = 0,
                         int cameraWidth = 640, int cameraHeight = 480, QWidget* parent = nullptr);
    ~MediaPipe2ManoViewer() override;

  private:
    struct ProcessedFrame {
        QImage image;
        m2m::FrameResult result;
        double processingMilliseconds = 0;
        QString error;
    };

    using TriangleMesh = open3d::geometry::TriangleMesh;

    void buildUi();
    void initializeScene();
    void submitFrame();
    void presentFrame();
    void drawOverlay(QImage& image, const m2m::FrameResult& result) const;
    void updateScene(const m2m::FrameResult& result);
    void setView(const QString& name);
    void stopWithError(const QString& message);

    std::string resourceRoot_;
    m2m::RuntimeConfig config_;
    std::unique_ptr<m2m::Runtime> runtime_;
    cv::VideoCapture camera_;
    QThreadPool processingPool_;
    QFutureWatcher<ProcessedFrame> frameWatcher_;
    QTimer captureTimer_;
    QElapsedTimer clock_;

    QLabel* videoLabel_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QPushButton* pauseButton_ = nullptr;
    bool paused_ = false;
    bool stopping_ = false;

    open3d::visualization::Visualizer visualizer_;
    std::array<std::shared_ptr<TriangleMesh>, 2> handMeshes_;
    std::array<bool, 2> handMeshAdded_{};
    std::shared_ptr<TriangleMesh> ballMesh_;
    std::vector<Eigen::Vector3d> ballBaseVertices_;
    std::shared_ptr<TriangleMesh> buttonBase_;
    std::shared_ptr<TriangleMesh> buttonCap_;
    std::vector<Eigen::Vector3d> buttonCapVertices_;
};
