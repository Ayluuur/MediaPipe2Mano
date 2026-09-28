#include "MediaPipe2ManoViewer.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <stdexcept>

namespace {

Eigen::Vector3d vector3(const m2m::Vector3& value) {
    return {value.x, value.y, value.z};
}

Eigen::Matrix3d rotation(const m2m::Quaternion& value) {
    return Eigen::Quaterniond(value.w, value.x, value.y, value.z).normalized().toRotationMatrix();
}

std::shared_ptr<open3d::geometry::TriangleMesh> centeredBox(double width, double height, double depth) {
    auto mesh = open3d::geometry::TriangleMesh::CreateBox(width, height, depth);
    mesh->Translate({-width / 2, -height / 2, 0});
    mesh->ComputeVertexNormals();
    return mesh;
}

} // namespace

MediaPipe2ManoViewer::MediaPipe2ManoViewer(std::string resourceRoot, m2m::RuntimeConfig config,
                                           int cameraIndex, int cameraWidth, int cameraHeight,
                                           QWidget* parent)
    : QMainWindow(parent), resourceRoot_(std::move(resourceRoot)), config_(std::move(config)) {
    buildUi();
    runtime_ = std::make_unique<m2m::Runtime>(resourceRoot_, config_);
    if (!camera_.open(cameraIndex))
        throw std::runtime_error("Cannot open the selected camera");
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, cameraWidth);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, cameraHeight);
    camera_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    processingPool_.setMaxThreadCount(1);
    processingPool_.setExpiryTimeout(-1);
    initializeScene();

    connect(&captureTimer_, &QTimer::timeout, this, [this] { submitFrame(); });
    connect(&frameWatcher_, &QFutureWatcher<ProcessedFrame>::finished, this, [this] { presentFrame(); });
    clock_.start();
    captureTimer_.start(16);
}

MediaPipe2ManoViewer::~MediaPipe2ManoViewer() {
    stopping_ = true;
    captureTimer_.stop();
    frameWatcher_.waitForFinished();
    processingPool_.waitForDone();
    camera_.release();
    visualizer_.DestroyVisualizerWindow();
}

void MediaPipe2ManoViewer::buildUi() {
    setWindowTitle(QStringLiteral("MediaPipe2Mano Viewer"));
    resize(920, 720);
    auto* central = new QWidget(this);
    auto* layout = new QVBoxLayout(central);
    videoLabel_ = new QLabel(QStringLiteral("正在初始化摄像头…"), central);
    videoLabel_->setAlignment(Qt::AlignCenter);
    videoLabel_->setMinimumSize(640, 480);
    videoLabel_->setStyleSheet(QStringLiteral("background:#181818;color:#ddd;"));
    layout->addWidget(videoLabel_, 1);

    auto* controls = new QHBoxLayout();
    pauseButton_ = new QPushButton(QStringLiteral("暂停"), central);
    auto* calibrateButton = new QPushButton(QStringLiteral("重新校准深度"), central);
    auto* frontButton = new QPushButton(QStringLiteral("正视图"), central);
    auto* depthButton = new QPushButton(QStringLiteral("深度视图"), central);
    controls->addWidget(pauseButton_);
    controls->addWidget(calibrateButton);
    controls->addWidget(frontButton);
    controls->addWidget(depthButton);
    controls->addStretch();
    layout->addLayout(controls);
    statusLabel_ = new QLabel(QStringLiteral("运行中"), central);
    layout->addWidget(statusLabel_);
    setCentralWidget(central);

    connect(pauseButton_, &QPushButton::clicked, this, [this] {
        paused_ = !paused_;
        pauseButton_->setText(paused_ ? QStringLiteral("继续") : QStringLiteral("暂停"));
    });
    connect(calibrateButton, &QPushButton::clicked, this,
            [this] { runtime_->requestDepthCalibration(clock_.nsecsElapsed() / 1e9); });
    connect(frontButton, &QPushButton::clicked, this, [this] { setView(QStringLiteral("front")); });
    connect(depthButton, &QPushButton::clicked, this, [this] { setView(QStringLiteral("depth")); });
}

void MediaPipe2ManoViewer::initializeScene() {
    if (!visualizer_.CreateVisualizerWindow("MediaPipe2Mano 3D", 960, 720, 960, 50, true))
        throw std::runtime_error("Open3D could not create its visualization window");
    auto& options = visualizer_.GetRenderOption();
    options.background_color_ = {0.03, 0.03, 0.03};
    options.light_on_ = true;
    options.mesh_show_back_face_ = true;
    auto bounds = std::make_shared<open3d::geometry::PointCloud>();
    bounds->points_ = {{-230, -180, -80}, {230, 180, 650}};
    bounds->colors_ = {options.background_color_, options.background_color_};
    visualizer_.AddGeometry(bounds, true);
    setView(QStringLiteral("front"));

    if (config_.interaction.enabled) {
        ballMesh_ = TriangleMesh::CreateSphere(config_.interaction.ballRadiusMillimeters, 28);
        ballBaseVertices_ = ballMesh_->vertices_;
        ballMesh_->PaintUniformColor({0.95, 0.72, 0.15});
        ballMesh_->ComputeVertexNormals();
        visualizer_.AddGeometry(ballMesh_, false);

        buttonBase_ = centeredBox(config_.interaction.buttonWidthMillimeters + 12,
                                  config_.interaction.buttonHeightMillimeters + 12, 8);
        buttonBase_->Translate(vector3(config_.interaction.buttonCenterMillimeters) +
                               Eigen::Vector3d(0, 0, config_.interaction.buttonTravelMillimeters + 10));
        buttonBase_->PaintUniformColor({0.22, 0.25, 0.30});
        visualizer_.AddGeometry(buttonBase_, false);
        buttonCap_ = centeredBox(config_.interaction.buttonWidthMillimeters,
                                 config_.interaction.buttonHeightMillimeters, 10);
        buttonCapVertices_ = buttonCap_->vertices_;
        visualizer_.AddGeometry(buttonCap_, false);
    }
}

void MediaPipe2ManoViewer::submitFrame() {
    if (stopping_ || paused_ || frameWatcher_.isRunning())
        return;
    const double timestamp = clock_.nsecsElapsed() / 1e9;
    frameWatcher_.setFuture(QtConcurrent::run(&processingPool_, [this, timestamp] {
        ProcessedFrame packet;
        try {
            const auto started = std::chrono::steady_clock::now();
            cv::Mat bgr;
            if (!camera_.read(bgr) || bgr.empty())
                throw std::runtime_error("Camera frame capture failed");
            m2m::ImageView input{bgr.data, bgr.cols, bgr.rows, std::ptrdiff_t(bgr.step),
                                 m2m::PixelFormat::Bgr24};
            packet.result = runtime_->process(input, timestamp);
            if (config_.mirrorInput)
                cv::flip(bgr, bgr, 1);
            cv::Mat rgb;
            cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
            packet.image = QImage(rgb.data, rgb.cols, rgb.rows, int(rgb.step), QImage::Format_RGB888).copy();
            packet.processingMilliseconds =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
        } catch (const std::exception& error) {
            packet.error = QString::fromUtf8(error.what());
        } catch (...) {
            packet.error = QStringLiteral("Unknown frame-processing failure");
        }
        return packet;
    }));
}

void MediaPipe2ManoViewer::presentFrame() {
    if (stopping_)
        return;
    ProcessedFrame packet = frameWatcher_.result();
    if (!packet.error.isEmpty()) {
        stopWithError(packet.error);
        return;
    }
    drawOverlay(packet.image, packet.result);
    videoLabel_->setPixmap(QPixmap::fromImage(packet.image)
                               .scaled(videoLabel_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    updateScene(packet.result);
    statusLabel_->setText(QStringLiteral("手数：%1　处理：%2 ms　%3")
                              .arg(packet.result.hands.size())
                              .arg(packet.processingMilliseconds, 0, 'f', 1)
                              .arg(QString::fromStdString(packet.result.calibrationStatus)));
}

void MediaPipe2ManoViewer::drawOverlay(QImage& image, const m2m::FrameResult& result) const {
    static constexpr int connections[][2] = {{0, 1},   {1, 2},   {2, 3},   {3, 4},   {0, 5},   {5, 6},
                                             {6, 7},   {7, 8},   {5, 9},   {9, 10},  {10, 11}, {11, 12},
                                             {9, 13},  {13, 14}, {14, 15}, {15, 16}, {13, 17}, {0, 17},
                                             {17, 18}, {18, 19}, {19, 20}};
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing);
    for (const auto& hand : result.hands) {
        const QColor color = hand.side == m2m::HandSide::Left ? QColor(70, 180, 255) : QColor(255, 170, 70);
        painter.setPen(QPen(color, 2));
        auto point = [&](int index) {
            return QPointF(hand.screenLandmarks[index].x * image.width(),
                           hand.screenLandmarks[index].y * image.height());
        };
        for (const auto& connection : connections)
            painter.drawLine(point(connection[0]), point(connection[1]));
        painter.setBrush(color);
        for (int index = 0; index < 21; ++index)
            painter.drawEllipse(point(index), 3, 3);
        const QString side =
            hand.side == m2m::HandSide::Left ? QStringLiteral("左手") : QStringLiteral("右手");
        const QString label = QStringLiteral("%1  %2%  Z=%3mm  IK=%4ms")
                                  .arg(side)
                                  .arg(hand.detectionScore * 100, 0, 'f', 0)
                                  .arg(hand.depthMillimeters, 0, 'f', 0)
                                  .arg(hand.ikSeconds * 1000, 0, 'f', 1);
        painter.drawText(point(0) + QPointF(8, -10), label);
    }
}

void MediaPipe2ManoViewer::updateScene(const m2m::FrameResult& result) {
    std::array<bool, 2> present{};
    for (const auto& hand : result.hands) {
        const int side = hand.side == m2m::HandSide::Left ? 0 : 1;
        if (!hand.meshReady)
            continue;
        present[side] = true;
        if (!handMeshes_[side])
            handMeshes_[side] = std::make_shared<TriangleMesh>();
        auto& mesh = *handMeshes_[side];
        mesh.vertices_.resize(hand.mesh.vertices.size());
        for (std::size_t index = 0; index < hand.mesh.vertices.size(); ++index)
            mesh.vertices_[index] = vector3(hand.mesh.vertices[index]);
        if (mesh.triangles_.empty()) {
            mesh.triangles_.reserve(hand.mesh.triangleIndices.size() / 3);
            for (std::size_t index = 0; index + 2 < hand.mesh.triangleIndices.size(); index += 3)
                mesh.triangles_.push_back({int(hand.mesh.triangleIndices[index]),
                                           int(hand.mesh.triangleIndices[index + 1]),
                                           int(hand.mesh.triangleIndices[index + 2])});
            mesh.PaintUniformColor(side == 0 ? Eigen::Vector3d(0.35, 0.65, 1)
                                             : Eigen::Vector3d(1, 0.65, 0.35));
        }
        mesh.ComputeVertexNormals();
        if (!handMeshAdded_[side]) {
            visualizer_.AddGeometry(handMeshes_[side], false);
            handMeshAdded_[side] = true;
        } else
            visualizer_.UpdateGeometry(handMeshes_[side]);
    }
    for (int side = 0; side < 2; ++side)
        if (handMeshAdded_[side] && !present[side]) {
            visualizer_.RemoveGeometry(handMeshes_[side], false);
            handMeshAdded_[side] = false;
        }

    if (ballMesh_) {
        const Eigen::Matrix3d orientation = rotation(result.ball.rotation);
        const Eigen::Vector3d center = vector3(result.ball.centerMillimeters);
        for (std::size_t index = 0; index < ballBaseVertices_.size(); ++index)
            ballMesh_->vertices_[index] =
                orientation * (ballBaseVertices_[index] * result.ball.scale) + center;
        ballMesh_->ComputeVertexNormals();
        visualizer_.UpdateGeometry(ballMesh_);
    }
    if (buttonCap_) {
        const Eigen::Vector3d offset =
            vector3(config_.interaction.buttonCenterMillimeters) +
            Eigen::Vector3d(0, 0, result.button.pressed ? config_.interaction.buttonTravelMillimeters : 0);
        for (std::size_t index = 0; index < buttonCapVertices_.size(); ++index)
            buttonCap_->vertices_[index] = buttonCapVertices_[index] + offset;
        buttonCap_->PaintUniformColor(result.button.pressed ? Eigen::Vector3d(0.15, 1, 0.35)
                                                            : Eigen::Vector3d(0.25, 0.48, 0.85));
        visualizer_.UpdateGeometry(buttonCap_);
    }
    visualizer_.UpdateRender();
    if (!visualizer_.PollEvents())
        close();
}

void MediaPipe2ManoViewer::setView(const QString& name) {
    auto& view = visualizer_.GetViewControl();
    const Eigen::Vector3d front = name == QStringLiteral("front")
                                      ? Eigen::Vector3d(0, 0, -1)
                                      : Eigen::Vector3d(0.28, -0.16, -1).normalized();
    view.SetFront(front);
    view.SetLookat({0, 0, 200});
    view.SetUp({0, 1, 0});
    view.SetZoom(0.72);
    view.ChangeFieldOfView(60 - view.GetFieldOfView());
    view.SetConstantZNear(10);
    view.SetConstantZFar(1500);
}

void MediaPipe2ManoViewer::stopWithError(const QString& message) {
    stopping_ = true;
    captureTimer_.stop();
    statusLabel_->setText(message);
    QMessageBox::critical(this, QStringLiteral("MediaPipe2Mano"), message);
}
