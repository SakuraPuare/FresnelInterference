#include "InputControlWidget.h"

#include "io/MVCameraInput.h"
#include "io/VideoInput.h"
#include "io/StaticImageInput.h"
#include "io/FrameManager.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QTime>
#include <QResizeEvent>

InputControlWidget::InputControlWidget(QWidget *parent)
    : QWidget(parent)
    , m_camera(nullptr)
    , m_frameTimer(new QTimer(this))
    , m_frameCount(0)
    , m_lastFrameTime(0)
    , m_currentPixmap()
{
    setupUI();

    m_frameTimer->setInterval(33); // ~30fps
    connect(m_frameTimer, &QTimer::timeout, this, &InputControlWidget::updateFrame);

    emit logMessage("输入控制模块已加载");
}

InputControlWidget::~InputControlWidget()
{
    if (m_camera && m_camera->isOpened()) {
        m_frameTimer->stop();
        m_camera.reset();
    }
}

void InputControlWidget::setupUI()
{
    QHBoxLayout* layout = new QHBoxLayout(this);

    // Image Display Area
    QGroupBox* imageGroup = new QGroupBox("图像显示");
    QVBoxLayout* imageLayout = new QVBoxLayout(imageGroup);
    
    m_imageLabel = new QLabel("无图像");
    m_imageLabel->setMinimumSize(640, 480);
    m_imageLabel->setFrameStyle(QFrame::Box);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setScaledContents(false);
    m_imageLabel->setStyleSheet("QLabel { border: 1px solid palette(mid); }");
    imageLayout->addWidget(m_imageLabel);
    layout->addWidget(imageGroup);

    // Control Panel
    QVBoxLayout* controlLayout = new QVBoxLayout();

    // Camera Control Group
    QGroupBox* cameraControlGroup = new QGroupBox("输入控制");
    QFormLayout* cameraFormLayout = new QFormLayout(cameraControlGroup);
    
    m_cameraTypeCombo = new QComboBox();
    m_cameraTypeCombo->addItems({"迈德威视相机", "普通相机", "图片/视频"});
    cameraFormLayout->addRow("输入源:", m_cameraTypeCombo);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_connectButton = new QPushButton("连接");
    m_disconnectButton = new QPushButton("断开");
    m_disconnectButton->setEnabled(false);
    buttonLayout->addWidget(m_connectButton);
    buttonLayout->addWidget(m_disconnectButton);
    cameraFormLayout->addRow(buttonLayout);

    QHBoxLayout* imageButtonLayout = new QHBoxLayout();
    m_captureButton = new QPushButton("捕获图像");
    m_saveButton = new QPushButton("保存图像");
    m_captureButton->setEnabled(false);
    m_saveButton->setEnabled(false);
    imageButtonLayout->addWidget(m_captureButton);
    imageButtonLayout->addWidget(m_saveButton);
    cameraFormLayout->addRow(imageButtonLayout);

    controlLayout->addWidget(cameraControlGroup);

    // Exposure Control Group
    m_exposureControlGroup = new QGroupBox("相机控制");
    QFormLayout* exposureLayout = new QFormLayout(m_exposureControlGroup);

    QHBoxLayout* exposureHLayout = new QHBoxLayout();
    m_exposureSlider = new QSlider(Qt::Horizontal);
    m_exposureSlider->setRange(100, 100000);
    m_exposureSlider->setValue(10000);
    m_exposureSpinBox = new QSpinBox();
    m_exposureSpinBox->setRange(100, 100000);
    m_exposureSpinBox->setValue(10000);
    exposureHLayout->addWidget(m_exposureSlider);
    exposureHLayout->addWidget(m_exposureSpinBox);
    exposureLayout->addRow("曝光时间(μs):", exposureHLayout);

    QHBoxLayout* gainHLayout = new QHBoxLayout();
    m_gainSlider = new QSlider(Qt::Horizontal);
    m_gainSlider->setRange(0, 100);
    m_gainSlider->setValue(0);
    m_gainSpinBox = new QSpinBox();
    m_gainSpinBox->setRange(0, 100);
    m_gainSpinBox->setValue(0);
    gainHLayout->addWidget(m_gainSlider);
    gainHLayout->addWidget(m_gainSpinBox);
    exposureLayout->addRow("增益:", gainHLayout);

    QHBoxLayout* gammaHLayout = new QHBoxLayout();
    m_gammaSlider = new QSlider(Qt::Horizontal);
    m_gammaSlider->setRange(50, 300);
    m_gammaSlider->setValue(100);
    m_gammaSpinBox = new QSpinBox();
    m_gammaSpinBox->setRange(50, 300);
    m_gammaSpinBox->setValue(100);
    gammaHLayout->addWidget(m_gammaSlider);
    gammaHLayout->addWidget(m_gammaSpinBox);
    exposureLayout->addRow("伽马:", gammaHLayout);

    controlLayout->addWidget(m_exposureControlGroup);
    
    controlLayout->addStretch();
    layout->addLayout(controlLayout);

    // Connections
    connect(m_cameraTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &InputControlWidget::onCameraTypeChanged);
    connect(m_connectButton, &QPushButton::clicked, this, &InputControlWidget::connectDevice);
    connect(m_disconnectButton, &QPushButton::clicked, this, &InputControlWidget::disconnectDevice);
    connect(m_captureButton, &QPushButton::clicked, this, &InputControlWidget::captureImage);
    connect(m_saveButton, &QPushButton::clicked, this, &InputControlWidget::saveImage);
    
    connect(m_exposureSlider, &QSlider::valueChanged, m_exposureSpinBox, &QSpinBox::setValue);
    connect(m_exposureSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), m_exposureSlider, &QSlider::setValue);
    connect(m_exposureSlider, &QSlider::valueChanged, this, &InputControlWidget::onExposureChanged);
    
    connect(m_gainSlider, &QSlider::valueChanged, m_gainSpinBox, &QSpinBox::setValue);
    connect(m_gainSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), m_gainSlider, &QSlider::setValue);
    connect(m_gainSlider, &QSlider::valueChanged, this, &InputControlWidget::onGainChanged);

    connect(m_gammaSlider, &QSlider::valueChanged, m_gammaSpinBox, &QSpinBox::setValue);
    connect(m_gammaSpinBox, QOverload<int>::of(&QSpinBox::valueChanged), m_gammaSlider, &QSlider::setValue);
    connect(m_gammaSlider, &QSlider::valueChanged, this, &InputControlWidget::onGammaChanged);

    onCameraTypeChanged(0); // Set initial state
}

void InputControlWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_currentPixmap.isNull()) {
        m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void InputControlWidget::updateFrame()
{
    if (!m_camera || !m_camera->isOpened()) {
        return;
    }
    
    // 性能优化：使用引用读取，减少内存拷贝
    static uint64_t lastFrameSeq = 0;
    uint64_t currentFrameSeq = m_camera->getFrameSequence();
    
    // 检查帧是否真的更新了
    bool frameUpdated = (currentFrameSeq != lastFrameSeq);
    
    const cv::Mat& frameRef = m_camera->readRef();
    if (frameRef.empty()) {
        // For video files, empty frame means end of video
        if (dynamic_cast<VideoInput*>(m_camera.get())) {
             emit logMessage("视频播放结束。");
             disconnectDevice();
        }
        return;
    }
    
    // 只在帧真正更新时才进行处理
    if (frameUpdated) {
        // 使用FrameManager进行零拷贝帧管理
        auto framePtr = FrameManager::getInstance().updateFrame(frameRef);
        if (framePtr) {
            m_currentFrame = framePtr->frame; // 保存本地引用用于显示
            lastFrameSeq = currentFrameSeq;
            
            // Display raw image - 只在帧更新时重新渲染
            m_currentPixmap = matToQPixmap(m_currentFrame);
            if (!m_currentPixmap.isNull()) {
                m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
            }
            
            // Emit frame for analysis tabs - 现在分析模块可以从FrameManager获取帧
            emit frameReady(m_currentFrame);
        }
    }
    
    // Calculate and display FPS - 即使帧没更新也要计算FPS
    m_frameCount++;
    qint64 currentTime = QTime::currentTime().msecsSinceStartOfDay();
    if (m_lastFrameTime == 0) {
        m_lastFrameTime = currentTime;
    } else if (currentTime - m_lastFrameTime >= 1000) {
        double fps = m_frameCount * 1000.0 / (currentTime - m_lastFrameTime);
        emit updateFps(fps);
        m_frameCount = 0;
        m_lastFrameTime = currentTime;
    }
}

void InputControlWidget::connectDevice()
{
    if (m_camera) {
        disconnectDevice();
    }

    try {
        int index = m_cameraTypeCombo->currentIndex();
        
        if (index == 0) { // "迈德威视相机"
            m_camera = std::make_unique<MVCameraInput>();
        } else if (index == 1) { // "普通相机"
            m_camera = std::make_unique<VideoInput>(0); // Default USB camera
        } else if (index == 2) { // "图片/视频"
            QString fileName = QFileDialog::getOpenFileName(this, "选择图片或视频文件", 
                "", "媒体文件 (*.png *.jpg *.bmp *.tiff *.mp4 *.avi *.mov *.mkv)");
            if (fileName.isEmpty()) return;

            QFileInfo fileInfo(fileName);
            QString suffix = fileInfo.suffix().toLower();
            if (suffix == "png" || suffix == "jpg" || suffix == "bmp" || suffix == "tiff") {
                m_camera = std::make_unique<StaticImageInput>(fileName.toStdString());
            } else {
                m_camera = std::make_unique<VideoInput>(fileName.toStdString());
            }
        }
        
        if (m_camera && m_camera->isOpened()) {
            m_connectButton->setEnabled(false);
            m_disconnectButton->setEnabled(true);
            m_cameraTypeCombo->setEnabled(false);
            m_captureButton->setEnabled(true);
            m_saveButton->setEnabled(true);
            
            m_frameTimer->start();
            emit logMessage(m_cameraTypeCombo->currentText() + "已连接");
        } else {
            m_camera.reset();
            QMessageBox::warning(this, "错误", "无法连接到 " + m_cameraTypeCombo->currentText());
            emit logMessage(m_cameraTypeCombo->currentText() + " 连接失败");
        }
    } catch (const std::exception& e) {
        m_camera.reset();
        QMessageBox::critical(this, "错误", QString("连接时出错: %1").arg(e.what()));
        emit logMessage(QString("连接异常: %1").arg(e.what()));
    }
}

void InputControlWidget::disconnectDevice()
{
    m_frameTimer->stop();
    m_camera.reset();
    
    m_connectButton->setEnabled(true);
    m_disconnectButton->setEnabled(false);
    m_cameraTypeCombo->setEnabled(true);
    m_captureButton->setEnabled(false);
    m_saveButton->setEnabled(false);
    
    m_currentPixmap = QPixmap();
    m_imageLabel->setText("无图像");
    emit logMessage(m_cameraTypeCombo->currentText() + "已断开连接");
    emit updateFps(0);
}


void InputControlWidget::captureImage()
{
    saveImage();
}

void InputControlWidget::saveImage()
{
    if (!m_currentFrame.empty()) {
        QString fileName = QFileDialog::getSaveFileName(this, "保存图像", 
            QString("capture_%1.png").arg(QTime::currentTime().toString("hhmmss")),
            "图像文件 (*.png *.jpg *.bmp)");
        if (!fileName.isEmpty()) {
            cv::imwrite(fileName.toStdString(), m_currentFrame);
            emit logMessage(QString("图像已保存: %1").arg(fileName));
        }
    }
}


void InputControlWidget::onCameraTypeChanged(int index)
{
    // "迈德威视相机"
    m_exposureControlGroup->setVisible(index == 0);

    // "图片/视频" doesn't need a connect button, it's implicit in file dialog
    if (index == 2) {
        m_connectButton->setText("打开文件");
    } else {
        m_connectButton->setText("连接");
    }
}

void InputControlWidget::onExposureChanged(int value)
{
    if (m_camera && m_camera->isOpened()) {
        MVCameraInput* mvCamera = dynamic_cast<MVCameraInput*>(m_camera.get());
        if (mvCamera) {
            if (mvCamera->setExposureTime(value)) {
                emit logMessage(QString("曝光时间已设置为: %1μs").arg(value));
            } else {
                emit logMessage("设置曝光时间失败");
            }
        }
    }
}

void InputControlWidget::onGainChanged(int value)
{
    if (m_camera && m_camera->isOpened()) {
        MVCameraInput* mvCamera = dynamic_cast<MVCameraInput*>(m_camera.get());
        if (mvCamera) {
            if (mvCamera->setGain(value)) {
                emit logMessage(QString("增益已设置为: %1").arg(value));
            } else {
                emit logMessage("设置增益失败");
            }
        }
    }
}

void InputControlWidget::onGammaChanged(int value)
{
    if (m_camera && m_camera->isOpened()) {
        MVCameraInput* mvCamera = dynamic_cast<MVCameraInput*>(m_camera.get());
        if (mvCamera) {
            double gamma = value / 100.0;
            if (mvCamera->setGamma(gamma)) {
                emit logMessage(QString("伽马值已设置为: %1").arg(gamma, 0, 'f', 2));
            } else {
                emit logMessage("设置伽马值失败");
            }
        }
    }
}


QPixmap InputControlWidget::matToQPixmap(const cv::Mat& mat)
{
    if (mat.empty()) {
        return QPixmap();
    }
    
    cv::Mat rgbMat;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
    } else if (mat.channels() == 1) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_GRAY2RGB);
    } else {
        rgbMat = mat;
    }
    
    QImage qimg(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
    return QPixmap::fromImage(qimg);
} 