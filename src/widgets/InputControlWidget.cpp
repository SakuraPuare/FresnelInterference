#include "InputControlWidget.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

#include "io/StaticImageInput.h"
#include "io/VideoInput.h"
#include "io/MVCameraInput.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QSlider>
#include <QSpinBox>
#include <QTime>
#include <QResizeEvent>
#include <opencv2/imgcodecs.hpp>
#include "utils/QtCvUtils.h"

// InputControlWidget：输入设备控制主界面
InputControlWidget::InputControlWidget(QWidget *parent)
    : QWidget(parent)
    , m_camera(nullptr)
    , m_frameTimer(new QTimer(this))
    , m_frameCount(0)
    , m_lastFrameTime(0)
    , m_currentFrame()
{
    setupUI();
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
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Camera type selection
    QHBoxLayout* topLayout = new QHBoxLayout();
    m_cameraTypeCombo = new QComboBox();
    m_cameraTypeCombo->addItem("迈德威视相机");
    m_cameraTypeCombo->addItem("普通相机");
    m_cameraTypeCombo->addItem("图片/视频");
    topLayout->addWidget(new QLabel("输入源:"));
    topLayout->addWidget(m_cameraTypeCombo);

    m_connectButton = new QPushButton("连接");
    m_disconnectButton = new QPushButton("断开");
    m_captureButton = new QPushButton("拍照");
    m_saveButton = new QPushButton("保存图片");
    m_disconnectButton->setEnabled(false);
    m_captureButton->setEnabled(false);
    m_saveButton->setEnabled(false);

    topLayout->addWidget(m_connectButton);
    topLayout->addWidget(m_disconnectButton);
    topLayout->addWidget(m_captureButton);
    topLayout->addWidget(m_saveButton);

    mainLayout->addLayout(topLayout);

    // Image display
    m_imageLabel = new QLabel();
    m_imageLabel->setMinimumSize(640, 480);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(m_imageLabel);

    setLayout(mainLayout);

    connect(m_connectButton, &QPushButton::clicked, this, &InputControlWidget::connectDevice);
    connect(m_disconnectButton, &QPushButton::clicked, this, &InputControlWidget::disconnectDevice);
    connect(m_captureButton, &QPushButton::clicked, this, &InputControlWidget::captureImage);
    connect(m_saveButton, &QPushButton::clicked, this, &InputControlWidget::saveImage);
    connect(m_cameraTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &InputControlWidget::onCameraTypeChanged);
}

void InputControlWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_currentFrame.empty()) {
        QPixmap pixmap = QtCvUtils::matToQPixmap(m_currentFrame);
        m_imageLabel->setPixmap(pixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void InputControlWidget::updateFrame()
{
    if (m_camera && m_camera->isOpened()) {
        const auto& frame = m_camera->read();
        if (!frame.empty()) {
            m_currentFrame = frame;
            emit frameReady(frame);
            QPixmap pixmap = QtCvUtils::matToQPixmap(frame);
            m_imageLabel->setPixmap(pixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
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
            
            m_frameTimer->start(33);
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
    emit inputSourceChanged();
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
    
    m_currentFrame = cv::Mat();
    m_imageLabel->setText("无图像");
    emit logMessage("输入设备已断开");
    emit updateFps(0);
    emit inputSourceChanged();
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
    disconnectDevice();

    // "图片/视频" doesn't need a connect button, it's implicit in file dialog
    if (index == 2) {
        m_connectButton->setText("打开文件");
    } else {
        m_connectButton->setText("连接");
    }
}

MVCameraInput* InputControlWidget::getCameraInput()
{
    return dynamic_cast<MVCameraInput*>(m_camera.get());
}

void InputControlWidget::setFrameRate(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setFrameRate(value)) {
            logMessage(QString("设置帧率 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::setExposure(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setExposureTime(value)) {
            logMessage(QString("设置曝光 %1 us 失败").arg(value));
        }
    }
}

void InputControlWidget::setGain(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setGain(value)) {
            logMessage(QString("设置增益 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::setGamma(double value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setGamma(value)) {
            logMessage(QString("设置伽马 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::setContrast(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setContrast(value)) {
            logMessage(QString("设置对比度 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::setSharpness(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setSharpness(value)) {
            logMessage(QString("设置锐度 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::setSaturation(int value)
{
    if (auto camera = getCameraInput()) {
        if (!camera->setSaturation(value)) {
            logMessage(QString("设置饱和度 %1 失败").arg(value));
        }
    }
}

void InputControlWidget::onExposureChanged(int value)
{
    // TODO: Implement this slot as needed
    logMessage(QString("onExposureChanged called: %1").arg(value));
}

void InputControlWidget::onGainChanged(int value)
{
    // TODO: Implement this slot as needed
    logMessage(QString("onGainChanged called: %1").arg(value));
}

void InputControlWidget::onGammaChanged(int value)
{
    // TODO: Implement this slot as needed
    logMessage(QString("onGammaChanged called: %1").arg(value));
}

#include "moc_InputControlWidget.cpp"

 