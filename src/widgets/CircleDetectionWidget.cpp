#include "CircleDetectionWidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>

#include <opencv2/imgproc.hpp>

CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent)
    , m_isDetectionActive(false)
{
    setupUI();
}

void CircleDetectionWidget::setupUI()
{
    QHBoxLayout* layout = new QHBoxLayout(this);
    
    // Image Display
    QGroupBox* imageGroup = new QGroupBox("圆检测结果");
    QVBoxLayout* imageLayout = new QVBoxLayout(imageGroup);
    
    m_imageLabel = new QLabel("等待输入图像...");
    m_imageLabel->setMinimumSize(640, 480);
    m_imageLabel->setFrameStyle(QFrame::Box);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setScaledContents(false);
    m_imageLabel->setStyleSheet("QLabel { background-color: #2b2b2b; color: white; }");
    imageLayout->addWidget(m_imageLabel);
    
    m_resultText = new QTextEdit();
    m_resultText->setMaximumHeight(100);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; }");
    imageLayout->addWidget(m_resultText);
    
    layout->addWidget(imageGroup);
    
    // Control Panel
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
    // Control Buttons
    QGroupBox* controlGroup = new QGroupBox("圆检测控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton("开始检测");
    m_stopButton = new QPushButton("停止检测");
    m_stopButton->setEnabled(false);
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(m_stopButton);
    controlGroupLayout->addLayout(buttonLayout);
    
    controlLayout->addWidget(controlGroup);
    
    // Parameters
    m_paramsGroup = new QGroupBox("HoughCircles参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    
    m_dpSlider = new QSlider(Qt::Horizontal);
    m_dpSlider->setRange(10, 50); // dp is float, use 10-50 and divide by 10.0
    m_dpSlider->setValue(10);
    paramsLayout->addRow("DP (x0.1):", m_dpSlider);
    
    m_minDistSlider = new QSlider(Qt::Horizontal);
    m_minDistSlider->setRange(10, 200);
    m_minDistSlider->setValue(50);
    paramsLayout->addRow("最小距离:", m_minDistSlider);
    
    m_cannyThreshSlider = new QSlider(Qt::Horizontal);
    m_cannyThreshSlider->setRange(10, 300);
    m_cannyThreshSlider->setValue(100);
    paramsLayout->addRow("Canny阈值:", m_cannyThreshSlider);
    
    m_centerThreshSlider = new QSlider(Qt::Horizontal);
    m_centerThreshSlider->setRange(10, 100);
    m_centerThreshSlider->setValue(30);
    paramsLayout->addRow("中心阈值:", m_centerThreshSlider);
    
    m_minRadiusSlider = new QSlider(Qt::Horizontal);
    m_minRadiusSlider->setRange(1, 100);
    m_minRadiusSlider->setValue(10);
    paramsLayout->addRow("最小半径:", m_minRadiusSlider);
    
    m_maxRadiusSlider = new QSlider(Qt::Horizontal);
    m_maxRadiusSlider->setRange(10, 300);
    m_maxRadiusSlider->setValue(100);
    paramsLayout->addRow("最大半径:", m_maxRadiusSlider);
    
    controlLayout->addWidget(m_paramsGroup);
    controlLayout->addStretch();
    
    layout->addLayout(controlLayout);
    
    // Connections
    connect(m_startButton, &QPushButton::clicked, this, &CircleDetectionWidget::startDetection);
    connect(m_stopButton, &QPushButton::clicked, this, &CircleDetectionWidget::stopDetection);
    connect(m_dpSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minDistSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_cannyThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_centerThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_maxRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
}

void CircleDetectionWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_currentPixmap.isNull()) {
        m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void CircleDetectionWidget::processFrame(const cv::Mat& frame)
{
    if (m_isDetectionActive && !frame.empty()) {
        cv::Mat result = performDetection(frame);
        m_currentPixmap = matToQPixmap(result);
        if (!m_currentPixmap.isNull()) {
            m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    }
}

void CircleDetectionWidget::startDetection()
{
    m_isDetectionActive = true;
    m_startButton->setEnabled(false);
    m_stopButton->setEnabled(true);
    emit logMessage("开始圆检测");
}

void CircleDetectionWidget::stopDetection()
{
    m_isDetectionActive = false;
    m_startButton->setEnabled(true);
    m_stopButton->setEnabled(false);
    m_imageLabel->setText("圆检测已停止");
    m_currentPixmap = QPixmap();
    emit logMessage("停止圆检测");
}

void CircleDetectionWidget::onParamsChanged()
{
    // This slot can be used if we need to do something specific when params change.
    // The detection uses the slider values directly, so this can be empty for now.
}


cv::Mat CircleDetectionWidget::performDetection(const cv::Mat& frame)
{
    cv::Mat gray, result = frame.clone();
    if(frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame;
    
    cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);
    
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 
                     m_dpSlider->value() / 10.0,
                     m_minDistSlider->value(),
                     m_cannyThreshSlider->value(),
                     m_centerThreshSlider->value(),
                     m_minRadiusSlider->value(),
                     m_maxRadiusSlider->value());
    
    QString resultText = QString("检测到 %1 个圆:\n").arg(circles.size());
    
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3f c = circles[i];
        cv::Point center(cvRound(c[0]), cvRound(c[1]));
        int radius = cvRound(c[2]);
        
        cv::circle(result, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0); // Center
        cv::circle(result, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0); // Outline
        
        resultText += QString("圆 %1: 中心(%2, %3), 半径: %4\n")
                     .arg(i+1).arg(center.x).arg(center.y).arg(radius);
    }
    
    m_resultText->setText(resultText);
    
    return result;
}

QPixmap CircleDetectionWidget::matToQPixmap(const cv::Mat& mat)
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