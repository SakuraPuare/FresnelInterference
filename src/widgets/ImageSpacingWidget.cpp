#include "ImageSpacingWidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QDoubleSpinBox>

#include <opencv2/imgproc.hpp>

ImageSpacingWidget::ImageSpacingWidget(QWidget *parent)
    : QWidget(parent)
    , m_isMeasurementActive(false)
{
    setupUI();
}

void ImageSpacingWidget::setupUI()
{
    QHBoxLayout* layout = new QHBoxLayout(this);
    
    // Image Display
    QGroupBox* imageGroup = new QGroupBox("间距测量结果");
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
    QGroupBox* controlGroup = new QGroupBox("间距测量控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton("开始测量");
    m_stopButton = new QPushButton("停止测量");
    m_stopButton->setEnabled(false);
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(m_stopButton);
    controlGroupLayout->addLayout(buttonLayout);
    
    controlLayout->addWidget(controlGroup);
    
    // Parameters
    m_paramsGroup = new QGroupBox("测量参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    
    m_pixelSizeSpin = new QDoubleSpinBox();
    m_pixelSizeSpin->setDecimals(3);
    m_pixelSizeSpin->setRange(0.1, 50.0);
    m_pixelSizeSpin->setSingleStep(0.1);
    m_pixelSizeSpin->setValue(3.45);
    paramsLayout->addRow("像素尺寸(μm):", m_pixelSizeSpin);
    
    controlLayout->addWidget(m_paramsGroup);
    controlLayout->addStretch();
    
    layout->addLayout(controlLayout);
    
    // Connections
    connect(m_startButton, &QPushButton::clicked, this, &ImageSpacingWidget::startMeasurement);
    connect(m_stopButton, &QPushButton::clicked, this, &ImageSpacingWidget::stopMeasurement);
    connect(m_pixelSizeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ImageSpacingWidget::onParamsChanged);
}

void ImageSpacingWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_currentPixmap.isNull()) {
        m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageSpacingWidget::processFrame(const cv::Mat& frame)
{
    if (m_isMeasurementActive && !frame.empty()) {
        cv::Mat result = performMeasurement(frame);
        m_currentPixmap = matToQPixmap(result);
        if (!m_currentPixmap.isNull()) {
            m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    }
}

void ImageSpacingWidget::startMeasurement()
{
    m_isMeasurementActive = true;
    m_startButton->setEnabled(false);
    m_stopButton->setEnabled(true);
    emit logMessage("开始大小像间距测量");
}

void ImageSpacingWidget::stopMeasurement()
{
    m_isMeasurementActive = false;
    m_startButton->setEnabled(true);
    m_stopButton->setEnabled(false);
    m_imageLabel->setText("间距测量已停止");
    m_currentPixmap = QPixmap();
    emit logMessage("停止间距测量");
}

void ImageSpacingWidget::onParamsChanged()
{
    // Can be used to trigger re-processing on param change if needed
}

cv::Mat ImageSpacingWidget::performMeasurement(const cv::Mat& frame)
{
    cv::Mat gray, result = frame.clone();
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    cv::drawContours(result, contours, -1, cv::Scalar(0, 255, 0), 2);
    
    double pixelSize = m_pixelSizeSpin->value();
    QString resultText = QString("像素尺寸: %1 μm\n").arg(pixelSize);
    resultText += QString("检测到 %1 个轮廓\n").arg(contours.size());
    
    if (contours.size() >= 2) {
        std::vector<std::pair<double, cv::Point2f>> areaCenters;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 100) { // Filter small contours
                cv::Moments m = cv::moments(contour);
                if(m.m00 == 0) continue;
                cv::Point2f center(m.m10/m.m00, m.m01/m.m00);
                areaCenters.push_back({area, center});
            }
        }
        
        if (areaCenters.size() >= 2) {
            std::sort(areaCenters.begin(), areaCenters.end(), 
                     [](const auto& a, const auto& b) { return a.first > b.first; });
            
            cv::Point2f center1 = areaCenters[0].second;
            cv::Point2f center2 = areaCenters[1].second;
            
            cv::circle(result, center1, 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(result, center2, 5, cv::Scalar(255, 0, 0), -1);
            cv::line(result, center1, center2, cv::Scalar(0, 0, 255), 2);
            
            double pixelDistance = cv::norm(center1 - center2);
            double realDistance = pixelDistance * pixelSize;
            
            resultText += QString("像素距离: %1 像素\n").arg(pixelDistance, 0, 'f', 2);
            resultText += QString("实际距离: %1 μm\n").arg(realDistance, 0, 'f', 2);
        }
    }
    
    m_resultText->setText(resultText);
    return result;
}

QPixmap ImageSpacingWidget::matToQPixmap(const cv::Mat& mat)
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