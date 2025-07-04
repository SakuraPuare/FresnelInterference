#include "ImageSpacingWidget.h"
#include "utils/QtCvUtils.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QSlider>
#include <QCheckBox>
#include <QTimer>

#include <opencv2/imgproc.hpp>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

// ImageSpacingWidget：像间距测量主界面
ImageSpacingWidget::ImageSpacingWidget(QWidget *parent)
    : QWidget(parent)
    , m_isFrozenForAnalysis(false)
    , m_pixelSize_um(3.45)
{
    setupUI();
    
    m_previewUpdateTimer = new QTimer(this);
    m_previewUpdateTimer->setSingleShot(true);
    m_previewUpdateTimer->setInterval(100); 

    connect(m_toggleButton, &QPushButton::toggled, this, &ImageSpacingWidget::toggleFreezeMode);
    
    connect(m_brightnessSlider, &QSlider::valueChanged, this, &ImageSpacingWidget::scheduleReanalysis);
    connect(m_contrastSlider, &QSlider::valueChanged, this, &ImageSpacingWidget::scheduleReanalysis);
    connect(m_gammaSlider, &QSlider::valueChanged, this, &ImageSpacingWidget::scheduleReanalysis);
    connect(m_peakThreshSlider, &QSlider::valueChanged, this, &ImageSpacingWidget::scheduleReanalysis);
    connect(m_minPeakDistSlider, &QSlider::valueChanged, this, &ImageSpacingWidget::scheduleReanalysis);
    connect(m_detectValleysCheck, &QCheckBox::toggled, this, &ImageSpacingWidget::scheduleReanalysis);

    connect(m_previewUpdateTimer, &QTimer::timeout, this, [this]() {
        if (m_isFrozenForAnalysis) {
            performMeasurement();
        } else {
            updatePreviewImage();
        }
    });
}

ImageSpacingWidget::~ImageSpacingWidget()
{
    delete m_previewUpdateTimer;
}

void ImageSpacingWidget::setPixelSize(double pixelSize_um)
{
    m_pixelSize_um = pixelSize_um;
    m_pixelSizeLabel->setText(QString("%1 μm").arg(m_pixelSize_um, 0, 'f', 2));
    if (m_isFrozenForAnalysis) {
        scheduleReanalysis();
    }
}

void ImageSpacingWidget::setupUI()
{
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    
    // Left Panel: Image Displays + Result Text
    QVBoxLayout* leftPanelLayout = new QVBoxLayout();
    
    QHBoxLayout* imageDisplayLayout = new QHBoxLayout();
    QGroupBox* originalGroup = new QGroupBox("原始图像 / 冻结帧");
    QVBoxLayout* originalLayout = new QVBoxLayout(originalGroup);
    m_originalImageLabel = new QLabel("等待输入图像...");
    m_originalImageLabel->setMinimumSize(400, 300);
    m_originalImageLabel->setFrameStyle(QFrame::Box);
    m_originalImageLabel->setAlignment(Qt::AlignCenter);
    m_originalImageLabel->setStyleSheet("QLabel { background-color: #2b2b2b; color: white; }");
    originalLayout->addWidget(m_originalImageLabel);
    
    QGroupBox* processedGroup = new QGroupBox("预处理与分析结果");
    QVBoxLayout* processedLayout = new QVBoxLayout(processedGroup);
    m_processedImageLabel = new QLabel("等待输入图像...");
    m_processedImageLabel->setMinimumSize(400, 300);
    m_processedImageLabel->setFrameStyle(QFrame::Box);
    m_processedImageLabel->setAlignment(Qt::AlignCenter);
    m_processedImageLabel->setStyleSheet("QLabel { background-color: #2b2b2b; color: white; }");
    processedLayout->addWidget(m_processedImageLabel);
    
    imageDisplayLayout->addWidget(originalGroup);
    imageDisplayLayout->addWidget(processedGroup);
    
    leftPanelLayout->addLayout(imageDisplayLayout);
    
    m_resultText = new QTextEdit();
    m_resultText->setMaximumHeight(120);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: 'Courier New', monospace; }");
    leftPanelLayout->addWidget(m_resultText);
    
    mainLayout->addLayout(leftPanelLayout, 4); 
    
    // Right Panel: Controls
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
    QGroupBox* controlGroup = new QGroupBox("分析控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    m_toggleButton = new QPushButton("冻结帧并调参");
    m_toggleButton->setCheckable(true);
    controlGroupLayout->addWidget(m_toggleButton);
    controlLayout->addWidget(controlGroup);
    
    m_paramsGroup = new QGroupBox("分析参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    
    m_peakThreshSlider = new QSlider(Qt::Horizontal);
    m_peakThreshSlider->setRange(1, 254); m_peakThreshSlider->setValue(50);
    paramsLayout->addRow("亮/暗检测阈值:", m_peakThreshSlider);

    m_minPeakDistSlider = new QSlider(Qt::Horizontal);
    m_minPeakDistSlider->setRange(1, 200); m_minPeakDistSlider->setValue(20);
    paramsLayout->addRow("最小特征间距(px):", m_minPeakDistSlider);
    
    m_detectValleysCheck = new QCheckBox("检测暗条纹 (波谷)");
    paramsLayout->addRow(m_detectValleysCheck);
    
    m_pixelSizeLabel = new QLabel(QString("%1 μm").arg(m_pixelSize_um, 0, 'f', 2));
    paramsLayout->addRow("像素尺寸(全局):", m_pixelSizeLabel);
    controlLayout->addWidget(m_paramsGroup);

    m_preprocessGroup = new QGroupBox("图像预处理");
    QFormLayout* preprocessLayout = new QFormLayout(m_preprocessGroup);
    m_brightnessSlider = new QSlider(Qt::Horizontal); m_brightnessSlider->setRange(-100, 100); m_brightnessSlider->setValue(0);
    preprocessLayout->addRow("亮度:", m_brightnessSlider);
    m_contrastSlider = new QSlider(Qt::Horizontal); m_contrastSlider->setRange(1, 200); m_contrastSlider->setValue(100);
    preprocessLayout->addRow("对比度(x0.02):", m_contrastSlider);
    m_gammaSlider = new QSlider(Qt::Horizontal); m_gammaSlider->setRange(10, 300); m_gammaSlider->setValue(100);
    preprocessLayout->addRow("伽马(x0.01):", m_gammaSlider);
    controlLayout->addWidget(m_preprocessGroup);
    
    controlLayout->addStretch();
    mainLayout->addLayout(controlLayout, 1);
}

void ImageSpacingWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (m_originalFrame.empty()) return;

    QPixmap originalPixmap;
    if (m_isFrozenForAnalysis) {
        originalPixmap = QtCvUtils::matToQPixmap(m_frozenFrame);
    } else {
        originalPixmap = QtCvUtils::matToQPixmap(m_originalFrame);
    }
    m_originalImageLabel->setPixmap(originalPixmap.scaled(m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    
    if (!m_currentPixmap.isNull()) {
        m_processedImageLabel->setPixmap(m_currentPixmap.scaled(m_processedImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageSpacingWidget::processFrame(const cv::Mat& frame)
{
    if (m_isFrozenForAnalysis) {
        return; // Ignore new frames when frozen
    }
    updateFrame(frame);
}

void ImageSpacingWidget::toggleFreezeMode(bool checked)
{
    m_isFrozenForAnalysis = checked;
    m_toggleButton->setText(checked ? "返回实时预览" : "冻结帧并调参");

    if (checked) {
        if (m_originalFrame.empty()) {
            m_resultText->setText("错误: 没有图像可以冻结。");
            m_isFrozenForAnalysis = false;
            m_toggleButton->setChecked(false);
            return;
        }
        m_frozenFrame = m_originalFrame.clone();
        performMeasurement(); 
    } else {
        m_resultText->clear();
        updateFrame(m_originalFrame); 
    }
}

void ImageSpacingWidget::scheduleReanalysis() {
    m_previewUpdateTimer->start();
}

void ImageSpacingWidget::updateFrame(const cv::Mat &frame) {
    if (frame.empty()) {
        m_originalImageLabel->setText("无图像输入");
        m_processedImageLabel->setText("无图像输入");
        m_currentFrame.release();
        m_originalFrame.release();
        m_frozenFrame.release();
        return;
    }
    
    m_originalFrame = frame.clone();

    QPixmap originalPixmap = QtCvUtils::matToQPixmap(m_originalFrame);
     if(!originalPixmap.isNull()){
        m_originalImageLabel->setPixmap(originalPixmap.scaled(
            m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    
    updatePreviewImage();
}

void ImageSpacingWidget::updatePreviewImage() {
    if (m_originalFrame.empty()) { return; }

    m_currentPixmap = QtCvUtils::preprocessToQPixmap(m_originalFrame,
                                                   m_brightnessSlider->value(),
                                                   m_contrastSlider->value(),
                                                   m_gammaSlider->value());
    if(!m_currentPixmap.isNull()){
        m_processedImageLabel->setPixmap(m_currentPixmap.scaled(
            m_processedImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageSpacingWidget::performMeasurement()
{
    if (m_frozenFrame.empty()) {
        m_resultText->setText("错误：没有冻结的图像用于分析。");
        return;
    }

    cv::Mat processedFrame;
    QtCvUtils::applyPreprocessing(m_frozenFrame, processedFrame,
                                 m_brightnessSlider->value(),
                                 m_contrastSlider->value(),
                                 m_gammaSlider->value());
    m_currentFrame = processedFrame.clone();

    cv::Mat gray_img;
    if (processedFrame.channels() == 3) {
        cv::cvtColor(processedFrame, gray_img, cv::COLOR_BGR2GRAY);
    } else {
        gray_img = processedFrame;
    }

    cv::threshold(gray_img, gray_img, m_peakThreshSlider->value(), 255, m_detectValleysCheck->isChecked() ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::sort(contours.begin(), contours.end(), [](const auto& c1, const auto& c2) {
        return cv::contourArea(c1) > cv::contourArea(c2);
    });

    // Prepare display images (clones, so we won't accumulate drawings)
    cv::Mat displayOriginal;
    if (m_frozenFrame.channels() == 1) {
        cv::cvtColor(m_frozenFrame, displayOriginal, cv::COLOR_GRAY2BGR);
    } else {
        displayOriginal = m_frozenFrame.clone();
    }

    cv::Mat displayProcessed;
    if (m_currentFrame.channels() == 1) {
        cv::cvtColor(m_currentFrame, displayProcessed, cv::COLOR_GRAY2BGR);
    } else {
        displayProcessed = m_currentFrame.clone();
    }

    // --------------------------------------------------
    // 1. Largest-area contour pair (basic analysis)
    // --------------------------------------------------
    QString result_string;
    if (contours.size() >= 2) {
        auto& c1 = contours[0];
        auto& c2 = contours[1];

        cv::Moments M1 = cv::moments(c1);
        cv::Moments M2 = cv::moments(c2);

        cv::Point2f center1(M1.m10 / M1.m00, M1.m01 / M1.m00);
        cv::Point2f center2(M2.m10 / M2.m00, M2.m01 / M2.m00);

        double dx_px = std::abs(center1.x - center2.x);
        double dy_px = std::abs(center1.y - center2.y);
        double dist_px = std::sqrt(dx_px * dx_px + dy_px * dy_px);
        
        double dist_um = dist_px * m_pixelSize_um;
        double dx_um = dx_px * m_pixelSize_um;
        double dy_um = dy_px * m_pixelSize_um;

        double angle_rad = std::atan2(center2.y - center1.y, center2.x - center1.x);
        double angle_deg = angle_rad * 180.0 / CV_PI;

        result_string.append(QString("检测到两个最大轮廓:\n"));
        result_string.append(QString("中心点1: (%1, %2)\n").arg(center1.x, 0, 'f', 1).arg(center1.y, 0, 'f', 1));
        result_string.append(QString("中心点2: (%1, %2)\n").arg(center2.x, 0, 'f', 1).arg(center2.y, 0, 'f', 1));
        result_string.append(QString("中心间距: %1 px (%2 μm)\n").arg(dist_px, 0, 'f', 2).arg(dist_um, 0, 'f', 2));
        result_string.append(QString("水平间距(dX): %1 px (%2 μm)\n").arg(dx_px, 0, 'f', 2).arg(dx_um, 0, 'f', 2));
        result_string.append(QString("垂直间距(dY): %1 px (%2 μm)\n").arg(dy_px, 0, 'f', 2).arg(dy_um, 0, 'f', 2));
        result_string.append(QString("连线角度: %1°\n").arg(angle_deg, 0, 'f', 2));
        
        // Draw on both original & processed display images
        drawResult(displayProcessed, {center1, center2}, angle_deg);
        drawResult(displayOriginal, {center1, center2}, angle_deg);
    } else {
        result_string = "未能检测到足够的目标（需要至少两个）。";
    }

    // --------------------------------------------------
    // 2. New contour-based analysis + weighted value
    // --------------------------------------------------
    cv::Mat thresh_img;
    cv::threshold(m_currentFrame, thresh_img, m_peakThreshSlider->value(), 255, m_detectValleysCheck->isChecked() ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours_new;
    cv::findContours(thresh_img, contours_new, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(contours_new.begin(), contours_new.end(), [](const auto& a, const auto& b){ return cv::contourArea(a) > cv::contourArea(b);} );

    QStringList contour_results;
    if (contours_new.size() >= 2) {
        cv::Moments M1 = cv::moments(contours_new[0]);
        cv::Moments M2 = cv::moments(contours_new[1]);
        if (M1.m00 > 0 && M2.m00 > 0) {
            cv::Point2f center1(M1.m10 / M1.m00, M1.m01 / M1.m00);
            cv::Point2f center2(M2.m10 / M2.m00, M2.m01 / M2.m00);
            double dx_px = std::abs(center1.x - center2.x);
            double dy_px = std::abs(center1.y - center2.y);
            double dist_px = std::sqrt(dx_px * dx_px + dy_px * dy_px);

            double dx_um = dx_px * m_pixelSize_um;
            double dy_um = dy_px * m_pixelSize_um;
            double dist_um = dist_px * m_pixelSize_um;
            double final_um = (dist_um + dx_um + dy_um) / 3.0;

            contour_results << "\n--- 轮廓分析 (新) ---";
            contour_results << QString("中心间距: %1 px (%2 μm)").arg(dist_px, 0, 'f', 2).arg(dist_um, 0, 'f', 2);
            contour_results << QString("水平间距: %1 px (%2 μm)").arg(dx_px, 0, 'f', 2).arg(dx_um, 0, 'f', 2);
            contour_results << QString("垂直间距: %1 px (%2 μm)").arg(dy_px, 0, 'f', 2).arg(dy_um, 0, 'f', 2);
            contour_results << QString("加权结果(平均): %1 μm").arg(final_um, 0, 'f', 2);

            // Draw on both display images (red line + circles)
            int thickness = std::max(1, displayProcessed.cols / 300);
            cv::Scalar drawColor(0,0,255);
            cv::line(displayProcessed, center1, center2, drawColor, thickness);
            cv::circle(displayProcessed, center1, 10, drawColor, thickness);
            cv::circle(displayProcessed, center2, 10, drawColor, thickness);

            cv::line(displayOriginal, center1, center2, drawColor, thickness);
            cv::circle(displayOriginal, center1, 10, drawColor, thickness);
            cv::circle(displayOriginal, center2, 10, drawColor, thickness);

            m_resultText->append(contour_results.join("\n"));
        }
    }

    // --------------------------------------------------
    // Update result text & image labels
    // --------------------------------------------------
    m_resultText->setText(result_string);

    QPixmap procPixmap = QtCvUtils::matToQPixmap(displayProcessed);
    QPixmap origPixmap = QtCvUtils::matToQPixmap(displayOriginal);

    if (!procPixmap.isNull()) {
        m_processedImageLabel->setPixmap(procPixmap.scaled(m_processedImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    if (!origPixmap.isNull()) {
        m_originalImageLabel->setPixmap(origPixmap.scaled(m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageSpacingWidget::drawResult(cv::Mat &displayImage, const std::vector<cv::Point2f> &centers, double angle_deg)
{
    if (centers.size() < 2) return;

    int thickness = std::max(1, displayImage.cols / 300);
    cv::Scalar color = cv::Scalar(0, 0, 255); // Red

    const auto& center1 = centers[0];
    const auto& center2 = centers[1];

    cv::line(displayImage, center1, center2, color, thickness);
    cv::circle(displayImage, center1, 10, color, thickness);
    cv::circle(displayImage, center2, 10, color, thickness);

    QString angle_text = QString::number(angle_deg, 'f', 1) + " deg";
    cv::Point text_pos((center1.x + center2.x) / 2, (center1.y + center2.y) / 2);
    cv::putText(displayImage, angle_text.toStdString(), text_pos, cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2);
}

void ImageSpacingWidget::drawResult(cv::Mat &displayImage, const std::vector<int> &peak_indices, double angle_deg) {
    cv::Mat rot_inv;
    cv::Point2f center(displayImage.cols/2.0, displayImage.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle_deg, 1.0);
    cv::invertAffineTransform(rot, rot_inv);

    int thickness = std::max(1, displayImage.cols / 300);
    cv::Scalar peak_color = cv::Scalar(0, 255, 0); // Green
    cv::Scalar line_color = cv::Scalar(0, 0, 255); // Red

    for (int peak_x : peak_indices) {
        // Create points for a vertical line at peak_x in the rotated coordinate system
        std::vector<cv::Point2f> line_points;
        line_points.push_back(cv::Point2f(peak_x, 0));
        line_points.push_back(cv::Point2f(peak_x, displayImage.rows));
        
        // Transform points back to original image coordinates
        std::vector<cv::Point2f> transformed_points;
        cv::transform(line_points, transformed_points, rot_inv);

        if (transformed_points.size() == 2) {
            cv::line(displayImage, transformed_points[0], transformed_points[1], peak_color, thickness);
        }
    }

    if (peak_indices.size() == 2) {
         // Create points for a horizontal line connecting the peaks in the rotated coordinate system
        std::vector<cv::Point2f> line_points;
        line_points.push_back(cv::Point2f(peak_indices[0], displayImage.rows / 2));
        line_points.push_back(cv::Point2f(peak_indices[1], displayImage.rows / 2));

        // Transform points back to original image coordinates
        std::vector<cv::Point2f> transformed_points;
        cv::transform(line_points, transformed_points, rot_inv);
        
        if (transformed_points.size() == 2) {
            cv::line(displayImage, transformed_points[0], transformed_points[1], line_color, thickness, cv::LINE_AA);
        }
    }
}

 