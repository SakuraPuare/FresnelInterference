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
    m_resultText->setMaximumHeight(100);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; }");
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
    if (m_originalFrame.channels() != 1) {
        cv::cvtColor(m_originalFrame, m_originalFrame, cv::COLOR_BGR2GRAY);
    }

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

    // --- Start: FFT-based Angle Detection ---
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(m_currentFrame.rows);
    int n = cv::getOptimalDFTSize(m_currentFrame.cols);
    cv::copyMakeBorder(m_currentFrame, padded, 0, m - m_currentFrame.rows, 0, n - m_currentFrame.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);
    cv::dft(complexI, complexI);
    cv::split(complexI, planes);
    cv::magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magI = planes[0];
    magI += cv::Scalar::all(1);
    cv::log(magI, magI);
    QtCvUtils::fftshift(magI);
    
    // Find the angle of the lines from the spectrum
    cv::Mat magI_clone = magI.clone();
    int cx = magI_clone.cols / 2;
    int cy = magI_clone.rows / 2;
    // Zero out the center of the spectrum to ignore the DC component
    cv::circle(magI_clone, cv::Point(cx, cy), 10, cv::Scalar(0), -1);

    // --- New Robust Angle Detection using PCA ---
    std::vector<cv::Point> locations;
    double maxVal = 0.0;
    cv::minMaxLoc(magI_clone, NULL, &maxVal, NULL, NULL);

    double angle_deg;

    // Only proceed if the signal is strong enough
    if (maxVal > 1e-6) {
        cv::Mat thresholded_mag;
        // Threshold to get high-energy points
        cv::threshold(magI_clone, thresholded_mag, maxVal * 0.5, 255, cv::THRESH_BINARY);
        cv::findNonZero(thresholded_mag, locations);
    }

    // Use PCA if enough points are found, otherwise fallback to max location
    if (locations.size() > 20) {
        cv::Mat data_pts(locations.size(), 2, CV_64F);
        for(size_t i = 0; i < locations.size(); i++) {
            // Center the points around the spectrum's origin for PCA
            data_pts.at<double>(i, 0) = locations[i].x - cx;
            data_pts.at<double>(i, 1) = locations[i].y - cy;
        }

        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
        // The first eigenvector is the direction of highest variance
        cv::Point2f eigenvector(pca_analysis.eigenvectors.at<double>(0, 0), pca_analysis.eigenvectors.at<double>(0, 1));
        angle_deg = atan2(eigenvector.y, eigenvector.x) * 180.0 / CV_PI;
    } else {
        // Fallback for weak signals or very few points
        cv::Point maxLoc;
        cv::minMaxLoc(magI_clone, NULL, NULL, NULL, &maxLoc);
        angle_deg = atan2(maxLoc.y - cy, maxLoc.x - cx) * 180.0 / CV_PI;
    }
    // --- End of PCA-based detection ---

    // Angle of the lines in the image is perpendicular to the angle in the frequency domain.
    double line_angle_deg = angle_deg - 90.0; 

    // The angle needed to rotate the image to make the lines vertical (target_angle = 90)
    // is rotation = target_angle - current_angle
    double rotation_angle = 90.0 - line_angle_deg;

    // Normalize rotation to the shortest path, e.g., -90 to 90
    if (rotation_angle > 90.0) rotation_angle -= 180.0;
    if (rotation_angle < -90.0) rotation_angle += 180.0;

    // For display, show a user-friendly angle (e.g., 0-180 degrees)
    double display_angle = line_angle_deg;
    if (display_angle < 0.0) display_angle += 180.0;

    // --- End: FFT-based Angle Detection ---

    cv::Mat rotatedFrame;
    cv::Point2f center(m_currentFrame.cols / 2.0, m_currentFrame.rows / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, rotation_angle, 1.0);
    cv::warpAffine(m_currentFrame, rotatedFrame, rot, m_currentFrame.size(), cv::INTER_CUBIC, cv::BORDER_REPLICATE);

    cv::Mat projection;
    // Now that the lines are vertical, we project horizontally
    cv::reduce(rotatedFrame, projection, 0, cv::REDUCE_AVG, CV_32F);

    std::vector<float> proj_vec;
    projection.row(0).copyTo(proj_vec);

    if (m_detectValleysCheck->isChecked()) {
        float max_val = *std::max_element(proj_vec.begin(), proj_vec.end());
        for (auto &v : proj_vec) { v = max_val - v; }
    }
    
    std::vector<int> peak_indices;
    QtCvUtils::findTwoStrongestPeaks(proj_vec, m_peakThreshSlider->value(), m_minPeakDistSlider->value(), peak_indices);

    QStringList results;
    results << "分析结果 (冻结帧):\n";
    results << QString("光斑倾斜角度: %1°\n").arg(display_angle, 0, 'f', 2);

    if (peak_indices.size() == 2) {
        double pixelDistance = std::abs(peak_indices[1] - peak_indices[0]);
        double realDistance = pixelDistance * m_pixelSize_um;
        
        results << "检测到 2 个光斑\n";
        results << QString("像素间距: %1 像素\n").arg(pixelDistance, 0, 'f', 2);
        results << QString("物理间距: %1 μm\n").arg(realDistance, 0, 'f', 2);
    } else {
        results << "未能检测到2个足够强的光斑以计算间距。\n";
    }
    
    m_resultText->setText(results.join(""));
    
    cv::Mat displayOriginal, displayProcessed;
    cv::cvtColor(m_frozenFrame, displayOriginal, cv::COLOR_GRAY2BGR, 3);
    cv::cvtColor(rotatedFrame, displayProcessed, cv::COLOR_GRAY2BGR); // Show the rotated image for verification
    
    // Draw results on both images
    drawResult(displayOriginal, peak_indices, line_angle_deg);
    // On the processed image, lines are now vertical, so we can draw them with angle 0 in their coordinate system
    drawResult(displayProcessed, peak_indices, 0); 

    m_currentPixmap = QtCvUtils::matToQPixmap(displayProcessed);
    QPixmap originalPixmap = QtCvUtils::matToQPixmap(displayOriginal);

    if (!originalPixmap.isNull()) {
        m_originalImageLabel->setPixmap(originalPixmap.scaled(m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    if (!m_currentPixmap.isNull()) {
        m_processedImageLabel->setPixmap(m_currentPixmap.scaled(m_processedImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void ImageSpacingWidget::drawResult(cv::Mat &displayImage, const std::vector<int> &peak_indices, double angle_deg) {
    cv::Mat rot_inv;
    cv::Point2f center(displayImage.cols/2.0, displayImage.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle_deg, 1.0);
    cv::invertAffineTransform(rot, rot_inv);

    for (int peak_x : peak_indices) {
        // Create points for a vertical line at peak_x in the rotated coordinate system
        std::vector<cv::Point2f> line_points;
        line_points.push_back(cv::Point2f(peak_x, 0));
        line_points.push_back(cv::Point2f(peak_x, displayImage.rows));
        
        // Transform points back to original image coordinates
        std::vector<cv::Point2f> transformed_points;
        cv::transform(line_points, transformed_points, rot_inv);

        if (transformed_points.size() == 2) {
            cv::line(displayImage, transformed_points[0], transformed_points[1], cv::Scalar(0, 255, 0), 2);
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
            cv::line(displayImage, transformed_points[0], transformed_points[1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
    }
}


 