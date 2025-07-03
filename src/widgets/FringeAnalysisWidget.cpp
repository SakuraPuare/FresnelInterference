#include "FringeAnalysisWidget.h"
#include "utils/QtCvUtils.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>
#include <QSpinBox>
#include <QCheckBox>
#include <QTimer>

#include <opencv2/imgproc.hpp>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

// FringeAnalysisWidget：干涉条纹分析主界面
FringeAnalysisWidget::FringeAnalysisWidget(QWidget *parent)
    : QWidget(parent)
    , m_pixelSize_um(3.45) // Default value
    , m_isFrozenForAnalysis(false)
{
    setupUI();

    m_previewUpdateTimer = new QTimer(this);
    m_previewUpdateTimer->setSingleShot(true);
    m_previewUpdateTimer->setInterval(100); // 100ms debounce time for smoother tuning

    connect(m_startButton, &QPushButton::toggled, this, &FringeAnalysisWidget::toggleFreezeMode);
    
    // Connect ALL parameter changes to schedule a re-analysis
    connect(m_brightnessSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::scheduleReanalysis);
    connect(m_contrastSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::scheduleReanalysis);
    connect(m_gammaSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::scheduleReanalysis);
    connect(m_peakThreshSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::scheduleReanalysis);
    connect(m_minPeakDistSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::scheduleReanalysis);
    connect(m_detectValleysCheck, &QCheckBox::toggled, this, &FringeAnalysisWidget::scheduleReanalysis);

    // The timer will trigger the actual analysis/preview update
    connect(m_previewUpdateTimer, &QTimer::timeout, this, [this]() {
        if (m_isFrozenForAnalysis) {
            performAnalysis();
        } else {
            updatePreviewImage();
        }
    });
}

FringeAnalysisWidget::~FringeAnalysisWidget()
{
    delete m_previewUpdateTimer;
}

void FringeAnalysisWidget::setPixelSize(double pixelSize_um)
{
    m_pixelSize_um = pixelSize_um;
    m_pixelSizeLabel->setText(QString("%1 μm").arg(m_pixelSize_um, 0, 'f', 2));
    scheduleReanalysis();
}

void FringeAnalysisWidget::setupUI()
{
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    
    // Image display area with two images
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

    // Main layout container for images and results text
    QVBoxLayout* leftPanelLayout = new QVBoxLayout();
    leftPanelLayout->addLayout(imageDisplayLayout);
    
    m_resultText = new QTextEdit();
    m_resultText->setMaximumHeight(100);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; }");
    leftPanelLayout->addWidget(m_resultText);
    
    mainLayout->addLayout(leftPanelLayout, 4); 
    
    // Right panel for controls
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
    QGroupBox* controlGroup = new QGroupBox("分析控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    
    m_startButton = new QPushButton("冻结帧并调参");
    m_startButton->setCheckable(true);
    controlGroupLayout->addWidget(m_startButton);
    controlLayout->addWidget(controlGroup);
    
    m_paramsGroup = new QGroupBox("分析参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    
    m_peakThreshSlider = new QSlider(Qt::Horizontal);
    m_peakThreshSlider->setRange(1, 254); m_peakThreshSlider->setValue(50);
    paramsLayout->addRow("亮/暗检测阈值:", m_peakThreshSlider);

    m_minPeakDistSlider = new QSlider(Qt::Horizontal);
    m_minPeakDistSlider->setRange(1, 100); m_minPeakDistSlider->setValue(10);
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

void FringeAnalysisWidget::resizeEvent(QResizeEvent* event)
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

void FringeAnalysisWidget::processFrame(const cv::Mat& frame)
{
    if (m_isFrozenForAnalysis) {
        return; // Ignore new frames when frozen
    }
    updateFrame(frame);
}

void FringeAnalysisWidget::toggleFreezeMode(bool checked)
{
    m_isFrozenForAnalysis = checked;
    m_startButton->setText(checked ? "返回实时预览" : "冻结帧并调参");

    if (checked) {
        // Entering freeze mode
        if (m_originalFrame.empty()) {
            m_resultText->setText("错误: 没有图像可以冻结。");
            m_isFrozenForAnalysis = false;
            m_startButton->setChecked(false);
            return;
        }
        m_frozenFrame = m_originalFrame.clone();
        performAnalysis(); // Perform initial analysis immediately
    } else {
        // Returning to live preview mode
        m_resultText->clear();
        updateFrame(m_originalFrame); // Refresh display with latest live frame
    }
}

void FringeAnalysisWidget::scheduleReanalysis() {
    // This function acts as a gatekeeper.
    // In freeze mode, any param change triggers a debounced re-analysis.
    // In live mode, pre-processing changes trigger a debounced preview update.
    m_previewUpdateTimer->start();
}

void FringeAnalysisWidget::updatePreviewImage() {
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

void FringeAnalysisWidget::performAnalysis()
{
    if (m_frozenFrame.empty()) {
        m_resultText->setText("错误：没有冻结的图像用于分析。");
        return;
    }

    // Always operate on the frozen frame
    cv::Mat processedFrame;
    QtCvUtils::applyPreprocessing(m_frozenFrame, processedFrame,
                                 m_brightnessSlider->value(),
                                 m_contrastSlider->value(),
                                 m_gammaSlider->value());
    m_currentFrame = processedFrame.clone();

    cv::Mat grayFrame = m_currentFrame; // applyPreprocessing已转灰度

    cv::Mat padded;
    int m = cv::getOptimalDFTSize(grayFrame.rows);
    int n = cv::getOptimalDFTSize(grayFrame.cols);
    cv::copyMakeBorder(grayFrame, padded, 0, m - grayFrame.rows, 0, n - grayFrame.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    
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
    cv::normalize(magI, magI, 0, 1, cv::NORM_MINMAX);

    cv::Point maxLoc;
    cv::minMaxLoc(magI, NULL, NULL, NULL, &maxLoc);

    double angle_rad = atan2(maxLoc.y - magI.rows / 2.0, maxLoc.x - magI.cols / 2.0);
    double angle_deg = (angle_rad * 180.0 / CV_PI);
    double rotation_angle = angle_deg - 90;

    cv::Mat rotatedFrame;
    cv::Point2f center(grayFrame.cols / 2.0, grayFrame.rows / 2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, rotation_angle, 1.0);
    cv::warpAffine(grayFrame, rotatedFrame, rot, grayFrame.size(), cv::INTER_CUBIC, cv::BORDER_REPLICATE);

    cv::Mat projection;
    cv::reduce(rotatedFrame, projection, 1, cv::REDUCE_AVG, CV_32F);

    std::vector<float> proj_vec;
    projection.col(0).copyTo(proj_vec);

    if (m_detectValleysCheck->isChecked()) {
        float max_val = *std::max_element(proj_vec.begin(), proj_vec.end());
        for (auto &v : proj_vec) { v = max_val - v; }
    }
    
    std::vector<int> peak_indices;
    int min_dist = m_minPeakDistSlider->value();
    float threshold = m_peakThreshSlider->value();
    QtCvUtils::findPeaks(proj_vec, threshold, min_dist, peak_indices);

    QStringList results;
    results << "分析结果 (冻结帧):\n";
    results << QString("条纹倾斜角度: %1°\n").arg(angle_deg, 0, 'f', 2);

    if (peak_indices.size() > 1) {
        std::vector<double> distances;
        for (size_t i = 0; i < peak_indices.size() - 1; ++i) {
            distances.push_back(static_cast<double>(peak_indices[i + 1] - peak_indices[i]));
        }

        double avg_pixel_distance = 0;
        if (distances.size() > 2) {
            double mean, stddev;
            QtCvUtils::calculateStats(distances, mean, stddev);
            std::vector<double> filtered_distances;
            for (double dist : distances) {
                if (std::abs(dist - mean) <= 1.5 * stddev) {
                    filtered_distances.push_back(dist);
                }
            }
            if (!filtered_distances.empty()) {
                avg_pixel_distance = std::accumulate(filtered_distances.begin(), filtered_distances.end(), 0.0) / filtered_distances.size();
                results << QString("（已过滤 %1 个异常值）\n").arg(distances.size() - filtered_distances.size());
            } else {
                avg_pixel_distance = mean;
                results << "（警告：所有间距均为异常值，使用原始平均值）\n";
            }
        } else if (!distances.empty()) {
            avg_pixel_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        }

        double avg_real_distance = avg_pixel_distance * m_pixelSize_um;
        results << QString("检测到 %1 个条纹\n").arg(peak_indices.size());
        results << QString("平均像素间距: %1 像素\n").arg(avg_pixel_distance, 0, 'f', 2);
        results << QString("平均物理间距: %1 μm\n").arg(avg_real_distance, 0, 'f', 2);
    } else {
        results << "未能检测到足够的条纹以计算间距。\n";
    }
    
    m_resultText->setText(results.join(""));
    
    // Create display images and draw results on both
    cv::Mat displayOriginal, displayProcessed;
    cv::cvtColor(m_frozenFrame, displayOriginal, cv::COLOR_GRAY2BGR, 3);
    cv::cvtColor(grayFrame, displayProcessed, cv::COLOR_GRAY2BGR);
    
    drawResult(displayOriginal, rotation_angle, peak_indices, projection);
    drawResult(displayProcessed, rotation_angle, peak_indices, projection);

    // Update labels
    m_currentPixmap = QtCvUtils::matToQPixmap(displayProcessed);
    QPixmap originalPixmap = QtCvUtils::matToQPixmap(displayOriginal);

    if (!originalPixmap.isNull()) {
        m_originalImageLabel->setPixmap(originalPixmap.scaled(m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    if (!m_currentPixmap.isNull()) {
        m_processedImageLabel->setPixmap(m_currentPixmap.scaled(m_processedImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void FringeAnalysisWidget::updateFrame(const cv::Mat &frame) {
    if (frame.empty()) {
        m_originalImageLabel->setText("无图像输入");
        m_processedImageLabel->setText("无图像输入");
        m_currentFrame.release();
        m_originalFrame.release();
        m_frozenFrame.release();
        return;
    }
    
    // Always update the live frame
    m_originalFrame = frame.clone(); 
    if (m_originalFrame.channels() != 1) {
        cv::cvtColor(m_originalFrame, m_originalFrame, cv::COLOR_BGR2GRAY);
    }

    // Display original frame on the left
    QPixmap originalPixmap = QtCvUtils::matToQPixmap(m_originalFrame);
     if(!originalPixmap.isNull()){
        m_originalImageLabel->setPixmap(originalPixmap.scaled(
            m_originalImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    
    // Update the preprocessed preview on the right
    updatePreviewImage();
}

void FringeAnalysisWidget::drawResult(cv::Mat &displayImage, double angle, const std::vector<int> &peak_indices, const cv::Mat &projection) {
    // This function can be expanded to draw detailed results on the image
    // For now, it just marks the detected peaks.

    // No need to rotate the display image here, we'll transform points instead.
    // The analysis gives us peak locations in the rotated-and-projected coordinate system.
    // We need to draw lines on the original-orientation (but preprocessed) image.
    
    cv::Mat rot_inv;
    cv::Point2f center(displayImage.cols/2.0, displayImage.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::invertAffineTransform(rot, rot_inv);

    for (int peak_y : peak_indices) {
        // Create points for a horizontal line at peak_y in the rotated coordinate system
        std::vector<cv::Point2f> line_points;
        line_points.push_back(cv::Point2f(0, peak_y));
        line_points.push_back(cv::Point2f(displayImage.cols, peak_y));
        
        // Transform points back to original image coordinates
        std::vector<cv::Point2f> transformed_points;
        cv::transform(line_points, transformed_points, rot_inv);

        if (transformed_points.size() == 2) {
            cv::line(displayImage, transformed_points[0], transformed_points[1], cv::Scalar(0, 255, 0), 1);
        }
    }
}