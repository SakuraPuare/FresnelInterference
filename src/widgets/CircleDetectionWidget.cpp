#include "CircleDetectionWidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>
#include <QDebug>
#include <utility>
#include <QTabWidget>
#include <QComboBox>
#include <QStackedWidget>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>

#include <opencv2/imgproc.hpp>

CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent)
    , m_isDetectionActive(false)
    , m_isRecording(false)
    , m_staticImageOffset(0,0)
    , m_nextTrackId(0)
    , m_frameCounter(0)
{
    setupUI();

    // Initialize color palette for tracks
    m_colorPalette << Qt::cyan << Qt::magenta << Qt::yellow << Qt::green << Qt::red << Qt::blue;
    
    // 初始化处理器参数
    updateProcessorParams();
}

void CircleDetectionWidget::setupUI()
{
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    
    // Left side: Image and Plot
    QVBoxLayout* displayLayout = new QVBoxLayout();
    
    QHBoxLayout* imageRowLayout = new QHBoxLayout();
    QGroupBox* origGroup = new QGroupBox("原始图像");
    QVBoxLayout* origLayout = new QVBoxLayout(origGroup);
    m_origImageLabel = new QLabel("等待输入图像...");
    m_origImageLabel->setMinimumSize(320, 240);
    m_origImageLabel->setFrameStyle(QFrame::Box);
    m_origImageLabel->setAlignment(Qt::AlignCenter);
    origLayout->addWidget(m_origImageLabel);
    imageRowLayout->addWidget(origGroup);
    
    QGroupBox* imageGroup = new QGroupBox("处理后检测结果");
    QVBoxLayout* imageLayout = new QVBoxLayout(imageGroup);
    m_imageLabel = new QLabel("等待输入图像...");
    m_imageLabel->setMinimumSize(320, 240);
    m_imageLabel->setFrameStyle(QFrame::Box);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    imageLayout->addWidget(m_imageLabel);
    imageRowLayout->addWidget(imageGroup);
    displayLayout->addLayout(imageRowLayout);

    m_resultText = new QTextEdit();
    m_resultText->setMaximumHeight(100);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; }");
    displayLayout->addWidget(m_resultText);

    // Plotting Area (remains the same)
    QHBoxLayout* plotsLayout = new QHBoxLayout();
    QChart *chartX = new QChart();
    chartX->setTitle("圆心X坐标 vs. 半径");
    m_plotViewX = new QChartView(chartX);
    m_plotViewX->setRenderHint(QPainter::Antialiasing);
    m_axisX_R = new QValueAxis();
    m_axisX_R->setTitleText("半径 (像素)");
    m_axisX_R->setRange(0, 100);
    chartX->addAxis(m_axisX_R, Qt::AlignBottom);
    m_axisX_X = new QValueAxis();
    m_axisX_X->setTitleText("X坐标 (像素)");
    m_axisX_X->setRange(0, 640);
    chartX->addAxis(m_axisX_X, Qt::AlignLeft);
    plotsLayout->addWidget(m_plotViewX);
    
    QChart *chartY = new QChart();
    chartY->setTitle("圆心Y坐标 vs. 半径");
    m_plotViewY = new QChartView(chartY);
    m_plotViewY->setRenderHint(QPainter::Antialiasing);
    m_axisY_R = new QValueAxis();
    m_axisY_R->setTitleText("半径 (像素)");
    m_axisY_R->setRange(0, 100);
    chartY->addAxis(m_axisY_R, Qt::AlignBottom);
    m_axisY_Y = new QValueAxis();
    m_axisY_Y->setTitleText("Y坐标 (像素)");
    m_axisY_Y->setRange(0, 480);
    chartY->addAxis(m_axisY_Y, Qt::AlignLeft);
    plotsLayout->addWidget(m_plotViewY);
    
    displayLayout->addLayout(plotsLayout);
    displayLayout->setStretchFactor(imageRowLayout, 1);
    displayLayout->setStretchFactor(plotsLayout, 1);
    mainLayout->addLayout(displayLayout, 2);
    
    // Right side: Control Panel
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
    // --- Start/Stop Controls ---
    QGroupBox* controlGroup = new QGroupBox("检测控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    m_startButton = new QPushButton("开始检测");
    m_stopButton = new QPushButton("停止检测");
    m_stopButton->setEnabled(false);
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(m_stopButton);
    controlGroupLayout->addLayout(buttonLayout);
    controlLayout->addWidget(controlGroup);

    // --- Algorithm Selection ---
    QGroupBox* algoGroup = new QGroupBox("算法选择");
    QVBoxLayout* algoLayout = new QVBoxLayout(algoGroup);
    m_algorithmSelector = new QComboBox();
    m_algorithmSelector->addItem("Hough圆检测", QVariant::fromValue(DetectionAlgorithm::Hough));
    m_algorithmSelector->addItem("二值化+几何中心", QVariant::fromValue(DetectionAlgorithm::BinaryCenter));
    algoLayout->addWidget(m_algorithmSelector);
    controlLayout->addWidget(algoGroup);
    
    // --- Algorithm Parameters Stack ---
    m_paramsStack = new QStackedWidget();
    
    // Hough Parameters
    m_houghParamsWidget = new QWidget();
    m_paramsGroup = new QGroupBox("HoughCircles参数");
    QFormLayout* houghParamsLayout = new QFormLayout(m_paramsGroup);
    m_dpSlider = new QSlider(Qt::Horizontal);
    m_dpSlider->setRange(10, 50); m_dpSlider->setValue(10);
    houghParamsLayout->addRow("DP (x0.1):", m_dpSlider);
    m_minDistSlider = new QSlider(Qt::Horizontal);
    m_minDistSlider->setRange(10, 200); m_minDistSlider->setValue(50);
    houghParamsLayout->addRow("最小距离:", m_minDistSlider);
    m_cannyThreshSlider = new QSlider(Qt::Horizontal);
    m_cannyThreshSlider->setRange(10, 300); m_cannyThreshSlider->setValue(100);
    houghParamsLayout->addRow("Canny阈值:", m_cannyThreshSlider);
    m_centerThreshSlider = new QSlider(Qt::Horizontal);
    m_centerThreshSlider->setRange(10, 100); m_centerThreshSlider->setValue(30);
    houghParamsLayout->addRow("中心阈值:", m_centerThreshSlider);
    m_minRadiusSlider = new QSlider(Qt::Horizontal);
    m_minRadiusSlider->setRange(1, 100); m_minRadiusSlider->setValue(10);
    houghParamsLayout->addRow("最小半径:", m_minRadiusSlider);
    m_maxRadiusSlider = new QSlider(Qt::Horizontal);
    m_maxRadiusSlider->setRange(10, 300); m_maxRadiusSlider->setValue(100);
    houghParamsLayout->addRow("最大半径:", m_maxRadiusSlider);
    QVBoxLayout* houghWidgetLayout = new QVBoxLayout(m_houghParamsWidget);
    houghWidgetLayout->addWidget(m_paramsGroup);
    houghWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_houghParamsWidget);

    // Binary Parameters
    m_binaryParamsWidget = new QWidget();
    QGroupBox* binaryParamsGroup = new QGroupBox("二值化参数");
    QFormLayout* binaryParamsLayout = new QFormLayout(binaryParamsGroup);
    m_binaryThreshSlider = new QSlider(Qt::Horizontal);
    m_binaryThreshSlider->setRange(0, 255); m_binaryThreshSlider->setValue(128);
    binaryParamsLayout->addRow("二值化阈值:", m_binaryThreshSlider);
    QVBoxLayout* binaryWidgetLayout = new QVBoxLayout(m_binaryParamsWidget);
    binaryWidgetLayout->addWidget(binaryParamsGroup);
    binaryWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_binaryParamsWidget);

    controlLayout->addWidget(m_paramsStack);

    // --- Record Controls ---
    m_recordGroup = new QGroupBox("记录控制");
    QVBoxLayout* recordLayout = new QVBoxLayout(m_recordGroup);
    m_startRecordButton = new QPushButton("开始记录");
    m_clearRecordButton = new QPushButton("清除记录");
    recordLayout->addWidget(m_startRecordButton);
    recordLayout->addWidget(m_clearRecordButton);
    controlLayout->addWidget(m_recordGroup);

    // --- Test Controls ---
    m_testGroup = new QGroupBox("静态图像测试");
    QFormLayout* testLayout = new QFormLayout(m_testGroup);
    m_offsetXSlider = new QSlider(Qt::Horizontal);
    m_offsetXSlider->setRange(-100, 100); m_offsetXSlider->setValue(0);
    testLayout->addRow("X 漂移方向:", m_offsetXSlider);
    m_offsetYSlider = new QSlider(Qt::Horizontal);
    m_offsetYSlider->setRange(-100, 100); m_offsetYSlider->setValue(0);
    testLayout->addRow("Y 漂移方向:", m_offsetYSlider);
    m_offsetLabel = new QLabel("漂移方向: (0, 0)");
    testLayout->addWidget(m_offsetLabel);
    m_zoomSlider = new QSlider(Qt::Horizontal);
    m_zoomSlider->setRange(100, 300); m_zoomSlider->setValue(100);
    testLayout->addRow("缩放/漂移:", m_zoomSlider);
    m_zoomLabel = new QLabel("缩放: 1.00x");
    testLayout->addWidget(m_zoomLabel);
    controlLayout->addWidget(m_testGroup);
    
    // --- Analysis Suggestions ---
    m_analysisGroup = new QGroupBox("分析建议");
    QVBoxLayout* analysisLayout = new QVBoxLayout(m_analysisGroup);
    m_suggestionText = new QTextEdit();
    m_suggestionText->setReadOnly(true);
    m_suggestionText->setMinimumHeight(80);
    analysisLayout->addWidget(m_suggestionText);
    controlLayout->addWidget(m_analysisGroup);
    
    controlLayout->addStretch();
    mainLayout->addLayout(controlLayout, 1);
    
    // Connections
    connect(m_startButton, &QPushButton::clicked, this, &CircleDetectionWidget::startDetection);
    connect(m_stopButton, &QPushButton::clicked, this, &CircleDetectionWidget::stopDetection);
    connect(m_algorithmSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CircleDetectionWidget::onAlgorithmChanged);

    // Param sliders
    connect(m_dpSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minDistSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_cannyThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_centerThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_maxRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_binaryThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);

    connect(m_startRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::startRecording);
    connect(m_clearRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::clearRecording);
    
    connect(m_offsetXSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::updateStaticImageOffset);
    connect(m_offsetYSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::updateStaticImageOffset);
    connect(m_zoomSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::updateStaticImageOffset);
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
    if (frame.empty()) return;

    frame.copyTo(m_originalFrame);
    m_frameCounter++;

    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::applyOffsetAndDetect()
{
    if (m_originalFrame.empty()) return;

    // 应用偏移和缩放变换
    cv::Mat modifiedFrame;
    double scale = m_zoomSlider->value() / 100.0;
    double dx = m_offsetXSlider->value();
    double dy = m_offsetYSlider->value();
    cv::Point2f center(m_originalFrame.cols / 2.0f, m_originalFrame.rows / 2.0f);
    cv::Mat rotMatrix = cv::getRotationMatrix2D(center, 0, scale);
    rotMatrix.at<double>(0, 2) += dx;
    rotMatrix.at<double>(1, 2) += dy;
    cv::warpAffine(m_originalFrame, modifiedFrame, rotMatrix, m_originalFrame.size());

    // 使用新的检测处理器
    DetectionResult result = m_processor.processFrame(modifiedFrame, m_frameCounter);
    
    // 显示结果图像
    m_origImageLabel->setPixmap(matToQPixmap(result.originalImage).scaled(m_origImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_imageLabel->setPixmap(matToQPixmap(result.processedImage).scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    
    // 更新结果文本
    QString resultText = QString("检测到 %1 个目标:\n").arg(result.circles.size());
    
    if (m_isRecording) {
        // 进入追踪模式
        matchAndTrackCircles(result.circles);
        
        QString trackedResultText;
        int trackedCount = 0;
        for (const auto& track : std::as_const(m_trackedObjects)) {
            if (track.framesSinceUpdate == 0 && !track.dataPoints.isEmpty()) {
                trackedCount++;
                const auto& lastData = track.dataPoints.last();
                cv::Point center(cvRound(lastData.x), cvRound(lastData.y));
                int radius = cvRound(lastData.radius);
                
                trackedResultText += QString("轨迹 %1: 中心(%2, %3), 半径: %4\n")
                                    .arg(track.id).arg(center.x).arg(center.y).arg(radius);
            }
        }
        resultText = QString("追踪到 %1 个目标:\n").arg(trackedCount) + trackedResultText;
    } else {
        // 显示检测结果
        for (size_t i = 0; i < result.circles.size(); i++) {
            cv::Vec3f c = result.circles[i];
            cv::Point center(cvRound(c[0]), cvRound(c[1]));
            int radius = cvRound(c[2]);
            
            resultText += QString("目标 %1: 中心(%2, %3), 半径: %4\n")
                         .arg(i+1).arg(center.x).arg(center.y).arg(radius);
        }
    }
    
    m_resultText->setText(resultText);
}

void CircleDetectionWidget::startDetection()
{
    m_isDetectionActive = true;
    m_startButton->setEnabled(false);
    m_stopButton->setEnabled(true);
    emit logMessage("开始圆检测");
    applyOffsetAndDetect();
}

void CircleDetectionWidget::stopDetection()
{
    m_isDetectionActive = false;
    m_startButton->setEnabled(true);
    m_stopButton->setEnabled(false);

    if (m_isRecording) {
        startRecording(); // This will stop recording and reset the button
    }

    m_imageLabel->setText("圆检测已停止");
    m_currentPixmap = QPixmap();
    emit logMessage("停止圆检测");
}

void CircleDetectionWidget::startRecording()
{
    m_isRecording = !m_isRecording;
    if (m_isRecording) {
        m_startRecordButton->setText("停止记录");
        emit logMessage("开始记录圆心坐标");
    } else {
        m_startRecordButton->setText("开始记录");
        emit logMessage("停止记录圆心坐标");
        updateAnalysisAndSuggestions(); // Final analysis
    }
}

void CircleDetectionWidget::clearRecording()
{
    for (const auto& track : std::as_const(m_trackedObjects)) {
        m_plotViewX->chart()->removeSeries(track.seriesXR);
        m_plotViewX->chart()->removeSeries(track.fitLineXR);
        m_plotViewY->chart()->removeSeries(track.seriesYR);
        m_plotViewY->chart()->removeSeries(track.fitLineYR);
        delete track.seriesXR;
        delete track.fitLineXR;
        delete track.seriesYR;
        delete track.fitLineYR;
    }
    m_trackedObjects.clear();
    m_nextTrackId = 0;
    m_suggestionText->clear();
    
    m_axisX_R->setRange(0, 100);
    m_axisX_X->setRange(0, m_originalFrame.cols > 0 ? m_originalFrame.cols : 640);
    m_axisY_R->setRange(0, 100);
    m_axisY_Y->setRange(0, m_originalFrame.rows > 0 ? m_originalFrame.rows : 480);

    emit logMessage("圆心记录已清除");
}

void CircleDetectionWidget::updateStaticImageOffset(int)
{
    int x = m_offsetXSlider->value();
    int y = m_offsetYSlider->value();
    m_staticImageOffset.setX(x);
    m_staticImageOffset.setY(y);
    m_offsetLabel->setText(QString("漂移方向: (%1, %2)").arg(x).arg(y));

    double scale = m_zoomSlider->value() / 100.0;
    m_zoomLabel->setText(QString("缩放: %1x").arg(scale, 0, 'f', 2));
    
    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::onParamsChanged()
{
    updateProcessorParams();
    
    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::onAlgorithmChanged(int index)
{
    auto algorithm = m_algorithmSelector->itemData(index).value<DetectionAlgorithm>();
    
    // 转换为处理器的算法枚举
    ::DetectionAlgorithm processorAlgorithm;
    if (algorithm == CircleDetectionWidget::DetectionAlgorithm::Hough) {
        processorAlgorithm = ::DetectionAlgorithm::Hough;
    } else {
        processorAlgorithm = ::DetectionAlgorithm::BinaryCenter;
    }
    
    m_processor.setAlgorithm(processorAlgorithm);
    m_paramsStack->setCurrentIndex(index);
    emit logMessage(QString("切换检测算法为: %1").arg(m_algorithmSelector->currentText()));
    
    // 强制重新检测
    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::updateProcessorParams()
{
    DetectionParams params;
    params.dp = m_dpSlider->value() / 10.0;
    params.minDist = m_minDistSlider->value();
    params.cannyThresh = m_cannyThreshSlider->value();
    params.centerThresh = m_centerThreshSlider->value();
    params.minRadius = m_minRadiusSlider->value();
    params.maxRadius = m_maxRadiusSlider->value();
    params.binaryThresh = m_binaryThreshSlider->value();
    
    m_processor.setParams(params);
}

void CircleDetectionWidget::matchAndTrackCircles(const std::vector<cv::Vec3f>& newCircles)
{
    const double MAX_DIST = 50.0; // Max distance to match a track
    const int MAX_FRAMES_UNSEEN = 15;

    // Mark all tracks as not updated
    for (auto it = m_trackedObjects.begin(); it != m_trackedObjects.end(); ++it) {
        it->framesSinceUpdate++;
    }

    QList<int> matchedTrackIds;

    for (const auto& circle : newCircles) {
        QPointF center(circle[0], circle[1]);
        double radius = circle[2];
        double bestDist = MAX_DIST;
        int bestTrackId = -1;

        for (auto it = m_trackedObjects.begin(); it != m_trackedObjects.end(); ++it) {
            if (matchedTrackIds.contains(it.key())) continue; // Already matched
            
            double dist = cv::norm(cv::Point2f(center.x(), center.y()) - cv::Point2f(it->lastCenter.x(), it->lastCenter.y()));
            if (dist < bestDist) {
                bestDist = dist;
                bestTrackId = it.key();
            }
        }

        if (bestTrackId != -1) { // Found a match
            TrackedObject& track = m_trackedObjects[bestTrackId];
            track.framesSinceUpdate = 0;
            track.lastCenter = center;
            
            CircleData dataPoint = { center.x(), center.y(), radius };
            track.dataPoints.append(dataPoint);
            track.seriesXR->append(radius, center.x());
            track.seriesYR->append(radius, center.y());
            
            matchedTrackIds.append(bestTrackId);
        } else { // No match, create new track
            int newId = m_nextTrackId++;
            TrackedObject newTrack;
            newTrack.id = newId;
            newTrack.lastCenter = center;
            newTrack.framesSinceUpdate = 0;
            newTrack.color = m_colorPalette[newId % m_colorPalette.size()];
            
            newTrack.seriesXR = new QScatterSeries();
            newTrack.seriesXR->setName(QString("Track %1").arg(newId));
            newTrack.seriesXR->setColor(newTrack.color);
            newTrack.seriesXR->setMarkerSize(8.0);
            
            newTrack.fitLineXR = new QLineSeries();
            newTrack.fitLineXR->setName(QString("Fit %1").arg(newId));
            newTrack.fitLineXR->setColor(newTrack.color);
            
            newTrack.seriesYR = new QScatterSeries();
            newTrack.seriesYR->setName(QString("Track %1").arg(newId));
            newTrack.seriesYR->setColor(newTrack.color);
            newTrack.seriesYR->setMarkerSize(8.0);

            newTrack.fitLineYR = new QLineSeries();
            newTrack.fitLineYR->setName(QString("Fit %1").arg(newId));
            newTrack.fitLineYR->setColor(newTrack.color);
            
            CircleData dataPoint = { center.x(), center.y(), radius };
            newTrack.dataPoints.append(dataPoint);
            newTrack.seriesXR->append(radius, center.x());
            newTrack.seriesYR->append(radius, center.y());

            m_plotViewX->chart()->addSeries(newTrack.seriesXR);
            m_plotViewX->chart()->addSeries(newTrack.fitLineXR);
            newTrack.seriesXR->attachAxis(m_axisX_R);
            newTrack.seriesXR->attachAxis(m_axisX_X);
            newTrack.fitLineXR->attachAxis(m_axisX_R);
            newTrack.fitLineXR->attachAxis(m_axisX_X);

            m_plotViewY->chart()->addSeries(newTrack.seriesYR);
            m_plotViewY->chart()->addSeries(newTrack.fitLineYR);
            newTrack.seriesYR->attachAxis(m_axisY_R);
            newTrack.seriesYR->attachAxis(m_axisY_Y);
            newTrack.fitLineYR->attachAxis(m_axisY_R);
            newTrack.fitLineYR->attachAxis(m_axisY_Y);

            m_trackedObjects.insert(newId, newTrack);
        }
    }

    // Remove old tracks
    QList<int> tracksToRemove;
    for (auto it = m_trackedObjects.begin(); it != m_trackedObjects.end(); ++it) {
        if (it->framesSinceUpdate > MAX_FRAMES_UNSEEN) {
            tracksToRemove.append(it.key());
        }
    }
    for (int id : tracksToRemove) {
        TrackedObject& track = m_trackedObjects[id];
        m_plotViewX->chart()->removeSeries(track.seriesXR);
        m_plotViewX->chart()->removeSeries(track.fitLineXR);
        m_plotViewY->chart()->removeSeries(track.seriesYR);
        m_plotViewY->chart()->removeSeries(track.fitLineYR);
        delete track.seriesXR;
        delete track.fitLineXR;
        delete track.seriesYR;
        delete track.fitLineYR;
        m_trackedObjects.remove(id);
    }

    if (m_isRecording) { // Live update
        updatePlotsAxes();
        updateAnalysisAndSuggestions();
    }
}

void CircleDetectionWidget::updateAnalysisAndSuggestions()
{
    QString suggestion;

    for (auto& track : m_trackedObjects) {
        if (track.dataPoints.size() < 2) continue;

        // Linear regression for X vs R
        double sumR = 0, sumX = 0, sumRR = 0, sumRX = 0;
        for (const auto& p : track.dataPoints) {
            sumR += p.radius;
            sumX += p.x;
            sumRR += p.radius * p.radius;
            sumRX += p.radius * p.x;
        }
        double n = track.dataPoints.size();
        double den = n * sumRR - sumR * sumR;
        if (std::abs(den) < 1e-9) continue; // Avoid division by zero

        double slopeX = (n * sumRX - sumR * sumX) / den;
        double interceptX = (sumX - slopeX * sumR) / n;

        // Linear regression for Y vs R
        double sumY = 0, sumRY = 0;
        for (const auto& p : track.dataPoints) {
            sumY += p.y;
            sumRY += p.radius * p.y;
        }
        double slopeY = (n * sumRY - sumR * sumY) / den;
        double interceptY = (sumY - slopeY * sumR) / n;

        // Update fit lines
        if (!track.dataPoints.isEmpty()) {
            double minR = track.dataPoints.first().radius, maxR = track.dataPoints.first().radius;
            for(const auto& p : track.dataPoints) {
                if (p.radius < minR) minR = p.radius;
                if (p.radius > maxR) maxR = p.radius;
            }
            track.fitLineXR->clear();
            track.fitLineXR->append(minR, slopeX * minR + interceptX);
            track.fitLineXR->append(maxR, slopeX * maxR + interceptX);
            
            track.fitLineYR->clear();
            track.fitLineYR->append(minR, slopeY * minR + interceptY);
            track.fitLineYR->append(maxR, slopeY * maxR + interceptY);
        }

        // Generate suggestion text
        double angle = atan2(slopeY, slopeX) * 180.0 / M_PI;
        double magnitude = std::sqrt(slopeX * slopeX + slopeY * slopeY);

        QString dir;
        if (angle >= -22.5 && angle < 22.5) dir = "右";
        else if (angle >= 22.5 && angle < 67.5) dir = "右下方";
        else if (angle >= 67.5 && angle < 112.5) dir = "下方";
        else if (angle >= 112.5 && angle < 157.5) dir = "左下方";
        else if (angle >= 157.5 || angle < -157.5) dir = "左";
        else if (angle >= -157.5 && angle < -112.5) dir = "左上方";
        else if (angle >= -112.5 && angle < -67.5) dir = "上方";
        else if (angle >= -67.5 && angle < -22.5) dir = "右上方";

        suggestion += QString("<b>轨迹 #%1:</b><br>"
                              "&nbsp;&nbsp;偏移方向: <b>%2</b> (角度: %3°)<br>"
                              "&nbsp;&nbsp;偏移幅度: %4 (像素/像素)<br>"
                              "&nbsp;&nbsp;<b>建议</b>: 向该方向的 <b>反方向</b> 移动光源。<br>")
                              .arg(track.id)
                              .arg(dir)
                              .arg(angle, 0, 'f', 1)
                              .arg(magnitude, 0, 'f', 3);
    }
    m_suggestionText->setHtml(suggestion);
}

void CircleDetectionWidget::updatePlotsAxes()
{
    if (m_trackedObjects.isEmpty()) return;

    // Find overall min/max across all visible tracks
    double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max(), maxY = std::numeric_limits<double>::lowest();
    double minR = std::numeric_limits<double>::max(), maxR = std::numeric_limits<double>::lowest();
    bool hasPoints = false;

    for(const auto& track : std::as_const(m_trackedObjects)) {
        for (const auto& data : track.dataPoints) {
            hasPoints = true;
            if (data.x < minX) minX = data.x;
            if (data.x > maxX) maxX = data.x;
            if (data.y < minY) minY = data.y;
            if (data.y > maxY) maxY = data.y;
            if (data.radius < minR) minR = data.radius;
            if (data.radius > maxR) maxR = data.radius;
        }
    }

    if (!hasPoints) return;

    double xMargin = (maxX - minX) * 0.1 + 10;
    double yMargin = (maxY - minY) * 0.1 + 10;
    double rMargin = (maxR - minR) * 0.1 + 5;

    m_axisX_R->setRange(minR - rMargin, maxR + rMargin);
    m_axisX_X->setRange(minX - xMargin, maxX + xMargin);
    
    m_axisY_R->setRange(minR - rMargin, maxR + rMargin);
    m_axisY_Y->setRange(minY - yMargin, maxY + yMargin);
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