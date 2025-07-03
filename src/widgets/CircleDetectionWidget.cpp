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
#include <QCheckBox>
#include <QComboBox>
#include <QStackedWidget>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

#include <opencv2/imgproc.hpp>
#include "io/FrameManager.h"
#include "AnalysisModule.h"
#include <QtConcurrent/QtConcurrent>

CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent)
    , m_isDetectionActive(false)
    , m_isRecording(false)
    , m_staticImageOffset(0,0)
    , m_currentAlgorithm(DetectionAlgorithm::Hough)
    , m_frameCounter(0)
    , m_isStaticImageSource(false)
{
    setupUI();

    // 初始化算法选择
    m_algorithmSelector->setCurrentIndex(0); // 默认选择Hough算法
    m_paramsStack->setCurrentIndex(0);
    
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
    m_algorithmSelector->addItem("几何中心检测", QVariant::fromValue(DetectionAlgorithm::GeometricCenter));
    algoLayout->addWidget(m_algorithmSelector);
    controlLayout->addWidget(algoGroup);
    
    // --- Algorithm Parameters Stack ---
    m_paramsStack = new QStackedWidget();
    
    // Hough Parameters
    m_houghParamsWidget = new QWidget();
    m_houghParamsGroup = new QGroupBox("Hough算法参数");
    QFormLayout* houghParamsLayout = new QFormLayout(m_houghParamsGroup);
    
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
    
    // 二值化预处理选项
    m_useBinaryCheckBox = new QCheckBox("使用二值化预处理");
    m_useBinaryCheckBox->setChecked(false);
    houghParamsLayout->addRow(m_useBinaryCheckBox);
    
    m_binaryThreshSlider = new QSlider(Qt::Horizontal);
    m_binaryThreshSlider->setRange(0, 255); m_binaryThreshSlider->setValue(128);
    m_binaryThreshSlider->setEnabled(false); // 默认禁用
    houghParamsLayout->addRow("二值化阈值:", m_binaryThreshSlider);
    
    QVBoxLayout* houghWidgetLayout = new QVBoxLayout(m_houghParamsWidget);
    houghWidgetLayout->addWidget(m_houghParamsGroup);
    houghWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_houghParamsWidget);

    // Geometric Center Parameters
    m_geometricParamsWidget = new QWidget();
    m_geometricParamsGroup = new QGroupBox("几何中心算法参数");
    QFormLayout* geometricParamsLayout = new QFormLayout(m_geometricParamsGroup);
    
    m_geometricBinaryThreshSlider = new QSlider(Qt::Horizontal);
    m_geometricBinaryThreshSlider->setRange(0, 255); m_geometricBinaryThreshSlider->setValue(128);
    geometricParamsLayout->addRow("二值化阈值:", m_geometricBinaryThreshSlider);
    
    m_inverseGeometricCheckBox = new QCheckBox("反向二值化");
    m_inverseGeometricCheckBox->setChecked(false);
    geometricParamsLayout->addRow(m_inverseGeometricCheckBox);
    
    QVBoxLayout* geometricWidgetLayout = new QVBoxLayout(m_geometricParamsWidget);
    geometricWidgetLayout->addWidget(m_geometricParamsGroup);
    geometricWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_geometricParamsWidget);

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

    // Hough param controls
    connect(m_dpSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minDistSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_cannyThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_centerThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_maxRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_useBinaryCheckBox, &QCheckBox::toggled, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_useBinaryCheckBox, &QCheckBox::toggled, m_binaryThreshSlider, &QSlider::setEnabled);
    connect(m_binaryThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    
    // Geometric center param controls
    connect(m_geometricBinaryThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_inverseGeometricCheckBox, &QCheckBox::toggled, this, &CircleDetectionWidget::onParamsChanged);

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
    m_originalFrame = frame.clone();
    m_frameCounter++;
    auto future = QtConcurrent::run([=]() {
        DetectionResult result = m_processor.processFrame(frame, m_frameCounter);
        // 多中心点处理：选择最大半径的圆为主分析点，其余也显示
        int mainIdx = -1;
        double maxR = -1;
        for (size_t i = 0; i < result.circles.size(); ++i) {
            if (result.circles[i][2] > maxR) {
                maxR = result.circles[i][2];
                mainIdx = i;
            }
        }
        // 只有在记录时才添加点
        if (m_isRecording && !result.circles.empty()) {
            for (const auto& c : result.circles) {
                m_analysisModule.addPoint(c[0], c[1], c[2]);
            }
        }
        // UI刷新（图片和检测结果始终显示，便于调参）
        QPixmap origPixmap = QtCvUtils::matToQPixmap(result.originalImage).scaled(m_origImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        QPixmap procPixmap = QtCvUtils::matToQPixmap(result.processedImage).scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        QString resultText = QString("检测到 %1 个目标:\n").arg(result.circles.size());
        for (size_t i = 0; i < result.circles.size(); ++i) {
            cv::Vec3f c = result.circles[i];
            resultText += QString("目标 %1: 中心(%2, %3), 半径: %4\n").arg(i+1).arg(int(c[0])).arg(int(c[1])).arg(int(c[2]));
        }
        QMetaObject::invokeMethod(this, [=]() {
            m_origImageLabel->setPixmap(origPixmap);
            m_imageLabel->setPixmap(procPixmap);
            m_resultText->setText(resultText);
            if (m_isRecording) {
                updateChartTracks();
                updateAdvice();
            }
        }, Qt::QueuedConnection);
    });
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
        m_analysisModule.clear();
    } else {
        m_startRecordButton->setText("开始记录");
        emit logMessage("停止记录圆心坐标");
    }
    updateChartTracks();
    updateAdvice();
}

void CircleDetectionWidget::clearRecording()
{
    m_analysisModule.clear();
    m_suggestionText->clear();
    m_resultText->clear();
    m_origImageLabel->clear();
    m_imageLabel->clear();
    updateChartTracks();
    emit logMessage("所有记录、图表、建议和显示内容已清除");
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

    // 判断是否为静态图像（静态图像时才允许平移/缩放）
    if (!m_originalFrame.empty() && m_isDetectionActive && m_isStaticImageSource) {
        // 对原始静态图像做仿射变换
        cv::Mat modifiedFrame;
        cv::Point2f center(m_originalFrame.cols / 2.0f, m_originalFrame.rows / 2.0f);
        cv::Mat rotMatrix = cv::getRotationMatrix2D(center, 0, scale);
        rotMatrix.at<double>(0, 2) += x;
        rotMatrix.at<double>(1, 2) += y;
        cv::warpAffine(m_originalFrame, modifiedFrame, rotMatrix, m_originalFrame.size());
        processFrame(modifiedFrame); // 送入变换后的帧
    }
}

void CircleDetectionWidget::onParamsChanged()
{
    updateProcessorParams();
    
    if (m_isDetectionActive) {
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
        processorAlgorithm = ::DetectionAlgorithm::GeometricCenter;
    }
    
    m_processor.setAlgorithm(processorAlgorithm);
    m_paramsStack->setCurrentIndex(index);
    m_currentAlgorithm = algorithm;
    emit logMessage(QString("切换检测算法为: %1").arg(m_algorithmSelector->currentText()));
    
    // 强制重新检测
    if (m_isDetectionActive) {
    }
}

void CircleDetectionWidget::updateProcessorParams()
{
    DetectionParams params;
    
    // Hough算法参数
    params.dp = m_dpSlider->value() / 10.0;
    params.minDist = m_minDistSlider->value();
    params.cannyThresh = m_cannyThreshSlider->value();
    params.centerThresh = m_centerThreshSlider->value();
    params.minRadius = m_minRadiusSlider->value();
    params.maxRadius = m_maxRadiusSlider->value();
    params.useBinaryPreprocessing = m_useBinaryCheckBox->isChecked();
    params.binaryThresh = m_binaryThreshSlider->value();
    
    // 几何中心算法参数
    params.geometricBinaryThresh = m_geometricBinaryThreshSlider->value();
    params.inverseGeometric = m_inverseGeometricCheckBox->isChecked();
    
    m_processor.setParams(params);
}

void CircleDetectionWidget::updateAnalysisAndSuggestions()
{
    QString suggestion = m_analysisModule.getSuggestion();
    m_suggestionText->setText(suggestion);
}



void CircleDetectionWidget::updateAdvice()
{
    if (!m_suggestionText) return;
    m_suggestionText->setHtml(m_analysisModule.getMoveAdvice());
}

void CircleDetectionWidget::updateChartTracks()
{
    auto chartX = m_plotViewX->chart();
    auto chartY = m_plotViewY->chart();
    chartX->removeAllSeries();
    chartY->removeAllSeries();
    // 批量添加点，性能优化
    const auto& points = m_analysisModule.getPoints();
    if (!points.empty()) {
        QScatterSeries* seriesX = new QScatterSeries();
        QScatterSeries* seriesY = new QScatterSeries();
        seriesX->setName("x-r");
        seriesY->setName("y-r");
        seriesX->setMarkerSize(6.0);
        seriesY->setMarkerSize(6.0);
        QVector<QPointF> batchX, batchY;
        batchX.reserve(points.size());
        batchY.reserve(points.size());
        for (const auto& [x, y, r] : points) {
            batchX.append(QPointF(r, x));
            batchY.append(QPointF(r, y));
        }
        seriesX->append(batchX);
        seriesY->append(batchY);
        chartX->addSeries(seriesX);
        chartY->addSeries(seriesY);
        seriesX->attachAxis(m_axisX_R);
        seriesX->attachAxis(m_axisX_X);
        seriesY->attachAxis(m_axisY_R);
        seriesY->attachAxis(m_axisY_Y);
    }
    // 拟合直线
    auto fitX = m_analysisModule.fitXR();
    auto fitY = m_analysisModule.fitYR();
    if (fitX) {
        QLineSeries* line = new QLineSeries();
        double rMin = m_axisX_R->min();
        double rMax = m_axisX_R->max();
        line->append(rMin, fitX->first * rMin + fitX->second);
        line->append(rMax, fitX->first * rMax + fitX->second);
        line->setName("x-r拟合");
        chartX->addSeries(line);
        line->attachAxis(m_axisX_R);
        line->attachAxis(m_axisX_X);
    }
    if (fitY) {
        QLineSeries* line = new QLineSeries();
        double rMin = m_axisY_R->min();
        double rMax = m_axisY_R->max();
        line->append(rMin, fitY->first * rMin + fitY->second);
        line->append(rMax, fitY->first * rMax + fitY->second);
        line->setName("y-r拟合");
        chartY->addSeries(line);
        line->attachAxis(m_axisY_R);
        line->attachAxis(m_axisY_Y);
    }
} 