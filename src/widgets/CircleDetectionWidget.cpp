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

// 头文件包含顺序：标准库、第三方库、本项目头文件（如有需要可调整）

// CircleDetectionWidget：圆检测主界面
CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent)
    , m_isDetectionActive(false)
    , m_isRecording(false)
    , m_currentAlgorithm(DetectionAlgorithm::Hough)
    , m_frameCounter(0)
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
    setupDisplayLayout(mainLayout);
    setupControlLayout(mainLayout);
    makeConnections();
}

void CircleDetectionWidget::setupDisplayLayout(QHBoxLayout* mainLayout) {
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

    // Plotting Area
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
}

void CircleDetectionWidget::setupControlLayout(QHBoxLayout* mainLayout) {
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
    
    // --- Algorithm Parameters ---
    setupParameterWidgets(controlLayout);

    // --- Record Controls ---
    m_recordGroup = new QGroupBox("记录控制");
    QVBoxLayout* recordLayout = new QVBoxLayout(m_recordGroup);
    m_startRecordButton = new QPushButton("开始记录");
    m_clearRecordButton = new QPushButton("清除记录");
    recordLayout->addWidget(m_startRecordButton);
    recordLayout->addWidget(m_clearRecordButton);
    controlLayout->addWidget(m_recordGroup);

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
}

void CircleDetectionWidget::setupParameterWidgets(QVBoxLayout* controlLayout) {
    m_paramsStack = new QStackedWidget();
    
    // Hough Parameters
    m_houghParamsWidget = new QWidget();
    m_houghParamsGroup = new QGroupBox("Hough算法参数");
    QFormLayout* houghParamsLayout = new QFormLayout(m_houghParamsGroup);
    
    // Helper lambda to create a slider with a label
    auto createSliderWithLabel = [&](QSlider* &slider, QLabel* &label, const QString& name, int min, int max, int val, double factor = 1.0, int precision = 0) {
        QHBoxLayout* hLayout = new QHBoxLayout();
        slider = new QSlider(Qt::Horizontal);
        slider->setRange(min, max);
        slider->setValue(val);

        QString labelText = (factor == 1.0) ? QString::number(val) : QString::number(val * factor, 'f', precision);
        label = new QLabel(labelText);
        label->setMinimumWidth(40); // For alignment

        hLayout->addWidget(slider);
        hLayout->addWidget(label);
        houghParamsLayout->addRow(name, hLayout);

        connect(slider, &QSlider::valueChanged, this, [=](int newValue) {
            QString newLabelText = (factor == 1.0) ? QString::number(newValue) : QString::number(newValue * factor, 'f', precision);
            label->setText(newLabelText);
        });
    };

    createSliderWithLabel(m_dpSlider, m_dpValueLabel, "DP (x0.1):", 10, 50, 10, 0.1, 1);
    createSliderWithLabel(m_minDistSlider, m_minDistValueLabel, "最小距离:", 10, 200, 50);
    createSliderWithLabel(m_cannyThreshSlider, m_cannyThreshValueLabel, "Canny阈值:", 10, 300, 100);
    createSliderWithLabel(m_centerThreshSlider, m_centerThreshValueLabel, "中心阈值:", 10, 100, 30);
    createSliderWithLabel(m_minRadiusSlider, m_minRadiusValueLabel, "最小半径:", 1, 100, 10);
    createSliderWithLabel(m_maxRadiusSlider, m_maxRadiusValueLabel, "最大半径:", 10, 300, 100);
    
    m_useBinaryCheckBox = new QCheckBox("使用二值化预处理");
    m_useBinaryCheckBox->setChecked(false);
    houghParamsLayout->addRow(m_useBinaryCheckBox);
    
    QHBoxLayout* binaryLayout = new QHBoxLayout();
    m_binaryThreshSlider = new QSlider(Qt::Horizontal);
    m_binaryThreshSlider->setRange(0, 255); m_binaryThreshSlider->setValue(128);
    m_binaryThreshValueLabel = new QLabel(QString::number(m_binaryThreshSlider->value()));
    m_binaryThreshValueLabel->setMinimumWidth(40);
    binaryLayout->addWidget(m_binaryThreshSlider);
    binaryLayout->addWidget(m_binaryThreshValueLabel);
    houghParamsLayout->addRow("二值化阈值:", binaryLayout);
    
    QWidget* binaryWidget = binaryLayout->parentWidget();
    binaryWidget->setEnabled(false);

    connect(m_binaryThreshSlider, &QSlider::valueChanged, m_binaryThreshValueLabel, qOverload<int>(&QLabel::setNum));
    
    QVBoxLayout* houghWidgetLayout = new QVBoxLayout(m_houghParamsWidget);
    houghWidgetLayout->addWidget(m_houghParamsGroup);
    houghWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_houghParamsWidget);

    // Geometric Center Parameters
    m_geometricParamsWidget = new QWidget();
    m_geometricParamsGroup = new QGroupBox("几何中心算法参数");
    QFormLayout* geometricParamsLayout = new QFormLayout(m_geometricParamsGroup);
    
    QHBoxLayout* geometricBinaryLayout = new QHBoxLayout();
    m_geometricBinaryThreshSlider = new QSlider(Qt::Horizontal);
    m_geometricBinaryThreshSlider->setRange(0, 255); m_geometricBinaryThreshSlider->setValue(128);
    m_geometricBinaryThreshValueLabel = new QLabel(QString::number(m_geometricBinaryThreshSlider->value()));
    m_geometricBinaryThreshValueLabel->setMinimumWidth(40);
    geometricBinaryLayout->addWidget(m_geometricBinaryThreshSlider);
    geometricBinaryLayout->addWidget(m_geometricBinaryThreshValueLabel);
    geometricParamsLayout->addRow("二值化阈值:", geometricBinaryLayout);

    connect(m_geometricBinaryThreshSlider, &QSlider::valueChanged, m_geometricBinaryThreshValueLabel, qOverload<int>(&QLabel::setNum));

    m_inverseGeometricCheckBox = new QCheckBox("反向二值化");
    m_inverseGeometricCheckBox->setChecked(false);
    geometricParamsLayout->addRow(m_inverseGeometricCheckBox);
    
    QVBoxLayout* geometricWidgetLayout = new QVBoxLayout(m_geometricParamsWidget);
    geometricWidgetLayout->addWidget(m_geometricParamsGroup);
    geometricWidgetLayout->setContentsMargins(0,0,0,0);
    m_paramsStack->addWidget(m_geometricParamsWidget);

    controlLayout->addWidget(m_paramsStack);
}

void CircleDetectionWidget::makeConnections() {
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
    connect(m_binaryThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);

    // Also connect the checkbox to enable/disable the binary threshold slider's container
    QWidget* binaryWidget = m_binaryThreshSlider->parentWidget();
    connect(m_useBinaryCheckBox, &QCheckBox::toggled, binaryWidget, &QWidget::setEnabled);
    
    // Geometric center param controls
    connect(m_geometricBinaryThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_inverseGeometricCheckBox, &QCheckBox::toggled, this, &CircleDetectionWidget::onParamsChanged);

    connect(m_startRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::startRecording);
    connect(m_clearRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::clearRecording);
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
    if (!m_isDetectionActive) return;
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
    m_isRecording = false;
    if (m_startRecordButton) m_startRecordButton->setText("开始记录");
    m_analysisModule.clear();
    
    // 清除处理器缓存，防止旧的检测结果被重新添加
    m_processor.clearCache();

    m_suggestionText->clear();
    m_resultText->clear();
    m_origImageLabel->clear();
    m_imageLabel->clear();

    // Directly clear charts and reset axes
    m_plotViewX->chart()->removeAllSeries();
    m_plotViewY->chart()->removeAllSeries();
    m_axisX_R->setRange(0, 100);
    m_axisX_X->setRange(0, m_originalFrame.cols > 0 ? m_originalFrame.cols : 640);
    m_axisY_R->setRange(0, 100);
    m_axisY_Y->setRange(0, m_originalFrame.rows > 0 ? m_originalFrame.rows : 480);

    emit logMessage("所有记录、图表、建议和显示内容已清除");
}

void CircleDetectionWidget::onParamsChanged()
{
    updateProcessorParams();
    
    if (m_isDetectionActive && !m_originalFrame.empty()) {
        processFrame(m_originalFrame);
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
    if (m_isDetectionActive && !m_originalFrame.empty()) {
        processFrame(m_originalFrame);
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

    const auto& points = m_analysisModule.getPoints();

    if (points.empty()) {
        // Reset axes to default when there's no data
        m_axisX_R->setRange(0, 100);
        m_axisX_X->setRange(0, m_originalFrame.cols > 0 ? m_originalFrame.cols : 640);
        m_axisY_R->setRange(0, 100);
        m_axisY_Y->setRange(0, m_originalFrame.rows > 0 ? m_originalFrame.rows : 480);
        return;
    }

    // 批量添加点，性能优化
    if (!points.empty()) {
        QScatterSeries* seriesX = new QScatterSeries();
        QScatterSeries* seriesY = new QScatterSeries();
        seriesX->setName("x-r");
        seriesY->setName("y-r");
        seriesX->setMarkerSize(10.0);
        seriesY->setMarkerSize(10.0);
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

        // Update axes ranges based on data
        double minR = std::numeric_limits<double>::max();
        double maxR = std::numeric_limits<double>::min();
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::min();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::min();

        for (const auto& p : points) {
            minR = std::min(minR, std::get<2>(p));
            maxR = std::max(maxR, std::get<2>(p));
            minX = std::min(minX, std::get<0>(p));
            maxX = std::max(maxX, std::get<0>(p));
            minY = std::min(minY, std::get<1>(p));
            maxY = std::max(maxY, std::get<1>(p));
        }

        double rMargin = (maxR - minR) * 0.1;
        double xMargin = (maxX - minX) * 0.1;
        double yMargin = (maxY - minY) * 0.1;

        m_axisX_R->setRange(minR - rMargin, maxR + rMargin);
        m_axisX_X->setRange(minX - xMargin, maxX + xMargin);
        m_axisY_R->setRange(minR - rMargin, maxR + rMargin);
        m_axisY_Y->setRange(minY - yMargin, maxY + yMargin);
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