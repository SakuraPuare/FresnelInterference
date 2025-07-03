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

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

#include <opencv2/imgproc.hpp>

CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent)
    , m_isDetectionActive(false)
    , m_isRecording(false)
    , m_staticImageOffset(0,0)
{
    setupUI();
}

void CircleDetectionWidget::setupUI()
{
    QHBoxLayout* mainLayout = new QHBoxLayout(this);
    
    // Left side: Image and Plot
    QVBoxLayout* displayLayout = new QVBoxLayout();
    
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
    displayLayout->addWidget(imageGroup);

    // Plotting Area
    QHBoxLayout* plotsLayout = new QHBoxLayout();
    
    // Plot for X vs Radius
    QChart *chartX = new QChart();
    chartX->setTitle("圆心X坐标 vs. 半径");
    m_seriesXR = new QScatterSeries();
    m_seriesXR->setName("X vs R");
    m_seriesXR->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    m_seriesXR->setMarkerSize(8.0);
    chartX->addSeries(m_seriesXR);
    chartX->createDefaultAxes();
    qobject_cast<QValueAxis*>(chartX->axes(Qt::Horizontal).first())->setTitleText("半径 (像素)");
    qobject_cast<QValueAxis*>(chartX->axes(Qt::Vertical).first())->setTitleText("X坐标 (像素)");
    m_plotViewX = new QChartView(chartX);
    m_plotViewX->setRenderHint(QPainter::Antialiasing);
    plotsLayout->addWidget(m_plotViewX);

    // Plot for Y vs Radius
    QChart *chartY = new QChart();
    chartY->setTitle("圆心Y坐标 vs. 半径");
    m_seriesYR = new QScatterSeries();
    m_seriesYR->setName("Y vs R");
    m_seriesYR->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    m_seriesYR->setMarkerSize(8.0);
    chartY->addSeries(m_seriesYR);
    chartY->createDefaultAxes();
    qobject_cast<QValueAxis*>(chartY->axes(Qt::Horizontal).first())->setTitleText("半径 (像素)");
    qobject_cast<QValueAxis*>(chartY->axes(Qt::Vertical).first())->setTitleText("Y坐标 (像素)");
    m_plotViewY = new QChartView(chartY);
    m_plotViewY->setRenderHint(QPainter::Antialiasing);
    plotsLayout->addWidget(m_plotViewY);
    
    // Add plots to the main display layout
    displayLayout->addLayout(plotsLayout);
    displayLayout->setStretchFactor(imageGroup, 2);
    displayLayout->setStretchFactor(plotsLayout, 1);

    mainLayout->addLayout(displayLayout, 2); // Assign more stretch factor
    
    // Right side: Control Panel
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
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

    // Record controls
    m_recordGroup = new QGroupBox("记录控制");
    QVBoxLayout* recordLayout = new QVBoxLayout(m_recordGroup);
    m_startRecordButton = new QPushButton("开始记录");
    m_clearRecordButton = new QPushButton("清除记录");
    recordLayout->addWidget(m_startRecordButton);
    recordLayout->addWidget(m_clearRecordButton);
    controlLayout->addWidget(m_recordGroup);

    // Parameters
    m_paramsGroup = new QGroupBox("HoughCircles参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    m_dpSlider = new QSlider(Qt::Horizontal);
    m_dpSlider->setRange(10, 50);
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

    // Test Controls
    m_testGroup = new QGroupBox("静态图像测试");
    QFormLayout* testLayout = new QFormLayout(m_testGroup);
    m_offsetXSlider = new QSlider(Qt::Horizontal);
    m_offsetXSlider->setRange(-100, 100);
    m_offsetXSlider->setValue(0);
    testLayout->addRow("X 偏移:", m_offsetXSlider);
    m_offsetYSlider = new QSlider(Qt::Horizontal);
    m_offsetYSlider->setRange(-100, 100);
    m_offsetYSlider->setValue(0);
    testLayout->addRow("Y 偏移:", m_offsetYSlider);
    m_offsetLabel = new QLabel("偏移: (0, 0)");
    testLayout->addWidget(m_offsetLabel);
    controlLayout->addWidget(m_testGroup);
    
    controlLayout->addStretch();
    mainLayout->addLayout(controlLayout, 1);
    
    // Connections
    connect(m_startButton, &QPushButton::clicked, this, &CircleDetectionWidget::startDetection);
    connect(m_stopButton, &QPushButton::clicked, this, &CircleDetectionWidget::stopDetection);
    connect(m_dpSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minDistSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_cannyThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_centerThreshSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_minRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);
    connect(m_maxRadiusSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::onParamsChanged);

    connect(m_startRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::startRecording);
    connect(m_clearRecordButton, &QPushButton::clicked, this, &CircleDetectionWidget::clearRecording);
    connect(m_offsetXSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::updateStaticImageOffset);
    connect(m_offsetYSlider, &QSlider::valueChanged, this, &CircleDetectionWidget::updateStaticImageOffset);
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

    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::applyOffsetAndDetect()
{
    if (m_originalFrame.empty()) return;

    cv::Mat modifiedFrame;

    if (!m_staticImageOffset.isNull()) {
        cv::Mat translationMat = (cv::Mat_<double>(2, 3) << 1, 0, m_staticImageOffset.x(), 0, 1, m_staticImageOffset.y());
        cv::warpAffine(m_originalFrame, modifiedFrame, translationMat, m_originalFrame.size());
    } else {
        m_originalFrame.copyTo(modifiedFrame);
    }

    cv::Mat result = performDetection(modifiedFrame);
    m_currentPixmap = matToQPixmap(result);
    if (!m_currentPixmap.isNull()) {
        m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
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
    }
}

void CircleDetectionWidget::clearRecording()
{
    m_seriesXR->clear();
    m_seriesYR->clear();
    m_recordedData.clear();
    
    auto reset_axis = [this](QChartView* view, int w, int h) {
        if (!view || !view->chart()) return;
        view->chart()->axes(Qt::Horizontal).first()->setRange(0, 100); // Default range
        view->chart()->axes(Qt::Vertical).first()->setRange(0, m_originalFrame.rows > 0 ? h : 480);
    };

    reset_axis(m_plotViewX, m_originalFrame.cols, m_originalFrame.cols);
    reset_axis(m_plotViewY, m_originalFrame.cols, m_originalFrame.rows);

    emit logMessage("圆心记录已清除");
}

void CircleDetectionWidget::updateStaticImageOffset(int)
{
    int x = m_offsetXSlider->value();
    int y = m_offsetYSlider->value();
    m_staticImageOffset.setX(x);
    m_staticImageOffset.setY(y);
    m_offsetLabel->setText(QString("偏移: (%1, %2)").arg(x).arg(y));
    
    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
}

void CircleDetectionWidget::onParamsChanged()
{
    if (m_isDetectionActive) {
        applyOffsetAndDetect();
    }
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
    
    if (!circles.empty()) {
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Vec3f c = circles[i];
            cv::Point center(cvRound(c[0]), cvRound(c[1]));
            int radius = cvRound(c[2]);
            
            cv::circle(result, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            cv::circle(result, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
            
            resultText += QString("圆 %1: 中心(%2, %3), 半径: %4\n")
                        .arg(i+1).arg(center.x).arg(center.y).arg(radius);
            
            if (m_isRecording) {
                CircleData dataPoint = { (double)center.x, (double)center.y, (double)radius };
                m_recordedData.append(dataPoint);
                
                m_seriesXR->append(dataPoint.radius, dataPoint.x);
                m_seriesYR->append(dataPoint.radius, dataPoint.y);
                updatePlotsAxes();
            }
        }
    }
    
    m_resultText->setText(resultText);
    
    return result;
}

void CircleDetectionWidget::updatePlotsAxes()
{
    if (m_recordedData.isEmpty()) return;

    double minX = m_recordedData.first().x;
    double maxX = m_recordedData.first().x;
    double minY = m_recordedData.first().y;
    double maxY = m_recordedData.first().y;
    double minR = m_recordedData.first().radius;
    double maxR = m_recordedData.first().radius;

    for (const auto& data : m_recordedData) {
        if (data.x < minX) minX = data.x;
        if (data.x > maxX) maxX = data.x;
        if (data.y < minY) minY = data.y;
        if (data.y > maxY) maxY = data.y;
        if (data.radius < minR) minR = data.radius;
        if (data.radius > maxR) maxR = data.radius;
    }

    double xMargin = (maxX - minX) * 0.1 + 10;
    double yMargin = (maxY - minY) * 0.1 + 10;
    double rMargin = (maxR - minR) * 0.1 + 5;

    m_plotViewX->chart()->axes(Qt::Horizontal).first()->setRange(minR - rMargin, maxR + rMargin);
    m_plotViewX->chart()->axes(Qt::Vertical).first()->setRange(minX - xMargin, maxX + xMargin);
    
    m_plotViewY->chart()->axes(Qt::Horizontal).first()->setRange(minR - rMargin, maxR + rMargin);
    m_plotViewY->chart()->axes(Qt::Vertical).first()->setRange(minY - yMargin, maxY + yMargin);
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