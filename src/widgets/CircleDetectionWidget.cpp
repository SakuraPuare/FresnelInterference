#include "CircleDetectionWidget.h"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QResizeEvent>
#include <QSlider>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QtCharts>
#include <algorithm>
#include <opencv2/imgproc.hpp>

// CircleDetectionWidget：光路准直模块实现
CircleDetectionWidget::CircleDetectionWidget(QWidget *parent)
    : QWidget(parent), m_originalLabel(nullptr), m_processedLabel(nullptr),
      m_freezeButton(nullptr), m_algorithmCombo(nullptr),
      m_thresholdSlider(nullptr), m_resultText(nullptr),
      m_timer(new QTimer(this)), m_isFrozen(false), m_recordButton(nullptr),
      m_recordEnabled(false) {
  setupUI();

  m_timer->setSingleShot(true);
  m_timer->setInterval(300); // 300ms 防抖，减轻CPU负载
  connect(m_timer, &QTimer::timeout, this,
          &CircleDetectionWidget::handleTimeout);

  emit logMessage("光路准直模块已加载");
}

void CircleDetectionWidget::setupUI() {
  QHBoxLayout *mainLayout = new QHBoxLayout(this);

  // 左侧：原始与处理后图像
  QVBoxLayout *leftLayout = new QVBoxLayout();

  // 图像左右排版
  QHBoxLayout *imageRow = new QHBoxLayout();

  QGroupBox *origBox = new QGroupBox("原始/冻结图像");
  QVBoxLayout *origLayout = new QVBoxLayout(origBox);
  m_originalLabel = new QLabel("等待图像...");
  m_originalLabel->setMinimumSize(400, 300);
  m_originalLabel->setAlignment(Qt::AlignCenter);
  m_originalLabel->setStyleSheet(
      "QLabel { background-color: #2b2b2b; color: white; }");
  origLayout->addWidget(m_originalLabel);

  QGroupBox *procBox = new QGroupBox("检测结果");
  QVBoxLayout *procLayout = new QVBoxLayout(procBox);
  m_processedLabel = new QLabel("等待图像...");
  m_processedLabel->setMinimumSize(400, 300);
  m_processedLabel->setAlignment(Qt::AlignCenter);
  m_processedLabel->setStyleSheet(
      "QLabel { background-color: #2b2b2b; color: white; }");
  procLayout->addWidget(m_processedLabel);

  imageRow->addWidget(origBox);
  imageRow->addWidget(procBox);

  leftLayout->addLayout(imageRow);

  m_resultText = new QTextEdit();
  m_resultText->setMaximumHeight(120);
  m_resultText->setReadOnly(true);
  m_resultText->setStyleSheet(
      "QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: "
      "'Courier New', monospace; }");
  leftLayout->addWidget(m_resultText);

  mainLayout->addLayout(leftLayout, 3);

  // 右侧：控制
  QVBoxLayout *rightLayout = new QVBoxLayout();

  QGroupBox *controlBox = new QGroupBox("控制");
  QVBoxLayout *controlLayout = new QVBoxLayout(controlBox);

  m_freezeButton = new QPushButton("冻结帧并调参");
  m_freezeButton->setCheckable(true);
  controlLayout->addWidget(m_freezeButton);

  connect(m_freezeButton, &QPushButton::toggled, this,
          &CircleDetectionWidget::toggleFreezeMode);

  // 参数
  QGroupBox *paramBox = new QGroupBox("检测参数");
  QFormLayout *paramLayout = new QFormLayout(paramBox);

  m_algorithmCombo = new QComboBox();
  m_algorithmCombo->addItem(
      "Hough 圆检测",
      static_cast<int>(CircleDetectionProcessor::Algorithm::HoughTransform));
  m_algorithmCombo->addItem(
      "几何中心",
      static_cast<int>(CircleDetectionProcessor::Algorithm::GeometricCenter));
  paramLayout->addRow("算法:", m_algorithmCombo);

  m_thresholdSlider = new QSlider(Qt::Horizontal);
  m_thresholdSlider->setRange(0, 255);
  m_thresholdSlider->setValue(120);
  paramLayout->addRow("二值阈值:", m_thresholdSlider);

  connect(m_algorithmCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &CircleDetectionWidget::onParamsChanged);
  connect(m_thresholdSlider, &QSlider::valueChanged, this,
          &CircleDetectionWidget::onParamsChanged);

  rightLayout->addWidget(controlBox);
  rightLayout->addWidget(paramBox);

  // 记录按钮
  m_recordButton = new QPushButton("开始记录");
  m_recordButton->setCheckable(true);
  connect(m_recordButton, &QPushButton::toggled, this,
          &CircleDetectionWidget::onRecordToggled);
  rightLayout->addWidget(m_recordButton);

  rightLayout->addStretch();

  // 图表
  setupCharts();
  QGroupBox *chartBox = new QGroupBox("X-R / Y-R 曲线");
  QVBoxLayout *chartLayout = new QVBoxLayout(chartBox);
  chartLayout->addWidget(m_xrChartView);
  chartLayout->addWidget(m_yrChartView);
  mainLayout->addWidget(chartBox, 2);

  mainLayout->addLayout(rightLayout, 1);
}

void CircleDetectionWidget::processFrame(const cv::Mat &frame) {
  if (frame.empty())
    return;

  if (m_isFrozen)
    return; // 冻结时忽略实时帧

  m_originalFrame = frame.clone();

  // 实时更新预览（不做复杂分析）
  updatePreviewImage();
}

void CircleDetectionWidget::toggleFreezeMode(bool checked) {
  m_isFrozen = checked;
  m_freezeButton->setText(checked ? "返回实时预览" : "冻结帧并调参");

  if (checked) {
    if (m_originalFrame.empty()) {
      emit logMessage("错误：没有图像可以冻结");
      m_freezeButton->setChecked(false);
      m_isFrozen = false;
      return;
    }
    m_frozenFrame = m_originalFrame.clone();
    performAnalysis();
  } else {
    m_resultText->clear();
    updatePreviewImage();
  }
}

void CircleDetectionWidget::onParamsChanged() {
  // 更新算法与阈值
  int algoVal = m_algorithmCombo->currentData().toInt();
  m_processor.setAlgorithm(
      static_cast<CircleDetectionProcessor::Algorithm>(algoVal));
  m_processor.setThreshold(m_thresholdSlider->value());

  // 防抖处理
  m_timer->start();
}

void CircleDetectionWidget::handleTimeout() {
  if (m_isFrozen) {
    performAnalysis();
  } else {
    updatePreviewImage();
  }
}

void CircleDetectionWidget::updatePreviewImage() {
  if (m_originalFrame.empty())
    return;
  QPixmap pixmap = QtCvUtils::matToQPixmap(m_originalFrame);
  m_originalLabel->setPixmap(pixmap.scaled(
      m_originalLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void CircleDetectionWidget::performAnalysis() {
  if (m_frozenFrame.empty())
    return;

  // 检测圆
  auto result = m_processor.process(m_frozenFrame);

  // 绘制结果图像（保持 BGR，QtCvUtils 内部会转换）
  cv::Mat disp = m_frozenFrame.clone();
  if (result.valid) {
    cv::circle(disp, result.center, static_cast<int>(result.radius),
               {0, 255, 0}, 2);
    cv::circle(disp, result.center, 2, {255, 0, 0}, 2);
  }

  QPixmap processedPixmap = QtCvUtils::matToQPixmap(disp);
  m_currentPixmap = processedPixmap;
  m_processedLabel->setPixmap(processedPixmap.scaled(
      m_processedLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

  displayResult(result);
}

void CircleDetectionWidget::displayResult(
    const CircleDetectionProcessor::Result &result) {
  if (!result.valid) {
    m_resultText->setText("未检测到圆形光源");
    return;
  }

  if (m_recordEnabled) {
    m_analysis.addPoint(result.center.x, result.center.y, result.radius);
    // 更新图表
    updateCharts();
  }

  QString text;
  text += QString("圆心: ( %1 , %2 )\n")
              .arg(result.center.x, 0, 'f', 1)
              .arg(result.center.y, 0, 'f', 1);
  text += QString("半径: %1 px\n").arg(result.radius, 0, 'f', 1);

  // 拟合信息
  auto fitX = m_analysis.fitXR();
  auto fitY = m_analysis.fitYR();
  if (fitX) {
    text += QString("X-R 斜率: %1\n").arg(fitX->first, 0, 'f', 4);
  }
  if (fitY) {
    text += QString("Y-R 斜率: %1\n").arg(fitY->first, 0, 'f', 4);
  }

  text += "----------------------------\n";
  text += m_analysis.getSuggestion();

  m_resultText->setText(text);
}

void CircleDetectionWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  if (!m_originalFrame.empty()) {
    QPixmap pixmap = QtCvUtils::matToQPixmap(m_originalFrame);
    m_originalLabel->setPixmap(pixmap.scaled(m_originalLabel->size(),
                                             Qt::KeepAspectRatio,
                                             Qt::SmoothTransformation));
  }
  if (!m_currentPixmap.isNull()) {
    m_processedLabel->setPixmap(
        m_currentPixmap.scaled(m_processedLabel->size(), Qt::KeepAspectRatio,
                               Qt::SmoothTransformation));
  }
}

void CircleDetectionWidget::setupCharts() {
  // XR chart
  m_xrChart = new QChart();
  m_xrScatter = new QScatterSeries();
  m_xrScatter->setName("X vs R");
  m_xrScatter->setMarkerSize(6);
  m_xrFitLine = new QLineSeries();
  m_xrFitLine->setName("X-R Fit");
  m_xrChart->addSeries(m_xrScatter);
  m_xrChart->addSeries(m_xrFitLine);
  m_xrChart->createDefaultAxes();
  m_xrChart->setTitle("X vs Radius");
  m_xrChartView = new QChartView(m_xrChart);
  m_xrChartView->setMinimumHeight(200);

  // YR chart
  m_yrChart = new QChart();
  m_yrScatter = new QScatterSeries();
  m_yrScatter->setName("Y vs R");
  m_yrScatter->setMarkerSize(6);
  m_yrFitLine = new QLineSeries();
  m_yrFitLine->setName("Y-R Fit");
  m_yrChart->addSeries(m_yrScatter);
  m_yrChart->addSeries(m_yrFitLine);
  m_yrChart->createDefaultAxes();
  m_yrChart->setTitle("Y vs Radius");
  m_yrChartView = new QChartView(m_yrChart);
  m_yrChartView->setMinimumHeight(200);
}

void CircleDetectionWidget::updateCharts() {
  // Append latest point
  const auto &pts = m_analysis.getPoints();
  if (pts.empty())
    return;
  auto [x, y, r] = pts.back();
  m_xrScatter->append(r, x);
  m_yrScatter->append(r, y);

  // Update fit lines
  auto fitX = m_analysis.fitXR();
  if (fitX) {
    m_xrFitLine->clear();
    double k = fitX->first;
    double b = fitX->second;
    // Use min and max r in current points
    double rMin = r, rMax = r;
    for (const auto &[xx, yy, rr] : pts) {
      rMin = std::min(rMin, rr);
      rMax = std::max(rMax, rr);
    }
    m_xrFitLine->append(rMin, k * rMin + b);
    m_xrFitLine->append(rMax, k * rMax + b);
  }

  auto fitY = m_analysis.fitYR();
  if (fitY) {
    m_yrFitLine->clear();
    double k = fitY->first;
    double b = fitY->second;
    double rMin = r, rMax = r;
    for (const auto &[xx, yy, rr] : pts) {
      rMin = std::min(rMin, rr);
      rMax = std::max(rMax, rr);
    }
    m_yrFitLine->append(rMin, k * rMin + b);
    m_yrFitLine->append(rMax, k * rMax + b);
  }
}

void CircleDetectionWidget::onRecordToggled(bool checked) {
  m_recordEnabled = checked;
  m_recordButton->setText(checked ? "停止记录" : "开始记录");
  if (!checked) {
    // Clear data
    m_analysis.clear();
    m_xrScatter->clear();
    m_yrScatter->clear();
    m_xrFitLine->clear();
    m_yrFitLine->clear();
  }
}
