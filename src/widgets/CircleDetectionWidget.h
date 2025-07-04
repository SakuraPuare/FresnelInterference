#pragma once

#include <QWidget>
#include <QPixmap>
#include <QTimer>
#include "CircleDetectionProcessor.h"
#include "AnalysisModule.h"
#include "utils/QtCvUtils.h"
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QChartGlobal>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QComboBox;
class QSlider;
class QTextEdit;
QT_END_NAMESPACE

// CircleDetectionWidget：光路准直模块主界面
class CircleDetectionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CircleDetectionWidget(QWidget* parent = nullptr);

signals:
    void logMessage(const QString& message);

public slots:
    // 接收输入图像帧
    void processFrame(const cv::Mat& frame);

private slots:
    void toggleFreezeMode(bool checked);
    void onParamsChanged();
    void handleTimeout();
    void onRecordToggled(bool checked);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void setupUI();
    void updatePreviewImage();
    void performAnalysis();
    void displayResult(const CircleDetectionProcessor::Result& result);
    void updateCharts();
    void setupCharts();

    // UI 元件
    QLabel* m_originalLabel;
    QLabel* m_processedLabel;
    QPushButton* m_freezeButton;
    QPushButton* m_recordButton;
    QComboBox* m_algorithmCombo;
    QSlider* m_thresholdSlider;
    QTextEdit* m_resultText;

    // 定时器用于防抖更新
    QTimer* m_timer;

    // 数据
    cv::Mat m_originalFrame;   // 实时帧
    cv::Mat m_frozenFrame;     // 冻结帧
    QPixmap m_currentPixmap;
    bool m_isFrozen;
    bool m_recordEnabled;

    CircleDetectionProcessor m_processor;
    AnalysisModule m_analysis;

    // Charts
    QChartView* m_xrChartView;
    QChartView* m_yrChartView;
    QChart* m_xrChart;
    QChart* m_yrChart;
    QScatterSeries* m_xrScatter;
    QScatterSeries* m_yrScatter;
    QLineSeries* m_xrFitLine;
    QLineSeries* m_yrFitLine;
};
