#ifndef CIRCLE_DETECTION_WIDGET_H
#define CIRCLE_DETECTION_WIDGET_H

// 标准 C++ 头文件
#include <memory>

// Qt 核心头文件
#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <QList>
#include <QPointF>
#include <QColor>

// Qt Charts 头文件
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>

// 第三方库头文件
#include <opencv2/opencv.hpp>

// 本项目头文件
#include "AnalysisModule.h"
#include "CircleDetectionProcessor.h"
#include "utils/QtCvUtils.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QSlider;
class QTextEdit;
class QGroupBox;
class QChartView;
class QScatterSeries;
class QLineSeries;
class QValueAxis;
class QCheckBox;
class QComboBox;
class QStackedWidget;
class QHBoxLayout;
class QVBoxLayout;
QT_END_NAMESPACE

/**
 * @brief 圆检测界面组件
 * 
 * 提供圆检测功能的完整用户界面，包括算法选择、参数调整、
 * 实时检测、数据记录和分析建议等功能。
 */
class CircleDetectionWidget : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief 支持的检测算法类型
     */
    enum class DetectionAlgorithm {
        Hough,          ///< Hough 圆检测算法
        GeometricCenter ///< 几何中心检测算法
    };
    Q_ENUM(DetectionAlgorithm)

    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit CircleDetectionWidget(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数
     */
    ~CircleDetectionWidget() override = default;

public slots:
    /**
     * @brief 处理输入的图像帧
     * @param frame 输入的 OpenCV 图像
     */
    void processFrame(const cv::Mat& frame);

signals:
    /**
     * @brief 发送日志消息信号
     * @param message 日志消息内容
     */
    void logMessage(const QString& message);

private slots:
    void startDetection();
    void stopDetection();
    void startRecording();
    void clearRecording();
    void onParamsChanged();
    void onAlgorithmChanged(int index);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    // UI 初始化方法
    void setupUI();
    void initializeAlgorithm();
    void setupDisplayLayout(QHBoxLayout* mainLayout);
    void setupImageDisplayArea(QVBoxLayout* displayLayout);
    void setupResultTextArea(QVBoxLayout* displayLayout);
    void setupChartArea(QVBoxLayout* displayLayout);
    void setupXChart(QHBoxLayout* plotsLayout);
    void setupYChart(QHBoxLayout* plotsLayout);
    void setupControlLayout(QHBoxLayout* mainLayout);
    void setupDetectionControls(QVBoxLayout* controlLayout);
    void setupAlgorithmSelection(QVBoxLayout* controlLayout);
    void setupRecordingControls(QVBoxLayout* controlLayout);
    void setupAnalysisSuggestions(QVBoxLayout* controlLayout);
    void setupParameterWidgets(QVBoxLayout* controlLayout);
    void makeConnections();

    // 辅助方法
    void updateAnalysisAndSuggestions();
    void matchAndTrackCircles(const std::vector<cv::Vec3f>& newCircles);
    void updateProcessorParams();
    void updateTable();
    void updateAdvice();
    void updateChartTracks();

    // --- UI Elements ---
    QLabel* m_imageLabel;       // Processed image display
    QLabel* m_origImageLabel;   // Original image display
    QTextEdit* m_resultText;
    QTextEdit* m_suggestionText;
    QChartView *m_plotViewX, *m_plotViewY;

    // --- Controls ---
    QPushButton *m_startButton, *m_stopButton;
    QPushButton *m_startRecordButton, *m_clearRecordButton;
    QGroupBox *m_recordGroup, *m_analysisGroup;
    
    // --- Algorithm Selection ---
    QComboBox* m_algorithmSelector;
    QStackedWidget* m_paramsStack;
    QWidget* m_houghParamsWidget;
    QWidget* m_geometricParamsWidget;
    
    // --- Hough Algorithm Parameters ---
    QGroupBox* m_houghParamsGroup;
    QSlider *m_dpSlider, *m_minDistSlider, *m_cannyThreshSlider, *m_centerThreshSlider, *m_minRadiusSlider,
            *m_maxRadiusSlider;
    QLabel *m_dpValueLabel, *m_minDistValueLabel, *m_cannyThreshValueLabel, *m_centerThreshValueLabel,
           *m_minRadiusValueLabel, *m_maxRadiusValueLabel;
    QCheckBox* m_useBinaryCheckBox;
    QSlider* m_binaryThreshSlider;
    QLabel* m_binaryThreshValueLabel;
    QWidget* m_binaryControlsWidget;
    
    // --- Geometric Center Algorithm Parameters ---
    QGroupBox* m_geometricParamsGroup;
    QSlider* m_geometricBinaryThreshSlider;
    QLabel* m_geometricBinaryThreshValueLabel;
    QCheckBox* m_inverseGeometricCheckBox;

    // --- Plotting Axes ---
    QValueAxis *m_axisX_R, *m_axisX_X, *m_axisY_R, *m_axisY_Y;

    // --- 状态变量 ---
    bool m_isDetectionActive;
    bool m_isRecording;
    cv::Mat m_originalFrame;
    QPixmap m_currentPixmap;
    DetectionAlgorithm m_currentAlgorithm;
    int m_frameCounter;
    
    // --- 核心模块 ---
    CircleDetectionProcessor m_processor;
    AnalysisModule m_analysisModule;
};

#endif // CIRCLEDETECTIONWIDGET_H 