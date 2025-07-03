#ifndef CIRCLE_DETECTION_WIDGET_H
#define CIRCLE_DETECTION_WIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include "AnalysisModule.h"
#include "utils/QtCvUtils.h"

// --- Begin Qt ---
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
#include <QList>
#include <QPointF>
#include <QColor>
// --- End Qt ---

#include <opencv2/opencv.hpp>
#include "CircleDetectionProcessor.h"

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
QT_END_NAMESPACE

class CircleDetectionWidget : public QWidget
{
    Q_OBJECT

public:
    enum class DetectionAlgorithm {
        Hough,
        GeometricCenter
    };
    Q_ENUM(DetectionAlgorithm)

    explicit CircleDetectionWidget(QWidget *parent = nullptr);
    ~CircleDetectionWidget() override = default;

    void setStaticImageSource(bool isStatic) { m_isStaticImageSource = isStatic; }

public slots:
    void processFrame(const cv::Mat& frame);

signals:
    void logMessage(const QString& message);

private slots:
    void startDetection();
    void stopDetection();
    void startRecording();
    void clearRecording();
    void updateStaticImageOffset(int);
    void onParamsChanged();
    void onAlgorithmChanged(int index);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    // Setup and Helper methods
    void setupUI();
    void updatePlotsAxes();
    void updateAnalysisAndSuggestions();
    void matchAndTrackCircles(const std::vector<cv::Vec3f>& newCircles);
    void applyOffsetAndDetect();
    
    // 更新参数到处理器
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
    QSlider *m_dpSlider, *m_minDistSlider, *m_cannyThreshSlider, *m_centerThreshSlider, *m_minRadiusSlider, *m_maxRadiusSlider;
    QCheckBox* m_useBinaryCheckBox;
    QSlider* m_binaryThreshSlider;
    
    // --- Geometric Center Algorithm Parameters ---
    QGroupBox* m_geometricParamsGroup;
    QSlider* m_geometricBinaryThreshSlider;
    QCheckBox* m_inverseGeometricCheckBox;

    // --- Test Controls ---
    QGroupBox* m_testGroup;
    QSlider *m_offsetXSlider, *m_offsetYSlider, *m_zoomSlider;
    QLabel *m_offsetLabel, *m_zoomLabel;

    // --- Plotting Axes ---
    QValueAxis *m_axisX_R, *m_axisX_X, *m_axisY_R, *m_axisY_Y;

    // --- State ---
    bool m_isDetectionActive;
    bool m_isRecording;
    cv::Mat m_originalFrame;
    QPixmap m_currentPixmap;
    QPoint m_staticImageOffset;
    DetectionAlgorithm m_currentAlgorithm;
    
    // 检测处理器模块
    CircleDetectionProcessor m_processor;
    int m_frameCounter;

    // --- Tracking ---
    // struct CircleData { double x, y, radius; };
    // struct TrackedObject { ... };
    // QMap<int, TrackedObject> m_trackedObjects;
    // int m_nextTrackId;
    // QList<QColor> m_colorPalette;
    AnalysisModule m_analysisModule;
    // 静态图像源标志
    bool m_isStaticImageSource;
};

#endif // CIRCLEDETECTIONWIDGET_H 