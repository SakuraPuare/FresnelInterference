#ifndef CIRCLE_DETECTION_WIDGET_H
#define CIRCLE_DETECTION_WIDGET_H

// --- Begin Qt Charts ---
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QScatterSeries>
// --- End Qt Charts ---

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <opencv2/opencv.hpp>
#include <QList>
#include <QPointF>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QSlider;
class QTextEdit;
class QGroupBox;
class QChartView;
class QScatterSeries;
QT_END_NAMESPACE

struct CircleData {
    double x;
    double y;
    double radius;
};

class CircleDetectionWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CircleDetectionWidget(QWidget *parent = nullptr);

public slots:
    void processFrame(const cv::Mat& frame);

signals:
    void logMessage(const QString& message);

private slots:
    void startDetection();
    void stopDetection();
    void onParamsChanged();
    void startRecording();
    void clearRecording();
    void updateStaticImageOffset(int);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void setupUI();
    void updatePlotsAxes();
    QPixmap matToQPixmap(const cv::Mat& mat);
    void applyOffsetAndDetect();
    cv::Mat performDetection(const cv::Mat& frame);

    // UI
    QLabel* m_imageLabel;
    QTextEdit* m_resultText;
    QPushButton* m_startButton;
    QPushButton* m_stopButton;
    QGroupBox* m_paramsGroup;
    QSlider* m_dpSlider;
    QSlider* m_minDistSlider;
    QSlider* m_cannyThreshSlider;
    QSlider* m_centerThreshSlider;
    QSlider* m_minRadiusSlider;
    QSlider* m_maxRadiusSlider;

    // --- New UI elements for recording and plotting ---
    QGroupBox* m_recordGroup;
    QPushButton* m_startRecordButton;
    QPushButton* m_clearRecordButton;
    QChartView* m_plotViewX;
    QChartView* m_plotViewY;

    QGroupBox* m_testGroup;
    QSlider* m_offsetXSlider;
    QSlider* m_offsetYSlider;
    QLabel* m_offsetLabel;
    // --- End New UI ---

    // State
    bool m_isDetectionActive;
    QPixmap m_currentPixmap;
    cv::Mat m_originalFrame;

    // --- New state variables ---
    bool m_isRecording;
    QPoint m_staticImageOffset;
    QScatterSeries* m_seriesXR;
    QScatterSeries* m_seriesYR;
    QList<CircleData> m_recordedData;
};

#endif // CIRCLE_DETECTION_WIDGET_H 