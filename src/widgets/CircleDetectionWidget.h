#ifndef CIRCLE_DETECTION_WIDGET_H
#define CIRCLE_DETECTION_WIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QSlider;
class QTextEdit;
class QGroupBox;
QT_END_NAMESPACE

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

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void setupUI();
    QPixmap matToQPixmap(const cv::Mat& mat);
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

    // State
    bool m_isDetectionActive;
    QPixmap m_currentPixmap;
};

#endif // CIRCLE_DETECTION_WIDGET_H 