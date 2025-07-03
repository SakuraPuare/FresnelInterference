#ifndef IMAGE_SPACING_WIDGET_H
#define IMAGE_SPACING_WIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QTextEdit;
class QGroupBox;
class QDoubleSpinBox;
QT_END_NAMESPACE

class ImageSpacingWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageSpacingWidget(QWidget *parent = nullptr);

public slots:
    void processFrame(const cv::Mat& frame);

signals:
    void logMessage(const QString& message);

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void startMeasurement();
    void stopMeasurement();
    void onParamsChanged();

private:
    void setupUI();
    QPixmap matToQPixmap(const cv::Mat& mat);
    cv::Mat performMeasurement(const cv::Mat& frame);

    // UI
    QLabel* m_imageLabel;
    QTextEdit* m_resultText;
    QPushButton* m_startButton;
    QPushButton* m_stopButton;
    QGroupBox* m_paramsGroup;
    QDoubleSpinBox* m_pixelSizeSpin;

    // State
    bool m_isMeasurementActive;
    QPixmap m_currentPixmap;
};

#endif // IMAGE_SPACING_WIDGET_H 