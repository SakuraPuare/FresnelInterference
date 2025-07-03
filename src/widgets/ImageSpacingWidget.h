#ifndef IMAGE_SPACING_WIDGET_H
#define IMAGE_SPACING_WIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <opencv2/opencv.hpp>
#include "utils/QtCvUtils.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QTextEdit;
class QGroupBox;
class QSlider;
class QCheckBox;
class QTimer;
QT_END_NAMESPACE

class ImageSpacingWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageSpacingWidget(QWidget *parent = nullptr);
    ~ImageSpacingWidget();

public slots:
    void processFrame(const cv::Mat& frame);
    void setPixelSize(double pixelSize_um);

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void toggleFreezeMode(bool checked);
    void scheduleReanalysis();
    void performMeasurement();
    void updatePreviewImage();

private:
    void setupUI();
    void drawResult(cv::Mat &displayImage, const std::vector<int> &peak_indices, double angle_deg);
    void updateFrame(const cv::Mat &frame);

    // UI
    QLabel* m_originalImageLabel;
    QLabel* m_processedImageLabel;
    QTextEdit* m_resultText;
    QPushButton* m_toggleButton;
    QGroupBox* m_paramsGroup;
    QGroupBox* m_preprocessGroup;
    QLabel* m_pixelSizeLabel;
    QSlider* m_brightnessSlider;
    QSlider* m_contrastSlider;
    QSlider* m_gammaSlider;
    QSlider* m_peakThreshSlider;
    QSlider* m_minPeakDistSlider;
    QCheckBox* m_detectValleysCheck;

    // State
    bool m_isFrozenForAnalysis;
    QPixmap m_currentPixmap;
    double m_pixelSize_um;
    
    cv::Mat m_originalFrame;
    cv::Mat m_frozenFrame;
    cv::Mat m_currentFrame;
    QTimer* m_previewUpdateTimer;
};

#endif // IMAGE_SPACING_WIDGET_H