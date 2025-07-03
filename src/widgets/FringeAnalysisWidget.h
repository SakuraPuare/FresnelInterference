#ifndef FRINGE_ANALYSIS_WIDGET_H
#define FRINGE_ANALYSIS_WIDGET_H

#include <QWidget>
#include <QPixmap>
#include <QResizeEvent>
#include <opencv2/opencv.hpp>
#include <vector>

// Forward declarations
QT_BEGIN_NAMESPACE
class QLabel;
class QPushButton;
class QSlider;
class QTextEdit;
class QGroupBox;
class QCheckBox;
class QTimer;
QT_END_NAMESPACE

class FringeAnalysisWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FringeAnalysisWidget(QWidget *parent = nullptr);
    ~FringeAnalysisWidget();

public slots:
    void processFrame(const cv::Mat& frame);
    void setPixelSize(double pixelSize_um);

private slots:
    void toggleFreezeMode(bool checked);
    void scheduleReanalysis();
    void performAnalysis();
    void updatePreviewImage();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    // Private methods
    void setupUI();
    void updateFrame(const cv::Mat &frame);
    QPixmap matToQPixmap(const cv::Mat& mat);
    void applyPreprocessing(const cv::Mat &src, cv::Mat &dst);
    void drawResult(cv::Mat &displayImage, double angle,
                    const std::vector<int> &peak_indices,
                    const cv::Mat &projection);

    // UI Elements
    QLabel* m_originalImageLabel;
    QLabel* m_processedImageLabel;
    QTextEdit* m_resultText;
    QPushButton* m_startButton;
    QPushButton* m_stopButton;
    QGroupBox* m_paramsGroup;
    QSlider* m_peakThreshSlider;
    QSlider* m_minPeakDistSlider;
    QCheckBox* m_detectValleysCheck;
    QLabel* m_pixelSizeLabel;
    QGroupBox* m_preprocessGroup;
    QSlider* m_brightnessSlider;
    QSlider* m_contrastSlider;
    QSlider* m_gammaSlider;
    
    // State
    bool m_isFrozenForAnalysis;
    cv::Mat m_currentFrame;
    cv::Mat m_originalFrame;
    cv::Mat m_frozenFrame;
    QPixmap m_currentPixmap;
    double m_pixelSize_um;
    QTimer* m_previewUpdateTimer;
};

#endif // FRINGE_ANALYSIS_WIDGET_H 