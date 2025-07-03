#ifndef FRINGE_ANALYSIS_WIDGET_H
#define FRINGE_ANALYSIS_WIDGET_H

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
class QComboBox;
class QCheckBox;
QT_END_NAMESPACE

class FringeAnalysisWidget : public QWidget
{
    Q_OBJECT

public:
    explicit FringeAnalysisWidget(QWidget *parent = nullptr);

public slots:
    void processFrame(const cv::Mat& frame);

signals:
    void logMessage(const QString& message);

private slots:
    void startAnalysis();
    void stopAnalysis();
    void onParamsChanged();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void setupUI();
    QPixmap matToQPixmap(const cv::Mat& mat);
    cv::Mat performAnalysis(const cv::Mat& frame);

    // UI
    QLabel* m_imageLabel;
    QTextEdit* m_resultText;
    QPushButton* m_startButton;
    QPushButton* m_stopButton;
    QGroupBox* m_paramsGroup;
    QComboBox* m_methodCombo;
    QSlider* m_threshSlider;
    QCheckBox* m_fftCheck;

    // State
    bool m_isAnalysisActive;
    QPixmap m_currentPixmap;
};

#endif // FRINGE_ANALYSIS_WIDGET_H 