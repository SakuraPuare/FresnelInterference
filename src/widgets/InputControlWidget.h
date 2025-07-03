#ifndef INPUT_CONTROL_WIDGET_H
#define INPUT_CONTROL_WIDGET_H

#include <QWidget>
#include <QTimer>
#include <memory>
#include <QPixmap>
#include <QResizeEvent>

#include "io/ImageInput.h"
#include <opencv2/opencv.hpp>
#include "utils/QtCvUtils.h"

QT_BEGIN_NAMESPACE
class QGroupBox;
class QLabel;
class QPushButton;
class QComboBox;
class QSlider;
class QSpinBox;
QT_END_NAMESPACE

class InputControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit InputControlWidget(QWidget *parent = nullptr);
    ~InputControlWidget();
    
signals:
    void frameReady(const cv::Mat& frame);
    void logMessage(const QString& message);
    void updateFps(double fps);

private slots:
    void connectDevice();
    void disconnectDevice();
    void captureImage();
    void saveImage();

    void onCameraTypeChanged(int index);
    void onExposureChanged(int value);
    void onGainChanged(int value);
    void onGammaChanged(int value);

    void updateFrame();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void setupUI();
    
    // UI
    QLabel* m_imageLabel;
    QComboBox* m_cameraTypeCombo;
    QPushButton* m_connectButton;
    QPushButton* m_disconnectButton;
    QPushButton* m_captureButton;
    QPushButton* m_saveButton;

    // Exposure Controls
    QGroupBox* m_exposureControlGroup;
    QSlider* m_exposureSlider;
    QSlider* m_gainSlider;
    QSlider* m_gammaSlider;
    QSpinBox* m_exposureSpinBox;
    QSpinBox* m_gainSpinBox;
    QSpinBox* m_gammaSpinBox;

    // Backend
    std::unique_ptr<ImageInput> m_camera;
    QTimer* m_frameTimer;
    cv::Mat m_currentFrame;
    QPixmap m_currentPixmap;

    // FPS calculation
    int m_frameCount;
    qint64 m_lastFrameTime;
};

#endif // INPUT_CONTROL_WIDGET_H 