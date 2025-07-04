#ifndef CAMERACONTROLWIDGET_H
#define CAMERACONTROLWIDGET_H

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

class CameraControlWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraControlWidget(QWidget *parent = nullptr);
    ~CameraControlWidget();

signals:
    void frameRateChanged(int value);
    void exposureChanged(int value);
    void gainChanged(int value);
    void gammaChanged(double value);
    void contrastChanged(int value);
    void sharpnessChanged(int value);
    void saturationChanged(int value);

public slots:
    void setFrameRate(int value);
    void setExposure(int value);
    void setGain(int value);
    void setGamma(double value);
    void setContrast(int value);
    void setSharpness(int value);
    void setSaturation(int value);

private:
    void setupUi();

    // UI Elements
    QSlider* m_frameRateSlider;
    QLabel* m_frameRateValueLabel;

    QSlider* m_exposureSlider;
    QLabel* m_exposureValueLabel;

    QSlider* m_gainSlider;
    QLabel* m_gainValueLabel;
    
    QSlider* m_gammaSlider;
    QLabel* m_gammaValueLabel;

    QSlider* m_contrastSlider;
    QLabel* m_contrastValueLabel;

    QSlider* m_sharpnessSlider;
    QLabel* m_sharpnessValueLabel;
    
    QSlider* m_saturationSlider;
    QLabel* m_saturationValueLabel;
};

#endif // CAMERACONTROLWIDGET_H 