#include "CameraControlWidget.h"
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

CameraControlWidget::CameraControlWidget(QWidget *parent) :
    QWidget(parent)
{
    setupUi();
}

CameraControlWidget::~CameraControlWidget()
{
}

void CameraControlWidget::setupUi()
{
    auto mainLayout = new QVBoxLayout(this);
    auto settingsGroup = new QGroupBox("相机参数", this);
    auto formLayout = new QFormLayout(settingsGroup);

    // Frame Rate
    m_frameRateSlider = new QSlider(Qt::Horizontal);
    m_frameRateValueLabel = new QLabel("0");
    m_frameRateSlider->setRange(1, 100); // 假设范围
    formLayout->addRow("帧率:", m_frameRateSlider);
    formLayout->addRow("", m_frameRateValueLabel);
    connect(m_frameRateSlider, &QSlider::valueChanged, this, [this](int value){
        m_frameRateValueLabel->setText(QString::number(value));
        emit frameRateChanged(value);
    });

    // Exposure
    m_exposureSlider = new QSlider(Qt::Horizontal);
    m_exposureValueLabel = new QLabel("0");
    m_exposureSlider->setRange(100, 100000); // 假设范围 (us)
    formLayout->addRow("曝光 (us):", m_exposureSlider);
    formLayout->addRow("", m_exposureValueLabel);
    connect(m_exposureSlider, &QSlider::valueChanged, this, [this](int value){
        m_exposureValueLabel->setText(QString::number(value));
        emit exposureChanged(value);
    });

    // Gain
    m_gainSlider = new QSlider(Qt::Horizontal);
    m_gainValueLabel = new QLabel("0");
    m_gainSlider->setRange(0, 100); // 假设范围
    formLayout->addRow("增益:", m_gainSlider);
    formLayout->addRow("", m_gainValueLabel);
    connect(m_gainSlider, &QSlider::valueChanged, this, [this](int value){
        m_gainValueLabel->setText(QString::number(value));
        emit gainChanged(value);
    });

    // Gamma
    m_gammaSlider = new QSlider(Qt::Horizontal);
    m_gammaValueLabel = new QLabel("0.0");
    m_gammaSlider->setRange(1, 400); // 对应 0.01 - 4.0
    formLayout->addRow("伽马:", m_gammaSlider);
    formLayout->addRow("", m_gammaValueLabel);
    connect(m_gammaSlider, &QSlider::valueChanged, this, [this](int value){
        double gamma = value / 100.0;
        m_gammaValueLabel->setText(QString::number(gamma, 'f', 2));
        emit gammaChanged(gamma);
    });
    
    // Contrast
    m_contrastSlider = new QSlider(Qt::Horizontal);
    m_contrastValueLabel = new QLabel("0");
    m_contrastSlider->setRange(0, 200); // 假设范围
    formLayout->addRow("对比度:", m_contrastSlider);
    formLayout->addRow("", m_contrastValueLabel);
    connect(m_contrastSlider, &QSlider::valueChanged, this, [this](int value){
        m_contrastValueLabel->setText(QString::number(value));
        emit contrastChanged(value);
    });

    // Sharpness
    m_sharpnessSlider = new QSlider(Qt::Horizontal);
    m_sharpnessValueLabel = new QLabel("0");
    m_sharpnessSlider->setRange(0, 100); // 假设范围
    formLayout->addRow("锐度:", m_sharpnessSlider);
    formLayout->addRow("", m_sharpnessValueLabel);
    connect(m_sharpnessSlider, &QSlider::valueChanged, this, [this](int value){
        m_sharpnessValueLabel->setText(QString::number(value));
        emit sharpnessChanged(value);
    });

    // Saturation
    m_saturationSlider = new QSlider(Qt::Horizontal);
    m_saturationValueLabel = new QLabel("0");
    m_saturationSlider->setRange(0, 200); // 假设范围
    formLayout->addRow("饱和度:", m_saturationSlider);
    formLayout->addRow("", m_saturationValueLabel);
    connect(m_saturationSlider, &QSlider::valueChanged, this, [this](int value){
        m_saturationValueLabel->setText(QString::number(value));
        emit saturationChanged(value);
    });

    settingsGroup->setLayout(formLayout);
    mainLayout->addWidget(settingsGroup);
    setLayout(mainLayout);
}

void CameraControlWidget::setFrameRate(int value)
{
    m_frameRateSlider->setValue(value);
    m_frameRateValueLabel->setText(QString::number(value));
}

void CameraControlWidget::setExposure(int value)
{
    m_exposureSlider->setValue(value);
    m_exposureValueLabel->setText(QString::number(value));
}

void CameraControlWidget::setGain(int value)
{
    m_gainSlider->setValue(value);
    m_gainValueLabel->setText(QString::number(value));
}

void CameraControlWidget::setGamma(double value)
{
    m_gammaSlider->setValue(static_cast<int>(value * 100));
    m_gammaValueLabel->setText(QString::number(value, 'f', 2));
}

void CameraControlWidget::setContrast(int value)
{
    m_contrastSlider->setValue(value);
    m_contrastValueLabel->setText(QString::number(value));
}

void CameraControlWidget::setSharpness(int value)
{
    m_sharpnessSlider->setValue(value);
    m_sharpnessValueLabel->setText(QString::number(value));
}

void CameraControlWidget::setSaturation(int value)
{
    m_saturationSlider->setValue(value);
    m_saturationValueLabel->setText(QString::number(value));
} 