#include "FringeAnalysisWidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTextEdit>
#include <QComboBox>
#include <QCheckBox>

#include <opencv2/imgproc.hpp>

FringeAnalysisWidget::FringeAnalysisWidget(QWidget *parent)
    : QWidget(parent)
    , m_isAnalysisActive(false)
{
    setupUI();
}

void FringeAnalysisWidget::setupUI()
{
    QHBoxLayout* layout = new QHBoxLayout(this);
    
    // Image Display
    QGroupBox* imageGroup = new QGroupBox("条纹分析结果");
    QVBoxLayout* imageLayout = new QVBoxLayout(imageGroup);
    
    m_imageLabel = new QLabel("等待输入图像...");
    m_imageLabel->setMinimumSize(640, 480);
    m_imageLabel->setFrameStyle(QFrame::Box);
    m_imageLabel->setAlignment(Qt::AlignCenter);
    m_imageLabel->setScaledContents(false);
    m_imageLabel->setStyleSheet("QLabel { background-color: #2b2b2b; color: white; }");
    imageLayout->addWidget(m_imageLabel);
    
    m_resultText = new QTextEdit();
    m_resultText->setMaximumHeight(100);
    m_resultText->setReadOnly(true);
    m_resultText->setStyleSheet("QTextEdit { background-color: #1e1e1e; color: #00ff00; }");
    imageLayout->addWidget(m_resultText);
    
    layout->addWidget(imageGroup);
    
    // Control Panel
    QVBoxLayout* controlLayout = new QVBoxLayout();
    
    // Control Buttons
    QGroupBox* controlGroup = new QGroupBox("条纹分析控制");
    QVBoxLayout* controlGroupLayout = new QVBoxLayout(controlGroup);
    
    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton("开始分析");
    m_stopButton = new QPushButton("停止分析");
    m_stopButton->setEnabled(false);
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(m_stopButton);
    controlGroupLayout->addLayout(buttonLayout);
    
    controlLayout->addWidget(controlGroup);
    
    // Parameters
    m_paramsGroup = new QGroupBox("分析参数");
    QFormLayout* paramsLayout = new QFormLayout(m_paramsGroup);
    
    m_methodCombo = new QComboBox();
    m_methodCombo->addItems({"FFT频域分析", "梯度检测", "峰值检测"});
    paramsLayout->addRow("分析方法:", m_methodCombo);
    
    m_threshSlider = new QSlider(Qt::Horizontal);
    m_threshSlider->setRange(1, 100);
    m_threshSlider->setValue(20);
    paramsLayout->addRow("检测阈值:", m_threshSlider);
    
    m_fftCheck = new QCheckBox("启用FFT频谱分析");
    m_fftCheck->setChecked(true);
    paramsLayout->addRow(m_fftCheck);
    
    controlLayout->addWidget(m_paramsGroup);
    controlLayout->addStretch();
    
    layout->addLayout(controlLayout);
    
    // Connections
    connect(m_startButton, &QPushButton::clicked, this, &FringeAnalysisWidget::startAnalysis);
    connect(m_stopButton, &QPushButton::clicked, this, &FringeAnalysisWidget::stopAnalysis);
    connect(m_methodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &FringeAnalysisWidget::onParamsChanged);
    connect(m_threshSlider, &QSlider::valueChanged, this, &FringeAnalysisWidget::onParamsChanged);
    connect(m_fftCheck, &QCheckBox::toggled, this, &FringeAnalysisWidget::onParamsChanged);
}

void FringeAnalysisWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!m_currentPixmap.isNull()) {
        m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void FringeAnalysisWidget::processFrame(const cv::Mat& frame)
{
    if (m_isAnalysisActive && !frame.empty()) {
        cv::Mat result = performAnalysis(frame);
        m_currentPixmap = matToQPixmap(result);
        if (!m_currentPixmap.isNull()) {
            m_imageLabel->setPixmap(m_currentPixmap.scaled(m_imageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        }
    }
}

void FringeAnalysisWidget::startAnalysis()
{
    m_isAnalysisActive = true;
    m_startButton->setEnabled(false);
    m_stopButton->setEnabled(true);
    emit logMessage("开始条纹分析");
}

void FringeAnalysisWidget::stopAnalysis()
{
    m_isAnalysisActive = false;
    m_startButton->setEnabled(true);
    m_stopButton->setEnabled(false);
    m_imageLabel->setText("条纹分析已停止");
    m_currentPixmap = QPixmap();
    emit logMessage("停止条纹分析");
}

void FringeAnalysisWidget::onParamsChanged()
{
    // Can be used to trigger re-processing on param change if needed
}

cv::Mat FringeAnalysisWidget::performAnalysis(const cv::Mat& frame)
{
    cv::Mat gray, result = frame.clone();
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    QString method = m_methodCombo->currentText();
    QString resultText = QString("分析方法: %1\n").arg(method);
    
    if (method == "FFT频域分析" && m_fftCheck->isChecked()) {
        cv::Mat floatGray;
        gray.convertTo(floatGray, CV_32F);
        
        cv::Mat fftResult;
        cv::dft(floatGray, fftResult, cv::DFT_COMPLEX_OUTPUT);
        
        std::vector<cv::Mat> planes;
        cv::split(fftResult, planes);
        cv::Mat magnitude;
        cv::magnitude(planes[0], planes[1], magnitude);
        
        magnitude += cv::Scalar::all(1);
        cv::log(magnitude, magnitude);
        
        cv::normalize(magnitude, magnitude, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::cvtColor(magnitude, result, cv::COLOR_GRAY2BGR);
        
        resultText += "FFT频域分析完成\n";

    } else if (method == "梯度检测") {
        cv::Mat gradX, gradY, grad;
        cv::Sobel(gray, gradX, CV_16S, 1, 0, 3);
        cv::Sobel(gray, gradY, CV_16S, 0, 1, 3);
        cv::convertScaleAbs(gradX, gradX);
        cv::convertScaleAbs(gradY, gradY);
        cv::addWeighted(gradX, 0.5, gradY, 0.5, 0, grad);
        
        cv::cvtColor(grad, result, cv::COLOR_GRAY2BGR);
        resultText += "梯度检测完成\n";

    } else if (method == "峰值检测") {
        cv::Mat blurred;
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
        
        int threshold = m_threshSlider->value();
        for (int y = 1; y < blurred.rows - 1; y++) {
            for (int x = 1; x < blurred.cols - 1; x++) {
                uchar center = blurred.at<uchar>(y, x);
                bool isLocalMax = true;
                if (center <= threshold) continue;

                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dy == 0 && dx == 0) continue;
                        if (blurred.at<uchar>(y + dy, x + dx) >= center) {
                            isLocalMax = false;
                            break;
                        }
                    }
                    if (!isLocalMax) break;
                }
                
                if (isLocalMax) {
                    cv::circle(result, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        resultText += "峰值检测完成\n";
    }
    
    m_resultText->setText(resultText);
    return result;
}

QPixmap FringeAnalysisWidget::matToQPixmap(const cv::Mat& mat)
{
    if (mat.empty()) {
        return QPixmap();
    }
    
    cv::Mat rgbMat;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
    } else if (mat.channels() == 1) {
        cv::cvtColor(mat, rgbMat, cv::COLOR_GRAY2RGB);
    } else {
        rgbMat = mat;
    }
    
    QImage qimg(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);
    return QPixmap::fromImage(qimg);
} 