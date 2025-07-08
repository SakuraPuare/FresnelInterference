#include "MainWindow.h"
#include "widgets/InputControlWidget.h"
#include "widgets/CircleDetectionWidget.h"
#include "widgets/FringeAnalysisWidget.h"
#include "widgets/ImageSpacingWidget.h"
#include "widgets/CameraControlWidget.h"

#include <QApplication>
#include <QTime>
#include <QDebug>
#include <QMessageBox>
#include <QInputDialog>

// 头文件包含顺序：标准库、第三方库、本项目头文件（如有需要可调整）

// MainWindow：主窗口实现
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_tabWidget(nullptr)
    , m_pixelSize_um(2.2) // Default pixel size in micrometers
{
    setupUI();
    setupMenuBar();
    setupStatusBar();
    
    logMessage("应用程序启动完成");

    // Connect log messages from various widgets to the main window's logger
    connect(m_inputControlWidget, &InputControlWidget::logMessage, this, &MainWindow::logMessage);
    connect(m_circleDetectionWidget, &CircleDetectionWidget::logMessage, this, &MainWindow::logMessage);

    // Connect global parameter changes
    connect(this, &MainWindow::pixelSizeChanged, m_fringeAnalysisWidget, &FringeAnalysisWidget::setPixelSize);
    connect(this, &MainWindow::pixelSizeChanged, m_imageSpacingWidget, &ImageSpacingWidget::setPixelSize);

    connect(m_inputControlWidget, &InputControlWidget::updateFps, m_fpsLabel, [this](double fps){
        m_fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    });

    // Connect the frame output from input widget to the analysis widgets
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_circleDetectionWidget, &CircleDetectionWidget::processFrame);
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_fringeAnalysisWidget, &FringeAnalysisWidget::processFrame);
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_imageSpacingWidget, &ImageSpacingWidget::processFrame);

    // Connect camera control signals
    connect(m_cameraControlWidget, &CameraControlWidget::frameRateChanged, m_inputControlWidget, &InputControlWidget::setFrameRate);
    connect(m_cameraControlWidget, &CameraControlWidget::exposureChanged, m_inputControlWidget, &InputControlWidget::setExposure);
    connect(m_cameraControlWidget, &CameraControlWidget::gainChanged, m_inputControlWidget, &InputControlWidget::setGain);
    connect(m_cameraControlWidget, &CameraControlWidget::gammaChanged, m_inputControlWidget, &InputControlWidget::setGamma);
    connect(m_cameraControlWidget, &CameraControlWidget::contrastChanged, m_inputControlWidget, &InputControlWidget::setContrast);
    connect(m_cameraControlWidget, &CameraControlWidget::sharpnessChanged, m_inputControlWidget, &InputControlWidget::setSharpness);
    connect(m_cameraControlWidget, &CameraControlWidget::saturationChanged, m_inputControlWidget, &InputControlWidget::setSaturation);

    // Update camera control UI when input source changes
    connect(m_inputControlWidget, &InputControlWidget::inputSourceChanged, this, &MainWindow::updateCameraParameters);

    // Initialize widgets with the default pixel size
    emit pixelSizeChanged(m_pixelSize_um);
}

MainWindow::~MainWindow()
{
    // Child widgets are automatically deleted by Qt's parent-child system
}

void MainWindow::setupUI()
{
    setWindowTitle("菲涅尔双棱镜干涉实验系统 v2.0");
    setMinimumSize(1200, 800);
    
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
    
    m_tabWidget = new QTabWidget(this);
    mainLayout->addWidget(m_tabWidget);
    
    // Create and add tabs
    m_inputControlWidget = new InputControlWidget();
    m_tabWidget->addTab(m_inputControlWidget, "输入控制");

    m_circleDetectionWidget = new CircleDetectionWidget();
    m_tabWidget->addTab(m_circleDetectionWidget, "光路准直");

    m_fringeAnalysisWidget = new FringeAnalysisWidget();
    m_tabWidget->addTab(m_fringeAnalysisWidget, "干涉条纹测量");

    m_imageSpacingWidget = new ImageSpacingWidget();
    m_tabWidget->addTab(m_imageSpacingWidget, "大小像间距测量");

    m_cameraControlWidget = new CameraControlWidget();
}

void MainWindow::setupMenuBar()
{
    QMenuBar* menuBar = this->menuBar();
    
    QMenu* fileMenu = menuBar->addMenu("文件");
    m_exitAction = new QAction("退出", this);
    m_exitAction->setShortcut(QKeySequence::Quit);
    connect(m_exitAction, &QAction::triggered, this, &QWidget::close);
    fileMenu->addAction(m_exitAction);
    
    QMenu* settingsMenu = menuBar->addMenu("设置");
    m_resetAction = new QAction("重置设置", this);
    // connect(m_resetAction, &QAction::triggered, this, &MainWindow::resetSettings); // TODO: Re-implement reset
    settingsMenu->addAction(m_resetAction);
    
    m_pixelSizeAction = new QAction("像素尺寸设置...", this);
    connect(m_pixelSizeAction, &QAction::triggered, this, &MainWindow::openPixelSizeDialog);
    settingsMenu->addAction(m_pixelSizeAction);
    
    m_openCameraControlAct = new QAction("相机参数设置...", this);
    m_openCameraControlAct->setEnabled(false); // Initially disabled
    connect(m_openCameraControlAct, &QAction::triggered, this, &MainWindow::openCameraControlWidget);
    settingsMenu->addAction(m_openCameraControlAct);
    
    QMenu* helpMenu = menuBar->addMenu("帮助");
    m_aboutAction = new QAction("关于", this);
    connect(m_aboutAction, &QAction::triggered, this, [this](){
        QMessageBox::about(this, "关于", 
            "菲涅尔双棱镜干涉实验系统\n"
            "版本: 2.0.0\n"
            "基于Qt6和OpenCV开发\n"
            "支持光路准直、干涉条纹测量和大小像间距测量");
    });
    helpMenu->addAction(m_aboutAction);
}

void MainWindow::setupStatusBar()
{
    m_statusLabel = new QLabel("就绪");
    m_fpsLabel = new QLabel("FPS: 0");
    m_progressBar = new QProgressBar();
    m_progressBar->setVisible(false);
    
    statusBar()->addWidget(m_statusLabel);
    statusBar()->addPermanentWidget(m_fpsLabel);
    statusBar()->addPermanentWidget(m_progressBar);
}

void MainWindow::logMessage(const QString& message)
{
    QString timestamp = QTime::currentTime().toString("hh:mm:ss");
    qDebug() << QString("[%1] %2").arg(timestamp, message);
    if (m_statusLabel) {
        m_statusLabel->setText(message);
    }
}

void MainWindow::openPixelSizeDialog()
{
    bool ok;
    double newPixelSize = QInputDialog::getDouble(this, "像素尺寸设置", 
        "请输入相机单个像素的尺寸 (μm):", m_pixelSize_um, 0.1, 100.0, 3, &ok);
    
    if (ok) {
        m_pixelSize_um = newPixelSize;
        logMessage(QString("全局像素尺寸已更新为 %1 μm").arg(m_pixelSize_um));
        emit pixelSizeChanged(m_pixelSize_um);
    }
}

void MainWindow::openCameraControlWidget()
{
    if (m_cameraControlWidget) {
        updateCameraParameters();
        m_cameraControlWidget->show();
        m_cameraControlWidget->raise();
        m_cameraControlWidget->activateWindow();
    }
}

void MainWindow::updateCameraParameters()
{
    auto cameraInput = m_inputControlWidget->getCameraInput();
    if (!cameraInput) {
        m_openCameraControlAct->setEnabled(false);
        return;
    }

    m_openCameraControlAct->setEnabled(true);

    // Update the control widget with current camera settings
    m_cameraControlWidget->setFrameRate(cameraInput->getFrameRate());
    m_cameraControlWidget->setExposure(cameraInput->getExposureTime());
    m_cameraControlWidget->setGain(cameraInput->getGain());
    m_cameraControlWidget->setGamma(cameraInput->getGamma());
    m_cameraControlWidget->setContrast(cameraInput->getContrast());
    m_cameraControlWidget->setSharpness(cameraInput->getSharpness());
    m_cameraControlWidget->setSaturation(cameraInput->getSaturation());
}

#include "moc_MainWindow.cpp" 