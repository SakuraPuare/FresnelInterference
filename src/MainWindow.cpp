#include "MainWindow.h"
#include "widgets/InputControlWidget.h"
#include "widgets/CircleDetectionWidget.h"
#include "widgets/FringeAnalysisWidget.h"
#include "widgets/ImageSpacingWidget.h"

#include <QApplication>
#include <QTime>
#include <QDebug>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_tabWidget(nullptr)
{
    setupUI();
    setupMenuBar();
    setupStatusBar();
    
    logMessage("应用程序启动完成");

    // Connect signals from widgets to the main window's status bar
    connect(m_inputControlWidget, &InputControlWidget::logMessage, this, &MainWindow::logMessage);
    connect(m_circleDetectionWidget, &CircleDetectionWidget::logMessage, this, &MainWindow::logMessage);
    connect(m_fringeAnalysisWidget, &FringeAnalysisWidget::logMessage, this, &MainWindow::logMessage);
    connect(m_imageSpacingWidget, &ImageSpacingWidget::logMessage, this, &MainWindow::logMessage);

    connect(m_inputControlWidget, &InputControlWidget::updateFps, m_fpsLabel, [this](double fps){
        m_fpsLabel->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    });

    // Connect the frame output from input widget to the analysis widgets
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_circleDetectionWidget, &CircleDetectionWidget::processFrame);
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_fringeAnalysisWidget, &FringeAnalysisWidget::processFrame);
    connect(m_inputControlWidget, &InputControlWidget::frameReady, m_imageSpacingWidget, &ImageSpacingWidget::processFrame);
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

#include "moc_MainWindow.cpp" 