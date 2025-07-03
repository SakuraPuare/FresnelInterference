#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTabWidget>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QProgressBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <QStyleFactory>

#include <opencv2/opencv.hpp>
#include <memory>

#include "io/ImageInput.h"

QT_BEGIN_NAMESPACE
class QTabWidget;
class QLabel;
class QProgressBar;
QT_END_NAMESPACE

class InputControlWidget;
class CircleDetectionWidget;
class FringeAnalysisWidget;
class ImageSpacingWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void pixelSizeChanged(double pixelSize_um);

private slots:
    void openPixelSizeDialog();

private:
    void setupUI();
    void setupMenuBar();
    void setupStatusBar();
    void logMessage(const QString& message);

private:
    // UI组件
    QTabWidget* m_tabWidget;
    
    // 状态栏组件
    QLabel* m_statusLabel;
    QLabel* m_fpsLabel;
    QProgressBar* m_progressBar;
    
    // 菜单和动作
    QAction* m_exitAction;
    QAction* m_aboutAction;
    QAction* m_resetAction;
    QAction* m_pixelSizeAction;

    // Tab Widgets
    InputControlWidget* m_inputControlWidget;
    CircleDetectionWidget* m_circleDetectionWidget;
    FringeAnalysisWidget* m_fringeAnalysisWidget;
    ImageSpacingWidget* m_imageSpacingWidget;

    // Global Settings
    double m_pixelSize_um;
};

#endif // MAINWINDOW_H