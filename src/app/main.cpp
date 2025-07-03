#include <QApplication>
#include <QStyleFactory>
#include <QDir>
#include "MainWindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    // 设置应用程序信息
    app.setApplicationName("菲涅尔双棱镜干涉实验");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("Physics Lab");
    
    // 设置现代化界面风格
    app.setStyle(QStyleFactory::create("Fusion"));
    
    MainWindow window;
    window.show();
    
    return app.exec();
} 