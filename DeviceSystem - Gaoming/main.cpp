//本项目采用 Qt 标准图形应用架构。main.cpp 作为程序入口
//只负责初始化 Qt 应用环境、创建主控窗口 DeviceSystem、显示窗口并启动事件循环。
//具体的业务逻辑，如界面管理、设备初始化、图像处理、雷达数据处理与检测流程控制
//都封装在 DeviceSystem 及其子模块中。
#include "DeviceSystem.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    DeviceSystem window;   //创建界面程序
    window.show();
    return app.exec();
}
