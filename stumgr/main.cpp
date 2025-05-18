#include "mainwindow.h"
#include <page_login.h>
#include <QApplication>
#include "stusql.h"
int main(int argc, char *argv[])
{
        QApplication a(argc, argv);
        stuSql* sql = stuSql::getinstance();  // 通过单例接口获取实例
        if (!sql || !sql->init()) {          // 检查实例是否创建成功并初始化
            qCritical() << "stuSql初始化失败，程序退出";
            return -1;
        }

    MainWindow w;
//    w.show();
    return a.exec();
}
