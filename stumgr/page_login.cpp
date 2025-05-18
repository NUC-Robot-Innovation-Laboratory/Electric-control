#include "page_login.h"
#include "ui_page_login.h"

Page_Login::Page_Login(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Page_Login)
{
    ui->setupUi(this);
}

Page_Login::~Page_Login()
{
    delete ui;
}

void Page_Login::on_btn_login_clicked()
{
    //数据库查找用户名和密码
    //失败就提示
    //成功进入主界面
    emit sendLoginSuccess();

}


void Page_Login::on_btn_exit_clicked()
{
    //退出
    exit(0);
}

