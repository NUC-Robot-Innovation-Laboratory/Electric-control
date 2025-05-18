/********************************************************************************
** Form generated from reading UI file 'page_login.ui'
**
** Created by: Qt User Interface Compiler version 6.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGE_LOGIN_H
#define UI_PAGE_LOGIN_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Page_Login
{
public:
    QGridLayout *gridLayout_2;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QPushButton *btn_login;
    QPushButton *btn_exit;
    QWidget *widget_2;
    QLabel *label;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label_2;
    QLabel *label_3;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *le_user;
    QLineEdit *lineEdit_2;

    void setupUi(QWidget *Page_Login)
    {
        if (Page_Login->objectName().isEmpty())
            Page_Login->setObjectName("Page_Login");
        Page_Login->resize(400, 240);
        Page_Login->setMinimumSize(QSize(400, 240));
        Page_Login->setMaximumSize(QSize(400, 240));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/icon.jpg"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        Page_Login->setWindowIcon(icon);
        gridLayout_2 = new QGridLayout(Page_Login);
        gridLayout_2->setObjectName("gridLayout_2");
        widget = new QWidget(Page_Login);
        widget->setObjectName("widget");
        widget->setMaximumSize(QSize(16777215, 40));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName("horizontalLayout");
        horizontalLayout->setContentsMargins(2, 0, 2, 0);
        btn_login = new QPushButton(widget);
        btn_login->setObjectName("btn_login");
        btn_login->setMaximumSize(QSize(16777215, 35));
        QFont font;
        font.setPointSize(11);
        btn_login->setFont(font);

        horizontalLayout->addWidget(btn_login);

        btn_exit = new QPushButton(widget);
        btn_exit->setObjectName("btn_exit");
        btn_exit->setMaximumSize(QSize(16777215, 35));
        btn_exit->setFont(font);

        horizontalLayout->addWidget(btn_exit);


        gridLayout_2->addWidget(widget, 3, 0, 1, 1);

        widget_2 = new QWidget(Page_Login);
        widget_2->setObjectName("widget_2");
        label = new QLabel(widget_2);
        label->setObjectName("label");
        label->setGeometry(QRect(60, 10, 261, 60));
        label->setMinimumSize(QSize(0, 60));
        label->setMaximumSize(QSize(16777215, 60));
        layoutWidget = new QWidget(widget_2);
        layoutWidget->setObjectName("layoutWidget");
        layoutWidget->setGeometry(QRect(10, 90, 71, 61));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName("label_2");
        label_2->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        verticalLayout->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName("label_3");
        label_3->setAlignment(Qt::AlignmentFlag::AlignRight|Qt::AlignmentFlag::AlignTrailing|Qt::AlignmentFlag::AlignVCenter);

        verticalLayout->addWidget(label_3);

        layoutWidget1 = new QWidget(widget_2);
        layoutWidget1->setObjectName("layoutWidget1");
        layoutWidget1->setGeometry(QRect(90, 90, 241, 61));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        le_user = new QLineEdit(layoutWidget1);
        le_user->setObjectName("le_user");
        le_user->setMaxLength(8);

        verticalLayout_2->addWidget(le_user);

        lineEdit_2 = new QLineEdit(layoutWidget1);
        lineEdit_2->setObjectName("lineEdit_2");
        lineEdit_2->setMaxLength(8);
        lineEdit_2->setEchoMode(QLineEdit::EchoMode::Password);

        verticalLayout_2->addWidget(lineEdit_2);


        gridLayout_2->addWidget(widget_2, 2, 0, 1, 1);


        retranslateUi(Page_Login);

        QMetaObject::connectSlotsByName(Page_Login);
    } // setupUi

    void retranslateUi(QWidget *Page_Login)
    {
        Page_Login->setWindowTitle(QCoreApplication::translate("Page_Login", "\347\231\273\345\275\225", nullptr));
#if QT_CONFIG(tooltip)
        Page_Login->setToolTip(QCoreApplication::translate("Page_Login", "\350\277\231\346\230\257\347\231\273\345\275\225\347\252\227\345\217\243", nullptr));
#endif // QT_CONFIG(tooltip)
        btn_login->setText(QCoreApplication::translate("Page_Login", "\347\231\273\345\275\225", nullptr));
        btn_exit->setText(QCoreApplication::translate("Page_Login", "\351\200\200\345\207\272", nullptr));
        label->setText(QCoreApplication::translate("Page_Login", "<html><head/><body><p align=\"center\"><span style=\" font-size:18pt;\">\345\255\246\347\224\237\344\277\241\346\201\257\347\256\241\347\220\206\347\263\273\347\273\237</span></p></body></html>", nullptr));
        label_2->setText(QCoreApplication::translate("Page_Login", "<html><head/><body><p><span style=\" font-size:14pt;\">\347\224\250\346\210\267\345\220\215</span></p></body></html>", nullptr));
        label_3->setText(QCoreApplication::translate("Page_Login", "<html><head/><body><p><span style=\" font-size:14pt;\">\345\257\206\347\240\201</span></p></body></html>", nullptr));
        le_user->setPlaceholderText(QCoreApplication::translate("Page_Login", "\350\257\267\350\276\223\345\205\245", nullptr));
        lineEdit_2->setPlaceholderText(QCoreApplication::translate("Page_Login", "\350\257\267\350\276\223\345\205\245", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Page_Login: public Ui_Page_Login {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGE_LOGIN_H
