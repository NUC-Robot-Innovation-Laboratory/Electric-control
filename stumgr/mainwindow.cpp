#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QKeyEvent>
#include <QCoreApplication>
#include <QHeaderView>
#include <QTableWidget>
#include <QList>
#include <QRandomGenerator>
#include <QTimer>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    ,m_ptrStuSql(nullptr)
{
    m_ptrStuSql = stuSql::getinstance();
    if (!m_ptrStuSql) {
        qCritical() << "Error: stuSql 单例初始化失败！";
        QTimer::singleShot(0, this, &QMainWindow::close);
        return;
    }

    // 2. 初始化数据库
    if (!m_ptrStuSql->init()) {
        qCritical() << "数据库初始化失败！";
        QTimer::singleShot(0, this, &QMainWindow::close);//通过定时器延迟执行主窗口的关闭操作。
        //界面可能尚未完全初始化（如 ui->setupUi(this) 可能还未调用）
        return;
    }

    ui->setupUi(this);
    m_dlgLogin.show();
    connect(&m_dlgLogin, &Page_Login::sendLoginSuccess, this, [this]{
        this->show();
    });

    // 初始化树状控件
    ui->treeWidget->clear();
    ui->treeWidget->setColumnCount(1);//用于设置树形控件（QTreeWidget）的列数为 1
    QStringList l;
    l << "学生信息管理系统";
    QTreeWidgetItem *pf = new QTreeWidgetItem(ui->treeWidget, l);
    ui->treeWidget->addTopLevelItem(pf);
    l.clear();
    l << "学生管理";
    QTreeWidgetItem *p1 = new QTreeWidgetItem(pf, l);
    l.clear();
    l << "管理员管理";
    QTreeWidgetItem *p2 = new QTreeWidgetItem(pf, l);
    pf->addChild(p1);
    pf->addChild(p2);
    ui->treeWidget->expandAll();


    // 初始化表格（替换原有的列宽设置逻辑）
    initTable();
    m_ptrStuSql = stuSql::getinstance();
    if (!m_ptrStuSql || !m_ptrStuSql->init()) {
        qCritical() << "stuSql 初始化失败，程序退出";
        QTimer::singleShot(0, this, &QMainWindow::close);
        return;
    }
    m_ptrStuSql->init();
    m_lNames << "王谷雪";
    m_lNames << "坚清心";
    m_lNames << "栋芳蕙";
    m_lNames << "姓山灵";
    m_lNames << "佛盼夏";
    m_lNames << "禽多";
    m_lNames << "柔小蕊";
    m_lNames << "庞锐志";
    m_lNames << "军和";
    m_lNames << "范代双";
    m_lNames << "系阳泽";
    m_lNames << "孔骊媛";
    m_lNames << "鞠绿旋";
    m_lNames << "席清妍";
    m_lNames << "卫永年";
    m_lNames << "奚濮存";
    m_lNames << "汤成益";
    m_lNames << "长孙霏";
    m_lNames << "卜彤";
    m_lNames << "函醉巧";
    m_lNames << "费茉莉";
    m_lNames << "洋逸";
    m_lNames << "建雨竹";
    m_lNames << "呼芳茵";
    m_lNames << "真灵";
    m_lNames << "同冰蝶";
    m_lNames << "冉从筠";
    m_lNames << "谭醉易";
    m_lNames << "释白安";
    m_lNames << "敛竹筱";
    m_lNames << "福易蓉";
    m_lNames << "庄恬欣";
    m_lNames << "琦问萍";
    m_lNames << "素乐家";
    m_lNames << "休桐";
    m_lNames << "随夏兰";
    m_lNames << "宇傲雪";
    m_lNames << "绪小春";
    m_lNames << "湛尔烟";
    m_lNames << "袭小蕾";
    m_lNames << "针白萱";
    m_lNames << "营秋柏";
    m_lNames << "候司晨";
    m_lNames << "项真洁";
    m_lNames << "宏醉山";
    m_lNames << "索半香";
    m_lNames << "汝晓燕";
    m_lNames << "唐梓彤";
    m_lNames << "左梦雨";
    m_lNames << "第五巧风";
    m_lNames << "蒯寅骏";
    m_lNames << "卓静云";
    m_lNames << "郯全";
    m_lNames << "笃驰皓";
    m_lNames << "位和豫";
    m_lNames << "阳雅懿";
    m_lNames << "绍振海";
    m_lNames << "巩乐悦";
    m_lNames << "阿文虹";
    m_lNames << "阚惜萍";
    m_lNames << "尾念蕾";
    m_lNames << "孝以旋";
    m_lNames << "薄夏菡";
    m_lNames << "藤曜";
    m_lNames << "秋英华";
    m_lNames << "家蓄";
    m_lNames << "贾寒梅";
    m_lNames << "锺同化";
    m_lNames << "郦偲偲";
    m_lNames << "初曼丽";
    m_lNames << "法嘉";
    m_lNames << "恽蓝";
    m_lNames << "平怜珊";
    m_lNames << "阴舒畅";
    m_lNames << "屠骏奇";
    m_lNames << "由冰心";
    m_lNames << "羊彦";
    m_lNames << "谷梁岳";
    m_lNames << "籍平安";
    m_lNames << "登咏思";
    m_lNames << "斯思天";
    m_lNames << "郏晓山";
    m_lNames << "迮鸿晖";
    m_lNames << "鲜以寒";
    m_lNames << "赵刚毅";
    m_lNames << "贝暄妍";
    m_lNames << "鲜于芷云";
    m_lNames << "说梓舒";
    m_lNames << "钭如彤";
    m_lNames << "廉曼卉";
    m_lNames << "宫雨旋";
    m_lNames << "澄国安";
    m_lNames << "昔姝惠";
    m_lNames << "肥智阳";
    m_lNames << "韶悦欣";
    m_lNames << "无晶瑶";
    m_lNames << "沃晓蕾";
    m_lNames << "潭鹏赋";
    m_lNames << "乔湛娟";
    m_lNames << "英若云";
    m_lNames << "羽学文";
    m_lNames << "贯国源";
    m_lNames << "及夏青";
    m_lNames << "枚含烟";
    m_lNames << "聊寄波";
    m_lNames << "房博瀚";
    m_lNames << "腾忆雪";
    m_lNames << "铎忆梅";
    m_lNames << "莫施诗";
    m_lNames << "独柔雅";
    m_lNames << "海鸿才";
    m_lNames << "陀天真";
    m_lNames << "酒晶霞";
    m_lNames << "仲孙娅童";
    m_lNames << "欧以";
    m_lNames << "谬碧菡";
    m_lNames << "错逸明";
    m_lNames << "濯施";
    m_lNames << "孛画";
    m_lNames << "惠新晴";
    m_lNames << "仍清淑";
    m_lNames << "侯诗槐";
    m_lNames << "世驰海";
    m_lNames << "骑薇";
    m_lNames << "司马莹华";
    m_lNames << "皇甫韶敏";
    m_lNames << "南门俊雅";
    m_lNames << "牵建元";
    m_lNames << "司空嘉志";
    m_lNames << "类忆彤";
    m_lNames << "钮笑晴";
    m_lNames << "干芳荃";
    m_lNames << "邬香卉";
    auto cnt=m_ptrStuSql->getStuCnt();
    ui->lb_cnt->setText(QString("学生数量:%1").arg(cnt));
    QList<StuInfo> lStudents=m_ptrStuSql->getPageStu(0,cnt);
    ui->tableWidget->clear();
    // 设置表头标签
    QStringList headers = {"序号", "姓名", "年龄", "年级", "班级", "学号", "电话", "微信"};
    ui->tableWidget->setHorizontalHeaderLabels(headers);
    ui->tableWidget->setRowCount(cnt);
    for(int i=0;i<lStudents.size();i++){
        ui->tableWidget->setItem(i,0,new QTableWidgetItem(QString::number(i)));
        ui->tableWidget->setItem(i,1,new QTableWidgetItem(lStudents[i].name));
        ui->tableWidget->setItem(i,2,new QTableWidgetItem(QString::number(lStudents[i].age)));
        ui->tableWidget->setItem(i,3,new QTableWidgetItem(QString::number(lStudents[i].grade)));
        ui->tableWidget->setItem(i,4,new QTableWidgetItem(QString::number(lStudents[i].uiclass)));
        ui->tableWidget->setItem(i,5,new QTableWidgetItem(QString::number(lStudents[i].studentid)));
        ui->tableWidget->setItem(i,6,new QTableWidgetItem(lStudents[i].phone));
        ui->tableWidget->setItem(i,7,new QTableWidgetItem(lStudents[i].wechat));
    }
//row：行号（i）；
//column：列号（0 到 7，共 8 列）；
//item：QTableWidgetItem 对象（表格单元格的内容）。

}

MainWindow::~MainWindow()
{
    delete ui;
}

// 新增：表格初始化及动态列宽调整函数
void MainWindow::initTable() {
    QTableWidget *tableWidget = ui->tableWidget;
    if (!tableWidget) return;

    QHeaderView *header = tableWidget->horizontalHeader();
    // 设置表头为交互模式（允许手动拖动调整列宽）
    header->setSectionResizeMode(QHeaderView::Interactive);
    // 关闭自动拉伸最后一列（因为我们要自定义比例）
    header->setStretchLastSection(false);

    int columnCount = tableWidget->columnCount();//统计有几列
    // 定义每列比例（根据实际列数调整此数组长度）
    QList<double> columnRatios = {1, 1, 1, 1, 1, 1.5, 2, 2};

    // 校验比例数组长度与列数是否匹配
    if (columnRatios.size() != columnCount) {
        qWarning() << "Column ratio count does not match table column count!";
        return;
    }

    // 计算总比例
    double totalRatio = 0;
    for (double ratio : columnRatios) {
        totalRatio += ratio;
    }

    // 初始化列宽
    int tableWidth = tableWidget->width();//获取表格的总宽度
    for (int i = 0; i < columnCount; ++i) {
        //static_cast 是一种显式类型转换运算符，转换为int类型
        int width = static_cast<int>((tableWidth * columnRatios[i]) / totalRatio);//按比例计算每列宽度
        header->resizeSection(i, width);//设置当前列的宽度
    }

    // 连接表头拉伸信号，动态调整列宽
    //当调整某一列宽度时，QHeaderView::sectionResized 信号会被触发
    //参数 logicalIndex 是被调整列的索引。
    connect(header, &QHeaderView::sectionResized, this, [this, columnRatios](int logicalIndex, int oldSize, int newSize) {
        Q_UNUSED(oldSize);//用于消除编译器未使用变量警告的宏
        Q_UNUSED(newSize);
        QTableWidget *table = ui->tableWidget;
        if (!table) return;

        QHeaderView *header = table->horizontalHeader();
        int columnCount = table->columnCount();
        double totalRatio = 0;
        for (double ratio : columnRatios) {
            totalRatio += ratio;
        }

        int tableWidth = table->width();
        int fixedWidth = 0;
        // 计算除当前调整列外其他列的总宽度（按比例）
        for (int i = 0; i < columnCount; ++i) {
            if (i != logicalIndex) {
                fixedWidth += static_cast<int>((tableWidth * columnRatios[i]) / totalRatio);
            }
        }

        // 剩余宽度分配给当前调整的列（保持比例）
        int remainingWidth = tableWidth - fixedWidth;
        header->resizeSection(logicalIndex, remainingWidth);
    });
}

void MainWindow::on_btn_exit_clicked()
{
    exit(0);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_F6){
        QFile f("D:\\Qt\\Qt6.9\\QtProject\\stumgr\\build\\Desktop_Qt_6_9_0_MinGW_64_bit-Debug\\debug\\stuqss.css");
        if (f.open(QIODevice::ReadOnly)) {
            QString strQss = f.readAll();
            this->setStyleSheet(strQss);
            m_dlgLogin.setStyleSheet(strQss);
            //f.close();
        }
    }
}


void MainWindow::on_btn_simulation_clicked()
{
    //制作1000条学生数据
    QRandomGenerator *g = QRandomGenerator::global();//获取全局随机数生成器实例，赋值给指针 g
    QRandomGenerator *c = QRandomGenerator::global();

    for(int i=0;i<m_lNames.size();i++){
            int grade = g->bounded(7, 9);  // 生成 7 或 8（bounded 上限是开区间）
            int uiclass = c->bounded(1, 8);
        StuInfo info;
        info.name=m_lNames[i];
        info.age = 17;
        if (i % 3 == 0) {  // i 是 3 的倍数时
            info.age = 16;
        } else if (i % 2 == 0) {  // i 不是 3 的倍数但 是 2 的倍数
            info.age = 18;
        } else if (i % 7 == 0) {  // i 不是 3/2 的倍数但 是 7 的倍数
            info.age = 17;
        }

        info.grade=grade;
        info.uiclass=uiclass;
        info.studentid=i;
        info.phone="13812345678";
        info.wechat="123456577";
        m_ptrStuSql->addStu(info);
    }
}




