#include "stusql.h"
#include <QMessageBox>
#include <QSqlError>
#include <QCoreApplication>
#include <QSqlQuery>
#include <QtDebug>
stuSql * stuSql::ptrstuSql=nullptr;
stuSql::stuSql(QObject *parent)
    : QObject{parent}
{

    /*if (!init()) {
        qDebug() << "构造函数警告：数据库初始化失败，后续功能无法使用";
        return; // 初始化失败时，终止后续操作
    }
    StuInfo testStu;
    testStu.name = "张三";
    testStu.age = 12;
    testStu.grade = 3;    // 年级
    testStu.uiclass = 2;  // 班级
    testStu.studentid = 20240001; // 唯一学号
    testStu.phone = "15940224444";
    testStu.wechat = "zhangsan_wechat";*/
    //
    //getPageStu(2,3);
    //getStuCnt();
    //delStu(1803);
    //clearStuTable()
    //testStu.grade = 888;
    //void UpdataStuInfo(testStu);
    /*UserInfo testUser;
    testUser.username = "admin";
    testUser.password = "123";
    testUser.auth = "admin";*/
    /*if (!isExit(testUser.username)) { // 避免重复添加
        if (AddUser(testUser)) {
            qDebug() << "构造函数：测试用户 [admin] 添加成功";
        } else {
            qDebug() << "构造函数：测试用户添加失败";
        }
    } else {
        qDebug() << "构造函数：用户 [admin] 已存在，跳过添加";
    }*/
    //updateUser(testUser);
    //AddUser(testUser);
    //delUser(testUser.username);
}

bool stuSql::init() {
    // 步骤1：检查数据库驱动是否存在
    if (QSqlDatabase::drivers().isEmpty()) {
        qDebug() << "错误：未找到数据库驱动";
        return false;
    }

    // 步骤2：添加SQLite数据库连接
    m_db = QSqlDatabase::addDatabase("QSQLITE");

    // 步骤3：设置数据库路径（可根据需求修改为动态路径）
    QString dbPath = "D:\\Qt\\Qt6.9\\QtProject\\stumgr\\db\\data.db";
    m_db.setDatabaseName(dbPath);//指定路径
    qDebug() << "数据库路径：" << dbPath;

    // 步骤4：打开数据库
    if (!m_db.open()) {
        qDebug() << "数据库打开失败，错误：" << m_db.lastError().text();
        return false;
    }

    qDebug() << "数据库初始化成功";
    return true;
}
quint32 stuSql::getStuCnt(){
    QSqlQuery sql(m_db);
    if (!sql.exec("SELECT COUNT(id) FROM student")) {
        qDebug() << "查询学生数量失败：" << sql.lastError().text();
        return 0;
    }
    // 直接读取第一条结果（COUNT 只有一行）
    if (sql.next()) {
        return sql.value(0).toUInt();
    }
    return 0;
}
QList<StuInfo> stuSql::getPageStu(quint32 page, quint32 uiCnt)
{
        // 1. 函数声明和参数说明
        // 函数返回值类型为 QList<StuInfo>，表示返回一个包含 StuInfo 结构体的列表，用于存储获取到的学生信息
        // 参数 page 表示要获取的页码
        // 参数 uiCnt 表示每页显示的学生数量
        QList<StuInfo> l;
        // 2. 创建一个空的 QList<StuInfo> 对象 l，用于存储从数据库中获取的学生信息

        QSqlQuery sql(m_db);
        // 3. 创建一个 QSqlQuery 对象 sql，用于执行 SQL 查询操作，传入已有的数据库连接对象 m_db

        QString strsql=QString("select * from student order by id limit %1 offset %2;")
                             .arg(uiCnt).arg(page*uiCnt);
        // 4. 构造 SQL 查询语句
        // 该语句的作用是从 student 表中按 id 排序，获取指定数量（uiCnt）的学生信息，
        // 并从指定的偏移量（page * uiCnt）开始获取，实现分页查询
        sql.exec(strsql);
        // 5. 执行构造好的 SQL 查询语句
        StuInfo info;
        while(sql.next()){
            // 7. 循环遍历查询结果集，sql.next() 用于移动到下一条记录，当还有记录时返回 true，否则返回 false
            info.id=sql.value(0).toUInt();//只要下一个不为空，计序列
            // 8. 从查询结果的当前记录中获取第 0 列的值（假设第 0 列是 id 列），并转换为无符号整数类型，赋值给 info 的 id 成员
            info.name=sql.value(1).toString();

            info.age=sql.value(2).toUInt();

            info.grade=sql.value(3).toUInt();

            info.uiclass=sql.value(4).toUInt();

            info.studentid=sql.value(5).toUInt();

            info.phone=sql.value(6).toString();

            info.wechat=sql.value(7).toString();

            l.push_back(info);
            //将填充好信息的 info 对象添加到 QList<StuInfo> 对象 l 中
        }
        return l;
        // 17. 返回存储了从数据库中获取的学生信息的 QList<StuInfo> 对象 l
}

bool stuSql::addStu(StuInfo info)
{
    QSqlQuery query(m_db);  // 使用 QSqlQuery 操作数据库

    // 显式指定列名（推荐），避免表结构变化导致的错误
    // id 列设为 null（自增主键会自动生成）
    QString insertSql = R"(
        INSERT INTO student
        (id, name, age, grade, class, studentid, phone, wechat)
        VALUES (null, :name, :age, :grade, :class, :studentid, :phone, :wechat)
    )";

    // 预处理 SQL 语句（参数化查询核心）
    query.prepare(insertSql);

    // 绑定参数
    query.bindValue(":name", info.name);        // 文本类型
    query.bindValue(":age", info.age);          // 整数类型
    query.bindValue(":grade", info.grade);      // 整数类型
    query.bindValue(":class", info.uiclass);    // 整数类型
    query.bindValue(":studentid", info.studentid); // 整数类型
    query.bindValue(":phone", info.phone);      // 文本类型
    query.bindValue(":wechat", info.wechat);    // 文本类型

    // 执行 SQL 并返回结果
    bool success = query.exec();
    if (!success) {
        // 输出具体错误信息（便于调试）
        qDebug() << "插入失败，SQL错误：" << query.lastError().text();
    }
    else{
        qDebug()<<"插入成功";
    }
    return success;  // 返回插入是否成功
}

bool stuSql::delStu(int id)
{
    QSqlQuery sql(m_db);
    return sql.exec(QString("delete from student where id = %1").arg(id));
}

bool stuSql::clearStuTable()
{
    QSqlQuery sql(m_db);
    return sql.exec("delete from student");
}

void stuSql::UpdataStuInfo(StuInfo info)
{
    QSqlQuery query(m_db);
    // 构造参数化的 SQL 更新语句
    QString updateSql = R"(
        UPDATE student
        SET name = :name,
            age = :age,
            grade = :grade,
            class = :class,
            studentid = :studentid,
            phone = :phone,
            wechat = :wechat
        WHERE id = :id
    )";

    // 预处理 SQL 语句
    query.prepare(updateSql);
    query.bindValue(":id", info.id);
    query.bindValue(":name", info.name);
    query.bindValue(":age", info.age);
    query.bindValue(":grade", info.grade);
    query.bindValue(":class", info.uiclass);
    query.bindValue(":studentid", info.studentid);
    query.bindValue(":phone", info.phone);
    query.bindValue(":wechat", info.wechat);

    // 执行 SQL 语句
    if (!query.exec()) {
        // 若执行失败，输出错误信息
        qDebug() << "更新学生信息失败，SQL 错误：" << query.lastError().text();
    }
}

QList<UserInfo> stuSql::getALLUser()
{
    QList<UserInfo> l;
    QSqlQuery sql(m_db);
    sql.exec("select * from username");
    UserInfo info;
    while(sql.next()){
        info.username=sql.value(0).toString();//只要下一个不为空，计序列
        info.auth=sql.value(1).toString();
        info.password=sql.value(2).toString();
        l.push_back(info);
    }
    return l;
}

bool stuSql::isExit(QString strUser)
{
    QSqlQuery sql(m_db);
    sql.exec(QString("select *from username where username='%1'").arg(strUser));
    return sql.next();
}

bool stuSql::updateUser(UserInfo info)
{
    QSqlQuery query(m_db);
    // 构造参数化的 SQL 更新语句
    QString updateSql = R"(
        UPDATE username
        SET password = :password,
            auth = :auth
        WHERE username = :username
    )";

    // 预处理 SQL 语句
    query.prepare(updateSql);

    // 绑定参数
    query.bindValue(":username", info.username);
    query.bindValue(":password", info.password);
    query.bindValue(":auth", info.auth);


    // 执行 SQL 语句
    bool success = query.exec();
    if (!success) {
        // 若执行失败，输出错误信息
        qDebug() << "更新用户信息失败，SQL 错误：" << query.lastError().text();
    }
    return success;
}

bool stuSql::AddUser(UserInfo info)
{
    QSqlQuery query(m_db);  // 使用 QSqlQuery 操作数据库

    // 显式指定列名（推荐），避免表结构变化导致的错误
    // id 列设为 null（自增主键会自动生成）
    QString insertSql = R"(
        INSERT INTO username (username, password, auth)
        VALUES (:username, :password, :auth)
    )";

    // 预处理 SQL 语句（参数化查询核心）
    query.prepare(insertSql);
    query.bindValue(":username", info.username);        // 文本类型
    query.bindValue(":password", info.password);      // 文本类型
    query.bindValue(":auth", info.auth);          // 文本类型
    // 执行 SQL 并返回结果
    bool success = query.exec();
    if (!success) {
        qDebug() << "插入失败，SQL错误：" << query.lastError().text();
    }
    return success;  // 返回插入是否成功
}

bool stuSql::delUser(QString strUserName)
{
    QSqlQuery sql(m_db);
    return sql.exec(QString("delete from username where username='%1'").arg(strUserName));
}
