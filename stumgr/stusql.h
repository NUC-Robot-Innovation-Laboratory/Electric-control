#ifndef STUSQL_H
#define STUSQL_H

#include <QObject>
#include <QSqlDatabase>

struct StuInfo {
    int id;
    QString name;
    quint8 age;
    quint16 grade;
    quint16 uiclass;
    quint32 studentid;
    QString phone;
    QString wechat;
};

struct UserInfo {
    QString username;
    QString password;
    QString auth;
};

class stuSql : public QObject {
    Q_OBJECT
public:
    static stuSql* ptrstuSql;
    static stuSql* getinstance() {
        if (nullptr == ptrstuSql) {
            ptrstuSql = new stuSql(nullptr); // 显式传递父对象为nullptr
        }
        return ptrstuSql;
    }

    bool init();             // 初始化数据库
    quint32 getStuCnt();     // 获取学生总数
    QList<StuInfo> getPageStu(quint32 page, quint32 uiCnt); // 分页查询学生
    bool addStu(StuInfo info); // 添加学生
    bool delStu(int id);     // 删除学生（按id）
    bool clearStuTable();    // 清空学生表
    void UpdataStuInfo(StuInfo info); // 更新学生信息
    QList<UserInfo> getALLUser();     // 获取所有用户
    bool isExit(QString strUser);     // 判断用户是否存在
    bool updateUser(UserInfo info);   // 更新用户信息
    bool AddUser(UserInfo info);      // 添加用户
    bool delUser(QString strUserName); // 删除用户

private:
    explicit stuSql(QObject* parent = nullptr); // 私有构造函数（带默认参数）
    QSqlDatabase m_db; // 数据库连接对象
};

#endif // STUSQL_H
