#include <QMetaType>
#include "common_types_extended.h"
#include "mainwindow.h"


#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //关闭FFmepg日志输出的，
    av_log_set_level(AV_LOG_QUIET);
    //跨线程数据传输，先要注册
    qRegisterMetaType<QHostAddress>("QHostAddress"); // ✅ 必须
    qRegisterMetaType<LidarFrame>("LidarFrame");
    qRegisterMetaType<QVector<M_PointXYZI>>("QVector<M_PointXYZI>");
    qRegisterMetaType<TimedFrame<QVector<M_PointXYZI>>>("TimedFrame<QVector<M_PointXYZI>>");
    qRegisterMetaType<QImage>("QImage");
    qRegisterMetaType<usv::BerthMeasureResult>("usv::BerthMeasureResult");

    MainWindow w;
    w.show();
    return a.exec();
}
