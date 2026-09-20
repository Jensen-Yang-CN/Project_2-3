#ifndef IMUWIDGET_H
#define IMUWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include "NaviProtocol.h" // 请确保此头文件包含 IMUParsedData 结构体

class ImuWidget : public QWidget {
    Q_OBJECT
public:
    explicit ImuWidget(QWidget *parent = nullptr);
    ~ImuWidget();

    // 设置面板顶部的标题文字
    void setPanelInfo(const QString &title);

public slots:
    // 接收来自 Worker 线程的解析数据
    void updateNavigationData(const IMUParsedData &data);

private:
    // 初始化 UI 布局和控件
    void setupLayout();

    // 辅助函数：快速创建名称和数值的标签对
    // 确保这里的参数列表最后有一个 int col
        void addDataRow(QGridLayout *layout, const QString &name, QLabel* &valLabel, int row, int col);

    // 私有 UI 成员变量（注意：不要带 ui-> 前缀直接访问）
    QLabel *lblTitle;
    QLabel *lblYawVal;
    QLabel *lblPitchVal;
    QLabel *lblRollVal;
    QLabel *lblLonVal;
    QLabel *lblLatVal;

    QLabel *lblEastRateVal;
    QLabel *lblNorthRateVal;
    QLabel *lblSkyRateVal;
    QLabel *lblXRateVal;
    QLabel *lblYRateVal;
};

#endif // IMUWIDGET_H
