#include "imuwidget.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QWidget>
#include <QDebug>
#include <QTimer>

ImuWidget::ImuWidget(QWidget *parent) : QWidget(parent) {
//    qDebug() << "--- 布局状态检查 ---";
//    qDebug() << "Layout 指针:" << this->layout();
//    qDebug() << "子对象列表是否含布局:" << this->findChild<QLayout*>();
//    qDebug() << "是否有布局属性:" << this->testAttribute(Qt::WA_WState_Created);
//    setupLayout();
    QTimer::singleShot(0, this, [this](){
            this->setupLayout();
        });
}

ImuWidget::~ImuWidget() {
    // 指针由 Qt 的父子对象树管理，无需手动 delete
}

void ImuWidget::setupLayout() {
    // 再次双重检查，如果此时已经有布局了，彻底销毁它
    if (this->layout()) {
        // 使用 deleteLater 确保在当前事件处理完后安全删除
        delete this->layout();
    }

    // 创建孤立布局
    QVBoxLayout *mainLayout = new QVBoxLayout();
    mainLayout->setContentsMargins(5, 5, 5, 5);
    mainLayout->setSpacing(5);

    // --- 界面元素构建 ---
    lblTitle = new QLabel("IMU 综合导航信息", this);
    lblTitle->setStyleSheet("background-color: #34495E; color: white; padding: 4px; font-weight: bold;");
    mainLayout->addWidget(lblTitle);

    QGridLayout *grid = new QGridLayout();
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(3, 1);

    // 第一区：姿态与位置
    addDataRow(grid, "经度(Lon):", lblLonVal,   0, 0);
    addDataRow(grid, "纬度(Lat):", lblLatVal,   0, 2);
    addDataRow(grid, "艏向(Yaw):", lblYawVal,   1, 0);
    addDataRow(grid, "纵摇(Pit):", lblPitchVal, 1, 2);
    addDataRow(grid, "横摇(Rol):", lblRollVal,  2, 0);

    // 分隔线
    QLabel* line = new QLabel(this);
    line->setFixedHeight(1);
    line->setStyleSheet("background-color: #BDC3C7;");
    grid->addWidget(line, 3, 0, 1, 4);

    // 第二区：速度信息
    addDataRow(grid, "东向(Ve):", lblEastRateVal,  4, 0);
    addDataRow(grid, "北向(Vn):", lblNorthRateVal, 4, 2);
    addDataRow(grid, "天向(Vu):", lblSkyRateVal,   5, 0);
    addDataRow(grid, "载系 Vx:",  lblXRateVal,     5, 2);
    addDataRow(grid, "载系 Vy:",  lblYRateVal,     6, 0);

    mainLayout->addLayout(grid);
    mainLayout->addStretch();

    // 最后安装
    this->setLayout(mainLayout);
}

//void ImuWidget::setupLayout() {

//    QVBoxLayout *mainLayout = new QVBoxLayout(this);
//    mainLayout->setContentsMargins(5, 5, 5, 5);

//    lblTitle = new QLabel("IMU 综合导航信息", this);
//    lblTitle->setStyleSheet("background-color: #34495E; color: white; padding: 4px; font-weight: bold;");
//    mainLayout->addWidget(lblTitle);

//    QGridLayout *grid = new QGridLayout();
//    grid->setColumnStretch(1, 1);
//    grid->setColumnStretch(3, 1);

//    // 第一区：姿态与位置 (两行两列)
//    addDataRow(grid, "经度(Lon):", lblLonVal,   0, 0);
//    addDataRow(grid, "纬度(Lat):", lblLatVal,   0, 2);
//    addDataRow(grid, "艏向(Yaw):", lblYawVal,   1, 0);
//    addDataRow(grid, "纵摇(Pit):", lblPitchVal, 1, 2);
//    addDataRow(grid, "横摇(Rol):", lblRollVal,  2, 0);

//    // 分隔线
//    QLabel* line = new QLabel(this);
//    line->setFixedHeight(1);
//    line->setStyleSheet("background-color: #BDC3C7;");
//    grid->addWidget(line, 3, 0, 1, 4);

//    // 第二区：速度信息 (两行两列)
//    addDataRow(grid, "东向(Ve):", lblEastRateVal,  4, 0);
//    addDataRow(grid, "北向(Vn):", lblNorthRateVal, 4, 2);
//    addDataRow(grid, "天向(Vu):", lblSkyRateVal,   5, 0);
//    addDataRow(grid, "载系 Vx:",  lblXRateVal,     5, 2);
//    addDataRow(grid, "载系 Vy:",  lblYRateVal,     6, 0);

//    mainLayout->addLayout(grid);
//    mainLayout->addStretch();
//}

void ImuWidget::addDataRow(QGridLayout *layout, const QString &name, QLabel* &valLabel, int row, int col) {
    QLabel *nameLbl = new QLabel(name, this);
    nameLbl->setStyleSheet("color: #7F8C8D; font-size: 9pt;");

    valLabel = new QLabel("0.000", this);
    valLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    valLabel->setStyleSheet("color: #27AE60; font-family: 'Consolas'; font-size: 11pt; font-weight: bold;");

    layout->addWidget(nameLbl, row, col);
    layout->addWidget(valLabel, row, col + 1);
}

void ImuWidget::updateNavigationData(const IMUParsedData &data) {
    // 更新文本
    lblYawVal->setText(QString::number(data.yaw, 'f', 2) + " °");
    lblPitchVal->setText(QString::number(data.pitch, 'f', 2) + " °");
    lblRollVal->setText(QString::number(data.roll, 'f', 2) + " °");
    lblLonVal->setText(QString::number(data.longitude, 'f', 7));
    lblLatVal->setText(QString::number(data.latitude, 'f', 7));

    auto formatV = [](double v) { return QString::number(v, 'f', 3) + " m/s"; };
    lblEastRateVal->setText(formatV(data.vEast));
    lblNorthRateVal->setText(formatV(data.vNorth));
    lblSkyRateVal->setText(formatV(data.vSky));
    lblXRateVal->setText(formatV(data.vx));
    lblYRateVal->setText(formatV(data.vy));

    // 状态样式
    QString normal = "color: #27AE60; font-family: 'Consolas'; font-size: 11pt; font-weight: bold;";
    QString warn   = "color: #E74C3C; font-family: 'Consolas'; font-size: 11pt; font-weight: bold;";
    lblYawVal->setStyleSheet(data.yawValid ? normal : warn);
}
