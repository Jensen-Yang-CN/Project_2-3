#include "adddevicedialog.h"
#include "ui_adddevicedialog.h"
#include <QDateTime>
#include <QDebug>
using namespace device;
AddDeviceDialog::AddDeviceDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AddDeviceDialog)
{

    ui->setupUi(this);
    //setWindowTitle("注册新设备");
    ui->typeCombo->clear(); // 确保从零开始添加
    if (ui->typeCombo->count() > 0) {
        ui->typeCombo->setCurrentIndex(0);
    }
    // 在 Designer 里给 typeCombo 添加选项时，记得设置 UserData 对应枚举值
    ui->typeCombo->addItem("速腾雷达 (RS)", static_cast<int>(DeviceType::Lidar_RS));
    ui->typeCombo->addItem("镭神雷达 (LS)", static_cast<int>(DeviceType::Lidar_LS));
    ui->typeCombo->addItem("惯导 (IMU)", static_cast<int>(DeviceType::IMU));
    ui->typeCombo->addItem("红外相机 (H264)", static_cast<int>(DeviceType::Camera_H264));
    ui->typeCombo->addItem("可见光相机 (H265)", static_cast<int>(DeviceType::Camera_H265));

    // 绑定类型切换信号，处理动态显示（比如隐藏非雷达的备用端口）
    connect(ui->typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &AddDeviceDialog::onTypeChanged);
}

AddDeviceDialog::~AddDeviceDialog()
{
    delete ui;
}

DeviceInfo AddDeviceDialog::getDeviceInfo() const
{
    DeviceInfo info;
    info.name = ui->nameEdit->text();
    info.type = static_cast<DeviceType>(ui->typeCombo->currentData().toInt());
    //info.type = toString(ui->typeCombo->currentData().toInt());
    info.ip = ui->ipEdit->text();
    info.port = ui->portEdit->text().toInt();

    // 如果是雷达，读取备用端口；否则设为 0
    bool isLidar = (info.type == DeviceType::Lidar_RS || info.type == DeviceType::Lidar_LS);
    info.extraPort = isLidar ? ui->extraPortEdit->text().toInt() : 0;

    // metadata 预留：可以在这里初始化一些默认值
    info.metadata["added_time"] = QDateTime::currentDateTime().toString();

    return info;

}

void AddDeviceDialog::onTypeChanged(int index)
{
    qDebug() << "Type changed to index:" << index;

}
