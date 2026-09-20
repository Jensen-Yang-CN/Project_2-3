#ifndef ADDDEVICEDIALOG_H
#define ADDDEVICEDIALOG_H

#include <QDialog>
#include "DeviceConfig.h"
namespace Ui {
class AddDeviceDialog;
}

class AddDeviceDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AddDeviceDialog(QWidget *parent = nullptr);
    ~AddDeviceDialog();

    device::DeviceInfo getDeviceInfo() const;
private slots:
    void onTypeChanged(int index);
private:
    Ui::AddDeviceDialog *ui;
};

#endif // ADDDEVICEDIALOG_H
