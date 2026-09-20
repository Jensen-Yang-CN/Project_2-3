#ifndef DIALOGDEVICEMANAGER_H
#define DIALOGDEVICEMANAGER_H

#include <QDialog>

namespace Ui {
class DialogDeviceManager;
}

class DialogDeviceManager : public QDialog
{
    Q_OBJECT

public:
    explicit DialogDeviceManager(QWidget *parent = nullptr);
    ~DialogDeviceManager();

private:
    Ui::DialogDeviceManager *ui;
};

#endif // DIALOGDEVICEMANAGER_H
