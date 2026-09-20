#include "dialogdevicemanager.h"
#include "ui_dialogdevicemanager.h"

DialogDeviceManager::DialogDeviceManager(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogDeviceManager)
{
    ui->setupUi(this);
}

DialogDeviceManager::~DialogDeviceManager()
{
    delete ui;
}
