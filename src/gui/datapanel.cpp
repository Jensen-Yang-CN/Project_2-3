#include "datapanel.h"
#include "ui_datapanel.h"
#include <QLayout>
#include <QVBoxLayout>
#include <QResizeEvent>
DataPanel::DataPanel(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DataPanel)
{
    ui->setupUi(this);

}

DataPanel::~DataPanel()
{
    delete ui;
}

void DataPanel::setPanelInfo(const QString &title, const QString &iconPath)
{
    // 假设你在 datapanel.ui 里放了一个叫 labelTitle 的 QLabel
    //    ui->labelTitle->setText(title);
    ui->groupBox->setTitle(title);
    if(!iconPath.isEmpty()){
        return;
    }
    //    // 如果有图标，可以设置给另一个 labelIcon
    //    if(!iconPath.isEmpty()){
    //        ui->labelIcon->setPixmap(QPixmap(iconPath));
    //     }
}

void DataPanel::setCentralWidget(QWidget *widget)
{

    // 获取 groupBox 的布局
    QLayout *layout = ui->groupBox->layout();

    if (layout) {
        // 如果设计器里已经给 groupBox 设了布局，直接加进去
        layout->addWidget(widget);
    } else {
        // 如果没有，手动创建一个垂直布局给 groupBox
        QVBoxLayout *newLayout = new QVBoxLayout(ui->groupBox);
        newLayout->setContentsMargins(0, 0, 0, 0);
        newLayout->addWidget(widget);
    }
    // 确保子控件会自动拉伸
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

}

void DataPanel::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    if (ui->groupBox) {
        // 强制让 groupBox 铺满整个 DataPanel 区域
        ui->groupBox->setGeometry(0, 0, this->width(), this->height());
    }
}
