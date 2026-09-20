#ifndef DATAPANEL_H
#define DATAPANEL_H

#include <QWidget>

namespace Ui {
class DataPanel;
}

class DataPanel : public QWidget
{
    Q_OBJECT

public:
    explicit DataPanel(QWidget *parent = nullptr);
    ~DataPanel();
    // 设置面板标题和图标的接口
    void setPanelInfo(const QString &title, const QString &iconPath = "");
    void setCentralWidget(QWidget *widget);
    void resizeEvent(QResizeEvent *event);
private:
    Ui::DataPanel *ui;
};

#endif // DATAPANEL_H
