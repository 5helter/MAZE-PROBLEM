#ifndef RESWIDGET_H
#define RESWIDGET_H

#include <QWidget>

namespace Ui {
class ResWidget;
}

class ResWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ResWidget(QWidget *parent = nullptr);
    ~ResWidget();
    void setAstar(int Pathcount,int opencount);
    void setBFs(int PAthcount,int Opencount);
    void setDFs(int PAthcount,int Opencount);

private:
    Ui::ResWidget *ui;
};

#endif // RESWIDGET_H
