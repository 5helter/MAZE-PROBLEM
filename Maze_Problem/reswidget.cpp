#include "reswidget.h"
#include "ui_reswidget.h"

ResWidget::ResWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ResWidget)
{
    ui->setupUi(this);
    setWindowTitle("运行结果");
}

ResWidget::~ResWidget()
{
    delete ui;
}

void ResWidget::setAstar(int Pathcount, int opencount)
{
    ui->Acount->setText(QString::number(Pathcount));
    ui->Acount2->setText(QString::number(opencount));
}

void ResWidget::setBFs(int PAthcount, int Opencount)
{
    ui->c1->setText(QString::number(PAthcount));
    ui->c2->setText(QString::number(Opencount));
}

void ResWidget::setDFs(int PAthcount, int Opencount)
{
    ui->label_8->setText(QString::number(PAthcount));
    ui->label_11->setText(QString::number(Opencount));
}
