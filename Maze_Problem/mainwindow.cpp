#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("迷宫问题");
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    DisGLWidget::HMode=ui->comboBox_2->currentIndex();
    ui->coreWidget->run(DisGLWidget::AlgorithmType(ui->comboBox->currentIndex()),ui->checkBox->isChecked());
}


void MainWindow::on_pushButton_2_clicked()
{
    ui->coreWidget->setaccount(ui->spinBox->value());
    ui->coreWidget->setdensity(ui->spinBox_2->value());
    ui->coreWidget->reinitial();
}


void MainWindow::on_pushButton_3_clicked()
{
    if(mode==0)
    {
        ui->coreWidget->setMode(1);
        ui->pushButton_3->setText("编辑完成");
        mode=1;
    }
    else
    {
        ui->coreWidget->setMode(0);
        ui->pushButton_3->setText("编辑地图");
        mode=0;
    }

}

void MainWindow::on_pushButton_4_clicked()
{
    ui->coreWidget->reswidget.show();
}

