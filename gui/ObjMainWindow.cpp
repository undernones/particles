#include "ObjMainWindow.h"
#include "ui_ObjMainWindow.h"

ObjMainWindow::ObjMainWindow(QWidget* parent)
:    QMainWindow(parent)
,    ui(new Ui_ObjMainWindow)
{
    ui->setupUi(this);
}

ObjMainWindow::~ObjMainWindow()
{
    delete ui;
}

void
ObjMainWindow::paused()
{
    ui->actionNext->setEnabled(true);
}

void
ObjMainWindow::resumed()
{
    ui->actionNext->setEnabled(false);
}

void
ObjMainWindow::stepped()
{
    ui->glWidget->update();

    //// Only update the matrix viewer if we are stepping interactively.
    //if (SimThread::instance().isPaused()) {
    //    ui->matrixViewer->refresh();
    //}
}
