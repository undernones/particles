#include "ObjMainWindow.h"
#include "ui_ObjMainWindow.h"
#include <iostream>

ObjMainWindow::ObjMainWindow(QWidget* parent)
:   QMainWindow(parent)
,   ui(new Ui_ObjMainWindow)
,   mTimer(-1)
{
    ui->setupUi(this);
}

ObjMainWindow::~ObjMainWindow()
{
    delete ui;
}

void
ObjMainWindow::refresh()
{
}

void
ObjMainWindow::updateStatusBar()
{
}

void
ObjMainWindow::rewind()
{
}

void
ObjMainWindow::togglePlayPause(bool toggled)
{
}

void
ObjMainWindow::nextTimestep()
{
}

void
ObjMainWindow::prevTimestep()
{
}

void
ObjMainWindow::frameLoaded()
{
}
