#include "ObjMainWindow.h"
#include "ui_ObjMainWindow.h"
#include "MeshSet.h"
#include "ObjLoadThread.h"
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
ObjMainWindow::timerEvent(QTimerEvent* event)
{
    nextFrame();
}

void
ObjMainWindow::closeEvent(QCloseEvent* event)
{
    ObjLoadThread::instance().quit();
    ObjLoadThread::instance().wait();
}

void
ObjMainWindow::refresh()
{
    updateStatusBar();

    ui->glWidget->setMesh(MeshSet::instance().current());
    ui->glWidget->repaint();
}

void
ObjMainWindow::updateStatusBar()
{
    uint32_t current = MeshSet::instance().currentFrame() + 1;
    uint32_t total = MeshSet::instance().size();
    statusBar()->showMessage(QString("%1 / %2 Frames").arg(current).arg(total));
}

void
ObjMainWindow::rewind()
{
    MeshSet::instance().rewind();
    prevFrame();
}

void
ObjMainWindow::togglePlayPause(bool toggled)
{
    if (toggled) {
        killTimer(mTimer);
    } else {
        mTimer = startTimer(33); // 30 fps
    }
    ui->actionNext->setEnabled(toggled);
    ui->actionPrevious->setEnabled(toggled);
}

void
ObjMainWindow::nextFrame()
{
    MeshSet::instance().next();
    refresh();
}

void
ObjMainWindow::prevFrame()
{
    MeshSet::instance().prev();
    refresh();
}

void
ObjMainWindow::frameLoaded()
{
    static bool first = true;
    if (first) {
        MeshSet::instance().rewind();
        first = false;
    }

    refresh();
}
