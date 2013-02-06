#include "SimMainWindow.h"
#include "ui_SimMainWindow.h"
#include <physics/SoftBody.h>
#include <simulator/SimThread.h>

SimMainWindow::SimMainWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui_SimMainWindow)
{
    ui->setupUi(this);

    SimThread& thread(SimThread::instance());

    connect(ui->actionPlayPause, SIGNAL(triggered()), &thread, SLOT(togglePausedState()));
    connect(ui->actionNext, SIGNAL(triggered()), &thread, SLOT(step()));

    connect(&thread, SIGNAL(paused()), SLOT(paused()));
    connect(&thread, SIGNAL(resumed()), SLOT(resumed()));

    ui->actionNext->setEnabled(thread.isPaused());
    ui->actionPlayPause->setChecked(thread.isPaused());
}

SimMainWindow::~SimMainWindow()
{
    delete ui;
}

void
SimMainWindow::setSoftBody(const SoftBody* body)
{
    const Eigen::Matrix3d* matrix = NULL;

    if (body != NULL) {
        matrix = &body->bases[0];
    }

    ui->glWidget->setBody(body);
    ui->matrixViewer->setMatrix(matrix);
}

void
SimMainWindow::paused()
{
    ui->actionNext->setEnabled(true);
}

void
SimMainWindow::resumed()
{
    ui->actionNext->setEnabled(false);
}

void
SimMainWindow::stepped()
{
    ui->glWidget->update();

    // Only update the matrix viewer if we are stepping interactively.
    if (SimThread::instance().isPaused()) {
        ui->matrixViewer->refresh();
    }
}
