#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <physics/SoftBody.h>
#include "SimThread.h"

MainWindow::MainWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui_MainWindow)
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

MainWindow::~MainWindow()
{
    delete ui;
}

void
MainWindow::setSoftBody(const SoftBody* body)
{
    const Eigen::Matrix3d* matrix = NULL;

    if (body != NULL) {
        matrix = &body->strains[0];
    }

    ui->glWidget->setBody(body);
    ui->matrixViewer->setMatrix(matrix);
}

void
MainWindow::paused()
{
    ui->actionNext->setEnabled(true);
}

void
MainWindow::resumed()
{
    ui->actionNext->setEnabled(false);
}

void
MainWindow::stepped()
{
    ui->glWidget->update();

    // Only update the matrix viewer if we are stepping interactively.
    if (SimThread::instance().isPaused()) {
        ui->matrixViewer->refresh();
    }
}
