#ifndef LIVESIM_MAINWINDOW_H
#define LIVESIM_MAINWINDOW_H

#include <QtGui/QMainWindow>

class SoftBody;
class Ui_SimMainWindow;
class SimMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SimMainWindow(QWidget* parent = NULL);
    ~SimMainWindow();

    void setSoftBody(const SoftBody* body);

private:
    Ui_SimMainWindow* ui;

public slots:
    void paused();
    void resumed();
    void stepped();
};

#endif // LIVESIM_MAINWINDOW_H
