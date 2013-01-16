#ifndef QT_OBJMAINWINDOW_H
#define QT_OBJMAINWINDOW_H

#include <QtGui/QMainWindow>

class Ui_ObjMainWindow;
class ObjMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ObjMainWindow(QWidget* parent = NULL);
    ~ObjMainWindow();

private:
    Ui_ObjMainWindow* ui;

public slots:
    void paused();
    void resumed();
    void stepped();
};

#endif // QT_OBJMAINWINDOW_H
