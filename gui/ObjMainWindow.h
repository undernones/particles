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
    int mTimer;

    void refresh();
    void updateStatusBar();

public slots:
    void rewind();
    void togglePlayPause(bool toggled);
    void nextTimestep();
    void prevTimestep();
    void frameLoaded();
};

#endif // QT_OBJMAINWINDOW_H
