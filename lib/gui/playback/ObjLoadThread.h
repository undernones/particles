#ifndef GUI_OBJLOADTHREAD_H
#define GUI_OBJLOADTHREAD_H

#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>

class ObjLoadThread : public QThread
{
    Q_OBJECT

public:
    static ObjLoadThread& instance();
    ~ObjLoadThread();

    void init(const std::string& dir);

protected:
    void run();

private:
    bool mQuitFlag;
    std::string mDir;

    explicit ObjLoadThread(QObject* parent = NULL);
    ObjLoadThread(const ObjLoadThread&);

signals:
    void frameLoaded();

public slots:
    void quit();
};

#endif // GUI_OBJLOADTHREAD_H
