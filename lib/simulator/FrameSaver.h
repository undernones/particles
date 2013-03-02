#ifndef GUI_FRAMESAVER_H
#define GUI_FRAMESAVER_H

#include <QtCore/QObject>

class FrameSaver : public QObject
{
    Q_OBJECT

public:
    explicit FrameSaver();
    ~FrameSaver();

private:
    double mElapsed;
    double mSpf;
    uint32_t mFrame;

signals:
    void savedFrame(uint32_t which);

public slots:
    void stepped();
};

#endif // GUI_FRAMESAVER_H
