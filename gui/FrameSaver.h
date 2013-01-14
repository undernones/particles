#ifndef GUI_FRAMESAVER_H
#define GUI_FRAMESAVER_H

#include <QtCore/QObject>

class SoftBody;
class FrameSaver : public QObject
{
    Q_OBJECT

public:
    explicit FrameSaver(const SoftBody& body, const std::string& framesDir, double dt, double fps);
    ~FrameSaver();

private:
    const SoftBody& mBody;
    std::string mFramesDir;
    double mDt;
    double mFps;
    double mElapsed;
    double mSpf;
    uint32_t mFrame;

public slots:
    void stepped();
};

#endif // GUI_FRAMESAVER_H
