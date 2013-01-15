#ifndef QT_GLWIDGET_H
#define QT_GLWIDGET_H

#include <QtGui/QMouseEvent>
#include <QtOpenGL/QGLWidget>
#include <Eigen>

class GlWidget : public QGLWidget
{
    Q_OBJECT

public:
    explicit GlWidget(QWidget* parent = NULL);
    virtual ~GlWidget();

protected:
    QPoint mLastMouse;

    virtual void renderAll()
    {
        // Do nothing. Let the kids figure it out.
    }

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    virtual void mousePressEvent(QMouseEvent*);
    virtual void mouseReleaseEvent(QMouseEvent*);
    virtual void mouseMoveEvent(QMouseEvent*);
    virtual void wheelEvent(QWheelEvent*);

private:
    GLfloat mLightPos[4];
    GLfloat mLightAmb[3];
    GLfloat mLightColor[3];

    Eigen::Vector3d mEye;
    Eigen::Vector3d mLookAt;

    GLfloat mRotX;
    GLfloat mRotY;

    void zoom(int delta);
};

#endif // QT_GLWIDGET_H
