#ifndef QT_GLWIDGET_H
#define QT_GLWIDGET_H

// What an ugly pain all this is! Not sure what to do about it, though.
#ifdef __clang__
#pragma clang diagnostic push
#if defined(__has_warning) && __has_warning("-Wunused-private-field")
#pragma clang diagnostic ignored "-Wunused-private-field"
#endif
#endif
#include <QtGui/QMouseEvent>
#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <QtOpenGL/QGLWidget>
#include <Eigen>

class Mesh;
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

    static void renderMesh(const Mesh& mesh);

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
