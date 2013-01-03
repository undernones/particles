#ifndef QT_GLWIDGET_H
#define QT_GLWIDGET_H

#include <QtOpenGL/QGLWidget>
#include <Eigen>

class SoftBody;
class GlWidget : public QGLWidget
{
    Q_OBJECT

public:
    explicit GlWidget(QWidget* parent = NULL);
    ~GlWidget();

    void setBody(const SoftBody* body) { mBody = body; }
    bool select(uint32_t index);

protected:
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
    QPoint mLastMouse;

    GLfloat mRotX;
    GLfloat mRotY;

    GLuint mSphereSolid;
    GLuint mSphereWire;
    uint32_t mSelected;

    const SoftBody* mBody;

    void initSphereList();
    void zoom(int delta);

    void renderBody() const;
    uint32_t select(const QPoint& clickPt) const;

signals:
    void selectionChanged(const GlWidget* sender) const;
};

#endif // QT_GLWIDGET_H
