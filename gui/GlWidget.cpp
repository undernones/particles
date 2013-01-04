#include "GlWidget.h"
#include <GLUT/glut.h>
#include <QtGui/QMouseEvent>
#include <physics/SoftBody.h>
#include <iostream>

using Eigen::Vector3d;

namespace
{

const double SPHERE_SIZE = 0.015;
const double WIRE_OFFSET = 0.0005;
const uint32_t MAX_UINT = std::numeric_limits<uint32_t>::max();

Vector3d rainbow[] = {
    Vector3d(1.00, 0.00, 0.00), // Red
    Vector3d(1.00, 0.50, 0.00), // Orange
    Vector3d(1.00, 1.00, 0.00), // Yellow
    Vector3d(0.00, 1.00, 0.00), // Green
    Vector3d(0.00, 0.00, 1.00), // Blue
    Vector3d(0.29, 0.00, 0.51), // Indigo
    Vector3d(0.55, 0.00, 1.00), // Violet
};

}

GlWidget::GlWidget(QWidget* parent) :
    QGLWidget(parent),
    mRotX(0.f), mRotY(0.f),
    mSelected(MAX_UINT),
    mBody(NULL)
{
    mLightPos[0] = 0.0f;
    mLightPos[1] = 2.0f;
    mLightPos[2] = 9.0f;
    mLightPos[3] = 1.0f;

    mLightColor[0] = 1.f;
    mLightColor[1] = 1.f;
    mLightColor[2] = 1.f;

    mLightAmb[0] = 0.09f;
    mLightAmb[1] = 0.09f;
    mLightAmb[2] = 0.09f;

    mEye = Vector3d(0, 0, 7);
    mLookAt = Vector3d(0, 0, 0);
}

GlWidget::~GlWidget()
{
}

void
GlWidget::initializeGL()
{
    glClearDepth(1.0f);

    glEnable(GL_CULL_FACE);
    glEnable(GL_POLYGON_SMOOTH);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_POINT_SMOOTH);
    glPointSize(5.0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // lighting

    glLightfv(GL_LIGHT0, GL_POSITION, mLightPos);
    glLightfv(GL_LIGHT0, GL_LINEAR_ATTENUATION, mLightPos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  mLightColor);
    glLightfv(GL_LIGHT0, GL_SPECULAR, mLightColor);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  mLightAmb);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    {
        GLfloat mat[3] = { 0.2f, 0.2f, 0.2f };
        glMaterialfv(GL_FRONT, GL_AMBIENT, mat);

        mat[0] = 0.7f; mat[1] = 0.7f; mat[2] = 0.7f;
        glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);

        mat[0] = 0.1f; mat[1] = 0.1f; mat[2] = 0.1f;
        glMaterialfv(GL_FRONT, GL_SPECULAR, mat);
        glMaterialf(GL_FRONT, GL_SHININESS, 128.0f);
    }

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, mLightAmb);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

    initSphereList();
}

void
GlWidget::resizeGL(int w, int h)
{
    if (h == 0) h = 1;

    glViewport(0, 0, (GLsizei)w, (GLsizei)h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLdouble)w / (GLdouble)h, 0.01, 1000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(mEye[0], mEye[1], mEye[2], mLookAt[0], mLookAt[1], mLookAt[2], 0, 1, 0);
}

void
GlWidget::paintGL()
{
    glClearColor(0.03f, 0.03f, 0.03f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(mEye[0], mEye[1], mEye[2], mLookAt[0], mLookAt[1], mLookAt[2], 0, 1, 0);

    glRotatef(mRotX, 1.0f, 0, 0);
    glRotated(mRotY, 0, 1.0f, 0);

    renderBody();

    glFlush();
}

void
GlWidget::mousePressEvent(QMouseEvent* event)
{
    mLastMouse = event->pos();

    if ((event->buttons() & Qt::LeftButton) && !(event->modifiers() & Qt::ControlModifier)) {
        // Select a particle
        uint32_t selected = select(mLastMouse);
        select(selected);
    }
}

void
GlWidget::mouseReleaseEvent(QMouseEvent*)
{
}

void
GlWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPoint dp = event->pos() - mLastMouse;

    double scale, len, theta;
    Vector3d neye, neye2;
    Vector3d f, r, u;

    if ((event->buttons() & Qt::LeftButton) && (event->modifiers() & Qt::ControlModifier)) {
        // translate
        f = mLookAt - mEye;
        u = Vector3d(0, 1, 0);

        // scale the change by how far away we are
        scale = sqrt(f.norm()) * 0.007;

        r = f.cross(u).normalized();
        u = r.cross(f).normalized();

        mEye += (-r * dp.x() * scale) + (u * dp.y() * scale);
        mLookAt += (-r * dp.x() * scale) + (u * dp.y() * scale);

    } else if (event->buttons() & Qt::MidButton) {
        zoom(dp.x());

    } else if (event->buttons() & Qt::RightButton) {
        // rotate

        neye = mEye - mLookAt;

        // first rotate in the x/z plane
        theta = -dp.x() * 0.007;
        neye2 = Vector3d(
            cos(theta) * neye[0] + sin(theta) * neye[2],
            neye[1],
            -sin(theta) * neye[0] + cos(theta) * neye[2]
        );


        // now rotate vertically
        theta = -dp.y() * 0.007;

        f = -neye2;
        u = Vector3d(0, 1, 0);
        r = f.cross(u).normalized();
        u = r.cross(f).normalized();
        len = f.norm();
        f.normalize();

        neye = len * (((float)cos(theta) * f) + ((float)sin(theta) * u));

        mEye = mLookAt - neye;
    }

    mLastMouse = event->pos();

    this->repaint();
}

void
GlWidget::wheelEvent(QWheelEvent* event)
{
    int delta = event->delta();
    zoom(delta);
    this->repaint();
}

void
GlWidget::initSphereList()
{
    mSphereSolid = glGenLists(2);
    mSphereWire = mSphereSolid + 1;

    glNewList(mSphereSolid, GL_COMPILE);
    glutSolidSphere(SPHERE_SIZE, 20, 20);
    glEndList();

    glNewList(mSphereWire, GL_COMPILE);
    glutWireSphere(SPHERE_SIZE + WIRE_OFFSET, 20, 20);
    glEndList();
}

void
GlWidget::zoom(int delta)
{
    Vector3d f = mLookAt - mEye;
    double len = f.norm();
    f.normalize();

    // scale the change by how far away we are
    len -= sqrt(len) * delta * 0.01;

    mEye = mLookAt - (len * f);

    // make sure the eye and lookat points are sufficiently far away
    // push the lookat point forward if it is too close
    if (len < 1) {
        mLookAt = mEye + f;
    }
}
 
void
GlWidget::renderBody() const
{
    if (mBody == NULL) return;

    bool hasSelected = mSelected < mBody->size();

    // Draw points
    uint32_t index = 0;
    for (auto p : mBody->posWorld) {
        Eigen::Vector3d color = rainbow[index % 7];
        if (hasSelected) {
            color *= 0.2;
        }
        glColor3d(color[0], color[1], color[2]);

        glPushMatrix();

        glTranslated(p[0], p[1], p[2]);
        glCallList(mSphereSolid);

        if (index == mSelected) {
            glDisable(GL_LIGHTING);
            glColor4d(1, 1, 1, 1);
            glCallList(mSphereWire);
            glEnable(GL_LIGHTING);
        }

        if (false) { // TODO: view forces
            const Vector3d& force = mBody->forces[index];
            glDisable(GL_LIGHTING);
            if (index == mSelected)
                glLineWidth(3);
            glBegin(GL_LINES);
            glVertex3d(0, 0, 0);
            glVertex3d(force[0], force[1], force[2]);
            glEnd();
            glEnable(GL_LIGHTING);
            glLineWidth(1);
            glEnable(GL_LIGHTING);
        }

        glPopMatrix();
        index++;
    }

    if (hasSelected && true) { // TODO: view neighborhoods
        const Vector3d& basePos = mBody->posWorld[mSelected];

        glDisable(GL_LIGHTING);
        for (auto& n : mBody->neighborhoods[mSelected]) {
            const Vector3d& x = mBody->posWorld[n.index];

            glPushMatrix();
            glTranslated(x[0], x[1], x[2]);
            glColor3d(1, 1, 0.6);
            glCallList(mSphereWire);
            glPopMatrix();
        }
        glEnable(GL_LIGHTING);

        /*
        BOOST_FOREACH (const Neighbor &neighbor, particles[mSelectedIndex].neighbors) {
            const Particle &n = particles[neighbor.index];
            const SlVector3 &pos = (mSpace == World) ? n.worldPos : n.materialPos;

            glPushMatrix();
            glTranslated(pos[0], pos[1], pos[2]);
            glColor3d(1, 1, 0.6);
            glCallList(mSphereWire);
            glPopMatrix();

            if (mVectors != ParticleViewer::NoVectors) {
                glLineWidth(2);
                SlVector3 neighborLoc = basePos + neighbor.u;
                glColor3d(1, 0, 0);
                glBegin(GL_LINES);
                glVertex3d(basePos[0], basePos[1], basePos[2]);
                glVertex3d(neighborLoc[0], neighborLoc[1], neighborLoc[2]);
                glEnd();
            }
        }
        */
    }
}

uint32_t
GlWidget::select(const QPoint& clickPt) const
{
    // http://nehe.gamedev.net/data/articles/article.asp?article=13

    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];
    GLfloat winX, winY, winZ;
    Vector3d pos;

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    winX = (GLfloat)clickPt.x();
    winY = (GLfloat)viewport[3] - (GLfloat)clickPt.y();
    glReadPixels(clickPt.x(), (GLint)winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    gluUnProject(winX, winY, winZ, modelview, projection, viewport, &pos[0], &pos[1], &pos[2]);

    uint32_t index = 0;
    for (auto x : mBody->posWorld) {
        if ((x - pos).norm() < SPHERE_SIZE + WIRE_OFFSET) {
            return index;
        }
        index++;
    }

    return MAX_UINT;
}

bool
GlWidget::select(uint32_t index)
{
    if (mBody == NULL) return false;

    if (index != mSelected) {
        mSelected = index;
        this->repaint();
        emit selectionChanged(this);
    }
    return index < mBody->size();
}
