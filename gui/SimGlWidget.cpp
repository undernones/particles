#include "SimGlWidget.h"
#include <GLUT/glut.h>
#include <geom/Mesh.h>
#include <physics/SoftBody.h>

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

void
renderMesh(const Mesh& mesh)
{
    glColor3d(1, 1, 1);
    glBegin(GL_TRIANGLES);
    auto n_it = mesh.normals().begin();
    for (auto f : mesh.faces()) {
        glNormal3dv(n_it->data());
        glVertex3dv(mesh.verts()[f[0]].data());
        glVertex3dv(mesh.verts()[f[1]].data());
        glVertex3dv(mesh.verts()[f[2]].data());
        ++n_it;
    }
    glEnd();
}

}

SimGlWidget::SimGlWidget(QWidget* parent)
:   GlWidget(parent)
,   mBody(NULL)
,   mSphereSolid(MAX_UINT)
,   mSphereWire(MAX_UINT)
,   mSelected(MAX_UINT)
{
}

SimGlWidget::~SimGlWidget()
{
}

void
SimGlWidget::renderAll()
{
    renderBody();
}

void
SimGlWidget::initializeGL()
{
    GlWidget::initializeGL();
    initSphereList();
}

void
SimGlWidget::initSphereList()
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
SimGlWidget::mousePressEvent(QMouseEvent* event)
{
    GlWidget::mousePressEvent(event);

    if ((event->buttons() & Qt::LeftButton) && !(event->modifiers() & Qt::ControlModifier)) {
        // Select a particle
        uint32_t selected = select(mLastMouse);
        select(selected);
    }
}
 
void
SimGlWidget::renderBody() const
{
    if (mBody == NULL) return;

    bool hasSelected = mSelected < mBody->size();

    // Draw points
    uint32_t index = 0;
    for (auto& p : mBody->posWorld) {
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
        //const Vector3d& basePos = mBody->posWorld[mSelected];

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

    if (true && mBody->hasMesh()) { // TODO: show mesh
        renderMesh(*mBody->mesh());
    }
}

uint32_t
SimGlWidget::select(const QPoint& clickPt) const
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
SimGlWidget::select(uint32_t index)
{
    if (mBody == NULL) return false;

    if (index != mSelected) {
        mSelected = index;
        this->repaint();
        emit selectionChanged(this);
    }
    return index < mBody->size();
}
