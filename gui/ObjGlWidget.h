#ifndef GUI_OBJGLWIDGET_H
#define GUI_OBJGLWIDGET_H

#include "GlWidget.h"

class Mesh;
class ObjGlWidget : public GlWidget
{
    Q_OBJECT

public:
    explicit ObjGlWidget(QWidget* parent = nullptr);
    virtual ~ObjGlWidget();

    void setMesh(const Mesh* mesh) { mMesh = mesh; }

protected:
    virtual void renderAll();

private:
    const Mesh* mMesh;
};

#endif // GUI_OBJGLWIDGET_H
