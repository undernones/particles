#include "ObjGlWidget.h"

ObjGlWidget::ObjGlWidget(QWidget* parent)
:   GlWidget(parent)
,   mMesh(nullptr)
{
}

ObjGlWidget::~ObjGlWidget()
{
}

void
ObjGlWidget::renderAll()
{
    if (mMesh != nullptr) {
        renderMesh(*mMesh);
    }
}
