#ifndef GUI_OBJGLWIDGET_H
#define GUI_OBJGLWIDGET_H

#include "GlWidget.h"

class ObjGlWidget : public GlWidget
{
    Q_OBJECT

public:
    explicit ObjGlWidget(QWidget* parent = nullptr);
    virtual ~ObjGlWidget();
};

#endif // GUI_OBJGLWIDGET_H
