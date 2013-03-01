#ifndef GUI_SIMGLWIDGET_H
#define GUI_SIMGLWIDGET_H

#include <gui/GlWidget.h>

class SoftBody;
class VectorList;
class SimGlWidget : public GlWidget
{
    Q_OBJECT

public:
    typedef enum { World, Rest } Space;

    explicit SimGlWidget(QWidget* parent = nullptr);
    virtual ~SimGlWidget();

    void setBody(const SoftBody* body);
    void setSpace(Space space) { mSpace = space; }
    bool select(uint32_t index);

protected:
    virtual void renderAll();
    virtual void initializeGL();
    virtual void mousePressEvent(QMouseEvent*);

private:
    const SoftBody* mBody;
    Space mSpace;
    const VectorList* mPositions;

    GLuint mSphereSolid;
    GLuint mSphereWire;
    uint32_t mSelected;

    void initSphereList();

    void renderBody() const;
    uint32_t select(const QPoint& clickPt) const;

signals:
    void selectionChanged(const GlWidget* sender) const;
};

#endif // GUI_SIMGLWIDGET_H
