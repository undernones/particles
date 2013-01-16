#ifndef GUI_MESHSET_H
#define GUI_MESHSET_H

#include <QtCore/QObject>
#include <geom/Collection.h>
#include <geom/Mesh.h>

class MeshSet : public QObject, public Collection<Mesh*>
{
    Q_OBJECT

public:
    static MeshSet& instance();
    ~MeshSet();

    const Mesh* current() const;
    inline uint32_t currentFrame() const { return mCurrent; }
    inline bool next() { return (mCurrent < size() - 1) && goTo(mCurrent + 1); }
    inline bool prev() { return (mCurrent > 0) && goTo(mCurrent - 1); }
    void rewind();
    bool goTo(uint32_t frame);

private:
    uint32_t mCurrent;

    MeshSet();
    MeshSet(const MeshSet& other);
    MeshSet& operator =(const MeshSet& rhs);

signals:
    void frameChanged() const;
    void frameCountChanged() const;
};

#endif // GUI_MESHSET_H
