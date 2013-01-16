#ifndef GUI_MESHSET_H
#define GUI_MESHSET_H

#include <geom/Collection.h>
#include <geom/Mesh.h>

class MeshSet : public Collection<Mesh*>
{
public:
    static MeshSet& instance();
    ~MeshSet();

private:
    MeshSet();
    MeshSet(const MeshSet& other);
    MeshSet& operator =(const MeshSet& rhs);
};

#endif // GUI_MESHSET_H
