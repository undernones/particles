#ifndef GEOM_COLLECTION_H
#define GEOM_COLLECTION_H

// TODO: Move this into a utils lib?

#include <vector>

template <class T>
class Collection
{
private:
    typedef std::vector<T> TCollection;
    TCollection mItems;

public:
    explicit Collection()
    {
    }

    explicit Collection(const Collection& other)
    {
        mItems = other.mItems;
    }

    virtual ~Collection()
    {
    }

    typedef typename TCollection::iterator iterator;
    typedef typename TCollection::const_iterator const_iterator;
    typedef typename TCollection::reverse_iterator reverse_iterator;
    typedef typename TCollection::const_reverse_iterator const_reverse_iterator;

    inline iterator               begin()         { return mItems.begin(); }
    inline const_iterator         begin()   const { return mItems.begin(); }
    inline iterator               end()           { return mItems.end(); }
    inline const_iterator         end()     const { return mItems.end(); }

    inline reverse_iterator       rbegin()        { return mItems.rbegin(); }
    inline const_reverse_iterator rbegin()  const { return mItems.rbegin(); }
    inline reverse_iterator       rend()          { return mItems.rend(); }
    inline const_reverse_iterator rend()    const { return mItems.rend(); }

    inline unsigned size()     const { return mItems.size(); }
    inline unsigned max_size() const { return mItems.max_size(); }
    inline unsigned capacity() const { return mItems.capacity(); }
    inline bool empty()        const { return mItems.empty(); }
    inline void reserve(unsigned n)  { mItems.reserve(n); }

    inline T&       operator[](unsigned n)       { return mItems[n]; }
    inline const T& operator[](unsigned n) const { return mItems[n]; }
    inline T&       at(unsigned n)               { return mItems.at(n); }
    inline const T& at(unsigned n) const         { return mItems.at(n); }

    inline T&       front()       { return mItems.front(); }
    inline const T& front() const { return mItems.front(); }
    inline T&       back()        { return mItems.back(); }
    inline const T& back() const  { return mItems.back(); }

    inline void push_back(const T &p) { mItems.push_back(p); }
    inline void clear() { mItems.clear(); }

    inline iterator insert(iterator position, const T& p)
    { return mItems.insert(position, p); }

    template<typename _InputIterator>
    inline void insert(iterator position, _InputIterator first, _InputIterator last)
    { mItems.insert(position, first, last); }
};

#endif // GEOM_COLLECTION_H
