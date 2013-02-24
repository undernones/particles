// Copyright (c) 2011, Regents of the University of Utah
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GEOM_KDTREE_H
#define GEOM_KDTREE_H

#include <vector>
#include <Eigen>
#include "VectorList.h"

class IntDoublePair
{
private:
	int _i;
	double _d;

public:
	IntDoublePair(int i, double d): _i(i), _d(d){}
	IntDoublePair() {}
	IntDoublePair &operator=(IntDoublePair id) {_i = id._i, _d = id._d; return *this;}
	inline int i() const {return _i;}
	inline int &i() {return _i;}
	inline double d() const {return _d;}
	inline double &d() {return _d;}
};
	
class KdTree
{
private:
    typedef Eigen::Vector3d Point;
    typedef VectorList PointList;

public:
	// Initialize kdtree for point in pts.  The points are *not* duplicated.
    // The tree only stores indices into this set of points.
	KdTree(const PointList& points);
	~KdTree() {delete [] tree; delete [] distCache;}

	// Return the num nearest neighbors within distance r of the query point.
    // If r <= 0, returns the num nearest neighbors.
	// If num <= 0 returns all points within a sphere of radius r of the query point.
	void neighbors(const PointList& points, const Point& p, int num,
                   double r, std::vector<unsigned>& neighbors);

	// Returns the nearest neighbor.  If r > 0 neighbor must be within distance
    // distance r, otherwise returns -1.
	int neighbor(const PointList& points, const Point& x, double r);

private:
	int* tree;
	int npts;
	double* distCache;
	void xsplit(const PointList& points,
                std::vector<IntDoublePair>& xSortedList,
                std::vector<IntDoublePair>& ySortedList,
                std::vector<IntDoublePair>& zSortedList,
                std::vector<IntDoublePair>& scratch, int start, int end);
	void ysplit(const PointList& points,
                std::vector<IntDoublePair>& xSortedList,
                std::vector<IntDoublePair>& ySortedList,
                std::vector<IntDoublePair>& zSortedList,
                std::vector<IntDoublePair>& scratch, int start, int end);
	void zsplit(const PointList& points,
                std::vector<IntDoublePair>& xSortedList,
                std::vector<IntDoublePair>& ySortedList,
                std::vector<IntDoublePair>& zSortedList,
                std::vector<IntDoublePair>& scratch, int start, int end);

	void neighborsRecurse(const PointList& points, const Point& p,
                          unsigned int num, 
                          double r, double r2,
                          std::vector<unsigned>& neighbors,
                          int start, int end, int plane);

	void neighborsRecurse(const PointList& points, const Point& p,
                          unsigned int num, 
                          std::vector<unsigned>& neighbors,
                          int start, int end, int plane);

	void neighborsRecurse(const PointList& points, const Point& p, 
                          double r, double r2,
                          std::vector<unsigned>& neighbors,
                          int start, int end, int plane);

	void neighborRecurse(const PointList& points, const Point& p,
                         double r, double r2,
                         int& neighbor, double& ndist,
                         int start, int end, int plane);

	void neighborRecurse(const PointList& points, const Point& p, 
                         int& neighbor, double& ndist,
                         int start, int end, int plane);
};

#endif // GEOM_KDTREE_H
