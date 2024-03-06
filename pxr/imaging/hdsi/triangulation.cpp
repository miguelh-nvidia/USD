#include <pxr/pxr.h>

#include "triangulation.h"

PXR_NAMESPACE_OPEN_SCOPE

// FacePoint

FacePoint::FacePoint(const GfVec3f& p,
    const GfVec3f& q,
    const GfVec3f& r) :
    _p(p),
    _q(q),
    _r(r),
    _direction(GfCross(p - q, r - q))
{
}

FacePoint::FacePoint(FacePoint const& other) :
    _p(other._p),
    _q(other._q),
    _r(other._r),
    _direction(other._direction)
{
}

bool
FacePoint::HasSameOrientation(const FacePoint& other) const
{
    auto const value = GfDot(_direction, other._direction);
    if (value < 0)
    {
        return false;
    }
    return true;
}

bool
FacePoint::HasPoint(const GfVec3f& point) const
{
    return _p == point || _q == point || _r == point;
}


bool
FacePoint::ContainsPoint(const GfVec3f& s) const
{
    return
        GfDot(GfCross(_q - _p, _r - _p), GfCross(_q - _p, s - _p)) > 0 &&
        GfDot(GfCross(_r - _q, _p - _q), GfCross(_r - _q, s - _q)) > 0 &&
        GfDot(GfCross(_p - _r, _q - _r), GfCross(_p - _r, s - _r)) > 0;
}

// Face

Face::Face(const VtVec3fArray& points,
    const VtIntArray& indices,
    const size_t indexStart,
    const size_t vertexCount) :
    _points(points),
    _indices(indices),
    _indexStart(indexStart),
    _vertexCount(vertexCount)
{
}

FacePoint
Face::GetReferencePoint() const
{
    size_t index = 0;
    for (size_t i = 1; i < size(); ++i)
    {
        const GfVec3f& p = (*this)[i];
        const GfVec3f& q = (*this)[index];
        if (std::make_tuple(p[0], p[1], p[2]) < 
            std::make_tuple(q[0], q[1], q[2])) {
            index = i;
        }
    }
    return FacePoint(
        (*this)[(index + size() - 1) % size()],
        (*this)[index],
        (*this)[(index + 1) % size()]
    );
}

bool
Face::IsConvex() const
{
    if (size() <= 3)
    {
        return true;
    }
    const FacePoint reference((*this)[size() - 1], (*this)[0], (*this)[1]);
    for (size_t i = 1; i < size(); ++i)
    {
        const size_t prevIndex = i - 1;
        const size_t nextIndex = (i + 1) % size();
        const FacePoint facePoint((*this)[prevIndex], (*this)[i], (*this)[nextIndex]);
        if (!facePoint.HasSameOrientation(reference))
        {
            return false;
        }
    }
    return true;
}

bool
Face::IsEarClipping(const FacePoint& reference, VtIntArray& perm, VtIntArray& faceCount) const
{
    // Create a double linked list
    std::vector<size_t> prevIndex(size(), 0);
    std::vector<size_t> nextIndex(size(), 0);
    for (size_t i = 0; i < size(); ++i)
    {
        prevIndex[i] = (i + size() - 1) % size();
        nextIndex[i] = (i + 1) % size();
    }

    // Find out if a point is convex
    const auto IsConvex = [&](const size_t i)
    {
        const FacePoint facePoint((*this)[prevIndex[i]], (*this)[i], (*this)[nextIndex[i]]);
        return facePoint.HasSameOrientation(reference);
    };

    // Find all concave points
    std::set<size_t> concave;
    for (size_t i = 0; i < size(); ++i)
    {
        if (!IsConvex(i))
        {
            concave.insert(concave.end(), i);
        }
    }

    // An ear is a convex shape not containing any other point.
    const auto IsEar = [&](const size_t i)
    {
        if (!IsConvex(i))
        {
            return false;
        }
        const FacePoint facePoint((*this)[prevIndex[i]], (*this)[i], (*this)[nextIndex[i]]);
        for (size_t vertex : concave)
        {
            auto const& point = (*this)[vertex];
            if (facePoint.HasPoint(point))
            {
                continue;
            }
            if (facePoint.ContainsPoint(point))
            {
                return false;
            }
        }
        return true;
    };

    // Find all ears.
    std::set<size_t> ears;
    for (size_t i = 0; i < size(); ++i)
    {
        if (IsEar(i))
        {
            ears.insert(i);
        }
    }

    // Proceed cutting ears.
    int earCounter = 0;
    while (!ears.empty())
    {
        const auto it = ears.begin();
        const size_t i = *it;

        // The following lines is a way to merge triangles into polygons
        perm.push_back(prevIndex[i]);
        perm.push_back(i);
        perm.push_back(nextIndex[i]);
        faceCount.push_back(3);
        ++earCounter;

        // Remove ear
        ears.erase(it);
        nextIndex[prevIndex[i]] = nextIndex[i];
        prevIndex[nextIndex[i]] = prevIndex[i];

        std::vector<size_t> adjacent;
        // Validate reduction to point or line
        for (size_t j : {prevIndex[i], nextIndex[i]})
        {
            if (prevIndex[j] == nextIndex[j])
            {
                concave.erase(j);
                ears.erase(j);
            }
            else
            {
                adjacent.push_back(j);
            }
        }

        // Update concave -> convex
        for (size_t j : adjacent)
        {
            if (IsConvex(j))
            {
                concave.erase(j);
            }
        }

        // Update ear status
        for (size_t j : adjacent)
        {
            if (IsEar(j))
            {
                ears.insert(j);
            }
            else
            {
                ears.erase(j);
            }
        }
    }
    if (earCounter != size() - 2)
    {
        // Could not triangulate.
        return false;
    }

    return true;
}

bool
Face::Triangulate(VtIntArray& indices, VtIntArray& counts) const
{
    if (!IsValid())
    {
        return false;
    }

    if (IsConvex())
    {
        return false;
    }

    // Calculate a point which belongs to the convex hull
    // Useful for the following calculations
    FacePoint const reference = GetReferencePoint();

    VtIntArray permutation;
    VtIntArray relCounts;
    if (IsEarClipping(reference, permutation, relCounts))
    {
        for (size_t i = 0; i < permutation.size(); ++i)
        {
            const size_t index = _indices[_indexStart + permutation[i]];
            indices.push_back(index);
        }
        for (size_t i = 0; i < relCounts.size(); ++i)
        {
            counts.push_back(relCounts[i]);
        }
        return true;
    }

    return false;
}

// Triangulation

Triangulation::Triangulation(
    const VtVec3fArray& points,
    const VtIntArray& indices,
    const VtIntArray& vertexCount) :
    _points(points),
    _indices(indices),
    _vertexCount(vertexCount),
    _tessellations()
{
}

void
Triangulation::Triangulate()
{
    size_t indexStart = 0;
    for (size_t i = 0; i < _vertexCount.size(); ++i)
    {
        int const numberOfVertices = _vertexCount[i];

        Face const face(_points, _indices, indexStart, numberOfVertices);
        VtIntArray indices;
        VtIntArray counts;
        if (face.Triangulate(indices, counts))
        {
            HdMeshTessellation tessellation = {i, counts, indices};
            _tessellations.push_back(tessellation);
        }
        indexStart += numberOfVertices;
    }
}

PXR_NAMESPACE_CLOSE_SCOPE
