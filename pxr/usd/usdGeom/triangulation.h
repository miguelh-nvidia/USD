#ifndef PXR_USD_USD_GEOM_TRIANGULATION_H
#define PXR_USD_USD_GEOM_TRIANGULATION_H

#include "pxr/pxr.h"

#include "pxr/base/vt/array.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"

#include "pxr/usd/usdGeom/mesh.h"

PXR_NAMESPACE_OPEN_SCOPE

/// \class FacePoint
/// A face point is the temporary representation of a point in a face expressed by its adjacency
/// to its previous and next points. The class will compute all combinations of half planes of
/// those points in order to compute properties.
class FacePoint
{
public:
    /// Creates a face point with vertices (p, q, r). Where \p represents the previous index to \q
    /// and \r represents the next index to \p.
    FacePoint(const GfVec3f& p, const GfVec3f& q, const GfVec3f& r);

    FacePoint(FacePoint const& other);

    /// Computes the distance to the half plane <q, p, r> and checks whether the distance is
    /// positive or negative.
    bool HasSameOrientation(const FacePoint& other) const;

    /// Computes two half planes <s, q, p> and <s, q, r> and checks whether those two planes
    /// face each other.
    bool IsVisible(const GfVec3f& s) const;

    /// Whether the point is one of the points consisting this FacePoint.
    bool HasPoint(const GfVec3f& point) const;

    /// Computes three half planes <q, p, r>; <p, q, r>; <r, p, q> and checks whether a point lies
    /// in all of the three planes. Equivalent to find if a point lies within a triangle.
    bool ContainsPoint(const GfVec3f& s) const;
private:
    GfVec3f _p;
    GfVec3f _q;
    GfVec3f _r;
    GfVec3d _direction;
};

/// \class Face
/// A face is a representation of a Face in a Mesh. It consist of single public method: FanTriangulate.
class Face
{
public:
    Face(const VtVec3fArray& points,
        const VtIntArray& indices,
        const size_t indexStart,
        const size_t vertexCount);

    /// Returns true if it is possible to triangulate a face, false otherwise.
    /// If it is possible it will return an array of indices and faces to replace.
    bool FanTriangulate(VtIntArray& indices, VtIntArray& faces) const;
private:
    size_t size() const
    {
        return _vertexCount;
    }

    GfVec3f const& operator[](const size_t index) const
    {
        size_t offset = _indices[_indexStart + index];
        return _points[offset];
    }

    bool IsValid() const
    {
        return _vertexCount >= 3;
    }

    FacePoint GetReferencePoint() const;

    /// Returns true if the current face is a convex shape, false otherwise.
    /// Checking if a face is convex is a linear operation and if it is convex,
    /// the original indices and faces will not suffer modifications.
    bool IsConvex() const;

    /// Returns true if the current face is a star shaped, false otherwise.
    /// A polygon is star shaped if there is at least one point in the polygon from which all points are visible.
    /// Checking if a face is star shaped is a linear operation and if it is star shaped,
    /// the original indices will be shifted upon triangulation
    bool IsStarShaped(const FacePoint& reference, size_t& index) const;

    /// Returns true if the current face was successfully ear clipped, false otherwise.
    /// A simple polygon should be able to triangulate. This will modify all indices and face counts.
    /// This implementation is O(n * r) where r are the number of concave vertices.
    bool IsEarClipping(const FacePoint& reference, VtIntArray& indices, VtIntArray& faceCount) const;


    // Utility function. Add an index to the indices array.
    void Add(VtIntArray& indices, const size_t index) const
    {
        const size_t newIndex = _indices[_indexStart + index];
        indices.push_back(newIndex);
    }
private:
    const VtVec3fArray _points;
    const VtIntArray _indices;
    const size_t _indexStart;
    const size_t _vertexCount;
};

/// \class Triangulation
/// Triangulation takes all points, indices and vertices of a Mesh and computes its proper triangulation.
class Triangulation
{
public:
    Triangulation(const VtVec3fArray& points,
        const VtIntArray& indices,
        const VtIntArray& vertexCount);

    bool FanTriangulate(VtIntArray& indices, VtIntArray& faces) const;
private:
    const VtVec3fArray _points;
    const VtIntArray _indices;
    const VtIntArray _vertexCount;
};

/// \class FanTriangulation
/// A wrapper for UsdGeomMesh to generate Triangulation.
class FanTriangulation
{
public:
    FanTriangulation(UsdGeomMesh& mesh);

    bool FanTriangulate();
private:
    bool FanTriangulate(const UsdTimeCode& timeCode);

    UsdGeomMesh m_mesh;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // USD_TRIANGULATION_H
