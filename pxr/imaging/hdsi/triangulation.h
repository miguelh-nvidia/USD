#ifndef TRIANGULATION_H_
#define TRIANGULATION_H_

#include <stack>

#include <pxr/pxr.h>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec3i.h>
#include <pxr/base/gf/plane.h>

#include <pxr/usd/usdGeom/mesh.h>

PXR_NAMESPACE_OPEN_SCOPE

// Point2D

class Point2D
{
public:
    Point2D(const size_t index, const double x, const double y);

    Point2D(const Point2D& other);

    Point2D();

    double Cross(const Point2D& other) const;

    friend bool operator <(const Point2D& lhs, const Point2D& rhs);

    friend bool operator ==(const Point2D& lhs, const Point2D& rhs);
public:
    size_t index;
    double x;
    double y;
};


enum class Turn {
    COLLINEAR,
    CLOCKWISE,
    COUNTERCLOCKWISE
};


static
Turn turn(const Point2D& a, const Point2D& b, const Point2D& c);

static
bool CCW(const Point2D& a, const Point2D& b, const Point2D& c);

static
bool CCW(const std::vector<Point2D>& polygon);

// Polygon2D

class Polygon2D
{
public:
    Polygon2D(const std::vector<Point2D>& points);

    Point2D const& Get(const size_t index) const;

    Point2D const& Prev(const size_t index) const;

    Point2D const& Next(const size_t index) const;

    std::tuple<size_t, size_t> AddDiagonal(const size_t i, const size_t j);

    void GetPolygons(std::vector<std::vector<Point2D>>& polygons) const;
private:
    const std::vector<Point2D>& points;
    std::vector<size_t> indices;
    std::vector<size_t> prev;
    std::vector<size_t> next;
};


// Segment2D

class Segment2D
{
public:
    Segment2D(const Point2D& p, const Point2D& q);

    Segment2D(const Segment2D& other);

    friend bool operator <(const Segment2D& lhs, const Segment2D& rhs);

    friend bool operator ==(const Segment2D& lhs, const Segment2D& rhs);
public:
    Point2D p;
    Point2D q;
private:
    bool IsVertical() const;
};


// MonotoneTriangulation

class MonotoneTriangulation
{
public:
    MonotoneTriangulation(const std::vector<Point2D>& polygon);

    void triangulate(std::vector<Point2D>& result) const;
private:
    const std::vector<Point2D>& polygon;
};

// SweepLineEvent

class SweepLineEvent
{
public:
    SweepLineEvent(
        const size_t index,
        const Point2D& prev,
        const Point2D& curr,
        const Point2D& next,
        const bool polygon_ccw);

    SweepLineEvent(const SweepLineEvent& other);

    size_t GetIndex() const;

    bool IsStart() const;

    bool IsSplit() const;

    bool IsEnd() const;

    bool IsMerge() const;

    bool IsPolygonAbove() const;

    Segment2D const& GetAbove() const;

    Segment2D const& GetBelow() const;

    Segment2D const& GetLeft() const;

    Segment2D const& GetRight() const;

    friend bool operator <(const SweepLineEvent& lhs, const SweepLineEvent& rhs);

private:
    size_t index;
    Point2D prev;
    Point2D curr;
    Point2D next;
    bool polygon_ccw;
    bool event_ccw;
    Segment2D prev_segment;
    Segment2D next_segment;
};

// Segment2DHelper

class Segment2DHelper
{
public:
    Segment2DHelper(const Segment2D& segment);

    Segment2DHelper(const Segment2D& segment, const size_t index, const bool merge);

    friend bool operator <(const Segment2DHelper& lhs, const Segment2DHelper& rhs);
public:
    const Segment2D segment;
    mutable size_t index;
    mutable bool merge;
};

// SweepLine2D

class SweepLine2D
{
public:
    SweepLine2D(const std::vector<Point2D>& points);

    void Sweep(std::vector<std::vector<Point2D>>& result) const;
private:
    size_t GetIndex(Polygon2D& polygon, const size_t index, const size_t index_copy, const Segment2D& segment) const;

    void HandleStart(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const;

    void HandleSplit(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const;

    void HandleEnd(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const;

    void HandleMerge(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const;

    void HandleRegular(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const;
private:
    const std::vector<Point2D>& points;
};


/// \class Face
/// A face is a representation of a Face in a Mesh. It consist of single public method: FanTriangulate.
class Face
{
public:
    Face(
        const VtVec3fArray& points,
        const VtIntArray& indices,
        const size_t indexStart,
        const size_t vertexCount);

    bool Triangulate(VtVec3iArray& indices, VtIntArray& flags) const;
private:
    size_t size() const
    {
        return vertexCount;
    }

    size_t const index(const size_t idx) const
    {
        return indices[indexStart + idx];
    }

    GfVec3f const& operator[](const size_t idx) const
    {
        return points[index(idx)];
    }

    bool IsValid() const
    {
        return vertexCount >= 3;
    }

    /// Returns true if the current face is a convex shape, false otherwise.
    /// Checking if a face is convex is a linear operation and if it is convex,
    /// the original indices and faces will not suffer modifications.
    bool IsConvex() const;

    int TriangulateFlag(const size_t p, const size_t q, const size_t r) const;

    void FanTriangulate(VtVec3iArray& indices, VtIntArray& flags) const;
private:
    const VtVec3fArray& points;
    const VtIntArray& indices;
    const size_t indexStart;
    const size_t vertexCount;
};

/// \class Triangulation
/// Triangulation takes all points, indices and vertices of a Mesh and computes its proper triangulation.
class Triangulation
{
public:
    Triangulation(
        const VtVec3fArray& points,
        const VtIntArray& indices,
        const VtIntArray& vertexCount);

    bool Triangulate();

    void GetIndices(VtVec3iArray& outIndices) const;

    void GetFlags(VtIntArray& outFlags) const;
private:
    const VtVec3fArray _points;
    const VtIntArray _indices;
    const VtIntArray _vertexCount;

    VtVec3iArray _triangulationIndices;
    VtIntArray _triangulationFlags;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // TRIANGULATION_H_
