#include <pxr/pxr.h>

#include "triangulation.h"
#include <iostream>

PXR_NAMESPACE_OPEN_SCOPE

// Point2D

Point2D::Point2D(const size_t index, const double x, const double y) :
    index(index),
    x(x),
    y(y)
{
}

Point2D::Point2D(const Point2D& other) :
    Point2D(other.index, other.x, other.y)
{
}

Point2D::Point2D() :
    Point2D(0, 0, 0)
{
}

double Point2D::Cross(const Point2D& other) const
{
    return x * other.y - other.x * y;
}

bool operator < (const Point2D& lhs, const Point2D& rhs)
{
    return std::tie(lhs.x, lhs.y) < std::tie(rhs.x, rhs.y);
}

bool operator == (const Point2D& lhs, const Point2D& rhs)
{
    return std::tie(lhs.x, lhs.y) == std::tie(rhs.x, rhs.y);
}


static
Turn turn(const Point2D& a, const Point2D& b, const Point2D& c)
{
    double value = a.Cross(b) + b.Cross(c) + c.Cross(a);
    if (value > 0) {
        return Turn::COUNTERCLOCKWISE;
    }
    else if (value < 0) {
        return Turn::CLOCKWISE;
    }
    else {
        return Turn::COLLINEAR;
    }
}


static
bool CCW(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return turn(a, b, c) == Turn::COUNTERCLOCKWISE;
}


static
bool CCW(const std::vector<Point2D>& polygon)
{
    double total = 0;
    const size_t size = polygon.size();
    for (size_t i = 0; i < size; ++i) {
        const Point2D& p = polygon[i];
        const Point2D& q = polygon[(i + 1) % size];
        total += p.Cross(q);
    }
    return total > 0;
}


// Polygon2D

Polygon2D::Polygon2D(const std::vector<Point2D>& points) :
    points(points),
    indices(points.size(), 0),
    prev(points.size(), 0),
    next(points.size(), 0)
{
    const size_t size = points.size();
    for (size_t i = 0; i < size; ++i) {
        indices[i] = i;
        prev[i] = (i + size - 1) % size;
        next[i] = (i + 1) % size;
    }
}

Point2D const& Polygon2D::Get(const size_t index) const
{
    return points[indices[index]];
}

Point2D const& Polygon2D::Prev(const size_t index) const
{
    return Get(prev[index]);
}

Point2D const& Polygon2D::Next(const size_t index) const
{
    return Get(next[index]);
}

std::tuple<size_t, size_t> Polygon2D::AddDiagonal(const size_t i, const size_t j)
{
    const size_t p = indices.size();
    indices.push_back(indices[i]);
    prev.push_back(prev[i]);
    next.push_back(j);

    const size_t q = indices.size();
    indices.push_back(indices[j]);
    prev.push_back(prev[j]);
    next.push_back(i);

    next[prev[i]] = p;
    next[prev[j]] = q;
    prev[j] = p;
    prev[i] = q;
    return std::make_tuple(p, q);
}

void Polygon2D::GetPolygons(std::vector<std::vector<Point2D>>& polygons) const
{
    const size_t size = indices.size();

    std::vector<bool> used(size, false);
    polygons.clear();
    for (size_t i = 0; i < size; ++i) {
        if (used[i]) {
            continue;
        }
        std::vector<Point2D> polygon;
        for (size_t j = i; !used[j]; j = next[j]) {
            polygon.push_back(Get(j));
            used[j] = true;
        }
        polygons.push_back(polygon);
    }
}

// Segment2D

Segment2D::Segment2D(const Point2D& p, const Point2D& q) :
    p(p),
    q(q)
{
}

Segment2D::Segment2D(const Segment2D& other) :
    Segment2D(other.p, other.q)
{
}

bool operator < (const Segment2D& lhs, const Segment2D& rhs)
{
    if (!lhs.IsVertical()) {
        const Point2D& l = lhs.p < lhs.q ? lhs.p : lhs.q;
        const Point2D& r = lhs.p < lhs.q ? lhs.q : lhs.p;
        const Turn u = turn(r, l, rhs.p);
        const Turn v = turn(r, l, rhs.q);
        if (u == Turn::COLLINEAR && v == Turn::COLLINEAR) {
            return lhs.p.y < rhs.p.y;
        }
        else if (u == Turn::COLLINEAR) {
            return v == Turn::CLOCKWISE;
        }
        else if (v == Turn::COLLINEAR) {
            return u == Turn::CLOCKWISE;
        }
        else if (u == v) {
            return u == Turn::CLOCKWISE;
        }
    }

    if (!rhs.IsVertical()) {
        const Point2D& l = rhs.p < rhs.q ? rhs.p : rhs.q;
        const Point2D& r = rhs.p < rhs.q ? rhs.q : rhs.p;
        const Turn u = turn(r, l, lhs.p);
        const Turn v = turn(r, l, lhs.q);
        if (u == Turn::COLLINEAR && v == Turn::COLLINEAR) {
            return lhs.p.y < rhs.p.y;
        }
        else if (u == Turn::COLLINEAR) {
            return v == Turn::COUNTERCLOCKWISE;
        }
        else if (v == Turn::COLLINEAR) {
            return u == Turn::COUNTERCLOCKWISE;
        }
        else if (u == v) {
            return u == Turn::COUNTERCLOCKWISE;
        }
    }

    return lhs.p.y < rhs.p.y;
}

bool operator ==(const Segment2D& lhs, const Segment2D& rhs)
{
    return (lhs.p == rhs.p && lhs.q == rhs.q);
}

bool Segment2D::IsVertical() const
{
    return p.x == q.x;
}

// MonotoneTriangulation

MonotoneTriangulation::MonotoneTriangulation(const std::vector<Point2D>& polygon) :
    polygon(polygon)
{
}

void MonotoneTriangulation::triangulate(std::vector<Point2D>& result) const
{
    const size_t size = polygon.size();
    const bool polygon_ccw = CCW(polygon);

    size_t left = 0;
    size_t right = 0;
    for (size_t i = 0; i < size; ++i) {
        if (polygon[i] < polygon[left]) {
            left = i;
        }
        else if (polygon[right] < polygon[i]) {
            right = i;
        }
    }

    const bool left_to_right_above = polygon[(left + size - 1) % size].y < polygon[(left + 1) % size].y;

    std::vector<std::tuple<Point2D, bool>> left_to_right;
    for (size_t curr = left; curr != right; curr = (curr + 1) % size) {
        left_to_right.push_back(std::make_tuple(polygon[curr], left_to_right_above));
    }

    std::vector<std::tuple<Point2D, bool>> right_to_left;
    for (size_t curr = right; curr != left; curr = (curr + 1) % size) {
        right_to_left.push_back(std::make_tuple(polygon[curr], !left_to_right_above));
    }
    std::reverse(right_to_left.begin(), right_to_left.end());

    std::vector<std::tuple<Point2D, bool>> u;
    u.reserve(left_to_right.size() + right_to_left.size());
    std::merge(
        left_to_right.begin(), left_to_right.end(),
        right_to_left.begin(), right_to_left.end(),
        std::back_inserter(u)
    );

    auto visible = [](const std::tuple<Point2D, bool>& a, const std::tuple<Point2D, bool>& b, const std::tuple<Point2D, bool>& c) {
        const bool is_ccw = CCW(std::get<0>(a), std::get<0>(b), std::get<0>(c));
        return std::get<1>(c) ? !is_ccw : is_ccw;
    };

    std::stack<std::tuple<Point2D, bool>> s;
    std::vector<std::tuple<Point2D, bool>> t;

    s.push(u[0]);
    s.push(u[1]);
    for (size_t j = 2; j < u.size() - 1; ++j) {
        if (std::get<1>(u[j]) != std::get<1>(s.top())) {
            while (!s.empty()) {
                const std::tuple<Point2D, bool>& v = s.top();
                s.pop();
                if (!s.empty()) {
                    t.push_back(s.top());
                    t.push_back(u[j]);
                    t.push_back(v);
                }
            }
            s.push(u[j - 1]);
            s.push(u[j]);
        }
        else {
            std::tuple<Point2D, bool>& v = s.top();
            s.pop();
            while (!s.empty() && visible(s.top(), v, u[j])) {
                t.push_back(s.top());
                t.push_back(u[j]);
                t.push_back(v);

                v = s.top();
                s.pop();
            }
            s.push(v);
            s.push(u[j]);
        }
    }

    while (!s.empty()) {
        const std::tuple<Point2D, bool>& v = s.top();
        s.pop();
        if (!s.empty()) {
            t.push_back(s.top());
            t.push_back(u.back());
            t.push_back(v);
        }
    }

    result.clear();
    for (size_t i = 0; i < t.size(); i += 3) {
        const Point2D& p = std::get<0>(t[i]);
        const Point2D& q = std::get<0>(t[i + 1]);
        const Point2D& r = std::get<0>(t[i + 2]);
        if (CCW(p, q, r) == polygon_ccw) {
            result.push_back(p);
            result.push_back(q);
            result.push_back(r);
        }
        else {
            result.push_back(p);
            result.push_back(r);
            result.push_back(q);
        }
    }
}

// SweepLineEvent

SweepLineEvent::SweepLineEvent(
    const size_t index,
    const Point2D& prev,
    const Point2D& curr,
    const Point2D& next,
    const bool polygon_ccw) :
    index(index),
    prev(prev),
    curr(curr),
    next(next),
    polygon_ccw(polygon_ccw),
    event_ccw(CCW(prev, curr, next)),
    prev_segment(Segment2D(prev, curr)),
    next_segment(Segment2D(curr, next))
{
}

SweepLineEvent::SweepLineEvent(const SweepLineEvent& other) :
    SweepLineEvent(other.index, other.prev, other.curr, other.next, other.polygon_ccw)
{
}

size_t SweepLineEvent::GetIndex() const
{
    return index;
}

bool SweepLineEvent::IsStart() const
{
    return curr < prev&& curr < next&& event_ccw == polygon_ccw;
}

bool SweepLineEvent::IsSplit() const
{
    return curr < prev&& curr < next&& event_ccw != polygon_ccw;
}

bool SweepLineEvent::IsEnd() const
{
    return prev < curr&& next < curr&& event_ccw == polygon_ccw;
}

bool SweepLineEvent::IsMerge() const
{
    return prev < curr&& next < curr&& event_ccw != polygon_ccw;
}

bool SweepLineEvent::IsPolygonAbove() const
{
    if (polygon_ccw) {
        return prev < curr&& curr < next;
    }
    else {
        return next < curr&& curr < prev;
    }
}

Segment2D const& SweepLineEvent::GetAbove() const
{
    if (curr < prev && curr < next)
        return event_ccw ? prev_segment : next_segment;
    else
        return event_ccw ? next_segment : prev_segment;
}

Segment2D const& SweepLineEvent::GetBelow() const
{
    if (curr < prev && curr < next)
        return event_ccw ? next_segment : prev_segment;
    else
        return event_ccw ? prev_segment : next_segment;
}

Segment2D const& SweepLineEvent::GetLeft() const
{
    if (prev < curr)
        return prev_segment;
    else
        return next_segment;
}

Segment2D const& SweepLineEvent::GetRight() const
{
    if (curr < prev)
        return prev_segment;
    else
        return next_segment;
}

bool operator <(const SweepLineEvent& lhs, const SweepLineEvent& rhs)
{
    return lhs.curr < rhs.curr;
}

// Segment2DHelper

Segment2DHelper::Segment2DHelper(const Segment2D& segment) :
    Segment2DHelper(segment, 0, false)
{
}

Segment2DHelper::Segment2DHelper(const Segment2D& segment, const size_t index, const bool merge) :
    segment(segment),
    index(index),
    merge(merge)
{

}

bool operator < (const Segment2DHelper& lhs, const Segment2DHelper& rhs)
{
    return lhs.segment < rhs.segment;
}

// SweepLine2D

SweepLine2D::SweepLine2D(const std::vector<Point2D>& points) :
    points(points)
{
}

void SweepLine2D::Sweep(std::vector<std::vector<Point2D>>& result) const
{
    const size_t size = points.size();
    const bool polygon_ccw = CCW(points);

    std::vector<SweepLineEvent> events;
    for (size_t i = 0; i < size; ++i) {
        events.push_back(
            SweepLineEvent(
                i,
                points[(i + size - 1) % size],
                points[i],
                points[(i + 1) % size],
                polygon_ccw
            )
        );
    }
    std::sort(events.begin(), events.end());

    Polygon2D polygon(points);
    std::set<Segment2DHelper> segments;
    for (SweepLineEvent event : events) {
        if (event.IsStart()) {
            HandleStart(event, polygon, segments);
        }
        else if (event.IsSplit()) {
            HandleSplit(event, polygon, segments);
        }
        else if (event.IsEnd()) {
            HandleEnd(event, polygon, segments);
        }
        else if (event.IsMerge()) {
            HandleMerge(event, polygon, segments);
        }
        else {
            HandleRegular(event, polygon, segments);
        }
    }
    result.clear();
    polygon.GetPolygons(result);
}

size_t SweepLine2D::GetIndex(Polygon2D& polygon, const size_t index, const size_t index_copy, const Segment2D& segment) const
{
    Segment2D segment_at_index(polygon.Get(index), polygon.Next(index));
    return (segment == segment_at_index) ? index : index_copy;
}

void SweepLine2D::HandleStart(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const
{
    segments.insert(Segment2DHelper(event.GetAbove()));
    segments.insert(Segment2DHelper(event.GetBelow(), event.GetIndex(), false));
}

void SweepLine2D::HandleSplit(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const
{
    auto above = segments.insert(Segment2DHelper(event.GetAbove())).first;
    auto below = segments.insert(Segment2DHelper(event.GetBelow())).first;
    auto lower = std::prev(below);
    auto indices = polygon.AddDiagonal(lower->index, event.GetIndex());
    lower->index = GetIndex(polygon, event.GetIndex(), std::get<1>(indices), below->segment);
    lower->merge = false;
    above->index = GetIndex(polygon, event.GetIndex(), std::get<1>(indices), above->segment);
    above->merge = false;
}

void SweepLine2D::HandleEnd(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const
{
    auto below = segments.find(Segment2DHelper(event.GetBelow()));
    auto above = segments.find(Segment2DHelper(event.GetAbove()));
    if (below->merge) {
        polygon.AddDiagonal(below->index, event.GetIndex());
    }
    segments.erase(below);
    segments.erase(above);
}

void SweepLine2D::HandleMerge(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const
{
    size_t index = event.GetIndex();
    auto below = segments.find(Segment2DHelper(event.GetBelow()));
    auto above = segments.find(Segment2DHelper(event.GetAbove()));
    if (above->merge) {
        const auto indices = polygon.AddDiagonal(above->index, index);
        index = GetIndex(polygon, index, std::get<1>(indices), below->segment);
    }
    segments.erase(above);

    auto lower = std::prev(below);
    if (lower->merge) {
        auto indices = polygon.AddDiagonal(lower->index, index);
        index = GetIndex(polygon, index, std::get<1>(indices), below->segment) == index ? std::get<1>(indices) : index;
    }
    lower->index = index;
    lower->merge = true;
    segments.erase(below);
}

void SweepLine2D::HandleRegular(const SweepLineEvent& event, Polygon2D& polygon, std::set<Segment2DHelper>& segments) const
{
    size_t index = event.GetIndex();
    const auto left = segments.find(Segment2DHelper(event.GetLeft()));
    if (event.IsPolygonAbove()) {
        if (left->merge) {
            const auto indices = polygon.AddDiagonal(left->index, index);
            index = GetIndex(polygon, index, std::get<1>(indices), event.GetRight());
        }
        segments.erase(left);
        segments.insert(Segment2DHelper(event.GetRight(), index, false));
    }
    else {
        auto below = std::prev(left);
        if (below->merge) {
            const auto indices = polygon.AddDiagonal(below->index, index);
            index = GetIndex(polygon, index, std::get<1>(indices), event.GetRight());
        }
        segments.erase(left);
        segments.insert(Segment2DHelper(event.GetRight()));
        below->index = index;
        below->merge = false;
    }
}


// Face

Face::Face(const VtVec3fArray& points,
    const VtIntArray& indices,
    const size_t indexStart,
    const size_t vertexCount) :
    points(points),
    indices(indices),
    indexStart(indexStart),
    vertexCount(vertexCount)
{
}

bool
Face::IsConvex() const
{
    if (size() <= 3)
    {
        return true;
    }
    const GfVec3f& prev = (*this)[size() - 1];
    const GfVec3f& curr = (*this)[0];
    const GfVec3f& next = (*this)[1];
    const GfVec3f& reference = GfCross(prev - curr, next - curr);
    for (size_t i = 1; i < size(); ++i)
    {
        const GfVec3f& prev = (*this)[i - 1];
        const GfVec3f& curr = (*this)[i];
        const GfVec3f& next = (*this)[(i + 1) % size()];
        const GfVec3f& direction = GfCross(prev - curr, next - curr);
        if (GfDot(reference, direction) < 0) {
            return false;
        }
    }
    return true;
}


int
Face::TriangulateFlag(const size_t p, const size_t q, const size_t r) const
{
    auto adjacent = [](const size_t i, const size_t j, const size_t length) {
        return (i + 1) % length == j || (i + length - 1) % length == j;
    };
    bool pq = adjacent(p, q, size());
    bool qr = adjacent(q, r, size());
    bool rp = adjacent(r, p, size());
    if (pq && qr && rp) {
        return 0; // Show all edges
    }
    if (pq && qr && !rp) {
        return 1; // hide [2-0]
    }
    if (!pq && qr && rp) {
        return 2; // hide [0-1]
    }
    if (!pq && qr && !rp) {
        return 3; // hide [0-1] and [2-0]
    }
    if (!pq && !qr && !rp) {
        return 4; // hide all
    }
}

void
Face::FanTriangulate(VtVec3iArray& indices, VtIntArray& flags) const
{
    for (size_t i = 0; i < size() - 2; ++i)
    {
        indices.push_back(GfVec3i(index(i), index(i + 1), index(i + 2)));
        flags.push_back(TriangulateFlag(i, i + 1, i + 2));
    }
}


bool
Face::Triangulate(VtVec3iArray& indices, VtIntArray& flags) const
{
    if (!IsValid())
    {
        // Invalid geometry will be later discarded
        // There is no modification to indices
        FanTriangulate(indices, flags);
        return true;
    }

    if (IsConvex())
    {
        // Convex shapes can always fan triangulate
        // There is no modification to indices
        FanTriangulate(indices, flags);
        return true;
    }

    const GfPlane plane((*this)[0], (*this)[1], (*this)[2]);
    const GfVec3d x = plane.Project((*this)[1] - (*this)[0]).GetNormalized();
    const GfVec3d y = GfCross(plane.GetNormal(), x);

    std::vector<Point2D> values;
    values.reserve(size());
    for (size_t i = 0; i < size(); ++i) {
        double px = GfDot((*this)[i], x);
        double py = GfDot((*this)[i], y);
        values.push_back(Point2D(i, px, py));
    }

    SweepLine2D sweepLine(values);
    std::vector<std::vector<Point2D>> components;
    sweepLine.Sweep(components);
    for (auto component : components) {
        MonotoneTriangulation triangulation(component);
        std::vector<Point2D> result;
        triangulation.triangulate(result);

        for (size_t i = 0; i < result.size(); i += 3) {
            const size_t p = result[i].index;
            const size_t q = result[i + 1].index;
            const size_t r = result[i + 2].index;
            indices.push_back(GfVec3i(index(p), index(q), index(r)));
            flags.push_back(TriangulateFlag(p, q, r));
        }
    }

    return true;
}

// Triangulation

Triangulation::Triangulation(
    const VtVec3fArray& points,
    const VtIntArray& indices,
    const VtIntArray& vertexCount) :
    _points(points),
    _indices(indices),
    _vertexCount(vertexCount),
    _triangulationIndices(),
    _triangulationFlags()
{
}

bool
Triangulation::Triangulate()
{
    bool flag = true;
    size_t indexStart = 0;
    for (size_t i = 0; i < _vertexCount.size(); ++i)
    {
        int const numberOfVertices = _vertexCount[i];

        Face const face(_points, _indices, indexStart, numberOfVertices);
        flag &= face.Triangulate(_triangulationIndices, _triangulationFlags);

        indexStart += numberOfVertices;
    }
    return flag;
}

void
Triangulation::GetIndices(VtVec3iArray& outIndices) const
{
    for (size_t i = 0; i < _triangulationIndices.size(); ++i)
    {
        GfVec3i const index = _triangulationIndices[i];

        outIndices.push_back(index);
    }
}


void
Triangulation::GetFlags(VtIntArray& outFlags) const
{
    for (size_t i = 0; i < _triangulationFlags.size(); ++i)
    {
        int const count = _triangulationFlags[i];

        outFlags.push_back(count);
    }
}

PXR_NAMESPACE_CLOSE_SCOPE
