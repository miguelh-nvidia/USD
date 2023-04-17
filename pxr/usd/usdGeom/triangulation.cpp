#include "pxr/pxr.h"
#include "pxr/usd/usdGeom/triangulation.h"

PXR_NAMESPACE_OPEN_SCOPE

// FacePoint

FacePoint::FacePoint(const GfVec3f& p,
    const GfVec3f& q,
    const GfVec3f& r) :
    m_p(p),
    m_q(q),
    m_r(r),
    m_direction(GfCross(p - q, r - q))
{
}

FacePoint::FacePoint(FacePoint const& other) :
    m_p(other.m_p),
    m_q(other.m_q),
    m_r(other.m_r),
    m_direction(other.m_direction)
{
}

bool
FacePoint::HasSameOrientation(const FacePoint& other) const
{
    auto const value = GfDot(m_direction, other.m_direction);
    if (value < 0)
    {
        return false;
    }
    return true;
}

bool
FacePoint::IsVisible(const GfVec3f& s) const
{
    return
        GfDot(GfCross(m_p - m_q, s - m_q), m_direction) >= 0 &&
        GfDot(GfCross(s - m_q, m_r - m_q), m_direction) >= 0;
}

bool
FacePoint::HasPoint(const GfVec3f& point) const
{
    return m_p == point || m_q == point || m_r == point;
}


bool
FacePoint::ContainsPoint(const GfVec3f& s) const
{
    return
        GfDot(GfCross(m_q - m_p, m_r - m_p), GfCross(m_q - m_p, s - m_p)) > 0 &&
        GfDot(GfCross(m_r - m_q, m_p - m_q), GfCross(m_r - m_q, s - m_q)) > 0 &&
        GfDot(GfCross(m_p - m_r, m_q - m_r), GfCross(m_p - m_r, s - m_r)) > 0;
}

// Face

Face::Face(const VtVec3fArray& points,
    const VtIntArray& indices,
    const size_t indexStart,
    const size_t vertexCount) :
    m_points(points),
    m_indices(indices),
    m_indexStart(indexStart),
    m_vertexCount(vertexCount)
{
}

FacePoint
Face::GetReference() const
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
Face::IsStarShaped(const FacePoint& reference, size_t& index) const
{
    std::vector<bool> visited(size(), false);
    for (size_t i = 0; i < size(); )
    {
        if (visited[i])
        {
            // Cycle complete, break.
            break;
        }
        visited[i] = true;
        size_t secondIndex = (i + 1) % size();
        size_t thirdIndex = (i + 2) % size();
        size_t triangles = 0;
        while (triangles < size() - 2)
        {
            const FacePoint facePoint((*this)[i], (*this)[secondIndex], (*this)[thirdIndex]);
            if (facePoint.HasSameOrientation(reference))
            {
                ++triangles;
                secondIndex = (secondIndex + 1) % size();
                thirdIndex = (thirdIndex + 1) % size();
            }
            else {
                i = secondIndex;
                break;
            }
        }
        if (triangles == size() - 2)
        {
            index = i;
            return true;
        }
    }
    return false;
}

bool
Face::IsEarClipping(const FacePoint& reference, VtIntArray& indices, VtIntArray& faceCount) const
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
        indices.push_back(prevIndex[i]);
        indices.push_back(i);
        indices.push_back(nextIndex[i]);
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
Face::FanTriangulate(VtIntArray& indices, VtIntArray& faces) const
{
    if (!IsValid())
    {
        // Invalid geometry will be later discarded
        // There is no modification to indices
        for (size_t i = 0; i < size(); ++i)
            add(indices, i);
        faces.push_back(size());
        return true;
    }

    if (IsConvex())
    {
        // Convex shapes can always fan triangulate
        // There is no modification to indices
        for (size_t i = 0; i < size(); ++i)
            add(indices, i);
        faces.push_back(size());
        return true;
    }

    // Calculate a point which belongs to the convex hull
    // Useful for the following calculations
    FacePoint const reference = GetReference();

    size_t visibleIndex;
    if (IsStarShaped(reference, visibleIndex))
    {
        // Star shapes can always fan triangulate
        // Indices may need to shift.
        for (size_t i = 0; i < size(); ++i)
            add(indices, (visibleIndex + i) % size());
        faces.push_back(size());
        return true;
    }

    VtIntArray tmpIndices;
    VtIntArray tmpFaceCount;
    if (IsEarClipping(reference, tmpIndices, tmpFaceCount))
    {
        // Is ear clipping should work
        for (size_t i = 0; i < tmpIndices.size(); ++i)
            add(indices, tmpIndices[i]);
        for (size_t i = 0; i < tmpFaceCount.size(); ++i)
            faces.push_back(tmpFaceCount[i]);
        return true;
    }

    // When everything else fails.
    return false;
}

// Triangulation

Triangulation::Triangulation(const VtVec3fArray& points,
    const VtIntArray& indices,
    const VtIntArray& vertexCount) :
    m_points(points),
    m_indices(indices),
    m_vertexCount(vertexCount)
{
}

bool
Triangulation::FanTriangulate(VtIntArray& indices, VtIntArray& faces) const
{
    size_t indexStart = 0;
    for (size_t i = 0; i < m_vertexCount.size(); ++i)
    {
        int numberOfVertices = m_vertexCount[i];

        Face const face(m_points, m_indices, indexStart, numberOfVertices);
        indexStart += numberOfVertices;
        if (!face.FanTriangulate(indices, faces))
        {
            return false;
        }
    }
    return true;
}

// FanTriangulation

FanTriangulation::FanTriangulation(UsdGeomMesh& mesh) : m_mesh(mesh)
{
}

bool
FanTriangulation::FanTriangulate(const UsdTimeCode& timeCode)
{
    UsdAttribute pointsAttr = m_mesh.GetPointsAttr();
    VtVec3fArray points;
    pointsAttr.Get(&points, timeCode);

    UsdAttribute indicesAttr = m_mesh.GetFaceVertexIndicesAttr();
    VtIntArray indices;
    indicesAttr.Get(&indices, timeCode);

    UsdAttribute vertexCountAttr = m_mesh.GetFaceVertexCountsAttr();
    VtIntArray vertexCount;
    vertexCountAttr.Get(&vertexCount, timeCode);

    Triangulation triangulation(points, indices, vertexCount);
    VtIntArray newIndices;
    VtIntArray newFaces;
    if (triangulation.FanTriangulate(newIndices, newFaces))
    {
        indicesAttr.Set(newIndices, timeCode);
        vertexCountAttr.Set(newFaces, timeCode);
        // TODO: Reshuffle normals?
        return true;
    }
    return false;
}

bool
FanTriangulation::FanTriangulate()
{
    UsdAttribute pointsAttr = m_mesh.GetPointsAttr();

    if (pointsAttr.ValueMightBeTimeVarying())
    {
        std::vector<double> times;
        if (!pointsAttr.GetTimeSamples(&times))
        {
            return false;
        }

        bool result = true;
        for (double time : times)
        {
            result &= FanTriangulate(UsdTimeCode(time));
        }
        return result;
    }
    else
    {
        return FanTriangulate(UsdTimeCode::Default());
    }
}
PXR_NAMESPACE_CLOSE_SCOPE
