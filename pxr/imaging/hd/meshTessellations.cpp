// TODO
#include "pxr/imaging/hd/meshTessellations.h"

PXR_NAMESPACE_OPEN_SCOPE

// Find if an edge(u, v) is found in edges.
bool IsEdge(
    const std::map<std::pair<int, int>, int>& edges, 
    const int u, 
    const int v
)
{
    const auto edge = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
    const auto it = edges.find(edge);
    if (it == edges.end()) {
        return false;
    }
    return it->second == 1;
}

// Given a triangle, returns its edge flag.
// edgeFlag is used for inner-line removal of non-triangle
// faces on wireframe shading.
//
//          0__                0  0   0__
//        _/|\ \_            _/.  ..   . \_
//      _/  | \  \_   ->   _/  .  . .   .  \_
//     /  A |C \ B \_     /  A .  .C .   . B \_
//    1-----2---3----4   1-----2  1---2   1----2
//
//  Type   EdgeFlag    Draw
//    -       0        show all edges
//    A       1        hide [2-0]
//    B       2        hide [0-1]
//    C       3        hide [0-1] and [2-0]
int EdgeFlag(
    const std::map<std::pair<int, int>, int>& edges, 
    GfVec3i& triangle
)
{
    const bool p = IsEdge(edges, triangle[0], triangle[1]);
    const bool q = IsEdge(edges, triangle[1], triangle[2]);
    const bool r = IsEdge(edges, triangle[2], triangle[0]);
    // Show all edges
    if (p && q && r) {
        return 0;
    }
    // One edge is missing
    if (p && q && !r) {
        return 1; // hide [2-0]
    }
    if (!p && q && r) {
        return 2; // hide [0-1]
    }
    if (p && !q && r) {
        triangle.Set(triangle[1], triangle[2], triangle[0]);
        return 2;
    }
    // Two edges are missing
    if (!p && q && !r) {
        return 3; // hide [0-1] and [2-0]
    }
    if (!p && !q && r) {
        triangle.Set(triangle[1], triangle[2], triangle[0]);
        return 3;
    }
    if (p && !q && !r) {
        triangle.Set(triangle[2], triangle[0], triangle[1]);
        return 3;
    }
    // all edges are missing?
    return 3;
}

void
HdMeshTessellations::ComputeTriangleIndices(
    VtIntArray& trianglesPerFaces,
    VtVec3iArray& triangleIndices, 
    VtIntArray& triangleFlags
) const
{
    size_t offset;

    // Compute triangles per face
    offset = 0;
    for (const auto tessellationsPerFace : faceTessellations)
    {
        int trianglesPerFace = 0;
        for (int i = 0; i < tessellationsPerFace; ++i)
        {
            const int trianglesPerTessellation = tessellationVertexCounts[offset + i] - 2;
            trianglesPerFace += trianglesPerTessellation;
        }
        trianglesPerFaces.push_back(trianglesPerFace);
        offset += tessellationsPerFace;
    }

    // Compute all triangles
    offset = 0;
    for (const auto tessellationVertexCount : tessellationVertexCounts)
    {
        for (int i = 0; i < tessellationVertexCount - 2; ++i)
        {
            GfVec3i triangle = {
                tessellationVertexIndices[offset],
                tessellationVertexIndices[offset + i + 1],
                tessellationVertexIndices[offset + i + 2]
            };
            triangleIndices.push_back(triangle);
        }
        offset += tessellationVertexCount;
    }

    // Categorize triangles.
    offset = 0;
    for (const auto trianglePerFace : trianglesPerFaces)
    {
        std::map<std::pair<int, int>, int> edges;
        for (int i = 0; i < trianglePerFace; ++i)
        {
            const GfVec3i& t = triangleIndices[offset + i];
            ++edges[std::make_pair(t[0] < t[1] ? t[0] : t[1], t[0] < t[1] ? t[1] : t[0])];
            ++edges[std::make_pair(t[1] < t[2] ? t[1] : t[2], t[1] < t[2] ? t[2] : t[1])];
            ++edges[std::make_pair(t[2] < t[0] ? t[2] : t[0], t[2] < t[0] ? t[0] : t[2])];
        }
        for (int i = 0; i < trianglePerFace; ++i)
        {
            GfVec3i& t = triangleIndices[offset + i];
            const int flag = EdgeFlag(edges, t);
            triangleFlags.push_back(flag);
        }
        offset += trianglePerFace;
    }
}

bool operator==(const HdMeshTessellations& lhs, const HdMeshTessellations& rhs)
{
    return lhs.faceIndices == rhs.faceIndices && 
           lhs.faceTessellations == rhs.faceTessellations && 
           lhs.tessellationVertexCounts == rhs.tessellationVertexCounts &&
           lhs.tessellationVertexIndices == rhs.tessellationVertexIndices;
}

PXR_NAMESPACE_CLOSE_SCOPE

