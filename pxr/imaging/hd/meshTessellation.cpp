// TODO
#include "pxr/imaging/hd/meshTessellation.h"

PXR_NAMESPACE_OPEN_SCOPE

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
HdMeshTessellation::ComputeTriangles(
    VtVec3iArray& triangles, VtIntArray& flags
) const
{
    std::map<std::pair<int, int>, int> edges;

    int offset = 0;
    for (const auto count : counts)
    {
        for (int i = 0; i < count; ++i)
        {
            const int u = indices[offset + i];
            const int v = indices[offset + (i + 1) % count];
            const auto edge = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
            ++edges[edge];
        }
        offset += count;
    }

    offset = 0;
    for (const auto count : counts)
    {
        for (int i = 0; i < count - 2; ++i)
        {
            GfVec3i triangle = {
                indices[offset],
                indices[offset + i + 1],
                indices[offset + i + 2]
            };
            const int flag = EdgeFlag(edges, triangle);
            triangles.push_back(triangle);
            flags.push_back(flag);
        }
        offset += count;
    }
}

bool operator==(const HdMeshTessellation& lhs, const HdMeshTessellation& rhs)
{
    return lhs.faceIndex == rhs.faceIndex && 
           lhs.counts == rhs.counts &&
           lhs.indices == rhs.indices;
}

PXR_NAMESPACE_CLOSE_SCOPE

