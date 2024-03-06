// TODO
#include "pxr/imaging/hd/meshTessellation.h"

PXR_NAMESPACE_OPEN_SCOPE

int IsEdge(
    const std::map<std::pair<int, int>, int>& edges, 
    const int u, 
    const int v
)
{
    const auto key = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
    const auto it = edges.find(key);
    return (it == edges.end()) ? false : it->second == 1;
}

int EdgeFlag(
    const std::map<std::pair<int, int>, int>& edges, 
    const GfVec3i& triangle
)
{
    bool pq = IsEdge(edges, triangle[0], triangle[1]);
    bool qr = IsEdge(edges, triangle[1], triangle[2]);
    bool rp = IsEdge(edges, triangle[2], triangle[0]);
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
HdMeshTessellation::ComputeTriangles(
    VtVec3iArray& triangles, VtIntArray& flags
) const
{
    std::map<std::pair<int, int>, int> edges;

    int startIndex = 0;
    for (const auto count : counts)
    {
        for (int i = 0; i < count; ++i)
        {
            const int u = indices[startIndex + i];
            const int v = indices[startIndex + (i + 1) % count];
            const auto key = (u < v) ? std::make_pair(u, v) : std::make_pair(v, u);
            ++edges[key];
        }
        startIndex += count;
    }

    startIndex = 0;
    for (const auto count : counts)
    {
        for (int i = 0; i < count - 2; ++i)
        {
            GfVec3i triangle = {
                indices[startIndex],
                indices[startIndex + i + 1],
                indices[startIndex + i + 2]
            };
            triangles.push_back(triangle);
            flags.push_back(EdgeFlag(edges, triangle));
        }
        startIndex += count;
    }
}

bool operator==(const HdMeshTessellation& lhs, const HdMeshTessellation& rhs)
{
    return lhs.faceIndex == rhs.faceIndex && 
           lhs.counts == rhs.counts &&
           lhs.indices == rhs.indices;
}

PXR_NAMESPACE_CLOSE_SCOPE

