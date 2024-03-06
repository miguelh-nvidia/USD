//
// Copyright 2024 Pixar
//
// Licensed under the Apache License, Version 2.0 (the "Apache License")
// with the following modification; you may not use this file except in
// compliance with the Apache License and the following modification to it:
// Section 6. Trademarks. is deleted and replaced with:
//
// 6. Trademarks. This License does not grant permission to use the trade
//    names, trademarks, service marks, or product names of the Licensor
//    and its affiliates, except as required to comply with Section 4(c) of
//    the License and to reproduce the content of the NOTICE file.
//
// You may obtain a copy of the Apache License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the Apache License with the above modification is
// distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied. See the Apache License for the specific
// language governing permissions and limitations under the Apache License.
//
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
    if (it == edges.end())
    {
        return false;
    }
    else {
        return it->second == 1;
    }
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

