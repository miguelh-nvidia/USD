#include "pxr/imaging/hdsi/triangulationSceneIndex.h"

#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
#include "pxr/imaging/hd/meshTriangulationSchema.h"
#include "pxr/imaging/hd/primvarSchema.h"
#include "pxr/imaging/hd/primvarsSchema.h"
#include "pxr/imaging/hd/tokens.h"
#include "pxr/imaging/hd/retainedDataSource.h"
#include "pxr/imaging/hd/subdivisionTagsSchema.h"

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>

#include <iostream>

PXR_NAMESPACE_OPEN_SCOPE

TriangulationSceneIndexRefPtr TriangulationSceneIndex::New(
    const HdSceneIndexBaseRefPtr& inputSceneIndex)
{
    return TfCreateRefPtr(new TriangulationSceneIndex(inputSceneIndex));
}

TriangulationSceneIndex::TriangulationSceneIndex(
    const HdSceneIndexBaseRefPtr& inputSceneIndex) :
    HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
{
}

HdSceneIndexPrim TriangulationSceneIndex::GetPrim(const SdfPath & primPath) const
{
    const auto it = this->cache.find(primPath);
    if (it != this->cache.end())
    {
        return it->second;
    }
    return this->_GetInputSceneIndex()->GetPrim(primPath);
}

SdfPathVector TriangulationSceneIndex::GetChildPrimPaths(const SdfPath & primPath) const
{
    return this->_GetInputSceneIndex()->GetChildPrimPaths(primPath);
}

void TriangulationSceneIndex::_PrimsAdded(
    const HdSceneIndexBase & sender,
    const HdSceneIndexObserver::AddedPrimEntries & entries)
{
    if (!_IsObserved())
    {
        return;
    }

    for (auto& entry : entries)
    {
        if (entry.primType != HdPrimTypeTokens->mesh)
        {
            continue;
        }

        const auto it = this->cache.find(entry.primPath);
        if (it != this->cache.end())
        {
            continue;
        }

        HdSceneIndexPrim sceneIndexPrim = this->_GetInputSceneIndex()->GetPrim(entry.primPath);
        HdMeshSchema meshSchema = HdMeshSchema::GetFromParent(sceneIndexPrim.dataSource);
        HdPrimvarsSchema primvarsSchema = HdPrimvarsSchema::GetFromParent(sceneIndexPrim.dataSource);

        auto faceVertexCountsDataSource = meshSchema.GetTopology().GetFaceVertexCounts();
        auto faceVertexCounts = faceVertexCountsDataSource->GetTypedValue(0.0f);

        auto faceVertexIndicesDataSource = meshSchema.GetTopology().GetFaceVertexIndices();
        auto faceVertexIndices = faceVertexIndicesDataSource->GetTypedValue(0.0f);

        auto primvarSchema = primvarsSchema.GetPrimvar(TfToken("points"));
        auto pointsDataSource = primvarSchema.GetPrimvarValue();
        auto pointsValue = pointsDataSource->GetValue(0);
        if (!pointsValue.CanCast<VtVec3fArray>()) {
            continue;
        }
        VtValue pointsTmp = VtValue::Cast<VtVec3fArray>(pointsValue);
        VtVec3fArray points = pointsTmp.UncheckedGet<VtVec3fArray>();

        Triangulation triangulation(points, faceVertexIndices, faceVertexCounts);
        triangulation.Triangulate();
        VtIntArray triangulationFlags;
        VtVec3iArray triangulationIndices;
        triangulation.GetFlags(triangulationFlags);
        triangulation.GetIndices(triangulationIndices);

        HdMeshSchema meshSchemaCopy =
            pxr::HdMeshSchema::Builder()
            .SetTopology(pxr::HdMeshTopologySchema::Builder()
                .SetFaceVertexCounts(meshSchema.GetTopology().GetFaceVertexCounts())
                .SetFaceVertexIndices(meshSchema.GetTopology().GetFaceVertexIndices())
                .SetHoleIndices(meshSchema.GetTopology().GetHoleIndices())
                .SetOrientation(meshSchema.GetTopology().GetOrientation())
                .SetTriangulation(
                    pxr::HdMeshTriangulationSchema::Builder()
                    .SetTriangleFlags(HdRetainedTypedSampledDataSource<VtIntArray>::New(triangulationFlags))
                    .SetTriangleIndices(HdRetainedTypedSampledDataSource<VtVec3iArray>::New(triangulationIndices))
                    .Build()
                )
                .Build())
            .SetSubdivisionScheme(meshSchema.GetSubdivisionScheme())
            .SetSubdivisionTags(meshSchema.GetSubdivisionTags().GetContainer())
            .SetGeomSubsets(meshSchema.GetGeomSubsets().GetContainer())
            .SetDoubleSided(meshSchema.GetDoubleSided())
            .Build();

        HdSceneIndexPrim prim;
        prim.primType = pxr::HdPrimTypeTokens->mesh;
        prim.dataSource = HdRetainedContainerDataSource::New(
            HdPrimTypeTokens->mesh, meshSchemaCopy.GetContainer(),
            HdPrimvarsSchemaTokens->primvars, primvarsSchema.GetContainer()
        );
        this->cache[entry.primPath] = prim;
    }

    _SendPrimsAdded(entries);
}

void TriangulationSceneIndex::_PrimsRemoved(
    const HdSceneIndexBase & sender,
    const HdSceneIndexObserver::RemovedPrimEntries & entries)
{
    if (!_IsObserved())
    {
        return;
    }

    // TODO: remove the result scene index prim
    _SendPrimsRemoved(entries);
}

void TriangulationSceneIndex::_PrimsDirtied(
    const HdSceneIndexBase & sender,
    const HdSceneIndexObserver::DirtiedPrimEntries & entries)
{
    if (!_IsObserved())
    {
        return;
    }

    // TODO: update our tessellation
    _SendPrimsDirtied(entries);
}

PXR_NAMESPACE_CLOSE_SCOPE
