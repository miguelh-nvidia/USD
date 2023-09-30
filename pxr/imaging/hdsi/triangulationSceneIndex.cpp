#include "pxr/imaging/hdsi/triangulationSceneIndex.h"

#include "pxr/imaging/hd/meshSchema.h"
#include "pxr/imaging/hd/meshTopologySchema.h"
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
    std::cout << "Created TriangulationSceneIndex" << std::endl;
    return TfCreateRefPtr(new TriangulationSceneIndex(inputSceneIndex));
}

TriangulationSceneIndex::TriangulationSceneIndex(
    const HdSceneIndexBaseRefPtr& inputSceneIndex) :
    HdSingleInputFilteringSceneIndexBase(inputSceneIndex)
{
    std::cout << "TriangulationSceneIndex::TriangulationSceneIndex" << std::endl;
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

auto printDataSourceNames = [&](pxr::HdDataSourceBaseHandle dataSource, const pxr::TfToken& name, size_t depth, auto& printDataSourceNames) mutable -> void
{
    if (auto containerDataSource = pxr::HdContainerDataSource::Cast(dataSource))
    {
        for (auto name : containerDataSource->GetNames())
        {
            for (size_t i = 0; i < depth; ++i)
                std::cout << "  ";
            std::cout << name.GetText() << std::endl;
            printDataSourceNames(containerDataSource->Get(name), name, depth + 1, printDataSourceNames);
        }
    }
    else if (auto sampledDataSource = pxr::HdSampledDataSource::Cast(dataSource)) {
        for (size_t i = 0; i < depth; ++i)
            std::cout << "  ";
        std::cout << name.GetText() << " " << sampledDataSource->GetValue(0) << std::endl;
    }
    else if (auto vectorDataSource = pxr::HdVectorDataSource::Cast(dataSource)) {
        for (size_t i = 0; i < depth; ++i)
            std::cout << "  ";
        std::cout << name.GetText() << " " << vectorDataSource->GetNumElements() << " elements" << std::endl;
    }
};

void TriangulationSceneIndex::_PrimsAdded(
    const HdSceneIndexBase & sender,
    const HdSceneIndexObserver::AddedPrimEntries & entries)
{
    if (!_IsObserved())
    {
        return;
    }

    std::cout << "PrimsAdded" << std::endl;
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
        std::cout << "Compute for " << entry.primPath << std::endl;
        //HdSceneIndexPrim sceneIndexPrim = this->_GetInputSceneIndex()->GetPrim(entry.primPath);
        //printDataSourceNames(sceneIndexPrim.dataSource, entry.primType, 1, printDataSourceNames);

        HdDataSourceLocator faceVertexCountsLocator(HdPrimTypeTokens->mesh, HdTokens->topology, HdMeshTopologySchemaTokens->faceVertexCounts);
        auto faceVertexCountsDataSource = HdIntArrayDataSource::Cast(GetDataSource(entry.primPath, faceVertexCountsLocator));
        auto faceVertexCounts = faceVertexCountsDataSource->GetTypedValue(0);
        std::cout << faceVertexCounts << std::endl;

        HdDataSourceLocator faceVertexIndicesLocator(HdPrimTypeTokens->mesh, HdTokens->topology, HdMeshTopologySchemaTokens->faceVertexIndices);
        auto faceVertexIndicesDataSource = HdIntArrayDataSource::Cast(GetDataSource(entry.primPath, faceVertexIndicesLocator));
        auto faceVertexIndices = faceVertexIndicesDataSource->GetTypedValue(0);
        std::cout << faceVertexIndices << std::endl;

        HdDataSourceLocator pointsLocator(TfToken("primvars"), TfToken("points"), HdPrimvarSchemaTokens->primvarValue);
        auto pointsDataSource = HdSampledDataSource::Cast(GetDataSource(entry.primPath, pointsLocator));
        auto pointsValue = pointsDataSource->GetValue(0);
        if (!pointsValue.CanCast<VtVec3fArray>()) {
            continue;
        }
        VtValue pointsTmp = VtValue::Cast<VtVec3fArray>(pointsValue);
        VtVec3fArray points = pointsTmp.UncheckedGet<VtVec3fArray>();
        std::cout << points << std::endl;

        Triangulation triangulation(points, faceVertexIndices, faceVertexCounts);
        triangulation.Triangulate();
        VtIntArray triangulationFlags;
        VtVec3iArray triangulationIndices;
        triangulation.GetFlags(triangulationFlags);
        triangulation.GetIndices(triangulationIndices);

        HdSceneIndexPrim prim;
        prim.primType = pxr::HdPrimTypeTokens->mesh;

        auto meshPointsDs = HdRetainedTypedSampledDataSource<VtVec3fArray>::New(points);
        auto faceVertexCountsDs = HdRetainedTypedSampledDataSource<VtIntArray>::New(faceVertexCounts);
        auto faceVertexIndicesDs = HdRetainedTypedSampledDataSource<VtIntArray>::New(faceVertexIndices);
        auto triangulationFlagsDs = HdRetainedTypedSampledDataSource<VtIntArray>::New(triangulationFlags);
        auto triangulationIndicesDs = HdRetainedTypedSampledDataSource<VtVec3iArray>::New(triangulationIndices);

        auto meshDs =
            pxr::HdMeshSchema::Builder()
            .SetTopology(pxr::HdMeshTopologySchema::Builder()
                .SetFaceVertexCounts(faceVertexCountsDs)
                .SetFaceVertexIndices(faceVertexIndicesDs)
                .SetTriangulationFlags(triangulationFlagsDs)
                .SetTriangulationIndices(triangulationIndicesDs)
                .Build())
            .SetDoubleSided(
                HdRetainedTypedSampledDataSource<bool>::New(true))
            .Build();
        auto primvarDs = HdRetainedContainerDataSource::New(
            HdPrimvarsSchemaTokens->points,
            HdPrimvarSchema::Builder()
            .SetPrimvarValue(meshPointsDs)
            .SetInterpolation(HdPrimvarSchema::
                BuildInterpolationDataSource(
                    HdPrimvarSchemaTokens->vertex))
            .SetRole(HdPrimvarSchema::
                BuildRoleDataSource(
                    HdPrimvarSchemaTokens->point))
            .Build()
        );
        prim.dataSource = HdRetainedContainerDataSource::New(
            HdPrimTypeTokens->mesh, meshDs,
            HdPrimvarsSchemaTokens->primvars, primvarDs
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