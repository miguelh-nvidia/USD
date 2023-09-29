#ifndef PXR_IMAGING_HDSI_TRIANGULATION_SCENE_INDEX_H
#define PXR_IMAGING_HDSI_TRIANGULATION_SCENE_INDEX_H

#include "pxr/pxr.h"
#include "pxr/imaging/hdsi/api.h"
#include "pxr/imaging/hd/filteringSceneIndex.h"

#include "pxr/imaging/hdsi/triangulation.h"

PXR_NAMESPACE_OPEN_SCOPE

TF_DECLARE_REF_PTRS(TriangulationSceneIndex);

class TriangulationSceneIndex : public HdSingleInputFilteringSceneIndexBase
{
public:
    HDSI_API
    static TriangulationSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr& inputSceneIndex);

    HDSI_API
    HdSceneIndexPrim GetPrim(const SdfPath& primPath) const override;

    HDSI_API
    SdfPathVector GetChildPrimPaths(const SdfPath& primPath) const override;

protected:

    TriangulationSceneIndex(
        const HdSceneIndexBaseRefPtr& inputSceneIndex);

    virtual void _PrimsAdded(const HdSceneIndexBase& sender,
        const HdSceneIndexObserver::AddedPrimEntries& entries) override;

    virtual void _PrimsRemoved(const HdSceneIndexBase& sender,
        const HdSceneIndexObserver::RemovedPrimEntries& entries) override;

    virtual void _PrimsDirtied(const HdSceneIndexBase& sender,
        const HdSceneIndexObserver::DirtiedPrimEntries& entries) override;

private:

    std::unordered_map<SdfPath, HdSceneIndexPrim, SdfPath::Hash> cache;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // PXR_IMAGING_HDSI_TRIANGULATION_SCENE_INDEX_H
