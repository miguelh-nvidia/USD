#ifndef PXR_IMAGING_HD_ST_TRIANGULATION_SCENE_INDEX_PLUGIN_H
#define PXR_IMAGING_HD_ST_TRIANGULATION_SCENE_INDEX_PLUGIN_H

#include "pxr/pxr.h"
#include "pxr/imaging/hdSt/api.h"
#include "pxr/imaging/hd/sceneIndexPlugin.h"

PXR_NAMESPACE_OPEN_SCOPE

class TriangulationSceneIndexPlugin : public HdSceneIndexPlugin
{
public:
    TriangulationSceneIndexPlugin();

protected:
    HdSceneIndexBaseRefPtr _AppendSceneIndex(
        const HdSceneIndexBaseRefPtr& inputScene,
        const HdContainerDataSourceHandle& inputArgs) override;
};

PXR_NAMESPACE_CLOSE_SCOPE

#endif // PXR_IMAGING_HD_ST_TRIANGULATION_SCENE_INDEX_PLUGIN_H