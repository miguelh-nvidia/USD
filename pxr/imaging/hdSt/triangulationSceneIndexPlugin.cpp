#include "pxr/imaging/hdSt/triangulationSceneIndexPlugin.h"

#include "pxr/imaging/hd/sceneIndexPluginRegistry.h"
#include "pxr/imaging/hdsi/triangulationSceneIndex.h"

PXR_NAMESPACE_OPEN_SCOPE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((sceneIndexPluginName, "TriangulationSceneIndexPlugin"))
);

static const char* const _pluginDisplayName = "GL";

TF_REGISTRY_FUNCTION(TfType)
{
    HdSceneIndexPluginRegistry::Define<TriangulationSceneIndexPlugin>();
}

TF_REGISTRY_FUNCTION(HdSceneIndexPlugin)
{
    const HdSceneIndexPluginRegistry::InsertionPhase insertionPhase = 0;

    HdSceneIndexPluginRegistry::GetInstance().RegisterSceneIndexForRenderer(
        _pluginDisplayName,
        _tokens->sceneIndexPluginName,
        nullptr,
        insertionPhase,
        HdSceneIndexPluginRegistry::InsertionOrderAtStart);
}

TriangulationSceneIndexPlugin::
TriangulationSceneIndexPlugin() = default;

HdSceneIndexBaseRefPtr
TriangulationSceneIndexPlugin::_AppendSceneIndex(
    const HdSceneIndexBaseRefPtr & inputScene,
    const HdContainerDataSourceHandle & inputArgs)
{
    return TriangulationSceneIndex::New(inputScene);
}

PXR_NAMESPACE_CLOSE_SCOPE
