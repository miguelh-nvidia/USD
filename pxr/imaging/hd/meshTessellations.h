// TODO
#ifndef PXR_IMAGING_HD_GEOM_TESSELLATIONS_H
#define PXR_IMAGING_HD_GEOM_TESSELLATIONS_H

#include "pxr/pxr.h"
#include "pxr/imaging/hd/api.h"
#include "pxr/imaging/hd/version.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/gf/vec3i.h"

PXR_NAMESPACE_OPEN_SCOPE

/// \class HdMeshTessellations
///
/// A mesh tessellation is linked to a mesh, they are referenced by faceIndex.
///
struct HdMeshTessellations {
    /// The face indices with tessellations.
    VtIntArray faceIndices;
    /// The number of tessellations per face.
    VtIntArray faceTessellations;
    /// The number of vertices per tessellation.
    VtIntArray tessellationVertexCounts;
    /// The array of indices of tessellation.
    VtIntArray tessellationVertexIndices;

    /// Conver the group of tessellations as triangulations
    void ComputeTriangleIndices(
        VtIntArray& trianglesPerFaces,
        VtVec3iArray& trianglesIndices, 
        VtIntArray& triangleFlags
    ) const;
};

HD_API
bool operator==(const HdMeshTessellations& lhs, const HdMeshTessellations& rhs);

PXR_NAMESPACE_CLOSE_SCOPE

#endif // PXR_IMAGING_HD_GEOM_TESSELLATIONS_H
