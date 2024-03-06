// TODO
#ifndef PXR_IMAGING_HD_GEOM_TESSELLATION_H
#define PXR_IMAGING_HD_GEOM_TESSELLATION_H

#include "pxr/pxr.h"
#include "pxr/imaging/hd/api.h"
#include "pxr/imaging/hd/version.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/gf/vec3i.h"

PXR_NAMESPACE_OPEN_SCOPE

/// \class HdMeshTessellation
///
/// TODO
///
struct HdMeshTessellation {
    /// TODO
    size_t faceIndex;
    /// TODO
    VtIntArray counts;
    /// TODO
    VtIntArray indices;

    /// TODO
    void ComputeTriangles(VtVec3iArray& triangles, VtIntArray& flags) const;
};

/// A group of geometry tessellations.
typedef std::vector<HdMeshTessellation> HdMeshTessellations;

HD_API
bool operator==(const HdMeshTessellation& lhs, const HdMeshTessellation& rhs);

PXR_NAMESPACE_CLOSE_SCOPE

#endif // PXR_IMAGING_HD_GEOM_TESSELLATION_H
