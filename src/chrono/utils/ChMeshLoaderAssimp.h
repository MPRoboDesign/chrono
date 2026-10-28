// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// =============================================================================
// Branch-local addition (not upstream Chrono).
// =============================================================================
//
// Loads a mesh file of any format Assimp supports (e.g. .dae, .stl, .fbx)
// into a ChTriangleMeshConnected. Chrono's own ChTriangleMeshConnected only
// natively parses Wavefront .obj (CreateFromWavefrontFile) and .stl
// (CreateFromSTLFile); consumers of ChVisualShapeModelFile (the OptiX sensor
// engine and the Irrlicht visualization system) both historically only
// called CreateFromWavefrontFile regardless of the referenced file's actual
// extension, which silently produces a zero-vertex mesh for any other
// format - see docs/chrono_installation.md in the quardet repo for the full
// writeup of the bug this fixes.
//
// =============================================================================

#ifndef CH_MESH_LOADER_ASSIMP_H
#define CH_MESH_LOADER_ASSIMP_H

#include "chrono/core/ChApiCE.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include <memory>
#include <string>

namespace chrono {
namespace utils {

/// Load a mesh file via Assimp (any format it supports - .dae, .stl, .fbx, etc.)
/// into a ChTriangleMeshConnected, correctly walking the scene's node
/// hierarchy and applying each node's accumulated transform to its
/// geometry. This step is required: Collada/.dae files in particular
/// commonly carry a root-level transform (e.g. a Y-up -> Z-up axis
/// conversion) that is not baked into the raw per-mesh vertex data - reading
/// vertices directly without applying node transforms yields incorrectly
/// oriented/placed (or otherwise wrong) geometry.
///
/// On failure (file not found, unsupported/corrupt format), returns an empty
/// (but non-null) ChTriangleMeshConnected and logs an error - callers should
/// not crash on a load failure, just render nothing for that shape.
ChApi std::shared_ptr<ChTriangleMeshConnected> LoadMeshWithAssimp(const std::string& filename);

}  // namespace utils
}  // namespace chrono

#endif
