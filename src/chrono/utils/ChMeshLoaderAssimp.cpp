// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// =============================================================================
// Branch-local addition (not upstream Chrono). See ChMeshLoaderAssimp.h.
// =============================================================================

#include "chrono/utils/ChMeshLoaderAssimp.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>

namespace chrono {
namespace utils {

// Recursively walk the Assimp node hierarchy, accumulating each node's local
// transform, and append every referenced mesh's geometry (transformed into
// the scene's root space) into the output buffers.
static void AppendAssimpNode(const aiScene* scene,
                             const aiNode* node,
                             const aiMatrix4x4& parent_transform,
                             std::vector<ChVector3d>& vertices,
                             std::vector<ChVector3d>& normals,
                             std::vector<ChVector3i>& face_v,
                             std::vector<ChVector3i>& face_n) {
    aiMatrix4x4 transform = parent_transform * node->mTransformation;
    aiMatrix3x3 normal_transform(transform);  // rotation/scale part, for normals
    normal_transform.Inverse().Transpose();

    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        const aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        unsigned int base_index = static_cast<unsigned int>(vertices.size());

        for (unsigned int v = 0; v < mesh->mNumVertices; v++) {
            aiVector3D p = transform * mesh->mVertices[v];
            vertices.emplace_back(p.x, p.y, p.z);
            if (mesh->HasNormals()) {
                aiVector3D n = (normal_transform * mesh->mNormals[v]).Normalize();
                normals.emplace_back(n.x, n.y, n.z);
            }
        }

        for (unsigned int f = 0; f < mesh->mNumFaces; f++) {
            const aiFace& face = mesh->mFaces[f];
            if (face.mNumIndices != 3)
                continue;  // triangulated by aiProcess_Triangulate, should not happen
            ChVector3i tri(base_index + face.mIndices[0], base_index + face.mIndices[1],
                           base_index + face.mIndices[2]);
            face_v.push_back(tri);
            if (mesh->HasNormals())
                face_n.push_back(tri);
        }
    }

    for (unsigned int c = 0; c < node->mNumChildren; c++)
        AppendAssimpNode(scene, node->mChildren[c], transform, vertices, normals, face_v, face_n);
}

std::shared_ptr<ChTriangleMeshConnected> LoadMeshWithAssimp(const std::string& filename) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        filename, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals);

    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    if (!scene || !scene->HasMeshes()) {
        std::cerr << "utils::LoadMeshWithAssimp: failed to load '" << filename << "': " << importer.GetErrorString()
                  << std::endl;
        return trimesh;  // empty mesh - caller renders nothing for this shape, but does not crash
    }

    AppendAssimpNode(scene, scene->mRootNode, aiMatrix4x4(), trimesh->GetCoordsVertices(),
                     trimesh->GetCoordsNormals(), trimesh->GetIndicesVertexes(), trimesh->GetIndicesNormals());

    return trimesh;
}

}  // namespace utils
}  // namespace chrono
