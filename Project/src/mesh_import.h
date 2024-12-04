#ifndef MESH_IMPORT_H
#define MESH_IMPORT_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <iostream>
#include "color.h"
#include "aabb.h"
struct Vertex
{
    float position[3];
    float normal[3];
    float texCoords[2];
    int materialIndex;
};

struct Material
{
    std::string name;
    std::shared_ptr<texture> diffuseTexture;
    color diffuseColor;
};

struct Mesh
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<shared_ptr<material>> materials; // Add materials to the mesh
    aabb bounding_box;
};

class MeshImporter
{
public:
    static bool LoadMesh(const std::string &filePath, std::vector<Mesh> &meshes)
    {
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(filePath,
                                                 aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals);

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            std::cerr << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
            return false;
        }

        processNode(scene->mRootNode, scene, meshes);
        return true;
    }

private:
    static void processNode(aiNode *node, const aiScene *scene, std::vector<Mesh> &meshes)
    {
        // Process all the node's meshes
        for (unsigned int i = 0; i < node->mNumMeshes; i++)
        {
            aiMesh *ai_mesh = scene->mMeshes[node->mMeshes[i]];
            meshes.push_back(processMesh(ai_mesh, scene));
        }
        // Then process children nodes
        for (unsigned int i = 0; i < node->mNumChildren; i++)
        {
            processNode(node->mChildren[i], scene, meshes);
        }
    }

    static Mesh processMesh(aiMesh *ai_mesh, const aiScene *scene)
    {
        Mesh mesh;

        // Process vertices
        for (unsigned int i = 0; i < ai_mesh->mNumVertices; i++)
        {
            Vertex vertex;
            vertex.position[0] = ai_mesh->mVertices[i].x;
            vertex.position[1] = ai_mesh->mVertices[i].y;
            vertex.position[2] = ai_mesh->mVertices[i].z;

            if (ai_mesh->HasNormals())
            {
                vertex.normal[0] = ai_mesh->mNormals[i].x;
                vertex.normal[1] = ai_mesh->mNormals[i].y;
                vertex.normal[2] = ai_mesh->mNormals[i].z;
            }

            if (ai_mesh->HasTextureCoords(0))
            {
                vertex.texCoords[0] = ai_mesh->mTextureCoords[0][i].x;
                vertex.texCoords[1] = 1.0f - ai_mesh->mTextureCoords[0][i].y;
            }
            else
            {
                vertex.texCoords[0] = 0.0f;
                vertex.texCoords[1] = 0.0f;
            }

            vertex.materialIndex = ai_mesh->mMaterialIndex;
            mesh.vertices.push_back(vertex);
        }

        // Process indices (no change)
        for (unsigned int i = 0; i < ai_mesh->mNumFaces; i++)
        {
            aiFace face = ai_mesh->mFaces[i];
            for (unsigned int j = 0; j < face.mNumIndices; j++)
            {
                unsigned int index = face.mIndices[j];
                if (index < ai_mesh->mNumVertices)
                {
                    mesh.indices.push_back(index);
                }
            }
        }

        mesh.bounding_box = aabb(point3(ai_mesh->mAABB.mMin.x, ai_mesh->mAABB.mMin.y, ai_mesh->mAABB.mMin.z),
                                 point3(ai_mesh->mAABB.mMax.x, ai_mesh->mAABB.mMax.y, ai_mesh->mAABB.mMax.z));

        // Process materials
        if (scene->HasMaterials())
        {
            std::clog << "has materials" << std::endl;
            for (unsigned int i = 0; i < scene->mNumMaterials; i++)
            {
                aiMaterial *ai_material = scene->mMaterials[i];
                shared_ptr<material> mat;

                // Load diffuse texture if available
                if (ai_material->GetTextureCount(aiTextureType_DIFFUSE) > 0)
                {
                    std::clog << "has diffuse" << std::endl;
                    aiString path;
                    if (ai_material->GetTexture(aiTextureType_DIFFUSE, 0, &path) == AI_SUCCESS)
                    {
                        std::clog << "Loading texture: " << path.C_Str() << std::endl;
                        // Use the actual texture path from the material
                        auto texture = make_shared<image_texture>(path.C_Str());
                        mat = make_shared<lambertian>(texture);
                    }
                }
                else
                {
                    std::clog << "no diffuse" << std::endl;
                    aiColor3D aiColor(0.f, 0.f, 0.f);
                    if (AI_SUCCESS == ai_material->Get(AI_MATKEY_COLOR_DIFFUSE, aiColor))
                    {
                        std::clog << "Loading color: " << aiColor.r << " " << aiColor.g << " " << aiColor.b << std::endl;
                        mat = make_shared<lambertian>(color(1.0, 0.0, 0.0));
                    }
                }

                if (!mat)
                {
                    std::clog << "Default material" << std::endl;
                    mat = make_shared<lambertian>(color(1.0, 0.0, 0.0)); // Default material
                }

                mesh.materials.push_back(mat);
            }
        }

        return mesh;
    }

public:
    void convertMeshToTriangles(const Mesh &mesh, std::vector<shared_ptr<hittable>> &triangles)
    {
        for (size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            unsigned int idx0 = mesh.indices[i];
            unsigned int idx1 = mesh.indices[i + 1];
            unsigned int idx2 = mesh.indices[i + 2];

            // Ensure indices are within bounds
            if (idx0 >= mesh.vertices.size() || idx1 >= mesh.vertices.size() || idx2 >= mesh.vertices.size())
                continue;

            const Vertex &v0 = mesh.vertices[idx0];
            const Vertex &v1 = mesh.vertices[idx1];
            const Vertex &v2 = mesh.vertices[idx2];

            // Apply scaling and translation to vertex positions
            point3 p0 = point3(v0.position[0], v0.position[1], v0.position[2]);
            point3 p1 = point3(v1.position[0], v1.position[1], v1.position[2]);
            point3 p2 = point3(v2.position[0], v2.position[1], v2.position[2]);

            // Compute edge vectors for the triangle
            vec3 aa = p1 - p0;
            vec3 ab = p2 - p0;

            // Retrieve the material for the triangle
            shared_ptr<material> mat = mesh.materials[v0.materialIndex];

            // Construct the triangle using the existing constructor
            vec3 uv0 = {v0.texCoords[0], v0.texCoords[1], 0};
            vec3 uv1 = {v1.texCoords[0], v1.texCoords[1], 0};
            vec3 uv2 = {v2.texCoords[0], v2.texCoords[1], 0};

            triangles.push_back(make_shared<tri>(p0, aa, ab, mat, uv0, uv1, uv2));
        }
    }

    void mesh_to_tri_mat(const Mesh &mesh, std::vector<shared_ptr<hittable>> &triangles, shared_ptr<material> mat)
    {
        for (size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            unsigned int idx0 = mesh.indices[i];
            unsigned int idx1 = mesh.indices[i + 1];
            unsigned int idx2 = mesh.indices[i + 2];

            // Ensure indices are within bounds
            if (idx0 >= mesh.vertices.size() || idx1 >= mesh.vertices.size() || idx2 >= mesh.vertices.size())
                continue;

            const Vertex &v0 = mesh.vertices[idx0];
            const Vertex &v1 = mesh.vertices[idx1];
            const Vertex &v2 = mesh.vertices[idx2];

            // Apply scaling and translation to vertex positions
            point3 p0 = point3(v0.position[0], v0.position[1], v0.position[2]);
            point3 p1 = point3(v1.position[0], v1.position[1], v1.position[2]);
            point3 p2 = point3(v2.position[0], v2.position[1], v2.position[2]);

            // Compute edge vectors for the triangle
            vec3 aa = p1 - p0;
            vec3 ab = p2 - p0;

            triangles.push_back(make_shared<triangle>(p0, aa, ab, mat));
        }
    }
};

#endif
