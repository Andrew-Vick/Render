#ifndef MESH_IMPORT_H
#define MESH_IMPORT_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <string>
#include <iostream>

struct Vertex
{
    float position[3];
    float normal[3];
    float texCoords[2];
};

struct Mesh
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
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
            else
            {
                // If no normals, set default normal (optional)
                vertex.normal[0] = 0.0f;
                vertex.normal[1] = 0.0f;
                vertex.normal[2] = 0.0f;
            }

            if (ai_mesh->HasTextureCoords(0))
            {
                vertex.texCoords[0] = ai_mesh->mTextureCoords[0][i].x;
                vertex.texCoords[1] = ai_mesh->mTextureCoords[0][i].y;
            }
            else
            {
                vertex.texCoords[0] = 0.0f;
                vertex.texCoords[1] = 0.0f;
            }

            mesh.vertices.push_back(vertex);
        }

        // Process indices
        for (unsigned int i = 0; i < ai_mesh->mNumFaces; i++)
        {
            aiFace face = ai_mesh->mFaces[i];
            if (face.mNumIndices != 3)
            {
                // Skip non-triangular faces
                continue;
            }
            mesh.indices.push_back(face.mIndices[0]);
            mesh.indices.push_back(face.mIndices[1]);
            mesh.indices.push_back(face.mIndices[2]);
        }

        return mesh;
    }

public:
    void convertMeshToTriangles(const Mesh &mesh, std::vector<shared_ptr<hittable>> &triangles, shared_ptr<material> mat)
    {
        for (size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            unsigned int idx0 = mesh.indices[i];
            unsigned int idx1 = mesh.indices[i + 1];
            unsigned int idx2 = mesh.indices[i + 2];

            if (idx0 >= mesh.vertices.size() || idx1 >= mesh.vertices.size() || idx2 >= mesh.vertices.size())
            {
                std::cerr << "ERROR: Vertex index out of bounds." << std::endl;
                continue;
            }

            Vertex v0 = mesh.vertices[idx0];
            Vertex v1 = mesh.vertices[idx1];
            Vertex v2 = mesh.vertices[idx2];

            triangles.push_back(make_shared<tri>(
                point3(v0.position[0], v0.position[1], v0.position[2]),
                point3(v1.position[0], v1.position[1], v1.position[2]),
                point3(v2.position[0], v2.position[1], v2.position[2]),
                mat));
        }
    }
};

#endif
