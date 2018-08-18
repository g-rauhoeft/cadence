#include <iostream>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>

typedef OpenMesh::PolyMesh_ArrayKernelT<> OpenMeshMesh;
typedef OpenMesh::Decimater::DecimaterT<OpenMeshMesh> DecimatOr;
typedef OpenMesh::Decimater::ModQuadricT<OpenMeshMesh>::Handle QuadricModule;

struct PipelineData {
    std::string inputFile;
    std::string inputType;
    std::string outputFile;
    std::string outputType;
    unsigned int flags;
};

void getFileType(const std::string &filename, std::string &filetype){
    std::size_t found = filename.find_last_of(".");
    
    if(found != std::string::npos && found < filename.size()-1){
        filetype = filename.substr(found+1, filename.size());
    }
}

void parseFlags(PipelineData &pipelineData, const int argc, const char* argv[]){
    pipelineData.flags = 0;
}

void parseArgs(PipelineData &pipelineData, const int argc, const char* argv[]){
    for(size_t i = 1; i < argc-1; i++){
        const char* &arg = argv[i];
        const char* &next = argv[i+1];
        if(std::strcmp(arg, "-i") == 0){
            pipelineData.inputFile = next;
            getFileType(next, pipelineData.inputType);
        }
        if(std::strcmp(arg, "-o") == 0){
            pipelineData.outputFile = next;
            getFileType(next, pipelineData.outputType);
        }
    }
}

const aiScene* importModel(Assimp::Importer &importer, const PipelineData pipelineData){
    return importer.ReadFile(pipelineData.inputFile, pipelineData.flags); 
}

const aiExportFormatDesc* findFormat(Assimp::Exporter &exporter, const PipelineData &pipelineData){
    for(size_t i = 0; i < exporter.GetExportFormatCount(); i++){
        const aiExportFormatDesc* const e = exporter.GetExportFormatDescription(i);
        if(e->fileExtension == pipelineData.outputType){
            return e;
        }
    }
    return 0;
}

void exportModel(Assimp::Exporter &exporter, const PipelineData &pipelineData, aiScene *scene){
    std::cout << "Exporting model..." << std::endl;
    const aiExportFormatDesc* const exportDescription = findFormat(exporter, pipelineData);
    exporter.Export(scene, exportDescription->id, pipelineData.outputFile);
}

void assimpToOpenMesh(const aiScene* &scene, OpenMeshMesh &target){
    if(scene->HasMeshes()){
        for(size_t meshIndex = 0; meshIndex < scene->mNumMeshes; meshIndex++){
            aiMesh* mesh = scene->mMeshes[meshIndex];
            std::vector<OpenMeshMesh::VertexHandle> vertexHandles;
            for(size_t vertexIndex = 0; vertexIndex < mesh->mNumVertices; vertexIndex++){
                aiVector3D &vertex = mesh->mVertices[vertexIndex];
                vertexHandles.push_back(target.add_vertex(OpenMeshMesh::Point(vertex.x, vertex.y, vertex.z)));
            }
            for(size_t faceIndex = 0; faceIndex < mesh->mNumFaces; faceIndex++){
                aiFace &face = mesh->mFaces[faceIndex];
                std::vector<OpenMeshMesh::VertexHandle> faceVertexHandles;
                for(size_t index = 0; index < face.mNumIndices; index++){
                    faceVertexHandles.push_back(vertexHandles[face.mIndices[index]]);
                }
                target.add_face(faceVertexHandles);
                faceVertexHandles.clear();
            }
            for(size_t normalIndex = 0; normalIndex < mesh->mNumVertices; normalIndex++){
                aiVector3D &normal = mesh->mNormals[normalIndex];
                // TODO: Add to OpenMeshMesh
            }
            vertexHandles.clear();
        }
    }
}

void decimate(OpenMeshMesh &mesh, const PipelineData &pipelineData){
    DecimatOr decimatOr(mesh);
    QuadricModule quadricModule;
    decimatOr.add(quadricModule);

    decimatOr.module(quadricModule).unset_max_err();
    decimatOr.initialize();
    decimatOr.decimate_to_faces(15000, 5000);
    mesh.garbage_collection();
}

void openMeshToAssimp(OpenMeshMesh &mesh, aiScene &scene){
    aiMesh* newMesh = new aiMesh;
    scene.mNumMeshes = 1;
    scene.mMeshes = new aiMesh*[1];
    scene.mMeshes[0] = newMesh;
    scene.mMeshes[0]->mMaterialIndex = 0;
    scene.mMaterials = new aiMaterial*[1];
    std::cout << scene.HasMeshes() << std::endl;
    aiMaterial* newMaterial = new aiMaterial;
    scene.mMaterials[0] = newMaterial;
    scene.mNumMaterials = 1;
    newMesh->mMaterialIndex = 0;

    scene.mRootNode = new aiNode();
    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mMeshes[0] = 0;
    scene.mRootNode->mNumMeshes = 1;

    newMesh->mNumVertices = mesh.n_vertices();
    newMesh->mVertices = new aiVector3D[mesh.n_vertices()];    

    std::cout << "Number of vertices: " << mesh.n_vertices() << std::endl;
    for(size_t vertexIndex = 0; vertexIndex < mesh.n_vertices(); vertexIndex++){
        const OpenMeshMesh::Point &point = mesh.point((OpenMeshMesh::VertexHandle)vertexIndex);
        //std::cout << point[0] << " " << point[1] << " " << point[2] << std::endl;
        newMesh->mVertices[vertexIndex] = aiVector3D(point[0], point[1], point[2]);
    }

    newMesh->mNumFaces = mesh.n_faces();
    newMesh->mFaces = new aiFace[mesh.n_faces()];
    std::vector<int> vertexIndices;
    std::cout << "Number of faces: " << mesh.n_faces() << std::endl;
    for(size_t faceIndex = 0; faceIndex < mesh.n_faces(); faceIndex++){
        aiFace newFace;
        size_t indexCount = 0;
        for(OpenMeshMesh::FaceVertexCCWIter faceIterator = mesh.fv_ccwiter((OpenMeshMesh::FaceHandle)faceIndex); faceIterator.is_valid(); faceIterator++){
            indexCount++;
            vertexIndices.push_back((*faceIterator).idx());
        }
        newFace.mNumIndices = indexCount;
        newFace.mIndices = new unsigned int[indexCount];
        for(size_t i = 0; i < indexCount; i++){
            newFace.mIndices[i] = vertexIndices[i];
        }
        newMesh->mFaces[faceIndex] = newFace;
        vertexIndices.clear();
    }
}

int main(const int argc, const char* argv[]){
    PipelineData pipelineData;
    parseArgs(pipelineData, argc, argv);
    parseFlags(pipelineData, argc, argv);
    Assimp::Importer importer;
    Assimp::Exporter exporter;

    const aiScene *scene = importModel(importer, pipelineData);
    if(!scene){
        std::cout << importer.GetErrorString() << std::endl;
        return -1;
    }

    OpenMeshMesh mesh;
    assimpToOpenMesh(scene, mesh);    
    std::cout << "Decimating" << std::endl;
    decimate(mesh, pipelineData);
    OpenMesh::IO::write_mesh(mesh, "openmeshoutput.obj");
    aiScene targetScene;
    openMeshToAssimp(mesh, targetScene);

    exportModel(exporter, pipelineData, &targetScene);

    return 0;
}