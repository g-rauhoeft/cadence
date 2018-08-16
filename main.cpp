#include <iostream>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::PolyMesh_ArrayKernelT<> OpenMeshMesh;

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

void exportModel(Assimp::Exporter &exporter, const PipelineData pipelineData, const aiScene* scene){
    std::cout << "Exporting model..." << std::endl;
    const aiExportFormatDesc* const exportDescription = findFormat(exporter, pipelineData);
    std::cout << "What!?" << std::endl;
    exporter.Export(scene, exportDescription->id, pipelineData.outputFile);
}

void assimpToOpenMesh(const aiScene* scene, OpenMeshMesh &target){
    if(scene->HasMeshes()){
        for(size_t meshIndex = 0; meshIndex < scene->mNumMeshes; meshIndex++){
            aiMesh* mesh = scene->mMeshes[meshIndex];
            OpenMeshMesh::VertexHandle vertexHandles[mesh->mNumVertices];
            for(size_t vertexIndex = 0; vertexIndex < mesh->mNumVertices; vertexIndex++){
                aiVector3D &vertex = mesh->mVertices[vertexIndex];
                vertexHandles[vertexIndex] = target.add_vertex(OpenMeshMesh::Point(vertex.x, vertex.y, vertex.z));
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
        }
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
    std::cout << "Scene " << scene << std::endl; 

    OpenMeshMesh mesh;
    assimpToOpenMesh(scene, mesh);    

    exportModel(exporter, pipelineData, scene);

    return 0;
}