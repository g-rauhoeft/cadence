#include <iostream>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModHausdorffT.hh>
#include <OpenMesh/Tools/Decimater/ModAspectRatioT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Tools/Decimater/ModEdgeLengthT.hh>
#include <OpenMesh/Core/IO/importer/ImporterT.hh>

typedef OpenMesh::DefaultTraits OpenMeshTraits;
typedef OpenMesh::TriMesh_ArrayKernelT<OpenMeshTraits> OpenMeshMesh;
typedef OpenMesh::Decimater::DecimaterT<OpenMeshMesh> DecimatOr;
typedef OpenMesh::Decimater::ModHausdorffT<OpenMeshMesh>::Handle HausdorffModule;
typedef OpenMesh::Decimater::ModAspectRatioT<OpenMeshMesh>::Handle AspectRatioModule;
typedef OpenMesh::Decimater::ModQuadricT<OpenMeshMesh>::Handle QuadricModule;
typedef OpenMesh::Decimater::ModEdgeLengthT<OpenMeshMesh>::Handle EdgeLengthModule;
typedef OpenMesh::IO::ImporterT<OpenMeshMesh> Importer;

struct PipelineData
{
    PipelineData() : inputFile(""),
                     inputType(""),
                     outputFile(""),
                     outputType(""),
                     faces(0),
                     preserveBoundary(false),
                     aspectRatio(false),
                     hausdorff(false),
                     edgeLength(false),
		     flags(0)
    {
    }
    std::string inputFile;
    std::string inputType;
    std::string outputFile;
    std::string outputType;
    size_t faces;
    bool preserveBoundary;
    bool aspectRatio;
    bool hausdorff;
    bool edgeLength;
    unsigned int flags;
};

void printPipelineData(const PipelineData &pipelineData)
{
    std::cout << "Input File: \t\t\t" << pipelineData.inputFile << std::endl;
    std::cout << "Input Type: \t\t\t" << pipelineData.inputType << std::endl;
    std::cout << "Output File: \t\t\t" << pipelineData.outputFile << std::endl;
    std::cout << "Output Type: \t\t\t" << pipelineData.outputType << std::endl;
    std::cout << "Faces: \t\t\t\t" << pipelineData.faces << std::endl;
    std::cout << "Aspect Ratio: \t\t\t" << pipelineData.aspectRatio << std::endl;
    std::cout << "Hausdorff: \t\t\t" << pipelineData.hausdorff << std::endl;
    std::cout << "Edge Length: \t\t\t" << pipelineData.edgeLength << std::endl;
    std::cout << "Flags: \t\t\t\t" << pipelineData.flags << std::endl;
}

void printStats(const OpenMeshMesh &mesh){
    std::cout << "Vertices: \t\t\t" << mesh.n_vertices() << std::endl;
    std::cout << "Faces: \t\t\t\t" << mesh.n_faces() << std::endl;
}

void getFileType(const std::string &filename, std::string &filetype)
{
    std::size_t found = filename.find_last_of(".");

    if (found != std::string::npos && found < filename.size() - 1)
    {
        filetype = filename.substr(found + 1, filename.size());
    }
}

void parseFlags(PipelineData &pipelineData, const int argc, const char *argv[])
{
    pipelineData.flags = aiProcess_JoinIdenticalVertices
                        | aiProcess_Triangulate
                        | aiProcess_FindInvalidData
                        | aiProcess_FindDegenerates
                        | aiProcess_ValidateDataStructure;
}

void parseArgs(PipelineData &pipelineData, const int argc, const char *argv[])
{
    for (size_t i = 1; i < argc-1; i++)
    {
        const char *&arg = argv[i];
        const char *&next = argv[i + 1];
        if (std::strcmp(arg, "-i") == 0)
        {
            pipelineData.inputFile = next;
            getFileType(next, pipelineData.inputType);
        }
        if (std::strcmp(arg, "-o") == 0)
        {
            pipelineData.outputFile = next;
            getFileType(next, pipelineData.outputType);
        }
        if (std::strcmp(arg, "-f") == 0)
        {
            pipelineData.faces = std::stoi(next);
        }
        if (std::strcmp(arg, "-v") == 0)
        {
            pipelineData.faces = std::stoi(next) * 3;
        }
        if (std::strcmp(arg, "--prioritize-aspect-ratio") == 0)
        {
            pipelineData.aspectRatio = true;
        }
        if (std::strcmp(arg, "--use-hausdorff") == 0)
        {
            pipelineData.hausdorff = true;
        }
	if (std::strcmp(arg, "--prioritize-short-edges") == 0)
	{
	    pipelineData.edgeLength = true;
	}
    }
}

const aiScene *importModel(Assimp::Importer &importer, const PipelineData pipelineData)
{
    return importer.ReadFile(pipelineData.inputFile, pipelineData.flags);
}

const aiExportFormatDesc *findFormat(Assimp::Exporter &exporter, const PipelineData &pipelineData)
{
    for (size_t i = 0; i < exporter.GetExportFormatCount(); i++)
    {
        const aiExportFormatDesc *const e = exporter.GetExportFormatDescription(i);
        if (e->fileExtension == pipelineData.outputType)
        {
            return e;
        }
    }
    return 0;
}

void exportModel(Assimp::Exporter &exporter, const PipelineData &pipelineData, aiScene *scene)
{
    const aiExportFormatDesc *const exportDescription = findFormat(exporter, pipelineData);
    exporter.Export(scene, exportDescription->id, pipelineData.outputFile);
}

void assimpToOpenMesh(const aiScene *&scene, OpenMeshMesh &target)
{
    if (scene->HasMeshes())
    {
        Importer importer(target);
        for (size_t meshIndex = 0; meshIndex < scene->mNumMeshes; meshIndex++)
        {
            aiMesh *mesh = scene->mMeshes[meshIndex];
            std::vector<OpenMeshMesh::VertexHandle> vertexHandles;
            std::cout << "Adding vertices..." << std::endl;
            for (size_t vertexIndex = 0; vertexIndex < mesh->mNumVertices; vertexIndex++)
            {
                aiVector3D &vertex = mesh->mVertices[vertexIndex];
                OpenMeshMesh::VertexHandle vertexHandle = importer.add_vertex(OpenMesh::Vec3f(vertex.x, vertex.y, vertex.z));
                vertexHandles.push_back(vertexHandle);
            }
            std::cout << "Adding faces..." << std::endl;
            for (size_t faceIndex = 0; faceIndex < mesh->mNumFaces; faceIndex++)
            {
                aiFace &face = mesh->mFaces[faceIndex];
                std::vector<OpenMeshMesh::VertexHandle> faceVertexHandles;
                //std::cout << "Face " << faceIndex << " ";
                for (size_t index = 0; index < face.mNumIndices; index++)
                {
                    //std::cout << face.mIndices[index] << " ";
                    faceVertexHandles.push_back(vertexHandles[face.mIndices[index]]);
                }
                //std::cout << std::endl;
                importer.add_face(faceVertexHandles);
                faceVertexHandles.clear();
            }
            std::cout << "Adding normals..." << std::endl;
            for (size_t normalIndex = 0; normalIndex < mesh->mNumVertices; normalIndex++)
            {
                aiVector3D &normal = mesh->mNormals[normalIndex];
            }
            vertexHandles.clear();
        }
    }
}

void decimate(OpenMeshMesh &mesh, const PipelineData &pipelineData)
{
    DecimatOr decimatOr(mesh);
    QuadricModule quadricModule;
    decimatOr.add(quadricModule);
    decimatOr.module(quadricModule).unset_max_err();

    if (pipelineData.hausdorff)
    {
        HausdorffModule hausdorff;
        decimatOr.add(hausdorff);
        decimatOr.module(hausdorff).set_binary(true);
    }

    if (pipelineData.aspectRatio)
    {
        AspectRatioModule aspectRatio;
        decimatOr.add(aspectRatio);
        decimatOr.module(aspectRatio).set_binary(true);
    }

    if (pipelineData.edgeLength)
    {
	EdgeLengthModule edgeLength;
	decimatOr.add(edgeLength);
	decimatOr.module(edgeLength).set_binary(true);
    }

    decimatOr.initialize();
    decimatOr.info(std::cout);
    size_t decimated = decimatOr.decimate_to(pipelineData.faces);
    std::cout << decimated << " collapses performed." << std::endl;
    mesh.garbage_collection();
}

void openMeshToAssimp(OpenMeshMesh &mesh, aiScene &scene)
{
    aiMesh *newMesh = new aiMesh;
    scene.mNumMeshes = 1;
    scene.mMeshes = new aiMesh *[1];
    scene.mMeshes[0] = newMesh;
    scene.mMeshes[0]->mMaterialIndex = 0;
    scene.mMaterials = new aiMaterial *[1];
    aiMaterial *newMaterial = new aiMaterial;
    scene.mMaterials[0] = newMaterial;
    scene.mNumMaterials = 1;
    newMesh->mMaterialIndex = 0;

    scene.mRootNode = new aiNode();
    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mMeshes[0] = 0;
    scene.mRootNode->mNumMeshes = 1;

    newMesh->mNumVertices = mesh.n_vertices();
    newMesh->mVertices = new aiVector3D[mesh.n_vertices()];

    for (size_t vertexIndex = 0; vertexIndex < mesh.n_vertices(); vertexIndex++)
    {
        const OpenMeshMesh::Point &point = mesh.point((OpenMeshMesh::VertexHandle)vertexIndex);
        newMesh->mVertices[vertexIndex] = aiVector3D(point[0], point[1], point[2]);
    }

    newMesh->mNumFaces = mesh.n_faces();
    newMesh->mFaces = new aiFace[mesh.n_faces()];
    std::vector<int> vertexIndices;
    for (size_t faceIndex = 0; faceIndex < mesh.n_faces(); faceIndex++)
    {
        aiFace newFace;
        size_t indexCount = 0;
        for (OpenMeshMesh::FaceVertexCCWIter faceIterator = mesh.fv_ccwiter((OpenMeshMesh::FaceHandle)faceIndex); faceIterator.is_valid(); faceIterator++)
        {
            indexCount++;
            vertexIndices.push_back((*faceIterator).idx());
        }
        newFace.mNumIndices = indexCount;
        newFace.mIndices = new unsigned int[indexCount];
        for (size_t i = 0; i < indexCount; i++)
        {
            newFace.mIndices[i] = vertexIndices[i];
        }
        newMesh->mFaces[faceIndex] = newFace;
        vertexIndices.clear();
    }
}

int main(const int argc, const char *argv[])
{
    PipelineData pipelineData;
    parseArgs(pipelineData, argc, argv);
    parseFlags(pipelineData, argc, argv);
    printPipelineData(pipelineData);
    Assimp::Importer importer;
    Assimp::Exporter exporter;

    const aiScene *scene = importModel(importer, pipelineData);
    if (!scene)
    {
        std::cout << importer.GetErrorString() << std::endl;
        return -1;
    }

    OpenMeshMesh mesh;
    assimpToOpenMesh(scene, mesh);
    std::cout << "Mesh stats before decimation:" << std::endl;
    printStats(mesh);
    //OpenMesh::IO::read_mesh(mesh, "groot.ply");
    std::cout << "Starting decimation..." << std::endl;
    decimate(mesh, pipelineData);

    std::cout << "Mesh stats after decimation:" << std::endl;
    printStats(mesh);
    //OpenMesh::IO::write_mesh(mesh, "test.obj");
    aiScene targetScene;
    openMeshToAssimp(mesh, targetScene);

    exportModel(exporter, pipelineData, &targetScene);

    return 0;
}
