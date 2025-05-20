
#include "../hMesh.h"
#include "TestFramework.h"

using namespace std;
using hmesh::CellType;
using hmesh::FaceType;
using VolumeMesh = hmesh::VolumeMesh<>;

void TestVolumeMesh() {
    tf::TestSuite suite("VolumeMesh");

    suite.addTestCase("Basic Ability Test 1", []() {
        VolumeMesh vol;
        ASSERT_EQUAL(vol.addVert({0, 0, 0}), 0);
        ASSERT_EQUAL(vol.addVert({1, 0, 0}), 1);
        ASSERT_EQUAL(vol.addVert({1, 1, 0}), 2);
        ASSERT_EQUAL(vol.addVert({0, 1, 0}), 3);
        ASSERT_EQUAL(vol.addVert({0, 0, 1}), 4);
        ASSERT_EQUAL(vol.addVert({1, 0, 1}), 5);
        ASSERT_EQUAL(vol.addVert({1, 1, 1}), 6);
        ASSERT_EQUAL(vol.addVert({0, 1, 1}), 7);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_HEX>({0, 1, 2, 3, 4, 5, 6, 7}), 0);
        ASSERT_EQUAL(vol.addVert({2, 0, 0}), 8);
        ASSERT_EQUAL(vol.addVert({2, 0, 1}), 9);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_PRISM>({1, 8, 2, 5, 9, 6}), 1);
        ASSERT_EQUAL(vol.addVert({0.5, 0.5, 2}), 10);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_PYRAMID>({4, 5, 6, 7, 10}), 2);
        ASSERT_EQUAL(vol.addVert({1, 0, -1}), 11);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_TET>({1, 2, 8, 11}), 3);
        ASSERT_EQUAL(vol.numVerts(), 12);
        ASSERT_EQUAL(vol.numEdges(), 24);
        ASSERT_EQUAL(vol.numFaces(), 17);
        ASSERT_EQUAL(vol.numCells(), 4);

        hmesh::io::SaveMesh(ROOT_PATH "/result/c1.mesh", vol);
        VolumeMesh vol2;
        hmesh::io::LoadMesh(ROOT_PATH "/result/c1.mesh", vol2);
        ASSERT_EQUAL(vol2.numVerts(), 12);
        ASSERT_EQUAL(vol2.numEdges(), 24);
        ASSERT_EQUAL(vol2.numFaces(), 17);
        ASSERT_EQUAL(vol2.numCells(), 4);
        hmesh::io::SaveMesh(ROOT_PATH "/result/c1_1.mesh", vol2);

        ASSERT_EQUAL(vol.vertEdges(0).size(), 3);

        // 验证邻接关系
        ASSERT_EQUAL(vol.vertEdges(0).size(), 3);
        ASSERT_EQUAL(vol.vertEdges(1).size(), 5);
        ASSERT_EQUAL(vol.vertEdges(2).size(), 5);
        ASSERT_EQUAL(vol.vertEdges(3).size(), 3);
        ASSERT_EQUAL(vol.vertEdges(4).size(), 4);
        ASSERT_EQUAL(vol.vertFaces(0).size(), 3);
        ASSERT_EQUAL(vol.vertFaces(1).size(), 7);
        ASSERT_EQUAL(vol.vertFaces(2).size(), 7);
        ASSERT_EQUAL(vol.vertFaces(4).size(), 5);
        ASSERT_EQUAL(vol.vertCells(0).size(), 1);
        ASSERT_EQUAL(vol.vertCells(1).size(), 3);
        ASSERT_EQUAL(vol.vertCells(4).size(), 2);
        ASSERT_EQUAL(vol.vertCells(5).size(), 3);
        ASSERT_EQUAL(vol.edgeFaces(0).size(), 2);
        ASSERT_EQUAL(vol.edgeFaces(1).size(), 4);
        ASSERT_EQUAL(vol.edgeFaces(8).size(), 3);
        ASSERT_EQUAL(vol.edgeFaces(9).size(), 4);
        ASSERT_EQUAL(vol.edgeCells(0).size(), 1);
        ASSERT_EQUAL(vol.edgeCells(1).size(), 3);
        ASSERT_EQUAL(vol.edgeCells(8).size(), 2);
        ASSERT_EQUAL(vol.edgeCells(9).size(), 3);
        ASSERT_EQUAL(vol.faceCells(0).size(), 1);
        ASSERT_EQUAL(vol.faceCells(4).size(), 2);
        ASSERT_EQUAL(vol.faceCells(1).size(), 2);
        ASSERT_EQUAL(vol.faceCells(6).size(), 2);
        ASSERT_EQUAL(vol.edgeFaces(vol.getCell(2).edges()[4]).size(), 2);
        ASSERT_EQUAL(vol.edgeFaces(vol.getCell(2).edges()[0]).size(), 3);
        ASSERT_EQUAL(vol.edgeFaces(vol.getCell(1).edges()[1]).size(), 3);
        ASSERT_EQUAL(vol.edgeCells(vol.getCell(1).edges()[1]).size(), 2);
        ASSERT_EQUAL(vol.edgeCells(vol.getCell(1).edges()[4]).size(), 1);
        ASSERT_EQUAL(vol.edgeFaces(vol.getCell(1).edges()[4]).size(), 2);
        ASSERT_EQUAL(vol.faceCells(vol.getCell(1).faces()[2]).size(), 1);
        ASSERT_EQUAL(vol.faceCells(vol.getCell(1).faces()[4]).size(), 2);
        ASSERT_EQUAL(vol.getCellIndex<CellType::CELL_HEX>({0, 1, 2, 3, 4, 5, 6, 7}), 0);
        ASSERT_EQUAL(vol.getCellIndex<CellType::CELL_HEX>({0, 1, 2, 3, 4, 5, 6, 8}), vol.kInvalidIndex);
        ASSERT_TRUE(vol.isCellRemoved(vol.numCells()));

        ASSERT_TRUE(vol.getVertAttributes().addAttribute<int>("id", numeric_limits<int>::max()));
        for (int i = 0; i < 12; ++i) {
            vol.getVertAttributes().getAttribute<int>("id")->setElement(i, i);
        }
        ASSERT_TRUE(vol.getCellAttributes().addAttribute<int>("testdata", numeric_limits<int>::max()));
        vol.getCellAttributes().getAttribute<int>("testdata")->setElement(0, 0);
        vol.getCellAttributes().getAttribute<int>("testdata")->setElement(1, 1);
        vol.getCellAttributes().getAttribute<int>("testdata")->setElement(2, 2);

        // R移除体
        vol.removeCell(0);
        ASSERT_EQUAL(vol.numVerts(), 12);
        ASSERT_EQUAL(vol.numEdges(), 19);
        ASSERT_EQUAL(vol.numFaces(), 13);
        ASSERT_EQUAL(vol.numCells(), 3);
        ASSERT_TRUE(vol.isCellRemoved(0));
        vol.removeIsolatedVerts();
        ASSERT_EQUAL(vol.numVerts(), 10);
        ASSERT_EQUAL(vol.numEdges(), 19);
        ASSERT_EQUAL(vol.numFaces(), 13);
        ASSERT_EQUAL(vol.numCells(), 3);
        ASSERT_TRUE(vol.isVertRemoved(0));

        vol.defragment();
        ASSERT_FALSE(vol.isVertRemoved(0));
        ASSERT_FALSE(vol.isCellRemoved(0));
        ASSERT_EQUAL(vol.getVertAttributes().getAttribute<int>("id")->getElement(0), 1);
        ASSERT_EQUAL(vol.getCellAttributes().getAttribute<int>("testdata")->getElement(0), 1);
    });

    suite.addTestCase("Basic Ability Test 2", []() {
        VolumeMesh vol;
        ASSERT_EQUAL(vol.addVert({0, 0, 0}), 0);
        ASSERT_EQUAL(vol.addVert({1, 0, 0}), 1);
        ASSERT_EQUAL(vol.addVert({1, 1, 0}), 2);
        ASSERT_EQUAL(vol.addVert({0, 1, 0}), 3);
        ASSERT_EQUAL(vol.addVert({0, 0, 1}), 4);
        ASSERT_EQUAL(vol.addVert({1, 0, 1}), 5);
        ASSERT_EQUAL(vol.addVert({1, 1, 1}), 6);
        ASSERT_EQUAL(vol.addVert({0, 1, 1}), 7);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_HEX>({0, 1, 2, 3, 4, 5, 6, 7}), 0);
        ASSERT_EQUAL(vol.addVert({2, 0, 0}), 8);
        ASSERT_EQUAL(vol.addCell<CellType::CELL_TET>({1, 2, 5, 8}), 1);

        ASSERT_EQUAL(vol.numVerts(), 9);
        ASSERT_EQUAL(vol.numEdges(), 16);
        ASSERT_EQUAL(vol.numFaces(), 10);
        ASSERT_EQUAL(vol.numCells(), 2);

        ASSERT_EQUAL(vol.vertEdges(1).size(), 4);
        ASSERT_EQUAL(vol.vertEdges(5).size(), 5);
        ASSERT_EQUAL(vol.vertFaces(1).size(), 6);
        ASSERT_EQUAL(vol.vertCells(1).size(), 2);
        ASSERT_EQUAL(vol.edgeFaces(5).size(), 4);
        ASSERT_EQUAL(vol.edgeFaces(1).size(), 4);
        ASSERT_EQUAL(vol.edgeFaces(12).size(), 2);
        ASSERT_EQUAL(vol.edgeCells(0).size(), 1);
        ASSERT_EQUAL(vol.edgeCells(1).size(), 2);
        ASSERT_EQUAL(vol.edgeCells(5).size(), 2);
        ASSERT_EQUAL(vol.edgeCells(12).size(), 1);
        ASSERT_EQUAL(vol.faceCells(0).size(), 1);
        ASSERT_EQUAL(vol.faceCells(4).size(), 1);
        ASSERT_EQUAL(vol.faceCells(5).size(), 1);
        ASSERT_EQUAL(vol.faceCells(6).size(), 1);
        ASSERT_EQUAL(vol.faceCells(7).size(), 1);
    });

    suite.addTestCase("Basic Ability Test 3", []() {
        VolumeMesh vol;
        ASSERT_EQUAL(vol.addVert({0, 0, 0}), 0);
        ASSERT_EQUAL(vol.addVert({1, 0, 0}), 1);
        ASSERT_EQUAL(vol.addVert({1, 1, 0}), 2);
        ASSERT_EQUAL(vol.addVert({0, 1, 0}), 3);
        ASSERT_EQUAL(vol.addVert({0, 0, 1}), 4);
        ASSERT_EQUAL(vol.addVert({1, 0, 1}), 5);
        ASSERT_EQUAL(vol.addVert({1, 1, 1}), 6);
        ASSERT_EQUAL(vol.addVert({0, 1, 1}), 7);
        ASSERT_EQUAL(vol.addVert({0.5, 0.5, 2}), 8);

        vector<size_t> verts, edges, faces;
        for (int i = 0; i < 9; ++i) {
            verts.push_back(i);
        }
        edges.push_back(vol.addEdge(0, 1));
        edges.push_back(vol.addEdge(1, 2));
        edges.push_back(vol.addEdge(2, 3));
        edges.push_back(vol.addEdge(3, 0));
        edges.push_back(vol.addEdge(4, 5));
        edges.push_back(vol.addEdge(5, 6));
        edges.push_back(vol.addEdge(6, 7));
        edges.push_back(vol.addEdge(7, 4));
        edges.push_back(vol.addEdge(0, 4));
        edges.push_back(vol.addEdge(1, 5));
        edges.push_back(vol.addEdge(2, 6));
        edges.push_back(vol.addEdge(3, 7));
        edges.push_back(vol.addEdge(4, 8));
        edges.push_back(vol.addEdge(5, 8));
        edges.push_back(vol.addEdge(6, 8));
        edges.push_back(vol.addEdge(7, 8));
        faces.push_back(vol.addFace<FaceType::FACE_QUAD>({3, 2, 1, 0}));
        faces.push_back(vol.addFace<FaceType::FACE_QUAD>({5, 4, 0, 1}));
        faces.push_back(vol.addFace<FaceType::FACE_QUAD>({7, 6, 2, 3}));
        faces.push_back(vol.addFace<FaceType::FACE_QUAD>({4, 7, 3, 0}));
        faces.push_back(vol.addFace<FaceType::FACE_QUAD>({6, 5, 1, 2}));
        faces.push_back(vol.addFace<FaceType::FACE_TRI>({4, 5, 8}));
        faces.push_back(vol.addFace<FaceType::FACE_TRI>({5, 6, 8}));
        faces.push_back(vol.addFace<FaceType::FACE_TRI>({6, 7, 8}));
        faces.push_back(vol.addFace<FaceType::FACE_TRI>({7, 4, 8}));

        vol.addCell<CellType::CELL_POLYHEDRON>(verts.data(), verts.size(), edges.data(), edges.size(), faces.data(), faces.size());

        ASSERT_EQUAL(vol.numVerts(), 9);
        ASSERT_EQUAL(vol.numEdges(), 16);
        ASSERT_EQUAL(vol.numFaces(), 9);
        ASSERT_EQUAL(vol.numCells(), 1);
        ASSERT_EQUAL(
            vol.getCellIndex<CellType::CELL_POLYHEDRON>(verts.data(), verts.size(), edges.data(), edges.size(), faces.data(), faces.size()),
            0);

        hmesh::io::SaveObj(ROOT_PATH "/result/c2.obj", vol);
    });
    suite.addTestCase("Basic Ability Test 4", []() {
        VolumeMesh v1, v2;
        v1.copyFrom(v2);
    });

    suite.addTestCase("TriangleMeshOperations Test 1", []() {
        VolumeMesh m;

        m.addVert({0, 0, 0});
        m.addVert({1, 0, 0});
        m.addVert({0, 1, 0});
        m.addVert({0, 0, 1});
        m.addVert({0, -1, 0});
        m.addEdge(0, 1);
        m.addCell<CellType::CELL_TET>({0, 1, 2, 3});
        m.addCell<CellType::CELL_TET>({1, 0, 4, 3});

        size_t newVid;
        hmesh::TetrahedralMeshOperations::collapseEdge(m, (size_t)0, newVid);

        ASSERT_EQUAL(m.numVerts(), 6);
        ASSERT_EQUAL(m.numEdges(), 0);
        ASSERT_EQUAL(m.numFaces(), 0);
        ASSERT_EQUAL(m.numCells(), 0);

        m.defragment();
    });

    suite.addTestCase("TriangleMeshOperations Test 2", []() {
        VolumeMesh m;

        m.addVert({0, 0, 0});
        m.addVert({1, 0, 0});
        m.addVert({0, 1, 0});
        m.addVert({0, 0, 1});
        m.addVert({0, -1, 0});
        m.addVert({1, -1, 0});
        m.addVert({-1, -1, 0});
        m.addEdge(0, 1);
        m.addEdge(0, 4);
        m.addCell<CellType::CELL_TET>({0, 1, 2, 3});
        m.addCell<CellType::CELL_TET>({1, 0, 4, 3});
        m.addCell<CellType::CELL_TET>({4, 5, 1, 3});
        m.addCell<CellType::CELL_TET>({6, 4, 0, 3});

        hmesh::io::SaveMesh(ROOT_PATH "/result/before_collapseEdge.mesh", m);
        size_t newVid;
        hmesh::TetrahedralMeshOperations::collapseEdge(m, (size_t)0, newVid);

        ASSERT_EQUAL(newVid, 7);
        ASSERT_EQUAL(m.numVerts(), 8);
        ASSERT_EQUAL(m.numEdges(), 9);
        ASSERT_EQUAL(m.numFaces(), 7);
        ASSERT_EQUAL(m.numCells(), 2);
        ASSERT_EQUAL(m.vertEdges(7).size(), 4);
        ASSERT_EQUAL(m.vertEdges(3).size(), 4);
        ASSERT_EQUAL(m.vertEdges(4).size(), 4);
        ASSERT_EQUAL(m.vertEdges(5).size(), 3);
        ASSERT_EQUAL(m.vertEdges(6).size(), 3);
        ASSERT_EQUAL(m.vertFaces(7).size(), 5);
        ASSERT_EQUAL(m.vertFaces(3).size(), 5);
        ASSERT_EQUAL(m.vertFaces(4).size(), 5);
        ASSERT_EQUAL(m.vertCells(5).size(), 1);
        ASSERT_EQUAL(m.vertCells(7).size(), 2);
        ASSERT_TRUE(m.isEdgeRemoved(0));
        ASSERT_TRUE(m.isEdgeRemoved(1));
        ASSERT_TRUE(m.isFaceRemoved(0));
        ASSERT_TRUE(m.isFaceRemoved(1));
        ASSERT_TRUE(m.isFaceRemoved(2));
        ASSERT_TRUE(m.isFaceRemoved(3));
        size_t maxFaceToCells = 0;
        for (size_t fid : m.faceIndices()) {
            maxFaceToCells = max(maxFaceToCells, m.faceCells(fid).size());
        }
        ASSERT_EQUAL(maxFaceToCells, 2);

        size_t maxEdgeToFaces = 0;
        size_t maxEdgeToCells = 0;
        for (size_t eid : m.edgeIndices()) {
            maxEdgeToFaces = max(maxEdgeToFaces, m.edgeFaces(eid).size());
            maxEdgeToCells = max(maxEdgeToCells, m.edgeCells(eid).size());
        }
        ASSERT_EQUAL(maxEdgeToFaces, 3);
        ASSERT_EQUAL(maxEdgeToCells, 2);

        size_t maxVertToEdges = 0;
        size_t maxVertToFaces = 0;
        size_t maxVertToCells = 0;
        for (size_t vid : m.vertIndices()) {
            maxVertToEdges = max(maxVertToEdges, m.vertEdges(vid).size());
            maxVertToFaces = max(maxVertToFaces, m.vertFaces(vid).size());
            maxVertToCells = max(maxVertToCells, m.vertCells(vid).size());
        }
        ASSERT_EQUAL(maxVertToEdges, 4);
        ASSERT_EQUAL(maxVertToFaces, 5);
        ASSERT_EQUAL(maxVertToCells, 2);

        m.defragment();
        hmesh::io::SaveMesh(ROOT_PATH "/result/after_collapseEdge.mesh", m);
    });

    suite.addTestCase("Test IO(Obj)", []() {
        VolumeMesh vol;
        hmesh::io::LoadMesh(ROOT_PATH "/data/tet.mesh", vol);
        hmesh::io::SaveMesh(ROOT_PATH "/result/tet.mesh", vol);

        // 遍历顶点索引
        {
            auto it = vol.vertIndexBegin();
            for (auto id : vol.vertIndices()) {
                ASSERT_EQUAL(id, *it);
                ++it;
            }
            ASSERT_TRUE(it == vol.vertIndexEnd());
        }
        // 遍历边索引
        {
            auto it = vol.edgeIndexBegin();
            for (auto id : vol.edgeIndices()) {
                ASSERT_EQUAL(id, *it);
                ++it;
            }
            ASSERT_TRUE(it == vol.edgeIndexEnd());
        }
        // 遍历面索引
        {
            auto it = vol.faceIndexBegin();
            for (auto id : vol.faceIndices()) {
                ASSERT_EQUAL(id, *it);
                ++it;
            }
            ASSERT_TRUE(it == vol.faceIndexEnd());
        }
        // 遍历体索引
        {
            auto it = vol.cellIndexBegin();
            for (auto id : vol.cellIndices()) {
                ASSERT_EQUAL(id, *it);
                ++it;
            }
            ASSERT_TRUE(it == vol.cellIndexEnd());
        }
    });

    suite.addTestCase("Test Attribute", []() {
        VolumeMesh m;
        const size_t VALUE_DEFAULT = m.kInvalidIndex;
        const size_t VALUE_1 = m.kInvalidIndex - 1;
        const size_t VALUE_2 = m.kInvalidIndex - 2;
        const size_t VALUE_3 = m.kInvalidIndex - 3;

        ASSERT_EQUAL(m.addVert({0, 0, 0}), 0);
        m.getVertAttributes().addAttribute<size_t>("attr", VALUE_DEFAULT);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(0), VALUE_DEFAULT);
        m.getVertAttributes().getAttribute<size_t>("attr")->setElement(0, VALUE_1);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(0), VALUE_1);
        ASSERT_EQUAL(m.addVert({1, 0, 0}), 1);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(1), VALUE_DEFAULT);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(0), VALUE_1);
        m.getVertAttributes().getAttribute<size_t>("attr")->setElement(1, VALUE_2);
        m.removeVert(0);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(0), VALUE_DEFAULT);
        ASSERT_EQUAL(m.getVertAttributes().getAttribute<size_t>("attr")->getElement(1), VALUE_2);

        VolumeMesh copy;
        copy.copyFrom(m);
        ASSERT_EQUAL(copy.getVertAttributes().getAttribute<size_t>("attr")->getElement(0), VALUE_DEFAULT);
        ASSERT_EQUAL(copy.getVertAttributes().getAttribute<size_t>("attr")->getElement(1), VALUE_2);
        ASSERT_NOT_EQUAL(copy.getVertAttributes().getAttribute<size_t>("attr")->getElement(1), VALUE_3);
        });

    suite.run();
}