
#include "../hMesh.h"
#include "TestFramework.h"

using namespace std;
using hmesh::FaceType;
using SurfaceMesh = hmesh::SurfaceMesh<>;

struct Vector3d {
    array<double, 3> position;
};

void TestSurfaceMesh() {
    tf::TestSuite suite("SurfaceMesh");

    suite.addTestCase("Basic Ability Test 1", []() {
        SurfaceMesh mesh;

        // 创建三棱柱
        ASSERT_EQUAL(mesh.addVert({0.0, 0.0, 0.0}), 0);                    // v0
        ASSERT_EQUAL(mesh.addVert({0.0, 1.0, 0.0}), 1);                    // v1
        ASSERT_EQUAL(mesh.addVert({1.0, 0.0, 0.0}), 2);                    // v2
        ASSERT_EQUAL(mesh.addVert({0.0, 0.0, 1.0}), 3);                    // v3
        ASSERT_EQUAL(mesh.addVert({0.0, 1.0, 1.0}), 4);                    // v4
        ASSERT_EQUAL(mesh.addVert({1.0, 0.0, 1.0}), 5);                    // v5
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_TRI>({0, 1, 2}), 0);      // f0
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_TRI>({3, 4, 5}), 1);      // f1
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_QUAD>({0, 1, 4, 3}), 2);  // f2
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_QUAD>({1, 4, 5, 2}), 3);  // f3
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_QUAD>({0, 3, 5, 2}), 4);  // f4

        ASSERT_EQUAL(mesh.numVerts(), 6);
        ASSERT_EQUAL(mesh.numEdges(), 9);
        ASSERT_EQUAL(mesh.numFaces(), 5);
        ASSERT_FALSE(mesh.isFaceRemoved(1));
        for (int i = 0; i < 6; ++i) {
            ASSERT_EQUAL(mesh.vertEdges(i).size(), 3);
            ASSERT_EQUAL(mesh.vertFaces(i).size(), 3);
        }
        for (int i = 0; i < 9; ++i) {
            ASSERT_EQUAL(mesh.edgeFaces(i).size(), 2);
        }
        ASSERT_TRUE((mesh.getVert(1) == array<double, 3>{0.0, 1.0, 0.0}));
        ASSERT_EQUAL(mesh.getEdge(0).numVerts(), 2);
        ASSERT_EQUAL(mesh.getFace(0).numVerts(), 3);
        ASSERT_EQUAL(mesh.getFace(0).numEdges(), 3);
        ASSERT_EQUAL(mesh.getFace(0).verts()[0], 0);
        ASSERT_EQUAL(mesh.getFace(0).verts()[1], 1);
        ASSERT_EQUAL(mesh.getFace(0).verts()[2], 2);
        ASSERT_EQUAL(mesh.getFace(0).edges()[0], 0);
        ASSERT_EQUAL(mesh.getFace(0).edges()[1], 1);
        ASSERT_EQUAL(mesh.getFace(0).edges()[2], 2);
        ASSERT_TRUE(mesh.getFace(0).type() == FaceType::FACE_TRI);
        ASSERT_TRUE(mesh.getFace(2).type() == FaceType::FACE_QUAD);
        ASSERT_EQUAL(mesh.getEdgeIndex(0, 1), 0);
        ASSERT_EQUAL(mesh.getFaceIndex<FaceType::FACE_TRI>({0, 1, 2}), 0);
        ASSERT_TRUE(mesh.isVertRemoved(mesh.numVerts()));
        ASSERT_TRUE(mesh.isEdgeRemoved(mesh.numEdges()));
        ASSERT_TRUE(mesh.isFaceRemoved(mesh.numFaces()));

        // 添加重复面(f4)
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_QUAD>({0, 3, 5, 2}), 4);
        ASSERT_EQUAL(mesh.numVerts(), 6);
        ASSERT_EQUAL(mesh.numEdges(), 9);
        ASSERT_EQUAL(mesh.numFaces(), 5);

        // 增加孤立顶点
        ASSERT_EQUAL(mesh.addVert({-1.0, 0.0, 0.0}), 6);  // v6
        ASSERT_EQUAL(mesh.numVerts(), 7);

        // 增加孤立边
        ASSERT_EQUAL(mesh.addEdge(6, 0), 9);
        ASSERT_EQUAL(mesh.numEdges(), 10);

        // 移除孤立边
        ASSERT_EQUAL(mesh.removeIsolatedEdges(), 1);
        ASSERT_TRUE(mesh.isEdgeRemoved(9));
        ASSERT_EQUAL(mesh.numEdges(), 9);

        // 移除孤立顶点
        ASSERT_EQUAL(mesh.removeIsolatedVerts(), 1);
        ASSERT_TRUE(mesh.isVertRemoved(6));
        ASSERT_EQUAL(mesh.numVerts(), 6);

        // 移除面
        mesh.removeFace(0);
        ASSERT_TRUE(mesh.isFaceRemoved(0));
        ASSERT_EQUAL(mesh.numFaces(), 4);
        ASSERT_EQUAL(mesh.vertFaces(0).size(), 2);
        ASSERT_EQUAL(mesh.vertEdges(0).size(), 3);
        ASSERT_EQUAL(mesh.edgeFaces(0).size(), 1);
        ASSERT_EQUAL(mesh.edgeFaces(1).size(), 1);
        ASSERT_EQUAL(mesh.edgeFaces(2).size(), 1);
        ASSERT_FALSE(mesh.isIndexContinuous());

        mesh.removeFace(2);
        ASSERT_TRUE(mesh.isFaceRemoved(2));
        ASSERT_EQUAL(mesh.numFaces(), 3);
        ASSERT_EQUAL(mesh.vertFaces(0).size(), 1);
        ASSERT_EQUAL(mesh.vertEdges(0).size(), 2);
        ASSERT_TRUE(mesh.isEdgeRemoved(0));

        // 重新创建面f2
        // 因为空闲索引存储在集合中，所以会选择最小索引。
        ASSERT_EQUAL(mesh.addFace<FaceType::FACE_QUAD>({0, 1, 4, 3}), 0);

        // 整理碎片
        mesh.defragment();
        ASSERT_EQUAL(mesh.numVerts(), 6);
        ASSERT_EQUAL(mesh.numEdges(), 9);
        ASSERT_EQUAL(mesh.numFaces(), 4);
        ASSERT_TRUE(mesh.isIndexContinuous());
        for (size_t vid : mesh.vertIndices()) {
            ASSERT_TRUE(vid >= 0 && vid < 6);
        }
        for (auto it = mesh.vertIndices().begin(); it != mesh.vertIndices().end(); ++it) {
            ASSERT_TRUE(*it >= 0 && *it < 6);
        }
        for (size_t eid : mesh.edgeIndices()) {
            ASSERT_TRUE(eid >= 0 && eid < 9);
        }
        for (auto it = mesh.edgeIndices().begin(); it != mesh.edgeIndices().end(); ++it) {
            ASSERT_TRUE(*it >= 0 && *it < 9);
        }
        for (size_t fid : mesh.faceIndices()) {
            ASSERT_TRUE(fid >= 0 && fid < 5);
        }
        for (auto it = mesh.faceIndices().begin(); it != mesh.faceIndices().end(); ++it) {
            ASSERT_TRUE(*it >= 0 && *it < 5);
        }

        // 清空网格
        mesh.clear();
        ASSERT_EQUAL(mesh.numVerts(), 0);
        ASSERT_EQUAL(mesh.numEdges(), 0);
        ASSERT_EQUAL(mesh.numFaces(), 0);
        ASSERT_TRUE(mesh.isIndexContinuous());
    });

    suite.addTestCase("Basic Ability Test 2", []() {
        SurfaceMesh mesh;

        vector<array<double, 3>> verts = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}, {0, 1, 0}, {1, 2, 0}, {2, 1, 0}, {3, 1, 0}};
        vector<vector<size_t>> faces = {{0, 1, 6, 5, 4}, {1, 2, 6}, {2, 3, 7, 6}};

        for (const auto& p : verts) {
            mesh.addVert(p);
        }
        for (const auto& f : faces) {
            if (f.size() == 3) {
                mesh.addFace<FaceType::FACE_TRI>(f.data(), f.size());
            } else if (f.size() == 4) {
                mesh.addFace<FaceType::FACE_QUAD>(f.data(), f.size());
            } else {
                mesh.addFace<FaceType::FACE_POLYGON>(f.data(), f.size());
            }
        }
        ASSERT_EQUAL(mesh.numVerts(), 8);
        ASSERT_EQUAL(mesh.numEdges(), 10);
        ASSERT_EQUAL(mesh.numFaces(), 3);
        ASSERT_TRUE(mesh.getFace(0).type() == FaceType::FACE_POLYGON);
        ASSERT_TRUE(mesh.getFace(1).type() == FaceType::FACE_TRI);
        ASSERT_TRUE(mesh.getFace(2).type() == FaceType::FACE_QUAD);
        ASSERT_EQUAL(mesh.getFace(0).verts()[2], 6);
        ASSERT_TRUE(mesh.edgeFaces(1).count(0) > 0);
        ASSERT_TRUE(mesh.edgeFaces(1).count(1) > 0);
        ASSERT_EQUAL(mesh.getFaceIndex<FaceType::FACE_POLYGON>({0, 1, 6, 5, 4}), 0);

        using TestAttribute = std::array<size_t, 2>;
        ASSERT_TRUE(mesh.getFaceAttributes().addAttribute<TestAttribute>("testdata"));
        ASSERT_FALSE(mesh.getFaceAttributes().hasAttribute("not exist"));
        auto testdata = mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata");
        for (size_t fid : mesh.faceIndices()) {
            testdata->setElement(fid, {fid, fid + 10});
        }

        // 删除多边形
        mesh.removeFace(0);
        ASSERT_TRUE(mesh.isFaceRemoved(0));
        ASSERT_TRUE(mesh.isEdgeRemoved(0));
        ASSERT_TRUE(mesh.isEdgeRemoved(2));
        ASSERT_TRUE(mesh.isEdgeRemoved(3));
        ASSERT_TRUE(mesh.isEdgeRemoved(4));
        ASSERT_FALSE(mesh.isIndexContinuous());
        ASSERT_EQUAL(mesh.edgeFaces(1).size(), 1);
        ASSERT_EQUAL(mesh.vertEdges(0).size(), 0);
        ASSERT_EQUAL(mesh.vertFaces(0).size(), 0);
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(1)[0], 1);
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(2)[0], 2);

        // 整理碎片
        mesh.defragment();
        ASSERT_TRUE(mesh.isIndexContinuous());
        ASSERT_FALSE(mesh.isFaceRemoved(0));
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(0)[0], 1);
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(0)[1], 11);
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(1)[0], 2);
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata")->getElement(1)[1], 12);

        mesh.clear();
        ASSERT_TRUE(mesh.getFaceAttributes().hasAttribute("testdata"));
        ASSERT_NOT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata"), nullptr);
        mesh.clear(false);
        ASSERT_FALSE(mesh.getFaceAttributes().hasAttribute("testdata"));
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata"), nullptr);

        ASSERT_TRUE(mesh.getFaceAttributes().addAttribute<TestAttribute>("testdata"));
        ASSERT_TRUE(mesh.getFaceAttributes().hasAttribute("testdata"));
        ASSERT_NOT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata"), nullptr);
        mesh.clearAttributes();
        ASSERT_FALSE(mesh.getFaceAttributes().hasAttribute("testdata"));
        ASSERT_EQUAL(mesh.getFaceAttributes().getAttribute<TestAttribute>("testdata"), nullptr);
    });

    suite.addTestCase("Basic Ability Test 3", []() {
        enum AttributeType { Color };

        hmesh::SurfaceMesh<int, std::array<double, 3>, AttributeType> mesh;
        mesh.addVert({0, 0, 0});
        mesh.getVertAttributes().addAttribute<double>(AttributeType::Color);
        mesh.getVertAttributes().getAttribute<double>(AttributeType::Color)->setElement(0, 1.0);
    });

    suite.addTestCase("Basic Ability Test 4", []() {
        SurfaceMesh m1, m2;
        m2.copyFrom(m1);
    });

    suite.addTestCase("TriangleMeshOperations Test 1", []() {
        SurfaceMesh m;
        m.addVert({0, 0, 0});
        m.addVert({1, 0, 0});
        m.addVert({2, 0, 0});
        m.addVert({0, 1, 0});
        m.addVert({1, 1, 0});
        m.addVert({2, 1, 0});
        m.addVert({1, 2, 0});
        m.addEdge(0, 1);
        m.addEdge(1, 2);
        m.addEdge(0, 3);
        m.addEdge(1, 3);
        m.addEdge(1, 4);
        m.addEdge(2, 4);
        m.addEdge(2, 5);
        m.addEdge(3, 4);
        m.addEdge(4, 5);
        m.addEdge(3, 6);
        m.addEdge(4, 6);
        m.addEdge(5, 6);
        m.addFace<FaceType::FACE_TRI>({0, 1, 3});
        m.addFace<FaceType::FACE_TRI>({1, 4, 3});
        m.addFace<FaceType::FACE_TRI>({1, 2, 4});
        m.addFace<FaceType::FACE_TRI>({2, 5, 4});
        m.addFace<FaceType::FACE_TRI>({3, 4, 6});
        m.addFace<FaceType::FACE_TRI>({4, 5, 6});
        hmesh::io::SaveObj(ROOT_PATH "/result/before_collapseEdge.obj", m);

        size_t newVid;
        hmesh::TriangleMeshOperations::collapseEdge(m, (size_t)4, newVid);

        ASSERT_EQUAL(newVid, 7);
        ASSERT_EQUAL(m.numVerts(), 8);
        ASSERT_EQUAL(m.numEdges(), 9);
        ASSERT_EQUAL(m.numFaces(), 4);
        ASSERT_TRUE(m.isEdgeRemoved(4));
        ASSERT_TRUE(m.isEdgeRemoved(3));
        ASSERT_TRUE(m.isEdgeRemoved(7));
        ASSERT_TRUE(m.isEdgeRemoved(1));
        ASSERT_TRUE(m.isEdgeRemoved(5));
        ASSERT_TRUE(m.isFaceRemoved(1));
        ASSERT_TRUE(m.isFaceRemoved(2));
        ASSERT_FALSE(m.isEdgeRemoved(12));
        ASSERT_FALSE(m.isEdgeRemoved(13));
        ASSERT_EQUAL(m.vertEdges(7).size(), 5);
        ASSERT_EQUAL(m.vertEdges(3).size(), 3);
        ASSERT_EQUAL(m.vertEdges(5).size(), 3);
        ASSERT_EQUAL(m.vertEdges(6).size(), 3);
        ASSERT_EQUAL(m.vertFaces(7).size(), 4);
        ASSERT_EQUAL(m.vertFaces(3).size(), 2);
        ASSERT_EQUAL(m.vertFaces(5).size(), 2);
        ASSERT_EQUAL(m.edgeFaces(12).size(), 2);
        ASSERT_EQUAL(m.edgeFaces(13).size(), 1);
        ASSERT_EQUAL(m.edgeFaces(8).size(), 2);
        ASSERT_EQUAL(m.edgeFaces(0).size(), 1);
        ASSERT_EQUAL(m.edgeFaces(12).count(0), 1);
        ASSERT_EQUAL(m.edgeFaces(12).count(4), 1);
        ASSERT_EQUAL(m.edgeFaces(8).count(5), 1);
        ASSERT_EQUAL(m.edgeFaces(8).count(3), 1);
        ASSERT_EQUAL(m.edgeFaces(13).count(3), 1);


        m.defragment();
        hmesh::io::SaveObj(ROOT_PATH "/result/after_collapseEdge.obj", m);
    });
    suite.addTestCase("TriangleMeshOperations Test 2", []() {
        SurfaceMesh m;
        m.addVert({0, 0, 0});
        m.addVert({1, 0, 0});
        m.addVert({0, 1, 0});
        m.addVert({1, 1, 0});
        m.addEdge(0, 1);
        m.addEdge(1, 2);
        m.addEdge(2, 0);
        m.addEdge(2, 0);
        m.addEdge(1, 3);
        m.addEdge(2, 3);
        m.addFace<FaceType::FACE_TRI>({0, 1, 2});
        m.addFace<FaceType::FACE_TRI>({1, 2, 3});

        size_t newVid;
        hmesh::TriangleMeshOperations::collapseEdge(m, (size_t)1, newVid);

        ASSERT_EQUAL(m.numVerts(), 5);
        ASSERT_EQUAL(m.numEdges(), 0);
        ASSERT_EQUAL(m.numFaces(), 0);

    });
    suite.addTestCase("Test IO(Obj)", []() {
        SurfaceMesh surf;
        hmesh::io::LoadObj(ROOT_PATH "/data/tri.obj", surf);
        hmesh::io::SaveObj(ROOT_PATH "/result/tri.obj", surf);
        surf.clear();
        hmesh::io::LoadObj(ROOT_PATH "/data/quadtri.obj", surf);
        hmesh::io::SaveObj(ROOT_PATH "/result/quadtri.obj", surf);
    });

    suite.run();
}