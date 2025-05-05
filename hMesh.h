/*
MIT License

Copyright (c) [2025] [dtc]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hmesh {
template <typename Index, typename VertexContainer, typename AttributeName>
class SurfaceMesh;
template <typename Index, typename VertexContainer, typename AttributeName>
class VolumeMesh;

/**
 * @brief 面类型
 */
enum class FaceType { FACE_TRI = 3, FACE_QUAD = 4, FACE_POLYGON = 5 };
/**
 * @brief 体类型
 */
enum class CellType { CELL_TET = 4, CELL_PYRAMID = 5, CELL_PRISM = 6, CELL_HEX = 8, CELL_POLYHEDRON = 9 };

/**
 * @brief 基本的体单元的描述符
 */
namespace CellDescriptors {
template <size_t MaxVertices, size_t MaxEdges, size_t MaxFaces>
struct CellDescriptor {
    size_t numVerts;                                                  // 顶点数量
    std::array<size_t, MaxVertices> numVertAdjacentEdge;              // 顶点的邻接边数量
    std::array<std::array<size_t, 4>, MaxVertices> vertAdjacentEdge;  // 顶点的邻接边
    std::array<size_t, MaxVertices> numVertAdjacentFace;              // 顶点的邻接面数量
    std::array<std::array<size_t, 4>, MaxVertices> vertAdjacentFace;  // 顶点的邻接面
    size_t numEdges;                                                  // 边数量
    std::array<std::array<size_t, 2>, MaxEdges> edgeVert;             // 边上顶点的定义
    std::array<size_t, MaxEdges> numEdgeAdjacentFace;                 // 边的邻接面数量
    std::array<std::array<size_t, 4>, MaxEdges> edgeAdjacentFace;     // 边的邻接边
    size_t numFaces;                                                  // 面数量
    std::array<size_t, MaxFaces> numVertsInFace;                      // 面上顶点的数量
    std::array<std::array<size_t, 4>, MaxFaces> faceVert;             // 面上顶点的定义
    std::array<size_t, MaxFaces> numEdgesInFace;                      // 面上边的数量
    std::array<std::array<size_t, 4>, MaxFaces> faceEdge;             // 面上边的定义
};
/**
 * @brief 四面体的描述符
 */
inline constexpr CellDescriptor<8, 12, 6> TET = {
    4,                                                   // 顶点数量
    {3, 3, 3, 3},                                        // 顶点的邻接边数量
    {{{0, 2, 3}, {0, 1, 4}, {1, 2, 5}, {3, 4, 5}}},      // 顶点的邻接边
    {3, 3, 3, 3},                                        // 顶点的邻接面数量
    {{{0, 2, 3}, {0, 1, 2}, {0, 1, 3}, {1, 2, 3}}},      // 顶点的邻接面
    6,                                                   // 边数量
    {{{0, 1}, {1, 2}, {2, 0}, {3, 0}, {3, 1}, {3, 2}}},  // 边上顶点的定义
    {2, 2, 2, 2, 2, 2},                                  // 边的邻接面数量
    {{{0, 2}, {0, 1}, {0, 3}, {2, 3}, {1, 2}, {1, 3}}},  // 边的邻接边
    4,                                                   // 面数量
    {3, 3, 3, 3},                                        // 面上顶点的数量
    {{{0, 1, 2}, {1, 3, 2}, {1, 0, 3}, {2, 3, 0}}},      // 面上顶点的定义
    {3, 3, 3, 3},                                        // 面上边的数量
    {{{0, 1, 2}, {4, 5, 1}, {0, 3, 4}, {5, 3, 2}}}       // 面上边的定义
};
/**
 * @brief 金字塔的描述符
 */
inline constexpr CellDescriptor<8, 12, 6> PYRAMID = {
    5,                                                                   // 顶点数量
    {3, 3, 3, 3, 4},                                                     // 顶点的邻接边数量
    {{{0, 3, 4}, {0, 1, 5}, {1, 2, 6}, {2, 3, 7}, {4, 5, 6, 7}}},        // 顶点的邻接边
    {3, 3, 3, 3, 4},                                                     // 顶点的邻接面数量
    {{{0, 1, 4}, {0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {1, 2, 3, 4}}},        // 顶点的邻接面
    8,                                                                   // 边数量
    {{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 0}, {4, 1}, {4, 2}, {4, 3}}},  // 边上顶点的定义
    {2, 2, 2, 2, 2, 2, 2, 2},                                            // 边的邻接面数量
    {{{0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 4}, {1, 2}, {2, 3}, {3, 4}}},  // 边的邻接边
    5,                                                                   // 面数量
    {4, 3, 3, 3, 3},                                                     // 面上顶点的数量
    {{{0, 1, 2, 3}, {1, 0, 4}, {2, 1, 4}, {3, 2, 4}, {0, 3, 4}}},        // 面上顶点的定义
    {4, 3, 3, 3, 3},                                                     // 面上边的数量
    {{{0, 1, 2, 3}, {0, 4, 5}, {1, 5, 6}, {2, 6, 7}, {3, 7, 4}}}         // 面上边的定义
};
/**
 * @brief 三棱柱的描述符
 */
inline constexpr CellDescriptor<8, 12, 6> PRISM = {
    6,                                                                           // 顶点数量
    {3, 3, 3, 3, 3, 3},                                                          // 顶点的邻接边数量
    {{{0, 2, 3}, {0, 1, 4}, {1, 2, 5}, {3, 6, 8}, {4, 6, 7}, {5, 7, 8}}},        // 顶点的邻接边
    {3, 3, 3, 3, 3, 3},                                                          // 顶点的邻接面数量
    {{{0, 2, 4}, {0, 2, 3}, {0, 3, 4}, {1, 2, 4}, {1, 2, 3}, {1, 3, 4}}},        // 顶点的邻接面
    9,                                                                           // 边数量
    {{{0, 1}, {1, 2}, {2, 0}, {3, 0}, {4, 1}, {5, 2}, {3, 4}, {4, 5}, {5, 3}}},  // 边上顶点的定义
    {2, 2, 2, 2, 2, 2, 2, 2, 2},                                                 // 边的邻接面数量
    {{{0, 2}, {0, 3}, {0, 4}, {2, 4}, {2, 3}, {3, 4}, {1, 2}, {1, 3}, {1, 4}}},  // 边的邻接边
    5,                                                                           // 面数量
    {3, 3, 4, 4, 4},                                                             // 面上顶点的数量
    {{{0, 1, 2}, {3, 5, 4}, {3, 4, 1, 0}, {4, 5, 2, 1}, {5, 3, 0, 2}}},          // 面上顶点的定义
    {3, 3, 4, 4, 4},                                                             // 面上边的数量
    {{{0, 1, 2}, {8, 7, 6}, {3, 6, 4, 0}, {5, 7, 1, 4}, {8, 3, 2, 5}}}           // 面上边的定义
};
/**
 * @brief 六面体的描述符
 */
inline constexpr CellDescriptor<8, 12, 6> HEX = {
    8,                                                                                                   // 顶点数量
    {3, 3, 3, 3, 3, 3, 3, 3},                                                                            // 顶点的邻接边数量
    {{{0, 3, 4}, {0, 1, 5}, {1, 2, 6}, {2, 3, 7}, {4, 8, 11}, {5, 8, 9}, {6, 9, 10}, {7, 10, 11}}},      // 顶点的邻接边
    {3, 3, 3, 3, 3, 3, 3, 3},                                                                            // 顶点的邻接面数量
    {{{0, 2, 5}, {0, 2, 4}, {0, 3, 4}, {0, 3, 5}, {1, 2, 5}, {1, 2, 4}, {1, 3, 4}, {1, 3, 5}}},          // 顶点的邻接面
    12,                                                                                                  // 边数量
    {{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 0}, {5, 1}, {6, 2}, {7, 3}, {4, 5}, {5, 6}, {6, 7}, {7, 4}}},  // 边上顶点的定义
    {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2},                                                                // 边的邻接面数量
    {{{0, 2}, {0, 4}, {0, 3}, {0, 5}, {2, 5}, {2, 4}, {3, 4}, {3, 5}, {1, 2}, {1, 4}, {1, 3}, {1, 5}}},  // 边的邻接边
    6,                                                                                                   // 面数量
    {4, 4, 4, 4, 4, 4},                                                                                  // 面上顶点的数量
    {{{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 4, 5, 1}, {2, 6, 7, 3}, {1, 5, 6, 2}, {0, 3, 7, 4}}},              // 面上顶点的定义
    {4, 4, 4, 4, 4, 4},                                                                                  // 面上边的数量
    {{{0, 1, 2, 3}, {8, 9, 10, 11}, {4, 8, 5, 0}, {6, 9, 10, 2}, {5, 9, 6, 1}, {3, 11, 7, 4}}}           // 面上边的定义
};
/**
 * @brief 获取指定单元类型的描述符（单元拓扑信息）
 * @param type 单元类型枚举值
 * @return 返回对应单元类型的常量描述符引用
 * @throws 当输入为CELL_POLYHEDRON或未知类型时抛出异常
 */
constexpr const CellDescriptor<8, 12, 6> &getDescriptor(CellType type) {
    switch (type) {
        case CellType::CELL_TET:
            return CellDescriptors::TET;
        case CellType::CELL_PYRAMID:
            return CellDescriptors::PYRAMID;
        case CellType::CELL_PRISM:
            return CellDescriptors::PRISM;
        case CellType::CELL_HEX:
            return CellDescriptors::HEX;
        case CellType::CELL_POLYHEDRON:
            throw "This cell type has no descriptor";
        default:
            throw "Unknown cell type";
    }
}
/**
 * @brief 获取指定单元类型中单个面可能包含的最大顶点数
 * @tparam CTYPE 单元类型模板参数，必须是CellType枚举值
 * @return 返回该单元类型中任意面的最大顶点数
 */
template <CellType CTYPE>
constexpr size_t maxNumVertsInFace() {
    static_assert(CTYPE != CellType::CELL_POLYHEDRON, "A polyhedron has any number of verts");
    constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
    size_t maxNum = 0;
    for (size_t i = 0; i < desc.numFaces; ++i) {
        maxNum = std::max<size_t>(maxNum, desc.numVertsInFace[i]);
    }
    return maxNum;
}
/**
 * @brief 获取指定单元类型中单个面可能包含的最大边数
 * @tparam CTYPE 单元类型模板参数，必须是CellType枚举值
 * @return 返回该单元类型中任意面的最大边数
 */
template <CellType CTYPE>
constexpr size_t maxNumEdgesInFace() {
    static_assert(CTYPE != CellType::CELL_POLYHEDRON, "A polyhedron has any number of edges");
    constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
    size_t maxNum = 0;
    for (size_t i = 0; i < desc.numFaces; ++i) {
        maxNum = std::max<size_t>(maxNum, desc.numEdgesInFace[i]);
    }
    return maxNum;
}
};  // namespace CellDescriptors

namespace detail {
template <typename Index>
class Edge {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    Edge() = default;
    Edge(Index a, Index b) : verts_{a, b} {}
    bool operator==(const Edge &other) const {
        return (verts_[0] == other.verts_[0] && verts_[1] == other.verts_[1]) ||
               (verts_[0] == other.verts_[1] && verts_[1] == other.verts_[0]);
    }
    bool operator!=(const Edge &other) const { return !(*this == other); }

    Index *verts() { return verts_.data(); }
    const Index *verts() const { return verts_.data(); }
    void setVerts(Index a, Index b) {
        verts_[0] = a;
        verts_[1] = b;
    }
    size_t numVerts() const { return 2; }

   private:
    std::array<Index, 2> verts_{};
};

/**
 * @brief 面基类模板
 * @tparam Index 索引类型
 */
template <typename Index>
class FaceBase {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    virtual ~FaceBase() = default;

    /**
     * @brief 比较两个面是否相等（考虑顶点顺序和旋转对称性）
     * @param other 要比较的另一个面对象
     * @return 如果两个面类型相同、顶点数相同且顶点序列相同（考虑旋转和反转）则返回true
     */
    bool operator==(const FaceBase &other) const {
        if (type_ != other.type_ || size_ != other.size_) {
            return false;
        }
        if (size_ == 0) {
            return true;
        }
        size_t start_pos = std::distance(verts_, std::min_element(verts_, verts_ + size_));
        size_t other_start_pos = std::distance(other.verts_, std::min_element(other.verts_, other.verts_ + other.size_));
        bool clockwise_match = true;
        for (size_t i = 0; i < size_; ++i) {
            if (verts_[(start_pos + i) % size_] != other.verts_[(other_start_pos + i) % size_]) {
                clockwise_match = false;
                break;
            }
        }
        if (clockwise_match) {
            return true;
        }
        for (size_t i = 0; i < size_; ++i) {
            if (verts_[(start_pos + i) % size_] != other.verts_[(other_start_pos + size_ - i) % size_]) {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const FaceBase &other) const { return !(*this == other); }

    virtual std::shared_ptr<FaceBase> clone() const = 0;

    FaceType type() const noexcept { return type_; }
    Index *verts() noexcept { return verts_; }
    const Index *verts() const noexcept { return verts_; }
    Index *edges() noexcept { return edges_; }
    const Index *edges() const noexcept { return edges_; }
    void setVerts(const Index *indices) {
        assert(indices != nullptr);
        std::copy_n(indices, size_, verts_);
    }
    void setEdges(const Index *indices) {
        assert(indices != nullptr);
        std::copy_n(indices, size_, edges_);
    }
    virtual size_t numVerts() const noexcept = 0;
    virtual size_t numEdges() const noexcept = 0;

   private:
    template <typename, FaceType, size_t>
    friend class BasicFace;
    template <typename>
    friend class PolygonFace;

    const FaceType type_;
    size_t size_;
    Index *verts_ = nullptr;
    Index *edges_ = nullptr;

    FaceBase(FaceType type, size_t size, Index *verts, Index *edges) : type_(type), size_(size), verts_(verts), edges_(edges) {}
    FaceBase(const FaceBase &) = delete;
    FaceBase(FaceBase &&) = delete;
    FaceBase &operator=(const FaceBase &) = delete;
    FaceBase &operator=(FaceBase &&) = delete;
};
/**
 * @brief 基础面类模板，用于表示具有固定顶点数的几何面
 * @tparam Index 顶点索引类型，必须是整数类型
 * @tparam TYPE 面的类型（如三角形、四边形）
 * @tparam NB_VERTS 面的顶点数量，必须大于0
 */
template <typename Index, FaceType TYPE, size_t NB_VERTS>
class BasicFace : public FaceBase<Index> {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");
    static_assert(NB_VERTS > 0, "Size must be positive");
    static_assert(TYPE != FaceType::FACE_POLYGON, "A polygon has any number of vertices and edges");

   public:
    BasicFace() : FaceBase<Index>(TYPE, NB_VERTS, verts_container_.data(), edges_container_.data()) {}
    BasicFace(const BasicFace &other) : FaceBase<Index>(TYPE, NB_VERTS, verts_container_.data(), edges_container_.data()) {
        std::copy(other.verts_container_.begin(), other.verts_container_.end(), verts_container_.begin());
        std::copy(other.edges_container_.begin(), other.edges_container_.end(), edges_container_.begin());
    }
    BasicFace(BasicFace &&other) noexcept
        : FaceBase<Index>(TYPE, NB_VERTS, verts_container_.data(), edges_container_.data()),
          verts_container_(std::move(other.verts_container_)),
          edges_container_(std::move(other.edges_container_)) {}
    BasicFace &operator=(const BasicFace &other) {
        if (this != &other) {
            std::copy(other.verts_container_.begin(), other.verts_container_.end(), verts_container_.begin());
            std::copy(other.edges_container_.begin(), other.edges_container_.end(), edges_container_.begin());
        }
        return *this;
    }
    BasicFace &operator=(BasicFace &&other) noexcept {
        if (this != &other) {
            verts_container_ = std::move(other.verts_container_);
            edges_container_ = std::move(other.edges_container_);
        }
        return *this;
    }

    std::shared_ptr<FaceBase> clone() const override { return std::make_shared<BasicFace>(static_cast<const BasicFace &>(*this)); }

    size_t numVerts() const noexcept override { return NB_VERTS; }
    size_t numEdges() const noexcept override { return NB_VERTS; }

   private:
    std::array<Index, NB_VERTS> verts_container_{};
    std::array<Index, NB_VERTS> edges_container_{};
};
/**
 * @brief 多边形面类模板，用于表示顶点数量可变的几何面
 * @tparam Index 顶点索引类型，必须是整数类型
 */
template <typename Index>
class PolygonFace : public FaceBase<Index> {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    explicit PolygonFace(size_t size = 0)
        : FaceBase<Index>(FaceType::FACE_POLYGON, size, nullptr, nullptr), verts_container_(size), edges_container_(size) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
    }
    PolygonFace(const PolygonFace &other)
        : FaceBase<Index>(FaceType::FACE_POLYGON, other.verts_container_.size(), nullptr, nullptr),
          verts_container_(other.verts_container_),
          edges_container_(other.edges_container_) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
    }
    PolygonFace(PolygonFace &&other) noexcept
        : FaceBase<Index>(FaceType::FACE_POLYGON, other.verts_container_.size(), nullptr, nullptr),
          verts_container_(std::move(other.verts_container_)),
          edges_container_(std::move(other.edges_container_)) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
    }
    PolygonFace &operator=(const PolygonFace &other) {
        if (this != &other) {
            verts_container_ = other.verts_container_;
            edges_container_ = other.edges_container_;
            size_ = verts_container_.size();
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
        }
        return *this;
    }
    PolygonFace &operator=(PolygonFace &&other) noexcept {
        if (this != &other) {
            verts_container_ = std::move(other.verts_container_);
            edges_container_ = std::move(other.edges_container_);
            size_ = verts_container_.size();
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
        }
        return *this;
    }

    std::shared_ptr<FaceBase> clone() const override { return std::make_shared<PolygonFace>(static_cast<const PolygonFace &>(*this)); }

    size_t numVerts() const noexcept override { return verts_container_.size(); }
    size_t numEdges() const noexcept override { return edges_container_.size(); }

   private:
    std::vector<Index> verts_container_{};
    std::vector<Index> edges_container_{};
};
template <typename Index>
using Triangle = BasicFace<Index, FaceType::FACE_TRI, 3>;
template <typename Index>
using Quadrilateral = BasicFace<Index, FaceType::FACE_QUAD, 4>;
template <typename Index>
using Polygon = PolygonFace<Index>;

/**
 * @brief 单元基类模板，用于表示网格中的基本单元
 * @tparam Index 顶点索引类型，必须是整数类型
 */
template <typename Index>
class CellBase {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    virtual ~CellBase() = default;

    bool operator==(const CellBase &other) const {
        if (type_ != other.type_ || verts_size_ != other.verts_size_) {
            return false;
        }
        if (verts_size_ == 0) {
            return true;
        }
        return isCellEqualExtra(other);
    }
    bool operator!=(const CellBase &other) const { return !(*this == other); }

    virtual std::shared_ptr<CellBase> clone() const = 0;

    CellType type() const noexcept { return type_; }
    Index *verts() noexcept { return verts_; }
    const Index *verts() const noexcept { return verts_; }
    Index *edges() noexcept { return edges_; }
    const Index *edges() const noexcept { return edges_; }
    Index *faces() noexcept { return faces_; }
    const Index *faces() const noexcept { return faces_; }
    void setVerts(const Index *indices) {
        assert(indices != nullptr);
        std::copy_n(indices, verts_size_, verts_);
    }
    void setEdges(const Index *indices) {
        assert(indices != nullptr);
        std::copy_n(indices, edges_size_, edges_);
    }
    void setFaces(const Index *indices) {
        assert(indices != nullptr);
        std::copy_n(indices, faces_size_, faces_);
    }

    virtual size_t numVerts() const noexcept = 0;
    virtual size_t numEdges() const noexcept = 0;
    virtual size_t numFaces() const noexcept = 0;

   private:
    template <typename, CellType, size_t, size_t, size_t>
    friend class BasicCell;
    template <typename>
    friend class PolyhedronCell;

    const CellType type_;
    size_t verts_size_;
    size_t edges_size_;
    size_t faces_size_;
    Index *verts_ = nullptr;
    Index *edges_ = nullptr;
    Index *faces_ = nullptr;

    CellBase(CellType type, size_t v_size, size_t e_size, size_t f_size, Index *verts, Index *edges, Index *faces)
        : type_(type), verts_size_(v_size), edges_size_(e_size), faces_size_(f_size), verts_(verts), edges_(edges), faces_(faces) {}
    CellBase(const CellBase &) = delete;
    CellBase(CellBase &&) = delete;
    CellBase &operator=(const CellBase &) = delete;
    CellBase &operator=(CellBase &&) = delete;

    virtual bool isCellEqualExtra(const CellBase &other) const {
        std::vector<Index> this_verts(verts_, verts_ + verts_size_);
        std::vector<Index> other_verts(other.verts_, other.verts_ + other.verts_size_);
        std::sort(this_verts.begin(), this_verts.end());
        std::sort(other_verts.begin(), other_verts.end());
        return this_verts == other_verts;
    }
};
/**
 * @brief 基础单元类模板，用于表示具有固定顶点数、边数和面数的几何单元
 * @tparam Index 顶点索引类型，必须是整数类型
 * @tparam TYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
 * @tparam NB_VERTS 单元的顶点数量，必须大于0
 * @tparam NB_EDGES 单元的边数量，必须大于0
 * @tparam NB_FACES 单元的面数量，必须大于0
 */
template <typename Index, CellType TYPE, size_t NB_VERTS, size_t NB_EDGES, size_t NB_FACES>
class BasicCell : public CellBase<Index> {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");
    static_assert(TYPE != CellType::CELL_POLYHEDRON, "A polyhedron has any multiple vertices, edges and faces");
    static_assert(NB_VERTS > 0, "Cell must have vertices");
    static_assert(NB_EDGES > 0, "Cell must have edges");
    static_assert(NB_FACES > 0, "Cell must have faces");

   public:
    BasicCell()
        : CellBase<Index>(TYPE, NB_VERTS, NB_EDGES, NB_FACES, verts_container_.data(), edges_container_.data(), faces_container_.data()) {}
    BasicCell(const BasicCell &other)
        : CellBase<Index>(other),
          verts_container_(other.verts_container_),
          edges_container_(other.edges_container_),
          faces_container_(other.faces_container_) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
        faces_ = faces_container_.data();
    }
    BasicCell(BasicCell &&other) noexcept
        : CellBase<Index>(std::move(other)),
          verts_container_(std::move(other.verts_container_)),
          edges_container_(std::move(other.edges_container_)),
          faces_container_(std::move(other.faces_container_)) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
        faces_ = faces_container_.data();
    }
    BasicCell &operator=(const BasicCell &other) {
        if (this != &other) {
            verts_container_ = other.verts_container_;
            edges_container_ = other.edges_container_;
            faces_container_ = other.faces_container_;
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
            faces_ = faces_container_.data();
        }
        return *this;
    }
    BasicCell &operator=(BasicCell &&other) noexcept {
        if (this != &other) {
            verts_container_ = std::move(other.verts_container_);
            edges_container_ = std::move(other.edges_container_);
            faces_container_ = std::move(other.faces_container_);
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
            faces_ = faces_container_.data();
        }
        return *this;
    }

    std::shared_ptr<CellBase> clone() const override {
        auto new_cell = std::make_shared<BasicCell>();
        *new_cell = *this;
        return new_cell;
    }

    size_t numVerts() const noexcept override { return NB_VERTS; }
    size_t numEdges() const noexcept override { return NB_EDGES; }
    size_t numFaces() const noexcept override { return NB_FACES; }

   private:
    std::array<Index, NB_VERTS> verts_container_{};
    std::array<Index, NB_EDGES> edges_container_{};
    std::array<Index, NB_FACES> faces_container_{};

    /**
     * @brief 比较当前单元与另一个单元是否相等（考虑顶点顺序无关的比较）
     * @param other 要比较的另一个单元
     * @return 如果顶点集合相同（不考虑顺序）返回true，否则返回false
     */
    bool isCellEqualExtra(const CellBase<Index> &other) const override {
        const Index *other_verts = other.verts();
        if (std::equal(verts_container_.begin(), verts_container_.end(), other_verts)) {
            return true;
        }
        std::array<Index, NB_VERTS> this_sorted = verts_container_;
        std::array<Index, NB_VERTS> other_sorted;
        std::copy_n(other_verts, NB_VERTS, other_sorted.begin());
        std::sort(this_sorted.begin(), this_sorted.end());
        std::sort(other_sorted.begin(), other_sorted.end());
        return this_sorted == other_sorted;
    }
};
/**
 * @brief 多面体单元类模板，用于表示顶点数量可变的几何单元
 * @tparam Index 顶点索引类型，必须是整数类型
 */
template <typename Index>
class PolyhedronCell : public CellBase<Index> {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    PolyhedronCell(size_t nVerts = 0, size_t nEdges = 0, size_t nFaces = 0)
        : CellBase<Index>(CellType::CELL_POLYHEDRON, nVerts, nEdges, nFaces, nullptr, nullptr, nullptr),
          verts_container_(nVerts),
          edges_container_(nEdges),
          faces_container_(nFaces) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
        faces_ = faces_container_.data();
    }
    PolyhedronCell(const PolyhedronCell &other)
        : CellBase<Index>(CellType::CELL_POLYHEDRON, other.verts_container_.size(), other.edges_container_.size(),
                          other.faces_container_.size(), nullptr, nullptr, nullptr),
          verts_container_(other.verts_container_),
          edges_container_(other.edges_container_),
          faces_container_(other.faces_container_) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
        faces_ = faces_container_.data();
    }
    PolyhedronCell(PolyhedronCell &&other) noexcept
        : CellBase<Index>(CellType::CELL_POLYHEDRON, other.verts_container_.size(), other.edges_container_.size(),
                          other.faces_container_.size(), nullptr, nullptr, nullptr),
          verts_container_(std::move(other.verts_container_)),
          edges_container_(std::move(other.edges_container_)),
          faces_container_(std::move(other.faces_container_)) {
        verts_ = verts_container_.data();
        edges_ = edges_container_.data();
        faces_ = faces_container_.data();
    }
    PolyhedronCell &operator=(const PolyhedronCell &other) {
        if (this != &other) {
            verts_container_ = other.verts_container_;
            edges_container_ = other.edges_container_;
            faces_container_ = other.faces_container_;
            verts_size_ = verts_container_.size();
            edges_size_ = edges_container_.size();
            faces_size_ = faces_container_.size();
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
            faces_ = faces_container_.data();
        }
        return *this;
    }
    PolyhedronCell &operator=(PolyhedronCell &&other) noexcept {
        if (this != &other) {
            verts_container_ = std::move(other.verts_container_);
            edges_container_ = std::move(other.edges_container_);
            faces_container_ = std::move(other.faces_container_);
            verts_size_ = verts_container_.size();
            edges_size_ = edges_container_.size();
            faces_size_ = faces_container_.size();
            verts_ = verts_container_.data();
            edges_ = edges_container_.data();
            faces_ = faces_container_.data();
        }
        return *this;
    }

    std::shared_ptr<CellBase> clone() const override {
        auto new_cell = std::make_shared<PolyhedronCell>();
        *new_cell = *this;
        return new_cell;
    }

    size_t numVerts() const noexcept override { return verts_container_.size(); }
    size_t numEdges() const noexcept override { return edges_container_.size(); }
    size_t numFaces() const noexcept override { return faces_container_.size(); }

   private:
    std::vector<Index> verts_container_;
    std::vector<Index> edges_container_;
    std::vector<Index> faces_container_;

    /**
     * @brief 比较当前多面体单元与另一个单元是否相等（考虑面集合的比较）
     * @param other 要比较的另一个单元
     * @return 如果面集合相同（不考虑顺序）返回true，否则返回false
     */
    bool isCellEqualExtra(const CellBase<Index> &other) const override {
        if (edges_container_.size() != other.numEdges() || faces_container_.size() != other.numFaces()) {
            return false;
        }
        assert(edges_container_.size() > 0 && faces_container_.size() > 0);
        std::vector<Index> this_faces(faces_container_);
        std::vector<Index> other_faces(other.faces(), other.faces() + other.numFaces());
        std::sort(this_faces.begin(), this_faces.end());
        std::sort(other_faces.begin(), other_faces.end());
        return this_faces == other_faces;
    }
};
template <typename Index>
using Tetrahedron = BasicCell<Index, CellType::CELL_TET, 4, 6, 4>;
template <typename Index>
using Pyramid = BasicCell<Index, CellType::CELL_PYRAMID, 5, 8, 5>;
template <typename Index>
using Prism = BasicCell<Index, CellType::CELL_PRISM, 6, 9, 5>;
template <typename Index>
using Hexahedron = BasicCell<Index, CellType::CELL_HEX, 8, 12, 6>;
template <typename Index>
using Polyhedron = PolyhedronCell<Index>;

/**
 * @brief 索引迭代器基类模板，用于遍历网格中的索引元素
 * @tparam MeshType 网格类型
 * @tparam Index 元素索引类型，必须是整数类型
 */
template <typename MeshType, typename Index>
class IndexIteratorBase {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    IndexIteratorBase(const MeshType *mesh, Index start, Index (MeshType::*next)(Index) const)
        : mesh_(mesh), current_(start), nextFunc_(next) {}
    Index operator*() const { return current_; }
    IndexIteratorBase &operator++() {
        current_ = (mesh_->*nextFunc_)(current_);
        return *this;
    }
    bool operator==(const IndexIteratorBase &other) const { return current_ == other.current_; }
    bool operator!=(const IndexIteratorBase &other) const { return current_ != other.current_; }

   private:
    const MeshType *mesh_;
    Index current_;
    Index (MeshType::*nextFunc_)(Index) const;
};
/**
 * @brief 迭代器范围封装结构体，用于表示一对迭代器定义的区间
 * @tparam Iter 迭代器类型
 */
template <typename Iter>
struct IteratorRange {
    Iter begin_, end_;
    Iter begin() const noexcept { return begin_; }
    Iter end() const noexcept { return end_; }
};

/**
 * @brief 属性存储基类模板，用于管理网格元素的属性数据
 * @tparam Index 元素索引类型，必须是整数类型
 */
template <typename Index>
class AttributeStorageBase {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    AttributeStorageBase() = default;
    virtual ~AttributeStorageBase() = default;
    AttributeStorageBase(const AttributeStorageBase &) = delete;
    AttributeStorageBase(AttributeStorageBase &&) = delete;
    AttributeStorageBase &operator=(const AttributeStorageBase &) = delete;
    AttributeStorageBase &operator=(AttributeStorageBase &&) = delete;

   protected:
    template <typename Index, typename AttributeName>
    friend class AttributeManager;

    /**
     * @brief 获取属性数据类型的大小
     * @return 返回属性数据类型T的大小（字节数）
     */
    virtual size_t typeSize() const = 0;
    /**
     * @brief 获取属性数据的类型信息
     * @return 返回属性数据类型T的type_info对象
     */
    virtual const std::type_info &typeInfo() const = 0;
    /**
     * @brief 处理元素复制时的属性数据拷贝
     * @param fromId 源元素的索引ID
     * @param toId 目标元素的索引ID
     */
    virtual void onElementCopy(Index fromId, Index toId) = 0;
    /**
     * @brief 处理存储空间大小调整时的操作
     * @param newSize 调整后的新大小
     */
    virtual void onResize(size_t newSize) = 0;
    /**
     * @brief 创建当前属性存储的深拷贝
     * @return 返回指向新拷贝的unique_ptr智能指针
     */
    virtual std::unique_ptr<AttributeStorageBase> clone() const = 0;
};
/**
 * @brief 属性存储模板类，用于存储和管理特定类型的网格属性数据
 * @tparam T 属性数据类型，必须是可平凡复制的类型
 * @tparam Index 元素索引类型，默认为size_t
 */
template <typename T, typename Index = size_t>
class AttributeStorage : public AttributeStorageBase<Index> {
    static_assert(std::is_trivially_copyable_v<T>, "Attribute type must be trivially copyable");
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    AttributeStorage() = default;
    ~AttributeStorage() = default;
    AttributeStorage(const AttributeStorage &) = delete;
    AttributeStorage(AttributeStorage &&) = delete;
    AttributeStorage &operator=(const AttributeStorage &) = delete;
    AttributeStorage &operator=(AttributeStorage &&) = delete;

    /**
     * @brief 获取指定ID对应的属性元素
     * @param id 要获取的元素的索引ID
     * @return 返回对应元素的常量引用
     */
    const T &getElement(Index id) const;
    /**
     * @brief 设置指定ID元素的属性值
     * @param id 要设置的元素的索引ID
     * @param value 要设置的属性值
     */
    void setElement(Index id, const T &value);

   private:
    /**
     * @brief 获取属性数据类型的大小
     * @return 返回属性数据类型T的大小（字节数）
     */
    size_t typeSize() const override { return sizeof(T); }
    /**
     * @brief 获取属性数据的类型信息
     * @return 返回属性数据类型T的type_info对象
     */
    const std::type_info &typeInfo() const override { return typeid(T); }
    /**
     * @brief 处理元素复制时的属性数据拷贝
     * @param fromId 源元素的索引ID
     * @param toId 目标元素的索引ID
     */
    void onElementCopy(Index fromId, Index toId) override;
    /**
     * @brief 处理存储空间大小调整时的操作
     * @param newSize 调整后的新大小
     */
    void onResize(size_t newSize) override;
    /**
     * @brief 创建当前属性存储的深拷贝
     * @return 返回指向新拷贝的unique_ptr智能指针
     */
    std::unique_ptr<AttributeStorageBase<Index>> clone() const override;

    /**
     * @brief 获取指定索引位置的属性数据指针
     * @param index 要获取的数据索引位置
     * @return 返回指向属性数据的指针
     */
    T *getPointer(size_t index);
    /**
     * @brief 获取指定索引位置的常量属性数据指针
     * @param index 要获取的数据索引位置
     * @return 返回指向常量属性数据的指针
     */
    const T *getPointer(size_t index) const;

   private:
    std::vector<unsigned char> data_{};
};
/**
 * @brief 属性存储模板类，用于存储和管理特定类型的网格属性数据
 * @tparam T 属性数据类型，必须是可平凡复制的类型
 * @tparam Index 元素索引类型，默认为size_t
 */
template <typename Index = size_t, typename AttributeName = std::string>
class AttributeManager {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    AttributeManager() = default;
    ~AttributeManager() = default;
    AttributeManager(const AttributeManager &) = delete;
    AttributeManager(AttributeManager &&) = delete;
    AttributeManager &operator=(const AttributeManager &) = delete;
    AttributeManager &operator=(AttributeManager &&) = delete;

    /**
     * @brief 添加指定类型的属性数据
     * @tparam T 要添加的属性数据类型
     * @param name 属性名称
     * @return 添加成功返回true，否则返回false
     */
    template <typename T>
    bool addAttribute(const AttributeName &name);
    /**
     * @brief 移除指定名称的属性数据
     * @param name 要移除的属性名称
     * @return 移除成功返回true，否则返回false
     */
    bool removeAttribute(const AttributeName &name);
    /**
     * @brief 检查是否存在指定名称的属性
     * @param name 要检查的属性名称
     * @return 存在返回true，否则返回false
     */
    bool hasAttribute(const AttributeName &name) const;
    /**
     * @brief 获取指定名称和类型的属性存储
     * @tparam T 期望的属性数据类型
     * @param name 要获取的属性名称
     * @return 返回指向属性存储的指针，若不存在或类型不匹配返回nullptr
     */
    template <typename T>
    AttributeStorage<T, Index> *getAttribute(const AttributeName &name);
    /**
     * @brief 获取指定名称和类型的常量属性存储
     * @tparam T 期望的属性数据类型
     * @param name 要获取的属性名称
     * @return 返回指向常量属性存储的指针，若不存在或类型不匹配返回nullptr
     */
    template <typename T>
    const AttributeStorage<T, Index> *getAttribute(const AttributeName &name) const;
    /**
     * @brief 获取所有属性名称列表
     * @return 返回包含所有属性名称的vector容器
     */
    std::vector<AttributeName> getAttributeNames() const;

   private:
    template <typename Index, typename VertexContainer, typename AttributeName>
    friend class SurfaceMesh;
    template <typename Index, typename VertexContainer, typename AttributeName>
    friend class VolumeMesh;

    size_t size_{0};
    std::unordered_map<AttributeName, std::unique_ptr<AttributeStorageBase<Index>>> attributes_{};

    /**
     * @brief 从另一个属性管理器复制所有属性数据
     * @param other 要复制的源属性管理器
     */
    void copyFrom(const AttributeManager &other);
    /**
     * @brief 处理元素复制时的属性数据拷贝操作
     * @param fromId 源元素的索引ID
     * @param toId 目标元素的索引ID
     */
    void onElementCopy(Index fromId, Index toId);
    /**
     * @brief 处理存储空间大小调整时的操作
     * @param newSize 调整后的新大小
     */
    void onResize(size_t newSize);
    /**
     * @brief 清除所有属性数据
     */
    void clearAttributes();
};
template <typename T, typename Index>
const T &AttributeStorage<T, Index>::getElement(Index id) const {
    size_t index = static_cast<size_t>(id);
    if (index >= data_.size() / sizeof(T)) {
        throw std::runtime_error("Element ID out of range");
    }
    return *reinterpret_cast<const T *>(&data_[index * sizeof(T)]);
}
template <typename T, typename Index>
void AttributeStorage<T, Index>::setElement(Index id, const T &value) {
    size_t index = static_cast<size_t>(id);
    if (index >= data_.size() / sizeof(T)) {
        throw std::runtime_error("Element ID out of range");
    }
    std::memcpy(&data_[index * sizeof(T)], &value, sizeof(T));
}
template <typename T, typename Index>
void AttributeStorage<T, Index>::onElementCopy(Index fromId, Index toId) {
    size_t fromIndex = static_cast<size_t>(fromId);
    size_t toIndex = static_cast<size_t>(toId);
    if (fromIndex >= data_.size() / sizeof(T) || toIndex >= data_.size() / sizeof(T)) {
        throw std::runtime_error("Element ID out of range");
    }
    std::memcpy(&data_[toIndex * sizeof(T)], &data_[fromIndex * sizeof(T)], sizeof(T));
}
template <typename T, typename Index>
void AttributeStorage<T, Index>::onResize(size_t newSize) {
    data_.resize(newSize * sizeof(T));
}
template <typename T, typename Index>
std::unique_ptr<AttributeStorageBase<Index>> AttributeStorage<T, Index>::clone() const {
    auto new_storage = std::make_unique<AttributeStorage<T, Index>>();
    new_storage->data_ = this->data_;
    return new_storage;
}
template <typename T, typename Index>
T *AttributeStorage<T, Index>::getPointer(size_t index) {
    return reinterpret_cast<T *>(&data_[index * sizeof(T)]);
}
template <typename T, typename Index>
const T *AttributeStorage<T, Index>::getPointer(size_t index) const {
    return reinterpret_cast<const T *>(&data_[index * sizeof(T)]);
}
template <typename Index, typename AttributeName>
template <typename T>
bool AttributeManager<Index, AttributeName>::addAttribute(const AttributeName &name) {
    if (hasAttribute(name)) return false;
    attributes_[name] = std::make_unique<AttributeStorage<T, Index>>();
    attributes_[name]->onResize(size_);
    return true;
}
template <typename Index, typename AttributeName>
bool AttributeManager<Index, AttributeName>::removeAttribute(const AttributeName &name) {
    return attributes_.erase(name) > 0;
}
template <typename Index, typename AttributeName>
bool AttributeManager<Index, AttributeName>::hasAttribute(const AttributeName &name) const {
    return attributes_.find(name) != attributes_.end();
}
template <typename Index, typename AttributeName>
template <typename T>
AttributeStorage<T, Index> *AttributeManager<Index, AttributeName>::getAttribute(const AttributeName &name) {
    auto it = attributes_.find(name);
    if (it == attributes_.end()) return nullptr;
    if (it->second->typeInfo() != typeid(T)) return nullptr;
    return static_cast<AttributeStorage<T, Index> *>(it->second.get());
}
template <typename Index, typename AttributeName>
template <typename T>
const AttributeStorage<T, Index> *AttributeManager<Index, AttributeName>::getAttribute(const AttributeName &name) const {
    auto it = attributes_.find(name);
    if (it == attributes_.end()) return nullptr;
    if (it->second->typeInfo() != typeid(T)) return nullptr;
    return static_cast<const AttributeStorage<T, Index> *>(it->second.get());
}
template <typename Index, typename AttributeName>
void AttributeManager<Index, AttributeName>::onElementCopy(Index fromId, Index toId) {
    for (auto &[name, storage] : attributes_) {
        storage->onElementCopy(fromId, toId);
    }
}
template <typename Index, typename AttributeName>
void AttributeManager<Index, AttributeName>::onResize(size_t newSize) {
    for (auto &[name, storage] : attributes_) {
        storage->onResize(newSize);
    }
    size_ = newSize;
}
template <typename Index, typename AttributeName>
void AttributeManager<Index, AttributeName>::clearAttributes() {
    attributes_.clear();
}
template <typename Index, typename AttributeName>
std::vector<AttributeName> AttributeManager<Index, AttributeName>::getAttributeNames() const {
    std::vector<AttributeName> names;
    names.reserve(attributes_.size());
    for (const auto &[name, _] : attributes_) {
        names.push_back(name);
    }
    return names;
}
template <typename Index, typename AttributeName>
void AttributeManager<Index, AttributeName>::copyFrom(const AttributeManager &other) {
    clearAttributes();
    size_ = other.size_;

    for (const auto &[name, storage] : other.attributes_) {
        attributes_[name] = storage->clone();
    }
}
}  // namespace detail

/**
 * @brief 表面网格类模板，提供基础的网格数据结构和属性管理
 * @tparam Index 顶点索引类型，必须是整数类型，默认为size_t
 * @tparam VertexContainer 顶点坐标容器类型，默认为std::array<double, 3>
 * @tparam AttributeName 属性名称类型，默认为std::string
 */
template <typename Index = size_t, typename VertexContainer = std::array<double, 3>, typename AttributeName = std::string>
class SurfaceMesh {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    using BaseClass = SurfaceMesh<Index, VertexContainer, AttributeName>;
    using EdgeContainer = detail::Edge<Index>;
    using FaceContainer = detail::FaceBase<Index>;
    using FacePtr = std::shared_ptr<FaceContainer>;
    using DeletionCallback = std::function<void(Index)>;
    using IndexIterator = detail::IndexIteratorBase<BaseClass, Index>;
    using IndexIteratorRange = detail::IteratorRange<IndexIterator>;
    using AttributeManager = detail::AttributeManager<Index, AttributeName>;

    /**
     * @brief 表示无效索引的常量值
     */
    constexpr static Index kInvalidIndex = std::numeric_limits<Index>::max();

    SurfaceMesh() = default;
    virtual ~SurfaceMesh() = default;
    SurfaceMesh(const SurfaceMesh &) = delete;
    SurfaceMesh(SurfaceMesh &&) = delete;
    SurfaceMesh &operator=(const SurfaceMesh &) = delete;
    SurfaceMesh &operator=(SurfaceMesh &&) = delete;

    /**
     * @brief 获取网格中有效顶点的数量
     * @return 返回当前网格中有效顶点的总数
     */
    size_t numVerts() const { return verts_.size() - freeVerts_.size(); }
    /**
     * @brief 获取网格中有效边的数量
     * @return 返回当前网格中有效边的总数
     */
    size_t numEdges() const { return edges_.size() - freeEdges_.size(); }
    /**
     * @brief 获取网格中有效面的数量
     * @return 返回当前网格中有效面的总数
     */
    size_t numFaces() const { return faces_.size() - freeFaces_.size(); }

    /**
     * @brief 检查网格索引是否连续无空洞（包含顶点/边/面）
     * @return 如果顶点、边和面的索引都连续无空洞则返回true，否则返回false
     */
    bool isIndexContinuous() const { return freeVerts_.empty() && freeEdges_.empty() && freeFaces_.empty(); }

    /**
     * @brief 添加一个新顶点到网格中
     * @param vert 要添加的顶点数据
     * @return 返回新顶点的索引ID
     */
    Index addVert(const VertexContainer &vert);
    /**
     * @brief 获取指定ID顶点的数据
     * @param id 要获取的顶点索引ID
     * @return 返回顶点坐标的引用
     */
    VertexContainer &getVert(Index id) {
        assert(id < verts_.size() && !isVertRemoved(id));
        return verts_[id];
    }
    /**
     * @brief 获取指定ID顶点的数据
     * @param id 要获取的顶点索引ID
     * @return 返回顶点坐标的常量引用
     */
    const VertexContainer &getVert(Index id) const {
        assert(id < verts_.size() && !isVertRemoved(id));
        return verts_[id];
    }
    /**
     * @brief 移除指定ID的顶点（移除操作不会导致其他顶点的索引改变，只会导致顶点索引不连续）
     * @param id 要移除的顶点索引ID
     */
    void removeVert(Index id);
    /**
     * @brief 检查指定ID顶点是否已被移除
     * @param id 要检查的顶点索引ID
     * @return 如果顶点已被移除返回true，否则返回false
     */
    bool isVertRemoved(Index id) const { return freeVerts_.count(id) != 0; }

    /**
     * @brief 在顶点a和b之间添加一条边
     * @param a 边的第一个顶点索引ID
     * @param b 边的第二个顶点索引ID
     * @return 返回新创建边的索引ID
     */
    Index addEdge(Index a, Index b);
    /**
     * @brief 获取指定ID的边数据
     * @param id 要获取的边索引ID
     * @return 返回边数据的常量引用
     */
    const EdgeContainer &getEdge(Index id) const {
        assert(id < edges_.size() && !isEdgeRemoved(id));
        return edges_[id];
    }
    /**
     * @brief 移除指定ID的边（移除操作不会导致其他边的索引改变，只会导致边索引不连续。）
     * @param id 要移除的边索引ID
     */
    void removeEdge(Index id);
    /**
     * @brief 检查指定ID的边是否已被移除
     * @param id 要检查的边索引ID
     * @return 如果边已被移除返回true，否则返回false
     */
    bool isEdgeRemoved(Index id) const { return freeEdges_.count(id) != 0; }
    /**
     * @brief 获取连接两个顶点的边的索引ID
     * @param a 边的第一个顶点索引ID
     * @param b 边的第二个顶点索引ID
     * @return 返回连接这两个顶点的边的索引ID，如果不存在则返回kInvalidIndex
     */
    Index getEdgeIndex(Index a, Index b) const;

    /**
     * @brief 添加一个新面到网格中
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param verts 构成面的顶点索引数组
     * @param numVerts 面的顶点数量
     * @return 返回新创建面的索引ID
     */
    template <FaceType FTYPE>
    Index addFace(const Index *verts, size_t numVerts);
    /**
     * @brief 添加一个新面到网格中（使用初始化列表）
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param verts 构成面的顶点索引初始化列表
     * @return 返回新创建面的索引ID
     */
    template <FaceType FTYPE>
    Index addFace(const std::initializer_list<Index> verts) {
        return addFace<FTYPE>(verts.begin(), verts.size());
    }
    /**
     * @brief 获取指定ID面的数据
     * @param id 要获取的面索引ID
     * @return 返回面数据的常量引用
     */
    const FaceContainer &getFace(Index id) const {
        assert(id < faces_.size() && !isFaceRemoved(id));
        return *faces_[id];
    }
    /**
     * @brief 移除指定ID的面
     * @param id 要移除的面索引ID
     */
    void removeFace(Index id);
    /**
     * @brief 检查指定ID的面是否已被移除
     * @param id 要检查的面索引ID
     * @return 如果面已被移除返回true，否则返回false
     */
    bool isFaceRemoved(Index id) const { return freeFaces_.count(id) != 0; }
    /**
     * @brief 获取由指定顶点构成的面索引ID
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param verts 构成面的顶点索引数组
     * @param numVerts 面的顶点数量
     * @return 返回匹配面的索引ID，如果不存在则返回kInvalidIndex
     */
    template <FaceType FTYPE>
    Index getFaceIndex(const Index *verts, size_t numVerts) const;
    /**
     * @brief 获取由指定顶点构成的面索引ID（使用初始化列表）
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param verts 构成面的顶点索引初始化列表
     * @return 返回匹配面的索引ID，如果不存在则返回kInvalidIndex
     */
    template <FaceType FTYPE>
    Index getFaceIndex(const std::initializer_list<Index> verts) const {
        return getFaceIndex<FTYPE>(verts.begin(), verts.size());
    }

    /**
     * @brief 获取与指定顶点相连的所有边的索引集合
     * @param vid 顶点索引ID
     * @return 返回包含所有相连边索引的常量set引用
     */
    const std::set<Index> &vertEdges(Index vid) const {
        assert(vid < vertToEdges_.size() && !isVertRemoved(vid));
        return vertToEdges_[vid];
    }
    /**
     * @brief 获取与指定顶点相连的所有面的索引集合
     * @param vid 顶点索引ID
     * @return 返回包含所有相连面索引的常量set引用
     */
    const std::set<Index> &vertFaces(Index vid) const {
        assert(vid < vertToFaces_.size() && !isVertRemoved(vid));
        return vertToFaces_[vid];
    }
    /**
     * @brief 获取与指定边相连的所有面的索引集合
     * @param eid 边索引ID
     * @return 返回包含所有相连面索引的常量set引用
     */
    const std::set<Index> &edgeFaces(Index eid) const {
        assert(eid < edgeToFaces_.size() && !isEdgeRemoved(eid));
        return edgeToFaces_[eid];
    }

    /**
     * @brief 获取顶点索引迭代器的起始位置
     * @return 返回指向第一个有效顶点索引的迭代器
     */
    IndexIterator vertIndexBegin() const { return IndexIterator(this, findFirstValidVert(), &SurfaceMesh::findNextValidVert); }
    /**
     * @brief 获取顶点索引迭代器的结束位置
     * @return 返回顶点索引迭代器的结束标记
     */
    IndexIterator vertIndexEnd() const { return IndexIterator(this, kInvalidIndex, &SurfaceMesh::findNextValidVert); }
    /**
     * @brief 获取顶点索引的范围迭代器
     * @return 返回包含所有有效顶点索引的范围迭代器
     */
    IndexIteratorRange vertIndices() const { return {vertIndexBegin(), vertIndexEnd()}; }
    /**
     * @brief 获取边索引迭代器的起始位置
     * @return 返回指向第一个有效边索引的迭代器
     */
    IndexIterator edgeIndexBegin() const { return IndexIterator(this, findFirstValidEdge(), &SurfaceMesh::findNextValidEdge); }
    /**
     * @brief 获取边索引迭代器的结束位置
     * @return 返回边索引迭代器的结束标记
     */
    IndexIterator edgeIndexEnd() const { return IndexIterator(this, kInvalidIndex, &SurfaceMesh::findNextValidEdge); }
    /**
     * @brief 获取边索引的范围迭代器
     * @return 返回包含所有有效边索引的范围迭代器
     */
    IndexIteratorRange edgeIndices() const { return {edgeIndexBegin(), edgeIndexEnd()}; }
    /**
     * @brief 获取面索引迭代器的起始位置
     * @return 返回指向第一个有效面索引的迭代器
     */
    IndexIterator faceIndexBegin() const { return IndexIterator(this, findFirstValidFace(), &SurfaceMesh::findNextValidFace); }
    /**
     * @brief 获取面索引迭代器的结束位置
     * @return 返回面索引迭代器的结束标记
     */
    IndexIterator faceIndexEnd() const { return IndexIterator(this, kInvalidIndex, &SurfaceMesh::findNextValidFace); }
    /**
     * @brief 获取面索引的范围迭代器
     * @return 返回包含所有有效面索引的范围迭代器
     */
    IndexIteratorRange faceIndices() const { return {faceIndexBegin(), faceIndexEnd()}; }

    /**
     * @brief 获取顶点属性管理器
     * @return 返回顶点属性管理器的引用
     */
    AttributeManager &getVertAttributes() { return vertAttributes_; }
    /**
     * @brief 获取边属性管理器
     * @return 返回边属性管理器的引用
     */
    AttributeManager &getEdgeAttributes() { return edgeAttributes_; }
    /**
     * @brief 获取面属性管理器
     * @return 返回面属性管理器的引用
     */
    AttributeManager &getFaceAttributes() { return faceAttributes_; }

    /**
     * @brief 设置顶点删除回调函数
     * @param cb 顶点删除时的回调函数
     */
    void setVertDeletionCallback(DeletionCallback cb) { vertDeletionCb_ = cb; }
    /**
     * @brief 设置边删除回调函数
     * @param cb 边删除时的回调函数
     */
    void setEdgeDeletionCallback(DeletionCallback cb) { edgeDeletionCb_ = cb; }
    /**
     * @brief 设置面删除回调函数
     * @param cb 面删除时的回调函数
     */
    void setFaceDeletionCallback(DeletionCallback cb) { faceDeletionCb_ = cb; }

    /**
     * @brief 移除孤立的顶点（不连接任何边的顶点）
     * @return 返回被移除的孤立顶点数量
     */
    size_t removeIsolatedVerts();
    /**
     * @brief 移除孤立的边（不连接任何面的边）
     * @return 返回被移除的孤立边数量
     */
    size_t removeIsolatedEdges();

    /**
     * @brief 执行网格碎片整理操作（优化内存布局，可使索引连续）
     */
    void defragment() {
        defragmentVertices();
        defragmentEdges();
        defragmentFaces();
    }

    /**
     * @brief 清空网格数据
     * @param keepAttributes 是否保留属性数据（true保留/false清除）
     */
    void clear(bool keepAttributes = true);
    /**
     * @brief 清除所有属性数据
     */
    void clearAttributes();

    /**
     * @brief 从另一个网格复制数据
     * @param other 要复制的源网格对象
     * @param copyAttributes 是否同时复制属性数据（true复制/false不复制）
     */
    void copyFrom(const SurfaceMesh &other, bool copyAttributes = true);

   protected:
    /**
     * @brief 创建指定类型的面对象
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param verts 构成面的顶点索引数组
     * @param numVerts 面的顶点数量
     * @return 返回新创建的面对象指针
     */
    template <FaceType FTYPE>
    FacePtr createFace(const Index *verts, size_t numVerts) const;

    /**
     * @brief 获取指定面对象的索引ID
     * @tparam FTYPE 面的类型（如三角形、四边形）
     * @param face 要查询的面对象
     * @return 返回该面对象的索引ID，如果不存在则返回kInvalidIndex
     */
    template <FaceType FTYPE>
    Index getFaceIndex(const FaceContainer &face) const;

    /**
     * @brief 分配并初始化一个新顶点
     * @param vert 顶点数据
     * @return 返回分配的顶点索引ID
     */
    virtual Index allocateVert(const VertexContainer &vert);
    /**
     * @brief 释放指定顶点
     * @param id 要释放的顶点索引ID
     */
    virtual void deallocateVert(Index id);
    /**
     * @brief 分配并初始化一个新边
     * @param edge 边数据
     * @return 返回分配的边索引ID
     */
    virtual Index allocateEdge(const EdgeContainer &edge);
    /**
     * @brief 释放指定边
     * @param id 要释放的边索引ID
     */
    virtual void deallocateEdge(Index id);
    /**
     * @brief 分配并初始化一个新面
     * @param face 面对象指针
     * @return 返回分配的面索引ID
     */
    virtual Index allocateFace(const FacePtr &face);
    /**
     * @brief 释放指定面
     * @param id 要释放的面索引ID
     */
    virtual void deallocateFace(Index id);

    /**
     * @brief 执行顶点数据的碎片整理（优化内存布局，可使索引连续）
     */
    void defragmentVertices();
    /**
     * @brief 执行边数据的碎片整理（优化内存布局，可使索引连续）
     */
    void defragmentEdges();
    /**
     * @brief 执行面数据的碎片整理（优化内存布局，可使索引连续）
     */
    void defragmentFaces();
    /**
     * @brief 碎片整理后更新顶点引用关系
     * @param newIndices 顶点索引映射表（旧索引→新索引）
     */
    void updateReferencesAfterDefragment(const std::vector<Index> &newIndices);
    /**
     * @brief 碎片整理后更新边引用关系
     * @param newIndices 边索引映射表（旧索引→新索引）
     */
    void updateEdgeReferencesAfterDefragment(const std::vector<Index> &newIndices);
    /**
     * @brief 碎片整理后更新面引用关系
     * @param newIndices 面索引映射表（旧索引→新索引）
     */
    void updateFaceReferencesAfterDefragment(const std::vector<Index> &newIndices);

    /**
     * @brief 查找第一个有效顶点索引
     * @return 返回第一个有效顶点的索引ID，若无则返回kInvalidIndex
     */
    Index findFirstValidVert() const;
    /**
     * @brief 查找当前顶点之后的下一个有效顶点索引
     * @param current 当前顶点索引ID
     * @return 返回下一个有效顶点的索引ID，若无则返回kInvalidIndex
     */

    Index findNextValidVert(Index current) const;
    /**
     * @brief 查找第一个有效边索引
     * @return 返回第一个有效边的索引ID，若无则返回kInvalidIndex
     */
    Index findFirstValidEdge() const;
    /**
     * @brief 查找当前边之后的下一个有效边索引
     * @param current 当前边索引ID
     * @return 返回下一个有效边的索引ID，若无则返回kInvalidIndex
     */
    Index findNextValidEdge(Index current) const;
    /**
     * @brief 查找第一个有效面索引
     * @return 返回第一个有效面的索引ID，若无则返回kInvalidIndex
     */
    Index findFirstValidFace() const;
    /**
     * @brief 查找当前面之后的下一个有效面索引
     * @param current 当前面索引ID
     * @return 返回下一个有效面的索引ID，若无则返回kInvalidIndex
     */
    Index findNextValidFace(Index current) const;

   protected:
    std::vector<VertexContainer> verts_;  // 顶点数据存储
    std::vector<EdgeContainer> edges_;    // 边数据存储
    std::vector<FacePtr> faces_;          // 面数据存储

    AttributeManager vertAttributes_;  // 顶点属性
    AttributeManager edgeAttributes_;  // 边属性
    AttributeManager faceAttributes_;  // 面属性

    std::vector<std::set<Index>> vertToEdges_;  // 顶点到边映射
    std::vector<std::set<Index>> vertToFaces_;  // 顶点到面映射
    std::vector<std::set<Index>> edgeToFaces_;  // 边到面映射

    std::unordered_set<Index> freeVerts_;  // 空闲顶点索引
    std::unordered_set<Index> freeEdges_;  // 空闲边索引
    std::unordered_set<Index> freeFaces_;  // 空闲面索引

    DeletionCallback vertDeletionCb_;  // 顶点删除回调
    DeletionCallback edgeDeletionCb_;  // 边删除回调
    DeletionCallback faceDeletionCb_;  // 面删除回调
};

/**
 * @brief 体网格类模板，提供三维体网格数据结构
 * @tparam Index 顶点/单元索引类型，必须是整数类型，默认为size_t
 * @tparam VertexContainer 顶点坐标容器类型，默认为std::array<double, 3>
 * @tparam AttributeName 属性名称类型，默认为std::string
 */
template <typename Index = size_t, typename VertexContainer = std::array<double, 3>, typename AttributeName = std::string>
class VolumeMesh : public SurfaceMesh<Index, VertexContainer, AttributeName> {
    static_assert(std::is_integral_v<Index>, "Index must be an integer type");

   public:
    using SuperClass = SurfaceMesh<Index, VertexContainer, AttributeName>;
    using BaseClass = VolumeMesh<Index, VertexContainer, AttributeName>;
    using CellContainer = detail::CellBase<Index>;
    using CellPtr = std::shared_ptr<CellContainer>;
    using DeletionCallback = typename SuperClass::DeletionCallback;
    using IndexIterator = detail::IndexIteratorBase<VolumeMesh, Index>;
    using IndexIteratorRange = detail::IteratorRange<IndexIterator>;
    using AttributeManager = typename SuperClass::AttributeManager;

    using SuperClass::kInvalidIndex;

    VolumeMesh() = default;
    ~VolumeMesh() = default;
    VolumeMesh(const VolumeMesh &) = delete;
    VolumeMesh(VolumeMesh &&) = delete;
    VolumeMesh &operator=(const VolumeMesh &) = delete;
    VolumeMesh &operator=(VolumeMesh &&) = delete;

    /**
     * @brief 获取网格中有效单元的数量
     * @return 返回当前网格中有效单元的总数
     */
    size_t numCells() const { return cells_.size() - freeCells_.size(); }
    /**
     * @brief 检查网格索引是否连续无空洞（包含顶点/边/面/单元）
     * @return 如果所有元素索引都连续无空洞则返回true，否则返回false
     */
    bool isIndexContinuous() const { return SuperClass::isIndexContinuous() && freeCells_.empty(); }

    /**
     * @brief 添加一个新单元到网格中
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param verts 构成单元的顶点索引数组
     * @param numVerts 单元的顶点数量
     * @param edges 构成单元的边索引数组（可选）
     * @param numEdges 单元的边数量（可选）
     * @param faces 构成单元的面索引数组（可选）
     * @param numFaces 单元的面数量（可选）
     * @return 返回新创建单元的索引ID
     */
    template <CellType CTYPE>
    Index addCell(const Index *verts, size_t numVerts, const Index *edges = nullptr, size_t numEdges = 0, const Index *faces = nullptr,
                  size_t numFaces = 0);
    /**
     * @brief 添加一个新单元到网格中（使用初始化列表）
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param verts 构成单元的顶点索引初始化列表
     * @param edges 构成单元的边索引初始化列表（可选）
     * @param faces 构成单元的面索引初始化列表（可选）
     * @return 返回新创建单元的索引ID
     */
    template <CellType CTYPE>
    Index addCell(const std::initializer_list<Index> verts, const std::initializer_list<Index> edges = {},
                  const std::initializer_list<Index> faces = {}) {
        return addCell<CTYPE>(verts.begin(), verts.size(), edges.begin(), edges.size(), faces.begin(), faces.size());
    }
    /**
     * @brief 获取指定ID的单元数据
     * @param id 要获取的单元索引ID
     * @return 返回单元数据的常量引用
     */
    const CellContainer &getCell(Index id) const {
        assert(id < cells_.size() && !isCellRemoved(id));
        return *cells_[id];
    }
    /**
     * @brief 移除指定ID的单元
     * @param id 要移除的单元索引ID
     */
    void removeCell(Index id);
    /**
     * @brief 检查指定ID的单元是否已被移除
     * @param id 要检查的单元索引ID
     * @return 如果单元已被移除返回true，否则返回false
     */
    bool isCellRemoved(Index id) const { return freeCells_.count(id) != 0; }
    /**
     * @brief 获取由指定元素构成的单元索引ID
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param verts 构成单元的顶点索引数组
     * @param numVerts 顶点数量
     * @param edges 构成单元的边索引数组（可选）
     * @param numEdges 边数量（可选）
     * @param faces 构成单元的面索引数组（可选）
     * @param numFaces 面数量（可选）
     * @return 返回匹配单元的索引ID，如果不存在则返回kInvalidIndex
     */
    template <CellType CTYPE>
    Index getCellIndex(const Index *verts, size_t numVerts, const Index *edges = nullptr, size_t numEdges = 0, const Index *faces = nullptr,
                       size_t numFaces = 0) const;
    /**
     * @brief 获取由指定元素构成的单元索引ID（使用初始化列表）
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param verts 构成单元的顶点索引初始化列表
     * @param edges 构成单元的边索引初始化列表（可选）
     * @param faces 构成单元的面索引初始化列表（可选）
     * @return 返回匹配单元的索引ID，如果不存在则返回kInvalidIndex
     */
    template <CellType CTYPE>
    Index getCellIndex(const std::initializer_list<Index> verts, const std::initializer_list<Index> edges = {},
                       const std::initializer_list<Index> faces = {}) const {
        return getCellIndex<CTYPE>(verts.begin(), verts.size(), edges.begin(), edges.size(), faces.begin(), faces.size());
    }

    /**
     * @brief 获取与指定顶点相连的所有单元的索引集合
     * @param vid 顶点索引ID
     * @return 返回包含所有相连单元索引的常量set引用
     */
    const std::set<Index> &vertCells(Index vid) const {
        assert(vid < vertToCells_.size() && !isVertRemoved(vid));
        return vertToCells_[vid];
    }
    /**
     * @brief 获取与指定边相连的所有单元的索引集合
     * @param eid 边索引ID
     * @return 返回包含所有相连单元索引的常量set引用
     */
    const std::set<Index> &edgeCells(Index eid) const {
        assert(eid < edgeToCells_.size() && !isEdgeRemoved(eid));
        return edgeToCells_[eid];
    }
    /**
     * @brief 获取与指定面相连的所有单元的索引集合
     * @param fid 面索引ID
     * @return 返回包含所有相连单元索引的常量set引用
     */
    const std::set<Index> &faceCells(Index fid) const {
        assert(fid < faceToCells_.size() && !isFaceRemoved(fid));
        return faceToCells_[fid];
    }

    /**
     * @brief 获取单元索引迭代器的起始位置
     * @return 返回指向第一个有效单元索引的迭代器
     */
    IndexIterator cellIndexBegin() const { return IndexIterator(this, findFirstValidCell(), &VolumeMesh::findNextValidCell); }
    /**
     * @brief 获取单元索引迭代器的结束位置
     * @return 返回单元索引迭代器的结束标记
     */
    IndexIterator cellIndexEnd() const { return IndexIterator(this, kInvalidIndex, &VolumeMesh::findNextValidCell); }
    /**
     * @brief 获取单元索引的范围迭代器
     * @return 返回包含所有有效单元索引的范围迭代器
     */
    IndexIteratorRange cellIndices() const { return {cellIndexBegin(), cellIndexEnd()}; }

    /**
     * @brief 获取单元属性管理器
     * @return 返回单元属性管理器的引用
     */
    AttributeManager &getCellAttributes() { return cellAttributes_; }

    /**
     * @brief 设置单元删除回调函数
     * @param cb 单元删除时的回调函数
     */
    void setCellDeletionCallback(DeletionCallback cb) { cellDeletionCb_ = cb; }

    /**
     * @brief 移除孤立的顶点（不连接任何边/面/单元的顶点）
     * @return 返回被移除的孤立顶点数量
     */
    size_t removeIsolatedVerts();
    /**
     * @brief 移除孤立的边（不连接任何面/单元的边）
     * @return 返回被移除的孤立边数量
     */
    size_t removeIsolatedEdges();
    /**
     * @brief 移除孤立的面（不连接任何单元的面）
     * @return 返回被移除的孤立面数量
     */
    size_t removeIsolatedFaces();

    /**
     * @brief 执行网格碎片整理操作（优化内存布局，可使索引连续）
     */
    void defragment() {
        SuperClass::defragment();
        defragmentCells();
    }

    /**
     * @brief 清空网格数据（包括顶点/边/面/单元）
     * @param keepAttributes 是否保留属性数据（true保留/false清除）
     */
    void clear(bool keepAttributes = true);
    /**
     * @brief 清除所有属性数据（顶点/边/面/单元）
     */
    void clearAttributes();

    /**
     * @brief 从另一个体网格复制数据
     * @param other 要复制的源体网格对象
     * @param copyAttributes 是否同时复制属性数据（true复制/false不复制）
     */
    void copyFrom(const VolumeMesh<Index, VertexContainer, AttributeName> &other, bool copyAttributes = true);

   protected:
    /**
     * @brief 创建指定类型的单元对象
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param verts 构成单元的顶点索引数组
     * @param numVerts 顶点数量
     * @param edges 构成单元的边索引数组
     * @param numEdges 边数量
     * @param faces 构成单元的面索引数组
     * @param numFaces 面数量
     * @return 返回新创建的单元对象指针
     */
    template <CellType CTYPE>
    CellPtr createCell(const Index *verts, size_t numVerts, const Index *edges, size_t numEdges, const Index *faces, size_t numFaces) const;

    /**
     * @brief 获取指定单元对象的索引ID
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param cell 要查询的单元对象
     * @return 返回该单元对象的索引ID，如果不存在则返回kInvalidIndex
     */
    template <CellType CTYPE>
    Index getCellIndex(const CellContainer &cell) const;

    /**
     * @brief 分配并初始化一个新顶点
     * @param vert 顶点坐标数据
     * @return 返回分配的顶点索引ID
     */
    Index allocateVert(const VertexContainer &vert) override;
    /**
     * @brief 释放指定顶点
     * @param id 要释放的顶点索引ID
     */
    void deallocateVert(Index id) override;
    /**
     * @brief 分配并初始化一个新边
     * @param edge 边数据
     * @return 返回分配的边索引ID
     */
    Index allocateEdge(const EdgeContainer &edge) override;
    /**
     * @brief 释放指定边
     * @param id 要释放的边索引ID
     */
    void deallocateEdge(Index id) override;
    /**
     * @brief 分配并初始化一个新面
     * @param face 面对象指针
     * @return 返回分配的面索引ID
     */
    Index allocateFace(const FacePtr &face) override;
    /**
     * @brief 释放指定面
     * @param id 要释放的面索引ID
     */

    void deallocateFace(Index id) override;
    /**
     * @brief 分配并初始化一个新单元
     * @param cell 单元对象指针
     * @return 返回分配的单元索引ID
     */

    Index allocateCell(const CellPtr &cell);
    /**
     * @brief 释放指定单元
     * @param id 要释放的单元索引ID
     */
    void deallocateCell(Index id);

    /**
     * @brief 自动生成单元的所有边（根据顶点连接关系）
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param cell 单元对象指针
     */
    template <CellType CTYPE>
    void autoGenerateEdges(CellPtr &cell);
    /**
     * @brief 自动生成单元的所有面（根据顶点/边连接关系）
     * @tparam CTYPE 单元类型（如四面体、金字塔、三棱柱、六面体）
     * @param cell 单元对象指针
     */
    template <CellType CTYPE>
    void autoGenerateFaces(CellPtr &cell);

    /**
     * @brief 执行顶点数据的碎片整理（优化内存布局）
     */
    void defragmentVertices();
    /**
     * @brief 执行边数据的碎片整理（优化内存布局）
     */
    void defragmentEdges();
    /**
     * @brief 执行面数据的碎片整理（优化内存布局）
     */
    void defragmentFaces();
    /**
     * @brief 执行单元数据的碎片整理（优化内存布局）
     */
    void defragmentCells();
    /**
     * @brief 碎片整理后更新顶点引用关系
     * @param newIndices 顶点索引映射表（旧索引→新索引）
     */
    void updateVertexReferencesAfterDefragment(const std::vector<Index> &newIndices);
    /**
     * @brief 碎片整理后更新边引用关系
     * @param newIndices 边索引映射表（旧索引→新索引）
     */
    void updateEdgeReferencesAfterDefragment(const std::vector<Index> &newIndices);
    /**
     * @brief 碎片整理后更新面引用关系
     * @param newIndices 面索引映射表（旧索引→新索引）
     */
    void updateFaceReferencesAfterDefragment(const std::vector<Index> &newIndices);
    /**
     * @brief 碎片整理后更新单元引用关系
     * @param newIndices 单元索引映射表（旧索引→新索引）
     */
    void updateCellReferencesAfterDefragment(const std::vector<Index> &newIndices);

    /**
     * @brief 查找第一个有效单元索引
     * @return 返回第一个有效单元的索引ID，若无则返回kInvalidIndex
     */
    Index findFirstValidCell() const;
    /**
     * @brief 查找当前单元之后的下一个有效单元索引
     * @param current 当前单元索引ID
     * @return 返回下一个有效单元的索引ID，若无则返回kInvalidIndex
     */
    Index findNextValidCell(Index current) const;

   protected:
    std::vector<CellPtr> cells_;                // 体数据存储
    AttributeManager cellAttributes_;           // 体属性
    std::vector<std::set<Index>> vertToCells_;  // 顶点到体映射
    std::vector<std::set<Index>> edgeToCells_;  // 边到体映射
    std::vector<std::set<Index>> faceToCells_;  // 面到体映射
    std::unordered_set<Index> freeCells_;       // 空闲体索引
    DeletionCallback cellDeletionCb_;           // 体删除回调
};

template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::addVert(const VertexContainer &vert) {
    return allocateVert(vert);
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::removeVert(Index id) {
    assert(id < verts_.size());
    if (isVertRemoved(id)) {
        return;
    }
    if (!vertToEdges_[id].empty() || !vertToFaces_[id].empty()) {
        throw std::runtime_error("Vertex still referenced");
    }
    if (vertDeletionCb_) {
        vertDeletionCb_(id);
    }
    deallocateVert(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::addEdge(Index a, Index b) {
    assert(a < verts_.size() && !isVertRemoved(a));
    assert(b < verts_.size() && !isVertRemoved(b));
    Index id = getEdgeIndex(a, b);
    if (id != kInvalidIndex) {
        return id;
    }
    EdgeContainer newEdge(a, b);
    id = allocateEdge(newEdge);
    vertToEdges_[a].insert(id);
    vertToEdges_[b].insert(id);
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::removeEdge(Index id) {
    assert(id < edges_.size());
    if (isEdgeRemoved(id)) {
        return;
    }
    if (!edgeToFaces_[id].empty()) {
        throw std::runtime_error("Edge still referenced");
    }
    const auto &edge = edges_[id];
    vertToEdges_[edge.verts()[0]].erase(id);
    vertToEdges_[edge.verts()[1]].erase(id);
    if (edgeDeletionCb_) {
        edgeDeletionCb_(id);
    }
    deallocateEdge(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::getEdgeIndex(Index a, Index b) const {
    for (Index eid : vertToEdges_[a]) {
        if (edges_[eid].verts()[0] == b || edges_[eid].verts()[1] == b) {
            return eid;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::removeFace(Index id) {
    assert(id < faces_.size());
    if (isFaceRemoved(id)) {
        return;
    }
    const auto &face = faces_[id];
    for (size_t i = 0; i < face->numVerts(); ++i) {
        Index vid = face->verts()[i];
        vertToFaces_[vid].erase(id);
    }
    for (size_t i = 0; i < face->numEdges(); ++i) {
        Index eid = face->edges()[i];
        edgeToFaces_[eid].erase(id);
        if (edgeToFaces_[eid].empty()) {
            removeEdge(eid);
        }
    }
    if (faceDeletionCb_) {
        faceDeletionCb_(id);
    }
    deallocateFace(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
size_t SurfaceMesh<Index, VertexContainer, AttributeName>::removeIsolatedVerts() {
    std::vector<Index> toRemove;
    for (Index vid = 0; vid < verts_.size(); ++vid) {
        if (!isVertRemoved(vid) && vertToEdges_[vid].empty() && vertToFaces_[vid].empty()) {
            toRemove.push_back(vid);
        }
    }
    for (Index vid : toRemove) {
        removeVert(vid);
    }
    return toRemove.size();
}
template <typename Index, typename VertexContainer, typename AttributeName>
size_t SurfaceMesh<Index, VertexContainer, AttributeName>::removeIsolatedEdges() {
    std::vector<Index> toRemove;
    for (Index eid = 0; eid < edges_.size(); ++eid) {
        if (!isEdgeRemoved(eid) && edgeToFaces_[eid].empty()) {
            toRemove.push_back(eid);
        }
    }
    for (Index eid : toRemove) {
        removeEdge(eid);
    }
    return toRemove.size();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::clear(bool keepAttributes) {
    verts_.clear();
    edges_.clear();
    faces_.clear();

    vertToEdges_.clear();
    vertToFaces_.clear();
    edgeToFaces_.clear();

    freeVerts_.clear();
    freeEdges_.clear();
    freeFaces_.clear();

    vertDeletionCb_ = nullptr;
    edgeDeletionCb_ = nullptr;
    faceDeletionCb_ = nullptr;

    vertAttributes_.onResize(0);
    edgeAttributes_.onResize(0);
    faceAttributes_.onResize(0);
    if (!keepAttributes) {
        vertAttributes_.clearAttributes();
        edgeAttributes_.clearAttributes();
        faceAttributes_.clearAttributes();
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::clearAttributes() {
    vertAttributes_.clearAttributes();
    edgeAttributes_.clearAttributes();
    faceAttributes_.clearAttributes();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::copyFrom(const SurfaceMesh<Index, VertexContainer, AttributeName> &other,
                                                                  bool copyAttributes) {
    verts_ = other.verts_;
    edges_ = other.edges_;
    faces_.clear();
    faces_.reserve(other.faces_.size());
    for (Index i = 0; i < other.faces_.size(); ++i) {
        faces_.push_back(other.faces_[i]->clone());
    }

    vertToEdges_ = other.vertToEdges_;
    vertToFaces_ = other.vertToFaces_;
    edgeToFaces_ = other.edgeToFaces_;

    freeVerts_ = other.freeVerts_;
    freeEdges_ = other.freeEdges_;
    freeFaces_ = other.freeFaces_;

    if (copyAttributes) {
        vertAttributes_.copyFrom(other.vertAttributes_);
        edgeAttributes_.copyFrom(other.edgeAttributes_);
        faceAttributes_.copyFrom(other.faceAttributes_);
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::allocateVert(const VertexContainer &vert) {
    Index id;
    if (freeVerts_.empty()) {
        id = static_cast<Index>(verts_.size());
        verts_.push_back(vert);
        vertAttributes_.onResize(verts_.size());
        vertToEdges_.emplace_back();
        vertToFaces_.emplace_back();
    } else {
        id = *freeVerts_.begin();
        freeVerts_.erase(freeVerts_.begin());
        verts_[id] = vert;
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::deallocateVert(Index id) {
    freeVerts_.insert(id);
    vertToEdges_[id].clear();
    vertToFaces_[id].clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::allocateEdge(const EdgeContainer &edge) {
    Index id;
    if (freeEdges_.empty()) {
        id = static_cast<Index>(edges_.size());
        edges_.push_back(edge);
        edgeAttributes_.onResize(edges_.size());
        edgeToFaces_.emplace_back();
    } else {
        id = *freeEdges_.begin();
        freeEdges_.erase(freeEdges_.begin());
        edges_[id] = edge;
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::deallocateEdge(Index id) {
    freeEdges_.insert(id);
    edgeToFaces_[id].clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::allocateFace(const FacePtr &face) {
    Index id;
    if (freeFaces_.empty()) {
        id = static_cast<Index>(faces_.size());
        faces_.push_back(face);
        faceAttributes_.onResize(faces_.size());
    } else {
        id = *freeFaces_.begin();
        freeFaces_.erase(freeFaces_.begin());
        faces_[id] = face;
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::deallocateFace(Index id) {
    freeFaces_.insert(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::defragmentVertices() {
    std::vector<Index> newIndices(verts_.size(), kInvalidIndex);
    std::vector<VertexContainer> newVerts;
    std::vector<std::set<Index>> newVertToEdges, newVertToFaces;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < verts_.size(); ++oldIndex) {
        if (!isVertRemoved(oldIndex)) {
            vertAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newVerts.push_back(verts_[oldIndex]);
            newVertToEdges.push_back(vertToEdges_[oldIndex]);
            newVertToFaces.push_back(vertToFaces_[oldIndex]);
        }
    }
    vertAttributes_.onResize(newIndex);

    updateReferencesAfterDefragment(newIndices);

    verts_ = std::move(newVerts);
    vertToEdges_ = std::move(newVertToEdges);
    vertToFaces_ = std::move(newVertToFaces);
    freeVerts_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::defragmentEdges() {
    std::vector<Index> newIndices(edges_.size(), kInvalidIndex);
    std::vector<EdgeContainer> newEdges;
    std::vector<std::set<Index>> newEdgeToFaces;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < edges_.size(); ++oldIndex) {
        if (!isEdgeRemoved(oldIndex)) {
            edgeAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newEdges.push_back(edges_[oldIndex]);
            newEdgeToFaces.push_back(edgeToFaces_[oldIndex]);
        }
    }
    edgeAttributes_.onResize(newIndex);

    updateEdgeReferencesAfterDefragment(newIndices);

    edges_ = std::move(newEdges);
    edgeToFaces_ = std::move(newEdgeToFaces);
    freeEdges_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::defragmentFaces() {
    std::vector<Index> newIndices(faces_.size(), kInvalidIndex);
    std::vector<FacePtr> newFaces;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < faces_.size(); ++oldIndex) {
        if (!isFaceRemoved(oldIndex)) {
            faceAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newFaces.push_back(faces_[oldIndex]);
        }
    }
    faceAttributes_.onResize(newIndex);

    updateFaceReferencesAfterDefragment(newIndices);

    faces_ = std::move(newFaces);
    freeFaces_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::updateReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    for (auto &edge : edges_) {
        for (size_t i = 0; i < 2; ++i) {
            Index oldIndex = edge.verts()[i];
            if (newIndices[oldIndex] != kInvalidIndex) {
                edge.verts()[i] = newIndices[oldIndex];
            }
        }
    }
    for (auto &face : faces_) {
        if (face) {
            for (size_t i = 0; i < face->numVerts(); ++i) {
                Index oldIndex = face->verts()[i];
                if (newIndices[oldIndex] != kInvalidIndex) {
                    face->verts()[i] = newIndices[oldIndex];
                }
            }
        }
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::updateEdgeReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    for (auto &edgeSet : vertToEdges_) {
        std::set<Index> newSet;
        for (Index oldIndex : edgeSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        edgeSet = std::move(newSet);
    }
    for (auto &face : faces_) {
        if (face) {
            for (size_t i = 0; i < face->numEdges(); ++i) {
                Index oldIndex = face->edges()[i];
                if (newIndices[oldIndex] != kInvalidIndex) {
                    face->edges()[i] = newIndices[oldIndex];
                }
            }
        }
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void SurfaceMesh<Index, VertexContainer, AttributeName>::updateFaceReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    for (auto &faceSet : vertToFaces_) {
        std::set<Index> newSet;
        for (Index oldIndex : faceSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        faceSet = std::move(newSet);
    }
    for (auto &faceSet : edgeToFaces_) {
        std::set<Index> newSet;
        for (Index oldIndex : faceSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        faceSet = std::move(newSet);
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findFirstValidVert() const {
    for (Index i = 0; i < verts_.size(); ++i) {
        if (!isVertRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findNextValidVert(Index current) const {
    for (Index i = current + 1; i < verts_.size(); ++i) {
        if (!isVertRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findFirstValidEdge() const {
    for (Index i = 0; i < edges_.size(); ++i) {
        if (!isEdgeRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findNextValidEdge(Index current) const {
    for (Index i = current + 1; i < edges_.size(); ++i) {
        if (!isEdgeRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findFirstValidFace() const {
    for (Index i = 0; i < faces_.size(); ++i) {
        if (!isFaceRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::findNextValidFace(Index current) const {
    for (Index i = current + 1; i < faces_.size(); ++i) {
        if (!isFaceRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <FaceType FTYPE>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::addFace(const Index *verts, size_t numVerts) {
    if constexpr (FTYPE != FaceType::FACE_POLYGON) {
        assert(numVerts == static_cast<size_t>(FTYPE));
    } else {
        assert(numVerts > 0);
    }

    // Validate vertices
    for (size_t i = 0; i < numVerts; ++i) {
        Index vid = verts[i];
        assert(vid < verts_.size() && !isVertRemoved(vid));
    }

    // Create face
    FacePtr face = createFace<FTYPE>(verts, numVerts);

    // Check for duplicate face
    Index id = getFaceIndex<FTYPE>(*face);
    if (id != kInvalidIndex) {
        return id;
    }

    // Create edges for the face
    if constexpr (FTYPE != FaceType::FACE_POLYGON) {
        std::array<Index, static_cast<size_t>(FTYPE)> edgeIndices;
        for (size_t i = 0; i < static_cast<size_t>(FTYPE); ++i) {
            Index a = verts[i];
            Index b = verts[(i + 1) % numVerts];
            edgeIndices[i] = addEdge(a, b);
        }
        face->setEdges(edgeIndices.data());
    } else {
        std::vector<Index> edgeIndices(numVerts);
        for (size_t i = 0; i < numVerts; ++i) {
            Index a = verts[i];
            Index b = verts[(i + 1) % numVerts];
            edgeIndices[i] = addEdge(a, b);
        }
        face->setEdges(edgeIndices.data());
    }

    id = allocateFace(face);

    // Update topology
    for (size_t i = 0; i < numVerts; ++i) {
        Index vid = verts[i];
        vertToFaces_[vid].insert(id);
    }
    for (size_t i = 0; i < numVerts; ++i) {
        Index eid = face->edges()[i];
        edgeToFaces_[eid].insert(id);
    }

    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <FaceType FTYPE>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::getFaceIndex(const Index *verts, size_t numVerts) const {
    if constexpr (FTYPE != FaceType::FACE_POLYGON) {
        assert(numVerts == static_cast<size_t>(FTYPE));
    } else {
        assert(numVerts > 0);
    }

    if constexpr (FTYPE == FaceType::FACE_TRI) {
        detail::Triangle<Index> face;
        face.setVerts(verts);
        return getFaceIndex<FTYPE>(face);
    } else if constexpr (FTYPE == FaceType::FACE_QUAD) {
        detail::Quadrilateral<Index> face;
        face.setVerts(verts);
        return getFaceIndex<FTYPE>(face);
    } else {
        detail::PolygonFace<Index> face(numVerts);
        face.setVerts(verts);
        return getFaceIndex<FTYPE>(face);
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <FaceType FTYPE>
typename SurfaceMesh<Index, VertexContainer, AttributeName>::FacePtr SurfaceMesh<Index, VertexContainer, AttributeName>::createFace(
    const Index *verts, size_t numVerts) const {
    if constexpr (FTYPE == FaceType::FACE_TRI) {
        auto face = std::make_shared<detail::Triangle<Index>>();
        face->setVerts(verts);
        return face;
    } else if constexpr (FTYPE == FaceType::FACE_QUAD) {
        auto face = std::make_shared<detail::Quadrilateral<Index>>();
        face->setVerts(verts);
        return face;
    } else {
        auto face = std::make_shared<detail::PolygonFace<Index>>(numVerts);
        face->setVerts(verts);
        return face;
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <FaceType FTYPE>
Index SurfaceMesh<Index, VertexContainer, AttributeName>::getFaceIndex(const FaceContainer &face) const {
    for (Index fid : vertToFaces_[face.verts()[0]]) {
        if (*faces_[fid] == face) {
            return fid;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::removeCell(Index id) {
    assert(id < cells_.size());
    if (isCellRemoved(id)) return;

    const auto &cell = cells_[id];

    // Remove from vertex mapping
    for (size_t i = 0; i < cell->numVerts(); ++i) {
        Index vid = cell->verts()[i];
        vertToCells_[vid].erase(id);
    }

    // Remove from edge mapping
    for (size_t i = 0; i < cell->numEdges(); ++i) {
        Index eid = cell->edges()[i];
        edgeToCells_[eid].erase(id);
        if (edgeToCells_[eid].empty() && edgeToFaces_[eid].empty()) {
            removeEdge(eid);
        }
    }

    // Remove from face mapping
    for (size_t i = 0; i < cell->numFaces(); ++i) {
        Index fid = cell->faces()[i];
        faceToCells_[fid].erase(id);
        if (faceToCells_[fid].empty()) {
            removeFace(fid);
        }
    }

    if (cellDeletionCb_) {
        cellDeletionCb_(id);
    }
    deallocateCell(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
size_t VolumeMesh<Index, VertexContainer, AttributeName>::removeIsolatedVerts() {
    std::vector<Index> toRemove;
    for (Index vid = 0; vid < verts_.size(); ++vid) {
        if (!isVertRemoved(vid) && vertToEdges_[vid].empty() && vertToFaces_[vid].empty() && vertToCells_[vid].empty()) {
            toRemove.push_back(vid);
        }
    }
    for (Index vid : toRemove) {
        removeVert(vid);
    }
    return toRemove.size();
}
template <typename Index, typename VertexContainer, typename AttributeName>
size_t VolumeMesh<Index, VertexContainer, AttributeName>::removeIsolatedEdges() {
    std::vector<Index> toRemove;
    for (Index eid = 0; eid < edges_.size(); ++eid) {
        if (!isEdgeRemoved(eid) && edgeToFaces_[eid].empty() && edgeToCells_[eid].empty()) {
            toRemove.push_back(eid);
        }
    }
    for (Index eid : toRemove) {
        removeEdge(eid);
    }
    return toRemove.size();
}
template <typename Index, typename VertexContainer, typename AttributeName>
size_t VolumeMesh<Index, VertexContainer, AttributeName>::removeIsolatedFaces() {
    std::vector<Index> toRemove;
    for (Index fid = 0; fid < faces_.size(); ++fid) {
        if (!isFaceRemoved(fid) && faceToCells_[fid].empty()) {
            toRemove.push_back(fid);
        }
    }
    for (Index fid : toRemove) {
        removeFace(fid);
    }
    return toRemove.size();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::clear(bool keepAttributes) {
    SuperClass::clear(keepAttributes);
    cells_.clear();
    vertToCells_.clear();
    edgeToCells_.clear();
    faceToCells_.clear();
    freeCells_.clear();
    cellDeletionCb_ = nullptr;
    cellAttributes_.onResize(0);
    if (!keepAttributes) {
        cellAttributes_.clearAttributes();
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::clearAttributes() {
    SuperClass::clearAttributes();
    cellAttributes_.clearAttributes();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::copyFrom(const VolumeMesh<Index, VertexContainer, AttributeName> &other,
                                                                 bool copyAttributes) {
    SuperClass::copyFrom(other, copyAttributes);
    cells_.clear();
    cells_.reserve(other.cells_.size());
    for (Index i = 0; i < other.cells_.size(); ++i) {
        cells_.push_back(other.cells_[i]->clone());
    }

    vertToCells_ = other.vertToCells_;
    edgeToCells_ = other.edgeToCells_;
    faceToCells_ = other.faceToCells_;

    freeCells_ = other.freeCells_;

    if (copyAttributes) {
        cellAttributes_.copyFrom(other.cellAttributes_);
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::allocateVert(const VertexContainer &vert) {
    Index id = SuperClass::allocateVert(vert);
    if (vertToCells_.size() < verts_.size()) {
        vertToCells_.resize(verts_.size());
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::deallocateVert(Index id) {
    vertToCells_[id].clear();
    SuperClass::deallocateVert(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::allocateEdge(const EdgeContainer &edge) {
    Index id = SuperClass::allocateEdge(edge);
    if (edgeToCells_.size() < edges_.size()) {
        edgeToCells_.resize(edges_.size());
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::deallocateEdge(Index id) {
    edgeToCells_[id].clear();
    SuperClass::deallocateEdge(id);
}

template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::allocateFace(const FacePtr &face) {
    Index id = SuperClass::allocateFace(face);
    if (faceToCells_.size() < faces_.size()) {
        faceToCells_.resize(faces_.size());
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::deallocateFace(Index id) {
    faceToCells_[id].clear();
    SuperClass::deallocateFace(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::allocateCell(const CellPtr &cell) {
    Index id;
    if (freeCells_.empty()) {
        id = static_cast<Index>(cells_.size());
        cells_.push_back(cell);
        cellAttributes_.onResize(cells_.size());
    } else {
        id = *freeCells_.begin();
        freeCells_.erase(freeCells_.begin());
        cells_[id] = cell;
    }
    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::deallocateCell(Index id) {
    freeCells_.insert(id);
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::defragmentVertices() {
    std::vector<Index> newIndices(verts_.size(), kInvalidIndex);
    std::vector<VertexContainer> newVerts;
    std::vector<std::set<Index>> newVertToEdges, newVertToFaces, newVertToCells;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < verts_.size(); ++oldIndex) {
        if (!isVertRemoved(oldIndex)) {
            vertAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newVerts.push_back(verts_[oldIndex]);
            newVertToEdges.push_back(vertToEdges_[oldIndex]);
            newVertToFaces.push_back(vertToFaces_[oldIndex]);
            newVertToCells.push_back(vertToCells_[oldIndex]);
        }
    }
    vertAttributes_.onResize(newIndex);

    updateVertexReferencesAfterDefragment(newIndices, newVerts.size());

    verts_ = std::move(newVerts);
    vertToEdges_ = std::move(newVertToEdges);
    vertToFaces_ = std::move(newVertToFaces);
    vertToCells_ = std::move(newVertToCells);
    freeVerts_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::defragmentEdges() {
    std::vector<Index> newIndices(edges_.size(), kInvalidIndex);
    std::vector<EdgeContainer> newEdges;
    std::vector<std::set<Index>> newEdgeToFaces, newEdgeToCells;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < edges_.size(); ++oldIndex) {
        if (!isEdgeRemoved(oldIndex)) {
            edgeAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newEdges.push_back(edges_[oldIndex]);
            newEdgeToFaces.push_back(edgeToFaces_[oldIndex]);
            newEdgeToCells.push_back(edgeToCells_[oldIndex]);
        }
    }
    edgeAttributes_.onResize(newIndex);

    updateEdgeReferencesAfterDefragment(newIndices, newEdges.size());

    edges_ = std::move(newEdges);
    edgeToFaces_ = std::move(newEdgeToFaces);
    edgeToCells_ = std::move(newEdgeToCells);
    freeEdges_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::defragmentFaces() {
    std::vector<Index> newIndices(faces_.size(), kInvalidIndex);
    std::vector<FacePtr> newFaces;
    std::vector<std::set<Index>> newFaceToCells;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < faces_.size(); ++oldIndex) {
        if (!isFaceRemoved(oldIndex)) {
            faceAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newFaces.push_back(faces_[oldIndex]);
            newFaceToCells.push_back(faceToCells_[oldIndex]);
        }
    }
    faceAttributes_.onResize(newIndex);

    updateFaceReferencesAfterDefragment(newIndices, newFaces.size());

    faces_ = std::move(newFaces);
    faceToCells_ = std::move(newFaceToCells);
    freeFaces_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::defragmentCells() {
    std::vector<Index> newIndices(cells_.size(), kInvalidIndex);
    std::vector<CellPtr> newCells;

    Index newIndex = 0;
    for (Index oldIndex = 0; oldIndex < cells_.size(); ++oldIndex) {
        if (!isCellRemoved(oldIndex)) {
            cellAttributes_.onElementCopy(oldIndex, newIndex);
            newIndices[oldIndex] = newIndex++;
            newCells.push_back(cells_[oldIndex]);
        }
    }
    cellAttributes_.onResize(newIndex);

    updateCellReferencesAfterDefragment(newIndices);

    cells_ = std::move(newCells);
    freeCells_.clear();
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::updateVertexReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    SuperClass::updateReferencesAfterDefragment(newIndices, newSize);
    for (auto &cell : cells_) {
        if (cell) {
            for (size_t i = 0; i < cell->numVerts(); ++i) {
                Index oldIndex = cell->verts()[i];
                if (newIndices[oldIndex] != kInvalidIndex) {
                    cell->verts()[i] = newIndices[oldIndex];
                }
            }
        }
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::updateEdgeReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    SuperClass::updateEdgeReferencesAfterDefragment(newIndices, newSize);
    for (auto &cell : cells_) {
        if (cell) {
            for (size_t i = 0; i < cell->numEdges(); ++i) {
                Index oldIndex = cell->edges()[i];
                if (newIndices[oldIndex] != kInvalidIndex) {
                    cell->edges()[i] = newIndices[oldIndex];
                }
            }
        }
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::updateFaceReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    SuperClass::updateFaceReferencesAfterDefragment(newIndices, newSize);
    for (auto &cell : cells_) {
        if (cell) {
            for (size_t i = 0; i < cell->numFaces(); ++i) {
                Index oldIndex = cell->faces()[i];
                if (newIndices[oldIndex] != kInvalidIndex) {
                    cell->faces()[i] = newIndices[oldIndex];
                }
            }
        }
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
void VolumeMesh<Index, VertexContainer, AttributeName>::updateCellReferencesAfterDefragment(const std::vector<Index> &newIndices) {
    for (auto &cellSet : vertToCells_) {
        std::set<Index> newSet;
        for (Index oldIndex : cellSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        cellSet = std::move(newSet);
    }
    for (auto &cellSet : edgeToCells_) {
        std::set<Index> newSet;
        for (Index oldIndex : cellSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        cellSet = std::move(newSet);
    }
    for (auto &cellSet : faceToCells_) {
        std::set<Index> newSet;
        for (Index oldIndex : cellSet) {
            if (newIndices[oldIndex] != kInvalidIndex) {
                newSet.insert(newIndices[oldIndex]);
            }
        }
        cellSet = std::move(newSet);
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::findFirstValidCell() const {
    for (Index i = 0; i < cells_.size(); ++i) {
        if (!isCellRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
Index VolumeMesh<Index, VertexContainer, AttributeName>::findNextValidCell(Index current) const {
    for (Index i = current + 1; i < cells_.size(); ++i) {
        if (!isCellRemoved(i)) {
            return i;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
Index VolumeMesh<Index, VertexContainer, AttributeName>::addCell(const Index *verts, size_t numVerts, const Index *edges, size_t numEdges,
                                                                 const Index *faces, size_t numFaces) {
    if constexpr (CTYPE != CellType::CELL_POLYHEDRON) {
        constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
        assert(numVerts == desc.numVerts);
        assert(numEdges == 0 || numEdges == desc.numEdges);
        assert(numFaces == 0 || numFaces == desc.numFaces);
    } else {
        if (numVerts == 0 || numEdges == 0 || numFaces == 0) {
            throw std::runtime_error("Polyhedron requires complete vertex, edge and face indices");
        }
    }

    // Validate inputs
    for (size_t i = 0; i < numVerts; ++i) {
        Index vid = verts[i];
        assert(vid < verts_.size() && !isVertRemoved(vid));
    }
    for (size_t i = 0; i < numEdges; ++i) {
        Index eid = edges[i];
        assert(eid < edges_.size() && !isEdgeRemoved(eid));
    }
    for (size_t i = 0; i < numFaces; ++i) {
        Index fid = faces[i];
        assert(fid < faces_.size() && !isFaceRemoved(fid));
    }

    // Create cell
    CellPtr cell = createCell<CTYPE>(verts, numVerts, edges, numEdges, faces, numFaces);

    // Check for duplicate cell
    Index id = getCellIndex<CTYPE>(*cell);
    if (id != kInvalidIndex) {
        return id;
    }

    // Auto-generate topology if needed
    if (numEdges == 0) autoGenerateEdges<CTYPE>(cell);
    if (numFaces == 0) autoGenerateFaces<CTYPE>(cell);

    id = allocateCell(cell);

    // Update topology
    for (size_t i = 0; i < numVerts; ++i) {
        Index vid = verts[i];
        vertToCells_[vid].insert(id);
    }
    for (size_t i = 0; i < cell->numEdges(); ++i) {
        Index eid = cell->edges()[i];
        edgeToCells_[eid].insert(id);
    }
    for (size_t i = 0; i < cell->numFaces(); ++i) {
        Index fid = cell->faces()[i];
        faceToCells_[fid].insert(id);
    }

    return id;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
Index VolumeMesh<Index, VertexContainer, AttributeName>::getCellIndex(const Index *verts, size_t numVerts, const Index *edges,
                                                                      size_t numEdges, const Index *faces, size_t numFaces) const {
    if constexpr (CTYPE != CellType::CELL_POLYHEDRON) {
        constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
        assert(numVerts == desc.numVerts);
        assert(numEdges == 0 || numEdges == desc.numEdges);
        assert(numFaces == 0 || numFaces == desc.numFaces);
    } else {
        if (numVerts == 0 || numEdges == 0 || numFaces == 0) {
            throw std::runtime_error("Polyhedron requires complete vertex, edge and face indices");
        }
    }

    if constexpr (CTYPE == CellType::CELL_TET) {
        detail::Tetrahedron<Index> cell;
        cell.setVerts(verts);
        if (numEdges > 0) cell.setEdges(edges);
        if (numFaces > 0) cell.setFaces(faces);
        return getCellIndex<CTYPE>(cell);
    } else if constexpr (CTYPE == CellType::CELL_PYRAMID) {
        detail::Pyramid<Index> cell;
        cell.setVerts(verts);
        if (numEdges > 0) cell.setEdges(edges);
        if (numFaces > 0) cell.setFaces(faces);
        return getCellIndex<CTYPE>(cell);
    } else if constexpr (CTYPE == CellType::CELL_PRISM) {
        detail::Prism<Index> cell;
        cell.setVerts(verts);
        if (numEdges > 0) cell.setEdges(edges);
        if (numFaces > 0) cell.setFaces(faces);
        return getCellIndex<CTYPE>(cell);
    } else if constexpr (CTYPE == CellType::CELL_HEX) {
        detail::Hexahedron<Index> cell;
        cell.setVerts(verts);
        if (numEdges > 0) cell.setEdges(edges);
        if (numFaces > 0) cell.setFaces(faces);
        return getCellIndex<CTYPE>(cell);
    } else {
        detail::PolyhedronCell<Index> cell(numVerts, numEdges, numFaces);
        cell.setVerts(verts);
        if (numEdges > 0) cell.setEdges(edges);
        if (numFaces > 0) cell.setFaces(faces);
        return getCellIndex<CTYPE>(cell);
    }
}
/// @name Protected Helper Functions
/// @{
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
typename VolumeMesh<Index, VertexContainer, AttributeName>::CellPtr VolumeMesh<Index, VertexContainer, AttributeName>::createCell(
    const Index *verts, size_t numVerts, const Index *edges, size_t numEdges, const Index *faces, size_t numFaces) const {
    CellPtr cell;
    if constexpr (CTYPE == CellType::CELL_TET) {
        cell = std::make_shared<detail::Tetrahedron<Index>>();
    } else if constexpr (CTYPE == CellType::CELL_PYRAMID) {
        cell = std::make_shared<detail::Pyramid<Index>>();
    } else if constexpr (CTYPE == CellType::CELL_PRISM) {
        cell = std::make_shared<detail::Prism<Index>>();
    } else if constexpr (CTYPE == CellType::CELL_HEX) {
        cell = std::make_shared<detail::Hexahedron<Index>>();
    } else {
        cell = std::make_shared<detail::PolyhedronCell<Index>>(numVerts, numEdges, numFaces);
    }
    cell->setVerts(verts);
    if (numEdges > 0) {
        cell->setEdges(edges);
    }
    if (numFaces > 0) {
        cell->setFaces(faces);
    }
    return cell;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
Index VolumeMesh<Index, VertexContainer, AttributeName>::getCellIndex(const CellContainer &cell) const {
    for (Index cid : vertToCells_[cell.verts()[0]]) {
        if (*cells_[cid] == cell) {
            return cid;
        }
    }
    return kInvalidIndex;
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
void VolumeMesh<Index, VertexContainer, AttributeName>::autoGenerateEdges(CellPtr &cell) {
    if constexpr (CTYPE != CellType::CELL_POLYHEDRON) {
        constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
        std::array<Index, desc.numEdges> edges;
        for (size_t i = 0; i < desc.numEdges; ++i) {
            edges[i] = addEdge(cell->verts()[desc.edgeVert[i][0]], cell->verts()[desc.edgeVert[i][1]]);
        }
        cell->setEdges(edges.data());
    } else {
        throw std::runtime_error("Polyhedrons cannot generate edges automatically");
    }
}
template <typename Index, typename VertexContainer, typename AttributeName>
template <CellType CTYPE>
void VolumeMesh<Index, VertexContainer, AttributeName>::autoGenerateFaces(CellPtr &cell) {
    if constexpr (CTYPE != CellType::CELL_POLYHEDRON) {
        constexpr auto &desc = CellDescriptors::getDescriptor(CTYPE);
        std::array<Index, desc.numFaces> faces;
        std::array<Index, CellDescriptors::maxNumVertsInFace<CTYPE>()> faceVerts;

        for (size_t i = 0; i < desc.numFaces; ++i) {
            for (size_t j = 0; j < desc.numVertsInFace[i]; ++j) {
                faceVerts[j] = cell->verts()[desc.faceVert[i][j]];
            }

            if (desc.numVertsInFace[i] == 3) {
                faces[i] = addFace<FaceType::FACE_TRI>(faceVerts.data(), desc.numVertsInFace[i]);
            } else if (desc.numVertsInFace[i] == 4) {
                faces[i] = addFace<FaceType::FACE_QUAD>(faceVerts.data(), desc.numVertsInFace[i]);
            }
        }
        cell->setFaces(faces.data());
    } else {
        throw std::runtime_error("Polyhedrons cannot generate faces automatically");
    }
}

namespace io {
namespace detail {
/**
 * @brief 去除字符串左侧空白字符
 * @param str 要处理的字符串
 * @return 返回处理后的字符串引用
 */

inline std::string &ltrim(std::string &str) {
    str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](int ch) { return !std::isspace(ch); }));
    return str;
}
/**
 * @brief 去除字符串右侧空白字符
 * @param str 要处理的字符串
 * @return 返回处理后的字符串引用
 */
inline std::string &rtrim(std::string &str) {
    str.erase(std::find_if(str.rbegin(), str.rend(), [](int ch) { return !std::isspace(ch); }).base(), str.end());
    return str;
}
/**
 * @brief 去除字符串两侧空白字符
 * @param str 要处理的字符串
 * @return 返回处理后的字符串引用
 */
inline std::string &trim(std::string &str) { return ltrim(rtrim(str)); }
/**
 * @brief 返回去除两侧空白后的字符串副本
 * @param str 要处理的字符串
 * @return 返回处理后的新字符串
 */
inline std::string trimmed(const std::string &str) {
    std::string s(str);
    return trim(s);
}
/**
 * @brief 将字符串转换为小写
 * @param str 要转换的字符串
 * @return 返回转换后的新字符串
 */
inline std::string toLower(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
    return str;
}
/**
 * @brief 将字符串转换为大写
 * @param str 要转换的字符串
 * @return 返回转换后的新字符串
 */
inline std::string toUpper(std::string str) {
    std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::toupper(c); });
    return str;
}
/**
 * @brief 按指定分隔符分割字符串
 * @param str 要分割的字符串
 * @param delimiter 分隔字符
 * @return 返回分割后的字符串数组
 */
inline std::vector<std::string> split(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}
/**
 * @brief 按指定分隔字符串分割字符串
 * @param str 要分割的字符串
 * @param delimiter 分隔字符串
 * @return 返回分割后的字符串数组
 */
inline std::vector<std::string> split(const std::string &str, const std::string &delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = str.find(delimiter);

    while (end != std::string::npos) {
        tokens.push_back(str.substr(start, end - start));
        start = end + delimiter.length();
        end = str.find(delimiter, start);
    }

    tokens.push_back(str.substr(start));
    return tokens;
}
/**
 * @brief 将字符串数组用指定分隔符连接
 * @param strings 要连接的字符串数组
 * @param delimiter 连接分隔符
 * @return 返回连接后的字符串
 */
inline std::string join(const std::vector<std::string> &strings, const std::string &delimiter) {
    if (strings.empty()) return "";

    std::string result;
    for (size_t i = 0; i < strings.size() - 1; ++i) {
        result += strings[i] + delimiter;
    }
    result += strings.back();
    return result;
}
/**
 * @brief 检查字符串是否以指定前缀开头
 * @param str 要检查的字符串
 * @param prefix 前缀字符串
 * @return 如果以指定前缀开头返回true
 */
inline bool startsWith(const std::string &str, const std::string &prefix) {
    return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
}
/**
 * @brief 检查字符串是否以指定后缀结尾
 * @param str 要检查的字符串
 * @param suffix 后缀字符串
 * @return 如果以指定后缀结尾返回true
 */
inline bool endsWith(const std::string &str, const std::string &suffix) {
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}
};  // namespace detail

/**
 * @brief 从OBJ文件加载网格数据到表面网格对象
 * @tparam Index 顶点/面索引类型
 * @tparam VertexContainer 顶点容器类型
 * @tparam AttributeName 属性名称类型
 * @param filename 要加载的OBJ文件路径
 * @param m 要填充数据的表面网格对象
 */
template <typename Index, typename VertexContainer, typename AttributeName>
void LoadObj(const std::string &filename, SurfaceMesh<Index, VertexContainer, AttributeName> &m) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    VertexContainer vertex;
    std::vector<Index> face;

    try {
        while (std::getline(f, line)) {
            detail::trim(line);

            if (line.empty() || line.front() == '#') {
                continue;
            }

            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "v") {
                iss >> vertex[0] >> vertex[1] >> vertex[2];
                m.addVert(vertex);
            } else if (prefix == "f") {
                std::string faceData;
                while (iss >> faceData) {
                    size_t pos = faceData.find('/');
                    if (pos != std::string::npos) {
                        faceData = faceData.substr(0, pos);
                    }
                    Index idx = static_cast<Index>(std::stoi(faceData) - 1);
                    face.push_back(idx);
                }

                if (face.size() == 3) {
                    m.addFace<FaceType::FACE_TRI>(face.data(), face.size());
                } else if (face.size() == 4) {
                    m.addFace<FaceType::FACE_QUAD>(face.data(), face.size());
                } else {
                    m.addFace<FaceType::FACE_POLYGON>(face.data(), face.size());
                }
                face.clear();
            }
        }
    } catch (const std::exception &) {
        throw std::runtime_error("Failed to parse file: " + filename);
    }
}
/**
 * @brief 将表面网格对象保存为OBJ文件
 * @tparam Index 顶点/面索引类型
 * @tparam VertexContainer 顶点容器类型
 * @tparam AttributeName 属性名称类型
 * @param filename 要保存的OBJ文件路径
 * @param mesh 要保存的表面网格对象
 * @param precision 浮点数输出精度（默认为6位）
 */
template <typename Index, typename VertexContainer, typename AttributeName>
void SaveObj(const std::string &filename, const SurfaceMesh<Index, VertexContainer, AttributeName> &mesh, int precision = 6) {
    if (!mesh.isIndexContinuous()) {
        throw std::runtime_error("The index needs to be continuous. This problem can be solved by calling 'defragment'.");
    }
    std::ofstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    f << std::fixed << std::setprecision(precision);

    const size_t vertexCount = mesh.numVerts();
    for (size_t i = 0; i < vertexCount; ++i) {
        const auto &vertex = mesh.getVert(static_cast<Index>(i));
        f << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
    }
    const size_t faceCount = mesh.numFaces();
    for (size_t i = 0; i < faceCount; ++i) {
        const auto &face = mesh.getFace(static_cast<Index>(i));
        f << "f";
        const size_t vertexCountInFace = face.numVerts();
        for (size_t j = 0; j < vertexCountInFace; ++j) {
            f << " " << (face.verts()[j] + 1);
        }
        f << "\n";
    }
}
/**
 * @brief 从网格文件加载体网格数据（不包括多面体）
 * @tparam Index 顶点/单元索引类型
 * @tparam VertexContainer 顶点容器类型
 * @tparam AttributeName 属性名称类型
 * @param filename 要加载的网格文件路径
 * @param m 要填充数据的体网格对象
 */
template <typename Index, typename VertexContainer, typename AttributeName>
void LoadMesh(const std::string &filename, VolumeMesh<Index, VertexContainer, AttributeName> &m) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    std::string section;
    VertexContainer vertex;
    std::array<Index, 8> cell;
    try {
        while (std::getline(f, line)) {
            detail::trim(line);
            if (line.empty() || line.front() == '#') {
                continue;
            }
            if (line == "Vertices" || line == "Tetrahedra" || line == "Pyramids" || line == "Prisms" || line == "Hexahedra") {
                section = line;
                std::getline(f, line);
                int count = std::stoi(line);
                if (section == "Vertices") {
                    for (int i = 0; i < count; ++i) {
                        std::getline(f, line);
                        std::istringstream iss(line);
                        iss >> vertex[0] >> vertex[1] >> vertex[2];
                        m.addVert(vertex);
                    }
                } else if (section == "Tetrahedra") {
                    for (int i = 0; i < count; ++i) {
                        std::getline(f, line);
                        std::istringstream iss(line);
                        for (int j = 0; j < 4; ++j) {
                            iss >> cell[j];
                            cell[j]--;
                        }
                        m.addCell<CellType::CELL_TET>(cell.data(), 4);
                    }
                } else if (section == "Pyramids") {
                    for (int i = 0; i < count; ++i) {
                        std::getline(f, line);
                        std::istringstream iss(line);
                        for (int j = 0; j < 5; ++j) {
                            iss >> cell[j];
                            cell[j]--;
                        }
                        m.addCell<CellType::CELL_PYRAMID>(cell.data(), 5);
                    }
                } else if (section == "Prisms") {
                    for (int i = 0; i < count; ++i) {
                        std::getline(f, line);
                        std::istringstream iss(line);
                        for (int j = 0; j < 6; ++j) {
                            iss >> cell[j];
                            cell[j]--;
                        }
                        m.addCell<CellType::CELL_PRISM>(cell.data(), 6);
                    }
                } else if (section == "Hexahedra") {
                    for (int i = 0; i < count; ++i) {
                        std::getline(f, line);
                        std::istringstream iss(line);
                        for (int j = 0; j < 8; ++j) {
                            iss >> cell[j];
                            cell[j]--;
                        }
                        m.addCell<CellType::CELL_HEX>(cell.data(), 8);
                    }
                }
            }
        }
    } catch (...) {
        throw std::runtime_error("Failed to parse file: " + filename);
    }
}
/**
 * @brief 将体网格对象保存为网格文件（不包括多面体）
 * @tparam Index 顶点/单元索引类型
 * @tparam VertexContainer 顶点容器类型
 * @tparam AttributeName 属性名称类型
 * @param filename 要保存的网格文件路径
 * @param m 要保存的体网格对象
 * @param precision 浮点数输出精度（默认为6位）
 */
template <typename Index, typename VertexContainer, typename AttributeName>
void SaveMesh(const std::string &filename, const VolumeMesh<Index, VertexContainer, AttributeName> &m, int precision = 6) {
    if (!m.isIndexContinuous()) {
        throw std::runtime_error("The index needs to be continuous. This problem can be solved by calling 'defragment'.");
    }
    std::ofstream f(filename);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    f << std::fixed << std::setprecision(precision);

    f << "MeshVersionFormatted 1\n\n";
    f << "Dimension 3\n\n";
    f << "Vertices\n" << m.numVerts() << "\n";
    for (size_t i = 0; i < m.numVerts(); ++i) {
        const auto &vertex = m.getVert(static_cast<Index>(i));
        f << vertex[0] << " " << vertex[1] << " " << vertex[2] << " -1\n";
    }
    f << "\n";

    std::unordered_map<CellType, std::vector<Index>> cellIndices;
    for (size_t i = 0; i < m.numCells(); ++i) {
        cellIndices[m.getCell(static_cast<Index>(i)).type()].push_back(static_cast<Index>(i));
    }

    auto writeCells = [&](CellType type, const char *sectionName) {
        const auto &indices = cellIndices[type];
        if (indices.empty()) return;

        f << sectionName << "\n" << indices.size() << "\n";
        for (auto cellIdx : indices) {
            const auto &cell = m.getCell(cellIdx);
            for (int j = 0; j < cell.numVerts(); ++j) {
                f << (cell.verts()[j] + 1) << " ";
            }
            f << "1\n";
        }
        f << "\n";
    };

    writeCells(CellType::CELL_TET, "Tetrahedra");
    writeCells(CellType::CELL_PYRAMID, "Pyramids");
    writeCells(CellType::CELL_PRISM, "Prisms");
    writeCells(CellType::CELL_HEX, "Hexahedra");
}
}  // namespace io
};  // namespace hmesh