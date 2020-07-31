//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Primitive.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;

// BVHAccel Local Declarations
struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3& bounds)
        : primitiveNumber(primitiveNumber),
        bounds(bounds),
        centroid(.5f * bounds.pMin + .5f * bounds.pMax) {}
    size_t primitiveNumber;
    Bounds3 bounds;
    Vector3f centroid;
};

struct BucketInfo {
    int count = 0;
    Bounds3 bounds;
};

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;

class BVHAccel {

public:
    // BVHAccel Public Types
    enum class SplitMethod { Middle, SAH };

    // BVHAccel Public Methods
    // BVHAccel(std::vector<Primitive*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::Middle);
    BVHAccel(std::vector<Primitive*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::SAH);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode* root;

    // BVHAccel Private Methods
    BVHBuildNode* recursiveBuild(std::vector<Primitive*>objects);
    BVHBuildNode* recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start,
        int end, int* totalNodes, std::vector<Primitive*>& orderedPrims);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Primitive*> primitives;
};

struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Primitive* object;
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;

    // BVHBuildNode Public Methods
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }

    void setAsLeaf(int first, int n, const Bounds3& b, Primitive* o) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
        left = nullptr; right = nullptr;
        leafNodes++;
        totalLeafNodes++;
        totalPrimitives += n;
        object = o;
    }

    void setAsInterior(int axis, BVHBuildNode* c0, BVHBuildNode* c1) {
        left = c0;
        right = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
        interiorNodes++;
    }

    bool isLeaf() {
        return left == nullptr && right == nullptr;
    }
};




#endif //RAYTRACING_BVH_H
