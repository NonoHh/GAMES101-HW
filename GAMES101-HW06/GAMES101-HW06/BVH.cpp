#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Primitive*> p, int maxPrimsInNode,
                   SplitMethod splitMethod) :
    maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
    primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty()) return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Primitive*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds = Union(centroidBounds,
                               objects[i]->getBounds().Centroid());
    int dim = centroidBounds.maxExtent();
    switch (dim) {
    case 0:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
        {
            return f1->getBounds().Centroid().x < f2->getBounds().Centroid().x;
        });
        break;
    case 1:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
        {
            return f1->getBounds().Centroid().y < f2->getBounds().Centroid().y;
        });
        break;
    case 2:
        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
        {
            return f1->getBounds().Centroid().z < f2->getBounds().Centroid().z;
        });
        break;
    }

    switch (splitMethod) {
    case SplitMethod::Middle: {
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Primitive*>(beginning, middling);
        auto rightshapes = std::vector<Primitive*>(middling, ending);
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);

        break;
    }
    case SplitMethod::SAH: default: {
        std::cout << "here" << std::endl;
        auto nPrimitives = objects.size();

        if (nPrimitives <= 2) {
            node->left = recursiveBuild(std::vector{objects[0]});
            node->right = recursiveBuild(std::vector{objects[1]});

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else {
            // Allocate _BucketInfo_ for SAH partition buckets
            const int nBuckets = 12;
            BucketInfo buckets[nBuckets];

            // Initialize _BucketInfo_ for SAH partition buckets
            for (int i = 0; i < nPrimitives; i++) {
                int b = nBuckets * centroidBounds.Offset(
                            objects.at(i)->getBounds().Centroid())[dim];
                if (b == nBuckets) {
                    b = nBuckets - 1;
                }
                assert(b >= 0 && b < nBuckets);
                buckets[b].count++;
                buckets[b].bounds = Union(buckets[b].bounds, objects.at(i)->getBounds().Centroid());
            }

            // Compute costs for splitting after each bucket
            float cost[nBuckets - 1];
            for (int i = 0; i < nBuckets - 1; ++i) {
                Bounds3 b0, b1;
                int count0 = 0, count1 = 0;
                for (int j = 0; j <= i; ++j) {
                    b0 = Union(b0, buckets[j].bounds);
                    count0 += buckets[j].count;
                }
                for (int j = i + 1; j < nBuckets; ++j) {
                    b1 = Union(b1, buckets[j].bounds);
                    count1 += buckets[j].count;
                }
                cost[i] = 1 + (count0 * b0.SurfaceArea() + count1 * b1.
                               SurfaceArea()) / bounds.SurfaceArea();
            }

            // Find bucket to split at that minimizes SAH metric
            float minCost = cost[0];
            int minCostSplitBucket = 0;
            for (int i = 1; i < nBuckets - 1; ++i) {
                if (cost[i] < minCost) {
                    minCost = cost[i];
                    minCostSplitBucket = i;
                }
            }

            // Either create leaf or split primitives at selected SAH
            // bucket
            float leafCost = nPrimitives;
            std::vector<Primitive*> leftshapes, rightshapes;
            if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
                for (int i = 0; i < objects.size(); i++) {
                    int b = nBuckets * centroidBounds.Offset(
                                objects.at(i)->getBounds().Centroid())[dim];
                    if (b == nBuckets) {
                        b = nBuckets - 1;
                    }
                    assert(b >= 0 && b < nBuckets);
                    if (b <= minCostSplitBucket) {
                        leftshapes.push_back(objects.at(i));
                    }else {
                        rightshapes.push_back(objects.at(i));
                    }
                }
                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);

                node->bounds = Union(node->left->bounds, node->right->bounds);
            }
            else {
                node->bounds = objects[0]->getBounds();
                node->object = objects[0];
                node->left = nullptr;
                node->right = nullptr;
            }
        }
        break;
    }
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root) return isect;
    isect = getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if (!node->bounds.IntersectP(ray)) {
        return Intersection();
    }
    if (node->isLeaf()) {
        return node->object->getIntersection(ray);
    }
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);
    return hit1.distance < hit2.distance ? hit1 : hit2;
}
