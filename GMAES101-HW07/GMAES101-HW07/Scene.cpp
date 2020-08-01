//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
    // judge whether this ray (from eye to p) can hit something (light or object)
    auto eye_inter = intersect(ray);

    // hit the light, return white color
    if (eye_inter.emit.norm() > 0) {
        return Vector3f(1);
    }

    // hit object
    auto light = Vector3f(0);
    if (eye_inter.happened)
    {
        // wo's direction is different with which in course
        const auto wo = normalize(-ray.direction);
        const auto p = eye_inter.coords;
        const auto normal = normalize(eye_inter.normal);

        // sample a light
        float pdf_light;
        Intersection inter;
        sampleLight(inter, pdf_light);
        const auto x = inter.coords;
        const auto ws = normalize(x - p);

        // if the sample light is not blocked
        if ((intersect(Ray(p, ws)).coords - x).norm() < 0.01)
        {
            const auto normal_prime = normalize(inter.normal);
            light += inter.emit * eye_inter.m->eval(wo, ws, normal) * dotProduct(ws, normal) * dotProduct(-ws, normal_prime) / ((x - p).norm() * (x - p).norm() * pdf_light);
        }

        //indirect light
        if (get_random_float() < RussianRoulette)
        {
            const auto wi = eye_inter.m->sample(wo, normal);
            light += castRay(Ray(p, wi), depth) * eye_inter.m->eval(wi, wo, normal) * dotProduct(wi, normal) / (eye_inter.m->pdf(wi, wo, normal) * RussianRoulette);
        }
    }
    return light;
}