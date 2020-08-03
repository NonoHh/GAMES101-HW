//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"

enum MaterialType { DIFFUSE};

class Material{

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior = 2;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;
    Vector2f roughness;
    float alphax, alphay;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0),Vector2f r = Vector2f(1,1));
    inline Material(float i, MaterialType t = DIFFUSE, Vector3f e = Vector3f(0, 0, 0), Vector2f r = Vector2f(1, 1));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

    inline float GGX_D(const Vector3f& wh);

    inline float RoughnessToAlpha(float roughness);
    inline float Lambda(const Vector3f& w);
    inline float Smith_G(const Vector3f& wo, const Vector3f& wi);
};

Material::Material(MaterialType t, Vector3f e,Vector2f r){
    m_type = t;
    //m_color = c;
    m_emission = e;
    roughness = r;
    alphax = RoughnessToAlpha(r.x);
    alphay = RoughnessToAlpha(r.y);
}

Material::Material(float i, MaterialType t, Vector3f e, Vector2f r) {
    ior = i;
    m_type = t;
    //m_color = c;
    m_emission = e;
    roughness = r;
    alphax = RoughnessToAlpha(r.x);
    alphay = RoughnessToAlpha(r.y);
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

inline float Material::RoughnessToAlpha(float roughness) {
    roughness = std::max(roughness, (float)1e-3);
    float x = std::log(roughness);
    return 1.62142f + 0.819955f * x + 0.1734f * x * x + 0.0171201f * x * x * x +
        0.000640711f * x * x * x * x;
}

inline float Material::GGX_D(const Vector3f& wh) {
    float tan2Theta = Tan2Theta(wh);
    if (std::isinf(tan2Theta)) return 0.;
    const float cos4Theta = Cos2Theta(wh) * Cos2Theta(wh);
    float e = (Cos2Phi(wh) / (alphax * alphax) +
        Sin2Phi(wh) / (alphay * alphay)) * tan2Theta;
    return 1 / (M_PI * alphax * alphay * cos4Theta * (1 + e) * (1 + e));
}

inline float Material::Lambda(const Vector3f& w) {
    float absTanTheta = std::abs(TanTheta(w));
    if (std::isinf(absTanTheta)) return 0.;
    float alpha = std::sqrt(Cos2Phi(w) * alphax * alphax + Sin2Phi(w) * alphay * alphay);
    float alpha2Tan2Theta = (alpha * absTanTheta) * (alpha * absTanTheta);
    return (-1 + std::sqrt(1.f + alpha2Tan2Theta)) / 2;
}

inline float Material::Smith_G(const Vector3f& wo, const Vector3f& wi) {
    return 1 / (1 + Lambda(wo) + Lambda(wi));
}

Vector3f Material::eval(const Vector3f& wi, const Vector3f& wo, const Vector3f& N) {
    switch (m_type) {
    case DIFFUSE:
    {
        // calculate the contribution of diffuse   model
        Vector3f diffuse = Vector3f(0);
        float cosalpha = dotProduct(N, wo);
        if (cosalpha > 0.0f) {
            diffuse = Kd / M_PI;
        }

        float cosThetaO = AbsCosTheta(wo), cosThetaI = AbsCosTheta(wi);
        if (cosThetaI == 0 || cosThetaO == 0) {
            return diffuse;
        }
        auto wh = normalize(wi + wo);
        if (wh.x == 0 && wh.y == 0 && wh.z == 0) {
            return diffuse;
        }
        float kr = 0;
        fresnel(wi, N, ior, kr);
        auto D = GGX_D(wh);
        auto G = Smith_G(wo, wi);

        return Vector3f(D * G * kr / (4 * dotProduct(N, wi) * dotProduct(N, wo))) + diffuse;
    }
    }
}

#endif //RAYTRACING_MATERIAL_H
