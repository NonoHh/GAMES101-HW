#pragma once
#include <iostream>
#include <cmath>
#include <random>

#undef M_PI
#define M_PI 3.141592653589793f

extern const float  EPSILON;
const float kInfinity = std::numeric_limits<float>::max();

inline float clamp(const float &lo, const float &hi, const float &v)
{ return std::max(lo, std::min(hi, v)); }

inline  bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ?
                  -0.5 * (b + sqrt(discr)) :
                  -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1) std::swap(x0, x1);
    return true;
}

inline float get_random_float()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

    return dist(rng);
}

inline void UpdateProgress(float progress)
{
    int barWidth = 100;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r"; 
    std::cout.flush();
}

// BSDF Inline Functions
inline float CosTheta(const Vector3f& w) { return w.z; }
inline float Cos2Theta(const Vector3f& w) { return w.z * w.z; }
inline float AbsCosTheta(const Vector3f& w) { return std::abs(w.z); }
inline float Sin2Theta(const Vector3f& w) {
    return std::max(0.0, 1.0 - Cos2Theta(w));
}

inline float SinTheta(const Vector3f& w) { return std::sqrt(Sin2Theta(w)); }

inline float TanTheta(const Vector3f& w) { return SinTheta(w) / CosTheta(w); }

inline float Tan2Theta(const Vector3f& w) {
    return Sin2Theta(w) / Cos2Theta(w);
}

inline float CosPhi(const Vector3f& w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 1 : clamp(w.x / sinTheta, -1, 1);
}

inline float SinPhi(const Vector3f& w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 0 : clamp(w.y / sinTheta, -1, 1);
}

inline float Cos2Phi(const Vector3f& w) { return CosPhi(w) * CosPhi(w); }

inline float Sin2Phi(const Vector3f& w) { return SinPhi(w) * SinPhi(w); }

inline float CosDPhi(const Vector3f& wa, const Vector3f& wb) {
    float waxy = wa.x * wa.x + wa.y * wa.y;
    float wbxy = wb.x * wb.x + wb.y * wb.y;
    if (waxy == 0 || wbxy == 0)
        return 1;
    return clamp((wa.x * wb.x + wa.y * wb.y) / std::sqrt(waxy * wbxy), -1, 1);
}
