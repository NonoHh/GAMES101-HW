//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "omp.h"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
const bool ANTIALIASING = true;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);


    omp_set_num_threads(8);
    omp_lock_t mylock;
    omp_init_lock(&mylock);

    // change the spp value to change sample ammount
    int spp = 16;
    int buffer_index = 0;
    int percent_counter = 0;
    const int kSumResolution = scene.height * scene.width * spp;
    std::cout << "SPP: " << spp << "\n";

#pragma omp parallel for
    for (int j = 0; j < scene.height; ++j) {

#pragma omp parallel for
        for (int i = 0; i < scene.width; ++i) {
            if (ANTIALIASING) {

#pragma omp parallel for
                for (int k = 0; k < spp; k++) {
                    // generate primary ray direction
                    float x = (2 * (i + get_random_float()) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                    float y = (1 - 2 * (j + get_random_float()) / (float)scene.height) * scale;
                    Vector3f dir = normalize(Vector3f(-x, y, 1));

                    // cast ray
                    framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;

#pragma omp atomic
                    percent_counter++;
                }
            }
            else {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                    imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
                Vector3f dir = normalize(Vector3f(-x, y, 1));

#pragma omp parallel for
                for (int k = 0; k < spp; k++) {
                    // cast ray
                    framebuffer[buffer_index] += scene.castRay(Ray(eye_pos, dir), 0) / spp;

#pragma omp atomic
                    percent_counter++;
                }
                buffer_index++;
            }
        }
        omp_set_lock(&mylock);
        UpdateProgress(percent_counter * 1.0 / kSumResolution);
        omp_unset_lock(&mylock);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
