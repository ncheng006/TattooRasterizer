#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {  // Estimate the lighting from this intersection coming directly from a light.
    // For this function, sample uniformly in a hemisphere.

    // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
    // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);

    // This is the same number of total samples as
    // estimate_direct_lighting_importance (outside of delta lights). We keep the
    // same number of samples for clarity of comparison.
    int num_samples = scene->lights.size() * ns_area_light;
    Vector3D L_out;

    // TODO (Part 3): Write your sampling loop here
    // TODO BEFORE YOU BEGIN
    // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading

    double pdf;
    Vector3D sample;
    for (int i = 0; i < num_samples; i++) {
        isect.bsdf->sample_f(w_out, &sample, &pdf);
        Ray sample_r = Ray(hit_p, o2w * sample);
        sample_r.min_t = EPS_F;
        Intersection inter;
        if (bvh->intersect(sample_r, &inter)) {
            auto L = inter.bsdf->get_emission();
            L_out += L * isect.bsdf->f(w_out, sample) * cos_theta(sample)/pdf;
        }
    }
    return L_out /num_samples;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);
    Vector3D L_ret(0,0,0);
    for (SceneLight* light : scene->lights) {
        Vector3D L_out(0, 0, 0);
        int num_samples = ns_area_light;
        if (light->is_delta_light()) {
            num_samples = 1;
        }
        for (int i = 0; i < num_samples; i++) {
            Vector3D wi;
            double distToLight, pdf;

            Vector3D emitted_radiance = light->sample_L(hit_p, &wi, &distToLight, &pdf);
            Vector3D w_in = w2o * wi;

            if (w_in.z >= 0) {
                Ray ra = Ray(hit_p, wi);
                ra.max_t = distToLight - EPS_F;
                ra.min_t = EPS_D;

                Intersection is;
                if (!bvh->intersect(ra, &is)) {
                    auto bsdf = isect.bsdf->f(w_out, w_in);
                    auto cos = cos_theta(w_in);
                    L_out += bsdf * emitted_radiance * cos / pdf;
                }
            }
        }
        L_ret += L_out / num_samples;
    }
    return L_ret;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

    return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
      return estimate_direct_lighting_hemisphere(r, isect);
  } else {
      return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D w_out = w2o * (-r.d);


    // TODO: Part 4, Task 2
    // Returns the one bounce radiance + radiance from extra bounces at this point.
    // Should be called recursively to simulate extra bounces.

    //The function at_least_one_bounce_radiance is the main implementation work for this Part 4. At a high level,
    // it should call the one_bounce_radiance function, and then recursively call itself to estimate the higher bounces.
    // This recursive call should take one random sample of a direction based on the BSDF at the hit point, trace a ray
    // in that sample direction, and recursively call itself on the new hit point.
    Vector3D L_out(0,0,0);

    Vector3D wi;
    double pdf;
    L_out = one_bounce_radiance(r, isect);

    Vector3D samples = isect.bsdf->sample_f(w_out, &wi, &pdf);
    Vector3D w_in = o2w * wi;

    auto max_depth = this->max_ray_depth;
    if (r.depth > 1 && (r.depth == max_depth || coin_flip(.65))) {
        Ray shadow_ray = Ray(hit_p, w_in);
        shadow_ray.min_t = EPS_F;
        Intersection is;
        shadow_ray.depth = r.depth - 1;
        if (!bvh->intersect(shadow_ray, &is)) {
            return L_out;
        }
        L_out += at_least_one_bounce_radiance(shadow_ray, is) * samples * cos_theta(wi) / (pdf * (.65));
    }
    return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
    Intersection isect;
    Vector3D L_out;

    // You will extend this in assignment 3-2.
    // If no intersection occurs, we simply return black.
    // This changes if you implement hemispherical lighting for extra credit.

    // The following line of code returns a debug color depending
    // on whether ray intersection with triangles or spheres has
    // been implemented.
    //
    // REMOVE THIS LINE when you are ready to begin Part 3.

    if (!bvh->intersect(r, &isect))
        return envLight ? envLight->sample_dir(r) : L_out;


    L_out = zero_bounce_radiance(r, isect);

    // TODO (Part 3): Return the direct illumination.

    // TODO (Part 4): Accumulate the "direct" and "indirect"
    // parts of global illumination into L_out rather than just direct

    return zero_bounce_radiance(r,isect) + at_least_one_bounce_radiance(r, isect);
//    L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
//    return L_out;

}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

    int num_samples = ns_aa;          // total samples to evaluate
    Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
    Vector3D res = Vector3D(0, 0, 0);
    float s1 = 0.0;
    float s2 = 0.0;
    int i = 0;
    int actualNumSamples = 0;

    for (i = 0; i < num_samples; i++) {
        Vector2D s = gridSampler->get_sample();
        Ray r = camera->generate_ray((float)(x + s.x)/(float)sampleBuffer.w, (float)(y + s.y)/(float)sampleBuffer.h);
        r.depth = max_ray_depth;
        Vector3D radiance = est_radiance_global_illumination(r);
        res += radiance;
        float illum = radiance.illum();
        s1 += illum;
        s2 += illum * illum;
        actualNumSamples++;

        if (i % samplesPerBatch == 0 && i > 0) {
            float mean = s1 / float(i + 1);
            float variance = (1.0 / i) * (s2 - ((s1 * s1) / float(i + 1)));
            float I = 1.96 * sqrt(variance) / sqrt(float(i + 1));
            if (I <= this->maxTolerance * mean) {
                break;
            }
        }
    }

    res = res/actualNumSamples;
    //Find the closest color to darker green, red, yellow, or black to result and set result to that color
    double rgb_avg = (res.x + res.y + res.z)/3;
    Vector3D grayscale_color = Vector3D(rgb_avg, rgb_avg, rgb_avg);

//      float probability = rgb_avg + 0.035; // never want complete opaqueness
//      if (probability < 0) {probability = 0;}
//      if (probability > 1) {probability = 1;}
//      // Make random generator
//      vector<int> vals(50, 0);
//      for (int j = 0; j < probability * 50; j++) {
//        vals[j] = 1;
//      }
//
//    int random = rand() % 10; // random value between 0-9
//    if (!vals[random]) {
//      grayscale_color = Vector3D(0,0,0); // black
//    } else {
//      grayscale_color = Vector3D(1,1,1); // white
//    }
    res = grayscale_color;

//    Vector3D green = Vector3D(0.19, 0.84, 0.78);
//    Vector3D red = Vector3D(0.15, 0.0, 0.15);
//    Vector3D yellow = Vector3D(.65, .35, 0.0);
//    Vector3D black = Vector3D(0.0, 0.0, 0.2);
//    Vector3D white = Vector3D(1.0, 1.0, 1.0);
//    Vector3D colors[5] = {green, red, yellow,black,white};
//    float min = 10000;
//    int minIndex = 0;
//    for (int i = 0; i < 4; i++) {
//        float dist = sqrt(pow(res.x - colors[i].x, 2) + pow(res.y - colors[i].y, 2) + pow(res.z - colors[i].z, 2));
//        if (dist < min) {
//            min = dist;
//            minIndex = i;
//        }
//    }
//
//    res = colors[minIndex];
    sampleBuffer.update_pixel(res, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
