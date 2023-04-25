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
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

//   make a coordinate system for a hit point
//   with N aligned with the Z direction.
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

//   Loop through all samples
  for (int i = 0; i < num_samples; i++) {
    // Sample hemisphere
    Vector3D sample = hemisphereSampler->get_sample();
    // Translate to world space
    Vector3D d = o2w * sample;
    // Determine direction

    Ray sampleRay = Ray(hit_p, d);
    sampleRay.min_t = EPS_F;
    Intersection intersection;

    // Figure out if the intersection exists
    if (bvh->intersect(sampleRay, &intersection)) {
      Vector3D e = intersection.bsdf->get_emission();
      Vector3D s = e * isect.bsdf->f(w_out, sample) * 2 * PI * cos_theta(sample);
      L_out += s;
    }
  }
  return L_out/ num_samples;
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

//   w_out points towards the source of the ray (e.g.,
//   toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;
  Vector3D result(0,0,0);

  // Loop through all the light sources
  for (auto light : this->scene->lights) {
    Vector3D wi;
    double pdf, distance = 0;

    int num_samples = ns_area_light;
    if (light->is_delta_light()) {
      num_samples = 1;
    }

    for (int i = 0; i < num_samples; i++) {
      Vector3D rad = light->sample_L(hit_p, &wi, &distance, &pdf);
      Vector3D w_in = w2o * wi;

      if (w_in.z >= 0) {
        Intersection i;
        Ray r = Ray(hit_p, wi);
        // Set min and max
        r.min_t = EPS_D;
        r.max_t = distance - EPS_F;
        if (!bvh->intersect(r, &i)) {
          L_out += (rad * isect.bsdf->f(w_out, w_in) * cos_theta(w_in))/pdf;
        }

      }

    }
    L_out = L_out/num_samples;
  }

  return L_out;
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
  }
  return estimate_direct_lighting_importance(r, isect);
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  if (r.depth != this->max_ray_depth) {
    L_out = one_bounce_radiance(r, isect);
  }

  double pdf;
  double rrprob = 0.35; // Termination probability for Russian Roulette
  Vector3D w_in;
  Vector3D sample = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  
  bool check2 = r.depth > 1 && (r.depth == max_ray_depth || coin_flip(1-rrprob));

  // If max_ray_depth > 1, then there will always be at least one indirect bounce
  if (check2) {
    Vector3D direction = o2w * w_in;
    Ray r = Ray(hit_p, direction);
    // Update ray
    r.min_t = EPS_F;
    r.depth -= 1;
    Intersection i;
    // Check for intersection
    if (bvh->intersect(r, &i) && r.depth != max_ray_depth) {
      // Recursive step
      L_out += sample *  at_least_one_bounce_radiance(r, i) * abs(cos_theta(w_in)) / pdf / (1-rrprob);
    }
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


//  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
//  L_out = estimate_direct_lighting_importance(r, isect) + zero_bounce_radiance(r, isect) ;
  L_out = estimate_direct_lighting_hemisphere(r, isect) + zero_bounce_radiance(r, isect) ;
  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
//  L_out = zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
//  L_out = at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y, vector<Vector3D>& data, int numCols) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.
  
  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  
  // TODO: the following is the part5, modified code
  
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
  Vector3D average = Vector3D(0, 0, 0);
  double s1, s2 = 0;
  int actualNumSamples = 0;

  // Generate num_sample camera rays
  for (int i = 0; i < num_samples; i++) {
    Vector2D sample = gridSampler->get_sample();
    // Generate ray
    Ray currRay = camera->generate_ray((x + sample.x)/sampleBuffer.w, (y + sample.y)/sampleBuffer.h);
    currRay.depth = max_ray_depth;
    Vector3D res = PathTracer::est_radiance_global_illumination(currRay);
    average += res;
    actualNumSamples++;

    // Update s1 and s2 for adaptive sampling
    s1 += res.illum();
    s2 += res.illum() * res.illum();

    // Check for pixel convergence
    if (i != 0 && i % samplesPerBatch == 0) {
      double mean = s1/actualNumSamples;
      double variance = (s2 - (s1 * s1)/actualNumSamples) / (actualNumSamples-1);
      // If we have reached convergence, break from loop
      double i_var = 1.96 * sqrt(variance) / sqrt(actualNumSamples);
      if (i_var <= maxTolerance * mean) {
        break;
      }
    }
  }
  average = average/actualNumSamples;
  
  // Convert to grayscale
  double rgb_avg = (average.x + average.y + average.z)/3;
  Vector3D grayscale_color = Vector3D(rgb_avg, rgb_avg, rgb_avg);
  
    float probability = rgb_avg + 0.035; // never want complete opaqueness
    if (probability < 0) {probability = 0;}
    if (probability > 1) {probability = 1;}
    // Make random generator
    vector<int> vals(50, 0);
    for (int i = 0; i < probability * 50; i++) {
      vals[i] = 1;
    }
  
  int random = rand() % 10; // random value between 0-9
  if (!vals[random]) {
    grayscale_color = Vector3D(0,0,0); // black
  } else {
    grayscale_color = Vector3D(1,1,1); // white
  }

  sampleBuffer.update_pixel(grayscale_color, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = actualNumSamples;
//  data[x * numCols + y] = grayscale_color;
//  data.push_back(grayscale_color);
//  cout << grayscale_color << "  ";
}

//void PathTracer::update_pixel(Vector3D color, int x, int y) {
//  sampleBuffer.update_pixel(color, x, y);
//}
void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
