#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>

#include "application/visual_debugger.h"

using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

    Vector3D MirrorBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D MirrorBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

        // TODO Project 3-2: Part 1
        // Implement MirrorBSDF
        return Vector3D();
    }

    void MirrorBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Mirror BSDF"))
        {
            DragDouble3("Reflectance", &reflectance[0], 0.005);
            ImGui::TreePop();
        }
    }

// Microfacet BSDF //

    double MicrofacetBSDF::G(const Vector3D wo, const Vector3D wi) {
        return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
    }

    double MicrofacetBSDF::D(const Vector3D h) {
        // TODO Project 3-2: Part 2
        // Compute Beckmann normal distribution function (NDF) here.
        // You will need the roughness alpha.

        double theta = getTheta(h);
        double exponent = (-tan(theta) * tan(theta)) / (alpha * alpha);
        double numerator = exp(exponent);
        double denominator = PI * alpha * alpha * pow(cos(theta), 4);


        return numerator / denominator;
    }

    Vector3D MicrofacetBSDF::F(const Vector3D wi) {
        // TODO Project 3-2: Part 2
        // Compute Fresnel term for reflection on dielectric-conductor interface.
        // You will need both eta and etaK, both of which are Vector3D.
        double costheta = cos(getTheta(wi));
        Vector3D eta_2_k_2 = eta * eta + k * k;

        Vector3D rs = (eta_2_k_2 - 2 * eta * costheta + pow(costheta, 2)) / (eta_2_k_2 + 2 * eta * costheta +  pow(costheta, 2));

        Vector3D rp = (eta_2_k_2 *  pow(costheta, 2) - 2 * eta * costheta + 1) / (eta_2_k_2 *  pow(costheta, 2) + 2 * eta * costheta + 1);

        return (rs + rp)/2;
    
    }

    Vector3D MicrofacetBSDF::f(const Vector3D wo, const Vector3D wi) {
        // TODO Project 3-2: Part 2
        // Implement microfacet model here.
      
        Vector3D n(0,0,1); // macro surface normal
        Vector3D h = wi + wo; // the half vector
        h.normalize();
        
        if (dot(n, wi) < 0 || dot(n, wo) < 0) {
          return Vector3D();
        }

        Vector3D numerator = F(wi) * G(wo,wi) * D(h);
        Vector3D denominator = 4 * dot(n, wo) * dot(n, wi);

        return numerator / denominator;

    }

    Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
        // TODO Project 3-2: Part 2
        // *Importance* sample Beckmann normal distribution function (NDF) here.
        // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
        //       and return the sampled BRDF value.
        
        // Generate random samples
        Vector2D sample = sampler.get_sample();
        double theta = atan(sqrt(-pow(alpha, 2) * log(1-sample.x)));
        double phi = 2 * PI * sample.y;

        // Sampled microfacet normal
        Vector3D h = Vector3D(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
        // Reflect wo according to h gives us wi
        *wi = -wo + 2 * dot(wo, h) * h;
        wi->normalize();

        // Check wi
        if (wi->z < 0) {
          pdf = 0;
          return Vector3D();
        }

        // pdf using beckmann NDF
        double pdf_theta = 2 * sin(theta) * exp(-pow(tan(theta),2)/pow(alpha,2)) / pow(alpha, 2) / pow(cos(theta), 3);
        double pdf_phi = 1 / (2 * PI);

        double pdf_h = pdf_theta * pdf_phi / sin(theta);
        *pdf = pdf_h / (4 * dot(*wi, h));

        // Default cosine hemisphere sampling code
//        *wi = cosineHemisphereSampler.get_sample(pdf);
  
        return MicrofacetBSDF::f(wo, *wi);
    }

    void MicrofacetBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Micofacet BSDF"))
        {
            DragDouble3("eta", &eta[0], 0.005);
            DragDouble3("K", &k[0], 0.005);
            DragDouble("alpha", &alpha, 0.005);
            ImGui::TreePop();
        }
    }

// Refraction BSDF //

    Vector3D RefractionBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D RefractionBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
        // TODO Project 3-2: Part 1
        // Implement RefractionBSDF
        return Vector3D();
    }

    void RefractionBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Refraction BSDF"))
        {
            DragDouble3("Transmittance", &transmittance[0], 0.005);
            DragDouble("ior", &ior, 0.005);
            ImGui::TreePop();
        }
    }

// Glass BSDF //

    Vector3D GlassBSDF::f(const Vector3D wo, const Vector3D wi) {
        return Vector3D();
    }

    Vector3D GlassBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {

        // TODO Project 3-2: Part 1
        // Compute Fresnel coefficient and either reflect or refract based on it.

        // compute Fresnel coefficient and use it as the probability of reflection
        // - Fundamentals of Computer Graphics page 305
        return Vector3D();
    }

    void GlassBSDF::render_debugger_node()
    {
        if (ImGui::TreeNode(this, "Refraction BSDF"))
        {
            DragDouble3("Reflectance", &reflectance[0], 0.005);
            DragDouble3("Transmittance", &transmittance[0], 0.005);
            DragDouble("ior", &ior, 0.005);
            ImGui::TreePop();
        }
    }

    void BSDF::reflect(const Vector3D wo, Vector3D* wi) {

        // TODO Project 3-2: Part 1
        // Implement reflection of wo about normal (0,0,1) and store result in wi.
        *wi = -wo + 2 * (wo * Vector3D(0, 0, 1)) * Vector3D(0, 0, 1);

    }

    bool BSDF::refract(const Vector3D wo, Vector3D* wi, double ior) {

        // TODO Project 3-2: Part 1
        // Use Snell's Law to refract wo surface and store result ray in wi.
        // Return false if refraction does not occur due to total internal reflection
        // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
        // ray entering the surface through vacuum.

        return true;

    }

} // namespace CGL
