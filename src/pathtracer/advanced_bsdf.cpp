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
        
        // Set pdf to 1 to handle delta BSDF
        *pdf = 1;
        
        // Store reflection of wo about normal in wi
        reflect(wo, wi);
        return reflectance / abs_cos_theta(*wi);
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
      
        double a = 0.05; // roughness alpha

        double theta = getTheta(h);
        double exponent = (-tan(theta) * tan(theta)) / (a * a);
        double numerator = exp(exponent);
        double denominator = PI * a * a * pow(cos(theta), 4);
    

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

        Vector3D numerator = F(wi) * G(wo,wi) * D(h);
        Vector3D denominator = 4 * dot(n, wo) * dot(n, wi);

        return numerator / denominator;

    }

    Vector3D MicrofacetBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
        // TODO Project 3-2: Part 2
        // *Importance* sample Beckmann normal distribution function (NDF) here.
        // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
        //       and return the sampled BRDF value.

        // Default cosine hemisphere sampling code
        *wi = cosineHemisphereSampler.get_sample(pdf);
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
        bool refracted = refract(wo, wi, ior);
        
        if (refracted) {
            *pdf = 1;
            double eta = wo.z > 0 ? 1.0/ior : ior;
            return transmittance / abs_cos_theta(*wi) / pow(eta, 2);
        }

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
        // Check for total internal reflection
        double eta = wo.z > 0 ? 1.0/ior : ior;
        if (1 - pow(eta, 2) * (1 - pow(wo.z, 2)) < 0) {
            reflect(wo, wi);
            *pdf = 1;
            return reflectance / abs_cos_theta(*wi);
        }
        
        // Calculate Schlick's reflection coefficient R
        double n1 = 1.0;
        double n2 = ior;
        double R0 = pow(((n1 - n2) / (n1 + n2)), 2);
        double R = R0 + (1.0 - R0) * pow((1.0 - abs_cos_theta(wo)), 5);
        
        if (coin_flip(R)) {
            bool refracted = refract(wo, wi, ior);
            *pdf = R;
            return R * reflectance / abs_cos_theta(*wi);
        } else {
            bool refracted = refract(wo, wi, ior);
            *pdf = 1.0-R;
            return (1.0-R) * transmittance / abs_cos_theta(*wi) / pow(eta, 2);
        }
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
        double eta = wo.z > 0 ? 1.0/ior : ior;

        // Check for total internal reflection
        if (1 - pow(eta, 2) * (1 - pow(wo.z, 2)) < 0) {
            return false;
        }
        
        // Set wi according to eta and the sign of wo.z
        wi->x = -eta * wo.x;
        wi->y = -eta * wo.y;
        
        int sign_z = wo.z > 0 ? 1 : -1;
        wi->z = (-1 * sign_z) * sqrt(1 - pow(eta, 2) * (1 - pow(wo.z, 2)));
        
        return true;

    }

} // namespace CGL
