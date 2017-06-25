#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AOIntegrator : public Integrator {
public:
	AOIntegrator(const PropertyList &props) {
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		/* computes ambient occlusion */
		Color3f result = Color3f(0.0f);

		/* for a sample ambient light source, add the illumination */
		Vector3f light = Warp::squareToCosineHemisphere(sampler->next2D());
		Vector3f worldLight = its.toWorld(light);
		if (VisibilityTester(scene, its, &worldLight)) {
			float cosTheta = Frame::cosTheta(light);
			result += cosTheta / M_PI;
		}
		float pdf = Warp::squareToCosineHemispherePdf(light);
		return result / pdf;
	}

	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir) const {
		Ray3f ray(its.p, *dir);
		// Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, INFINITY);
		return !scene->rayIntersect(ray);
	}

	/// Return a human-readable summary
	std::string toString() const {
		return "AOIntegrator[]";
	}
};

NORI_REGISTER_CLASS(AOIntegrator, "ao");
NORI_NAMESPACE_END