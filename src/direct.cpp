#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>


NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
	DirectIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return Color3f(0.0f);

		/* computes direct illumination from point light */
		Color3f result = Color3f(0.0f);

		/* Origin point from which we sample the emitter  */
		Point3f ref = its.p;
		/* Associated surface normal  */
		Normal3f n = its.shFrame.n.cwiseAbs();

		/* for each light source, add the illumination */
		for (Emitter *light : scene->getLights()) {
			EmitterQueryRecord eRec(its.p);
			eRec.emitter = light;
			Color3f direct = light->sample(eRec, sampler->next2D());
			Vector3f LocalWi = its.toLocal(eRec.wi);
			Vector3f LocalWo = its.toLocal(eRec.wi);
			/* check the visibility of the point being rendered, is it not in shadow? */
			if (VisibilityTester(scene, its, &eRec.wi, &eRec.dist)) {
				/*if not in shadow, get the BSDF and computer the illumination*/
				float cosTheta = its.shFrame.cosTheta(LocalWi);
				BSDFQueryRecord bRec(LocalWi, LocalWo, ESolidAngle);
				const BSDF *bsdf = its.mesh->getBSDF();
				Color3f bsdfDiff = bsdf->eval(bRec);
				result += light->eval(eRec) * fmax(0.0f, cosTheta) * bsdfDiff;
			}
		}
		return result;
	}

	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir, float *dist) const {
		Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, *dist);
		return !scene->rayIntersect(ray);
	}

	/// Return a human-readable summary
	std::string toString() const {
		return "DirectIntegrator[]";
	}
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END