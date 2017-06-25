#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

class DirectMaterialSampling : public Integrator {
public:
	DirectMaterialSampling(const PropertyList &props) {
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		Color3f result(0.0f), le(0.0f);

		// check if the ray intersect with anything, if not check enviroment emitter
		if (!scene->rayIntersect(ray, its)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, ray);
				Color3f env = e->eval(envEmitRec);
				result += env;
			}
			return result;
		}

		// if ray hits something, check if it is an emitter 
		if (its.mesh->isEmitter()) {
			Normal3f n = its.shFrame.n;
			const Emitter *e = its.mesh->getEmitter();
			EmitterQueryRecord emitRec(e, ray.o, its.p, n);
			le = e->eval(emitRec);
		}

		// if ray hits an non-emitter mesh, generate a new ray direction 
		BSDFQueryRecord bRec(its.toLocal(-ray.d));
		const BSDF *bsdf = its.mesh->getBSDF();
		Color3f bsdfDiff = bsdf->sample(bRec, sampler->next2D());
		//Color3f bsdfDiff = bsdf->eval(bRec) / bsdf->pdf(bRec); // = bsdf-sample / cosTheta
		Ray3f newRay = Ray3f(its.p, its.toWorld(bRec.wo));

		// check if new ray hit anything, if not, check the environment emitter 
		Intersection its_newRay;
		if (!scene->rayIntersect(newRay, its_newRay)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, newRay);
				Color3f env = e->eval(envEmitRec);
				float cosTheta = Frame::cosTheta(bRec.wo);
				result += env * bsdfDiff; // *fmax(0, cosTheta); // bsdf->sample already include the cosTheta
			}
			return le + result;
		}

		// if it hits something, check if it is an emmiter 
		if (its_newRay.mesh->isEmitter()) {
			Normal3f n = its_newRay.shFrame.n;
			const Emitter *e = its_newRay.mesh->getEmitter();
			EmitterQueryRecord emitRec(e, newRay.o, its_newRay.p, n);
			float cosTheta = Frame::cosTheta(bRec.wo);
			result += e->eval(emitRec) * bsdfDiff; // *fmax(0, cosTheta); // bsdf->sample already include the cosTheta
		}

		return le + result;
	}

	/// Return a human-readable summary
	std::string toString() const {
		return "DirectMaterialSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END