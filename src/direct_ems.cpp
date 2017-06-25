#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/warp.h>


NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public:
	DirectEmitterSampling(const PropertyList &props) {
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

		// if ray hits a mesh, sample a light and compute its contribution 
		EmitterQueryRecord dirRec(its.p);
		Color3f direct = scene->sampleDirect(dirRec, sampler->next2D());
		int v = VisibilityTester(scene, its, &dirRec.wi, &dirRec.dist);
		BSDFQueryRecord bRec(its.toLocal(-ray.d), its.toLocal(dirRec.wi), ESolidAngle);
		const BSDF *bsdf = its.mesh->getBSDF();
		Color3f bsdfDiff = bsdf->eval(bRec);
		float cosTheta = Frame::cosTheta(bRec.wo);
		result += direct * v * bsdfDiff * fmax(0.0f, cosTheta);

		return le + result;
	}

	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir, float *dist) const {
		Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, *dist - Epsilon);
		return !scene->rayIntersect(ray);
	}

	/// Return a human-readable summary
	std::string toString() const {
		return "DirectEmitterSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END