/*
    Path tracer skeleton implementation
    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class PathTracerNEE : public Integrator {
public:
    PathTracerNEE(const PropertyList &) { }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const {
        //throw NoriException("PathTracerNEE::Li() is not yet implemented!");
		/* Find the surface that is visible in the requested direction */
		Intersection its;
		Color3f le(0.0f), ld(0.0f), li(0.0f);
		Ray3f ray = _ray;

		///////////////////////////////
		///// Direct illumination /////
		///////////////////////////////
		// check if the ray intersect with anything, if not check enviroment emitter
		if (!scene->rayIntersect(ray, its)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, ray);
				Color3f env = e->eval(envEmitRec);
				le += env;
			}
			return le + ld + li;
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
		ld += direct * v * bsdfDiff * fmax(0.0f, cosTheta);

		/////////////////////////////////
		///// Indirect illumination /////
		/////////////////////////////////
		Color3f accBrdf(1.0f);
		int i = 0;
		float q = 0.1f;
		float deepth = 100;
		float xi = sampler->next2D().x();
		while (i < deepth && xi > q) {
			BSDFQueryRecord bRec_ind(its.toLocal(-ray.d));
			const BSDF *bsdf_ind = its.mesh->getBSDF();
			Color3f bsdfDiff_ind = bsdf_ind->sample(bRec_ind, sampler->next2D());
			accBrdf *= bsdfDiff_ind;
			if (bRec_ind.measure != EDiscrete) {
				accBrdf /= (1 - q);
			}
			ray = Ray3f(its.p, its.toWorld(bRec_ind.wo));

			// check if the ray intersect with anything, if not check enviroment emitter
			if (!scene->rayIntersect(ray, its)) {
				if (scene->hasEnvEmitter()) {
					const Emitter *e = scene->getEnvEmitter();
					EmitterQueryRecord envEmitRec(e, ray);
					Color3f env = e->eval(envEmitRec);
					if (bRec_ind.measure == EDiscrete) {
						li += env * accBrdf;
					}
				}
				return le + ld + li;
			}

			// if ray hits something, check if it is an emitter 
			if (its.mesh->isEmitter()) {
				Normal3f n = its.shFrame.n;
				const Emitter *e = its.mesh->getEmitter();
				EmitterQueryRecord emitRec(e, ray.o, its.p, n);
				if (bRec_ind.measure == EDiscrete) {
					li += e->eval(emitRec) * accBrdf;
				}
			}

			// if ray hits a mesh, sample a light and compute its contribution 
			EmitterQueryRecord dirRec(its.p);
			Color3f direct = scene->sampleDirect(dirRec, sampler->next2D());
			int v = VisibilityTester(scene, its, &dirRec.wi, &dirRec.dist);
			BSDFQueryRecord bRec(its.toLocal(-ray.d), its.toLocal(dirRec.wi), ESolidAngle);
			const BSDF *bsdf = its.mesh->getBSDF();
			Color3f bsdfDiff = bsdf->eval(bRec);
			float cosTheta = Frame::cosTheta(bRec.wo);
			li += direct * v * bsdfDiff * accBrdf * fmax(0.0f, cosTheta);
			i++;
		}

		return le + ld + li;
    }


	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir, float *dist) const {
		Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, *dist - Epsilon);
		return !scene->rayIntersect(ray);
	}


    std::string toString() const {
        return "PathTracerNEE[]";
    }
};

NORI_REGISTER_CLASS(PathTracerNEE, "path_nee");
NORI_NAMESPACE_END
