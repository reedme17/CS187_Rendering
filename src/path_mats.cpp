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

class PathTracerMATS : public Integrator {
public:
	PathTracerMATS(const PropertyList &) { }

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const {
		//throw NoriException("PathTracerMATS::Li() is not yet implemented!");
		Color3f le(0.0f), ld(0.0f), li(0.0f);
		Ray3f currentRay = _ray;

		///////////////////////////////
		///// Direct illumination /////
		///////////////////////////////
		// check if the ray intersect with anything, if not check enviroment emitter
		Intersection its;
		if (!scene->rayIntersect(currentRay, its)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, currentRay);
				Color3f env = e->eval(envEmitRec);
				le += env;
			}
			return le + ld + li;
		}

		// if ray hits something, check if it is an emitter 
		if (its.mesh->isEmitter()) {
			Normal3f n = its.shFrame.n;
			const Emitter *e = its.mesh->getEmitter();
			EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
			le += e->eval(emitRec);
		}

		// if ray hits an non-emitter mesh, generate a new ray direction 
		BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
		const BSDF *bsdf = its.mesh->getBSDF();
		Color3f bsdfDiff = bsdf->sample(bRec, sampler->next2D());
		currentRay = Ray3f(its.p, its.toWorld(bRec.wo));

		// check if new ray hit anything, if not, check the environment emitter 
		if (!scene->rayIntersect(currentRay, its)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, currentRay);
				Color3f env = e->eval(envEmitRec);
				ld += env * bsdfDiff; 
			}
			return le + ld + li;
		}

		// if it hits something, check if it is an emmiter 
		if (its.mesh->isEmitter()) {
			Normal3f n = its.shFrame.n;
			const Emitter *e = its.mesh->getEmitter();
			EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
			ld += e->eval(emitRec) * bsdfDiff; 
		}


		/////////////////////////////////
		///// Indirect illumination /////
		/////////////////////////////////
		// if ray hits an non-emitter mesh, start path tracing 
		int i = 0;
		float deepth = 100;
		float q = 1 - fmax(fmax(bsdfDiff.x(), bsdfDiff.y()), bsdfDiff.z());
		float xi = sampler->next2D().x();
		while (i < deepth && xi > q) {
			// generate a new ray direction 
			BSDFQueryRecord bRec_indirect(its.toLocal(-currentRay.d));
			const BSDF *bsdf_indirect = its.mesh->getBSDF();
			// Accumulate the brdf
			bsdfDiff *= bsdf_indirect->sample(bRec_indirect, sampler->next2D());
			bsdfDiff /= (1 - q);
			currentRay = Ray3f(its.p, its.toWorld(bRec_indirect.wo));

			Color3f recLe(0.0f);

			// check if new ray hit anything, if not, check the environment emitter 
			if (!scene->rayIntersect(currentRay, its)) {
				if (scene->hasEnvEmitter()) {
					const Emitter *e = scene->getEnvEmitter();
					EmitterQueryRecord envEmitRec(e, currentRay);
					Color3f env = e->eval(envEmitRec);
					recLe += env;
				}
				li += recLe * bsdfDiff;
				return le + ld + li;
			}

			// if it hits something, check if it is an emmiter 
			if (its.mesh->isEmitter()) {
				Normal3f n = its.shFrame.n;
				const Emitter *e = its.mesh->getEmitter();
				EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
				recLe += e->eval(emitRec);
			}
			li += recLe * bsdfDiff;

			i++; 
			q = 1 - fmax(fmax(bsdfDiff.x(), bsdfDiff.y()), bsdfDiff.z());
			xi = sampler->next2D().x();
		}


		return le + ld + li;
	}



	std::string toString() const {
		return "PathTracerMATS[]";
	}
};

NORI_REGISTER_CLASS(PathTracerMATS, "path_mats");
NORI_NAMESPACE_END
