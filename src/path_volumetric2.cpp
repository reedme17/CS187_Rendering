/*
Path tracer skeleton implementation
Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/medium.h>
#include <nori/bsdf.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class PathTracerVolumetric2 : public Integrator {
public:
	PathTracerVolumetric2(const PropertyList &) { }

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const {
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


		/////////////////////////////////
		///// Indirect illumination /////
		/////////////////////////////////
		// if ray hits an non-emitter mesh, start path tracing 
		Color3f accBrdf(1.0f);
		int i = 0;
		float deepth = 100;
		float q = 1 - accBrdf.maxCoeff();
		float xi = sampler->next2D().x();

		while (i < deepth && xi > q) {

			Color3f recLe(0.0f);

			// check if it is a medium, if yes, start volumetric path tracing 
			if (its.mesh->isMedium()) {
				const Medium *m = its.mesh->getMedium();
				float tmax = its.t;
				float t = m->sampleFreePath(sampler->next2D());

				// volume interaction 
				if (t < tmax) {
					Point3f p = currentRay.o + currentRay.d * t;
					MediumQueryRecord mRec(m, currentRay.o, p, its.shFrame.n);
					m->sample(mRec, sampler->next2D());
					accBrdf *= mRec.albedo;
					accBrdf /= (1 - q);
					currentRay = Ray3f(p, its.toWorld(mRec.wo));
				}

				// surface interaction 
				else {
					// generate a new ray direction 
					BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
					bRec.uv = its.uv;
					const BSDF *bsdf_indirect = its.mesh->getBSDF();
					MediumQueryRecord mRec(m, currentRay.o, its.p, its.shFrame.n);
					// Accumulate the brdf
					accBrdf *= bsdf_indirect->sample(bRec, sampler->next2D());
					accBrdf /= (1 - q);
					currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
				}


				// check if new ray hit anything, if not, check the environment emitter 
				if (!scene->rayIntersect(currentRay, its)) {
					if (scene->hasEnvEmitter()) {
						const Emitter *e = scene->getEnvEmitter();
						EmitterQueryRecord envEmitRec(e, currentRay);
						Color3f env = e->eval(envEmitRec);
						recLe += env;
					}
					li += recLe * accBrdf;
					return le + ld + li;
				}

				// if it hits something, check if it is an emmiter 
				if (its.mesh->isEmitter()) {
					Normal3f n = its.shFrame.n;
					const Emitter *e = its.mesh->getEmitter();
					EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
					recLe += e->eval(emitRec);
				}
			}


			// if it is NOT a medium, start normal path tracing 
			else {
				// generate a new ray direction 
				BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
				bRec.uv = its.uv;
				const BSDF *bsdf_indirect = its.mesh->getBSDF();
				// Accumulate the brdf
				accBrdf *= bsdf_indirect->sample(bRec, sampler->next2D());
				accBrdf /= (1 - q);
				currentRay = Ray3f(its.p, its.toWorld(bRec.wo));


				// check if new ray hit anything, if not, check the environment emitter 
				if (!scene->rayIntersect(currentRay, its)) {
					if (scene->hasEnvEmitter()) {
						const Emitter *e = scene->getEnvEmitter();
						EmitterQueryRecord envEmitRec(e, currentRay);
						Color3f env = e->eval(envEmitRec);
						recLe += env;
					}
					li += recLe * accBrdf;
					return le + ld + li;
				}

				// if it hits something, check if it is an emmiter 
				if (its.mesh->isEmitter()) {
					Normal3f n = its.shFrame.n;
					const Emitter *e = its.mesh->getEmitter();
					EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
					recLe += e->eval(emitRec);
				}
			}

			li += recLe * accBrdf;

			// Russian Roulette
			i++;
			q = 1 - accBrdf.maxCoeff();
			xi = sampler->next2D().x();
		}

		return le + ld + li;
	}


	std::string toString() const {
		return "Volumetric Path Tracing[]";
	}
};

NORI_REGISTER_CLASS(PathTracerVolumetric2, "path_vol2");
NORI_NAMESPACE_END
