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

class PathTracerVolumetric : public Integrator {
public:
	PathTracerVolumetric(const PropertyList &) { }

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const {
		Color3f le(0.0f), ld(0.0f), li(0.0f);
		Ray3f currentRay = _ray;
		Intersection its;

		if (scene->hasMedium()) {
			const Medium *m = scene->getMedium();

			// check if the ray intersect with anything, if not check enviroment emitter
			if (!scene->rayIntersect(currentRay, its)) {
				if (scene->hasEnvEmitter()) {
					const Emitter *e = scene->getEnvEmitter();
					EmitterQueryRecord envEmitRec(e, currentRay);
					Color3f env = e->eval(envEmitRec);
					MediumQueryRecord mRec(m, currentRay.o, its.p, its.shFrame.n);
					m->sample(mRec, sampler->next2D());
					le += env * mRec.tr;
				}
				return le + ld + li;
			}

			// if ray hits something, check if it is an emitter 
			if (its.mesh->isEmitter()) {
				Normal3f n = its.shFrame.n;
				const Emitter *e = its.mesh->getEmitter();
				EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
				MediumQueryRecord mRec(m, currentRay.o, its.p, its.shFrame.n);
				m->sample(mRec, sampler->next2D());
				le += e->eval(emitRec) * mRec.tr;
			}

			Color3f vPT(1.0f);
			int i = 0;
			float deepth = 100;
			float q = 1 - vPT.maxCoeff();
			float xi = sampler->next2D().x();

			while (i < deepth && xi > q && scene->rayIntersect(currentRay, its)) {
				float tmax = its.t;
				float t = m->sampleFreePath(sampler->next2D());

				Color3f recLe(0.0f);

				// volume interaction 
				if (t < tmax) {
					Point3f p = currentRay.o + currentRay.d * t;
					MediumQueryRecord mRec(m, currentRay.o, p, its.shFrame.n);
					m->sample(mRec, sampler->next2D());
					vPT *= mRec.sigma_s / mRec.sigma_t;
					vPT /= (1 - q);
					currentRay = Ray3f(p, its.toWorld(mRec.wo));
				}

				// surface interaction 
				else {
					// if it hits something, check if it is an emmiter 
					if (its.mesh->isEmitter()) {
						Normal3f n = its.shFrame.n;
						const Emitter *e = its.mesh->getEmitter();
						EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
						recLe += e->eval(emitRec);
					}
					li += recLe * vPT;

					// generate a new ray direction 
					BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
					bRec.uv = its.uv;
					const BSDF *bsdf_indirect = its.mesh->getBSDF();
					// Accumulate the brdf
					vPT *= bsdf_indirect->sample(bRec, sampler->next2D());
					vPT /= (1 - q);
					currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
				}

				i++;
				q = 1 - vPT.maxCoeff();
				xi = sampler->next2D().x();
			}

			return le + ld + li;
		}



		//////////////////////////////
		// Normal MIS path tracing ///
		//////////////////////////////
		else {
			bool hitMesh = scene->rayIntersect(currentRay, its);

			// check if the ray intersect with anything, if not check enviroment emitter
			if (!hitMesh) {
				if (scene->hasEnvEmitter()) {
					const Emitter *e = scene->getEnvEmitter();
					EmitterQueryRecord envEmitRec(e, currentRay);
					Color3f env = e->eval(envEmitRec);
					le += env;
				}
				return le + li;
			}

			// if ray hits something, check if it is an emitter 
			if (its.mesh->isEmitter()) {
				Normal3f n = its.shFrame.n;
				const Emitter *e = its.mesh->getEmitter();
				EmitterQueryRecord emitRec(e, currentRay.o, its.p, n);
				le += e->eval(emitRec);
			}

			// if ray hits a mesh, start mis path tracing 
			Color3f accBrdf(1.0f);
			int i = 0;
			float deepth = 100;
			float q = 1 - fmax(fmax(accBrdf.x(), accBrdf.y()), accBrdf.z());
			float xi = sampler->next2D().x();
			while (i < deepth && xi > q) {
				accBrdf /= (1 - q);
				Color3f lem_ind(0.0f), lmat_ind(0.0f);
				float pdf_em_wem(1.0f), pdf_em_wmat(1.0f), pdf_mat_wem(1.0f), pdf_mat_wmat(1.0f);

				//////////////////////
				// Emitter Sampling //
				//////////////////////
				const BSDF *bsdf = its.mesh->getBSDF();
				EmitterQueryRecord lRec(its.p);
				Color3f direct = scene->sampleDirect(lRec, sampler->next2D());
				if ((direct.array() != 0).any()) {
					BSDFQueryRecord bRec(its.toLocal(-currentRay.d), its.toLocal(lRec.wi), ESolidAngle);
					bRec.uv = its.uv;
					pdf_mat_wem = bsdf->pdf(bRec);
					pdf_em_wem = scene->pdfDirect(lRec);
					float weight = miWeight(pdf_em_wem, pdf_mat_wem);
					int v = VisibilityTester(scene, its, &lRec.wi, &lRec.dist);
					lem_ind += v * direct * weight * bsdf->eval(bRec) * accBrdf * fmax(0.0f, Frame::cosTheta(bRec.wo));
				}

				///////////////////
				// BRDF Sampling //
				///////////////////
				BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
				bRec.uv = its.uv;
				Color3f bsdfWeight = bsdf->sample(bRec, sampler->next2D());
				accBrdf *= bsdfWeight;
				currentRay = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));

				// check if new ray hit anything, if not, check the environment emitter 
				bool hitEmitter = false;
				bool outScene = false;
				if (!scene->rayIntersect(currentRay, its)) {
					if (scene->hasEnvEmitter()) {
						lRec = EmitterQueryRecord(scene->getEnvEmitter(), currentRay);
						hitEmitter = true;
					}
					outScene = true;
				}

				// if it hits something, check if it is an emmiter 
				else if (its.mesh->isEmitter()) {
					lRec = EmitterQueryRecord(its.mesh->getEmitter(), currentRay.o, its.p, its.shFrame.n);
					hitEmitter = true;
				}

				// compute the contribution of brdf sampling 
				if ((accBrdf.array() != 0).any() && hitEmitter) {
					Color3f radiance = lRec.emitter->eval(lRec);
					pdf_em_wmat = scene->pdfDirect(lRec);
					pdf_mat_wmat = bsdf->pdf(bRec);
					float weight = miWeight(pdf_mat_wmat, pdf_em_wmat);
					if (bRec.measure == EDiscrete) {
						weight = 1;
					}
					lmat_ind += accBrdf * radiance * weight;
				}

				// sum up the MIS sampling 
				li += lem_ind + lmat_ind;

				i++;
				q = 1 - fmax(fmax(accBrdf.x(), accBrdf.y()), accBrdf.z());
				xi = sampler->next2D().x();
			}








			return le + ld + li;
		}
	}






	inline float miWeight(float pdfA, float pdfB) const {
		pdfA *= pdfA; pdfB *= pdfB;
		return pdfA / (pdfA + pdfB);
	}


	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir, float *dist) const {
		Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, *dist - Epsilon);
		return !scene->rayIntersect(ray);
	}



	std::string toString() const {
		return "Volumetric Path Tracing[]";
	}
};

NORI_REGISTER_CLASS(PathTracerVolumetric, "path_vol");
NORI_NAMESPACE_END
