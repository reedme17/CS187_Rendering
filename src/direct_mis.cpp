#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>


NORI_NAMESPACE_BEGIN

class DirectImportanceSampling : public Integrator {
public:
	DirectImportanceSampling(const PropertyList &props) {
		/* No parameters this time */
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

		Color3f lem(0.0f), lmat(0.0f);
		float w_em(0.0f), w_mat(0.0f);
		MISSampling(scene, its, sampler, ray, lem, lmat, w_em, w_mat);

		result += le + w_em*lem + w_mat*lmat;
		return result;
	}



	void MISSampling(const Scene *scene, Intersection its, Sampler *sampler, const Ray3f &ray, 
		Color3f &lem, Color3f &lmat, float &w_em, float &w_mat) const {
		float pdf_em_wem(0.0f), pdf_em_wmat(0.0f), pdf_mat_wem(0.0f), pdf_mat_wmat(0.0f);

		//////////////////////
		// Emitter Sampling //
		//////////////////////
		// if ray hits a mesh, sample a light and compute its contribution 
		EmitterQueryRecord dirRec(its.p);
		Color3f direct = scene->sampleDirect(dirRec, sampler->next2D());
		pdf_em_wem = scene->pdfDirect(dirRec);
		int v = VisibilityTester(scene, its, &dirRec.wi, &dirRec.dist);
		BSDFQueryRecord bRec_em(its.toLocal(-ray.d), its.toLocal(dirRec.wi), ESolidAngle);
		bRec_em.uv = its.uv;
		const BSDF *bsdf_em = its.mesh->getBSDF();
		Color3f bsdfDiff_em = bsdf_em->eval(bRec_em);
		float cosTheta = Frame::cosTheta(bRec_em.wo);
		lem += direct * v * bsdfDiff_em * fmax(0.0f, cosTheta);

		///////////////////
		// BRDF Sampling //
		///////////////////
		// if ray hits an non-emitter mesh, generate a new ray direction 
		BSDFQueryRecord bRec_mats(its.toLocal(-ray.d));
		bRec_mats.uv = its.uv;
		const BSDF *bsdf_mat = its.mesh->getBSDF();
		Color3f bsdfDiff_mat = bsdf_mat->sample(bRec_mats, sampler->next2D());
		pdf_mat_wmat = bsdf_mat->pdf(bRec_mats);
		pdf_mat_wem = bsdf_mat->pdf(bRec_em);
		Ray3f newRay = Ray3f(its.p, its.toWorld(bRec_mats.wo));

		// check if new ray hit anything, if not, check the environment emitter 
		Intersection its_newRay;
		if (!scene->rayIntersect(newRay, its_newRay)) {
			if (scene->hasEnvEmitter()) {
				const Emitter *e = scene->getEnvEmitter();
				EmitterQueryRecord envEmitRec(e, newRay);
				Color3f env = e->eval(envEmitRec);
				pdf_em_wmat = scene->pdfDirect(envEmitRec);
				lmat += env * bsdfDiff_mat;
			}
		}

		// if it hits something, check if it is an emmiter 
		if (scene->rayIntersect(newRay, its_newRay)) {
			if (its_newRay.mesh->isEmitter()) {
				Normal3f n = its_newRay.shFrame.n;
				const Emitter *e = its_newRay.mesh->getEmitter();
				EmitterQueryRecord emitRec(e, newRay.o, its_newRay.p, n);
				pdf_em_wmat = scene->pdfDirect(emitRec);
				lmat += e->eval(emitRec) * bsdfDiff_mat;
			}
		}

		////////////
		// Weight //
		////////////
		w_em = fmax(0.0f, pdf_em_wem / (pdf_em_wem + pdf_mat_wem));
		w_mat = fmax(0.0f, pdf_mat_wmat / (pdf_em_wmat + pdf_mat_wmat));
	}


	/// check the visibility of the point being rendered, is it not in shadow? 
	bool VisibilityTester(const Scene *scene, Intersection its, Vector3f *dir, float *dist) const {
		Ray3f ray(TRay<Point3f, Vector3f>(its.p, *dir), Epsilon, *dist - Epsilon);
		return !scene->rayIntersect(ray);
	}

	/// Return a human-readable summary
	std::string toString() const {
		return "DirectImportanceSampling[]";
	}
};

NORI_REGISTER_CLASS(DirectImportanceSampling, "direct_mis");
NORI_NAMESPACE_END