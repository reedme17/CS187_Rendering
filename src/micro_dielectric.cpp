/*
Dielectric material skeleton implementation
Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/// Rough dielectric BSDF
class MicrofacetDielectric : public BSDF {
public:
	MicrofacetDielectric(const PropertyList &propList) {
		/* RMS surface roughness */
		m_alpha = propList.getFloat("alpha", 0.1f);

		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);

		//m_kd = propList.getColor("kd", Color3f(0.5f));
		//m_ks = 1 - m_kd.maxCoeff();
	}

	float microfacet_D(Vector3f wh, float alpha) const {
		if (alpha == 0) {
			return 0.0f;
		}
		float D = expf(-Frame::tanTheta(wh) * Frame::tanTheta(wh) / (alpha * alpha)) /
			(M_PI * alpha * alpha * powf(Frame::cosTheta(wh), 4));
		return D;
	}

	float microfacet_G(const BSDFQueryRecord &bRec, Vector3f wh, Normal3f n, float alpha) const {
		if (alpha == 0) {
			return 0.0f;
		}
		float b_wi = powf(alpha * Frame::tanTheta(bRec.wi), -1);
		float b_wo = powf(alpha * Frame::tanTheta(bRec.wo), -1);
		float c_wi = bRec.wi.dot(wh) / bRec.wi.dot(n);
		float c_wo = bRec.wo.dot(wh) / bRec.wo.dot(n);
		float x_wi = c_wi > 0.0f ? 1.0f : 0.0f;
		float x_wo = c_wo > 0.0f ? 1.0f : 0.0f;
		float shadow_wi = b_wi < 1.6f ? x_wi * (3.535f*b_wi + 2.181f*b_wi*b_wi) / (1.0f + 2.276f*b_wi + 2.577f*b_wi*b_wi) : 1.0f;
		float shadow_wo = b_wo < 1.6f ? x_wo * (3.535f*b_wo + 2.181f*b_wo*b_wo) / (1.0f + 2.276f*b_wo + 2.577f*b_wo*b_wo) : 1.0f;
		float G = shadow_wi * shadow_wo;
		return G;
	}


	virtual Color3f eval(const BSDFQueryRecord &bRec) const {
		if (bRec.measure != ESolidAngle)
			return Color3f(0.0f);

		float texture_alpha = m_alpha;

		if (m_texture != nullptr) {
			Point2f uv(bRec.uv);
			Color3f rgb = m_texture->sample(uv);
			float alpha_intensity = fabsf(0.21 * rgb.x() + 0.72 * rgb.y() + 0.07 * rgb.z());
			texture_alpha = fmax(0, m_alpha * alpha_intensity);
		}

		Normal3f n = Normal3f(0, 0, 1);
		float etaI = m_extIOR, etaT = m_intIOR;
		float cosThetaI = Frame::cosTheta(bRec.wi);
		if (cosThetaI < 0.0f) {
			n = Normal3f(0, 0, -1);
			std::swap(etaI, etaT);
		}

		Vector3f wh_r = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		Vector3f wh_t = (-etaI * bRec.wi - etaT * bRec.wo) / (-etaI * bRec.wi - etaT * bRec.wo).norm();

		float D_r = microfacet_D(wh_r, texture_alpha);
		float G_r = microfacet_G(bRec, wh_r, n, texture_alpha);
		float F_r = fresnel(wh_r.dot(bRec.wi), m_extIOR, m_intIOR);
		float cosThetaIO = 4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo);
		Color3f fr = D_r * F_r * G_r / cosThetaIO;

		float D_t = microfacet_D(wh_t, texture_alpha);
		float G_t = microfacet_G(bRec, wh_t, n, texture_alpha);
		float F_t = fresnel(wh_t.dot(bRec.wi), m_extIOR, m_intIOR);
		Color3f ft = (bRec.wi.dot(wh_t)*bRec.wo.dot(wh_t)) / (bRec.wi.dot(n)*bRec.wo.dot(n));
		ft *= (etaT*etaT*(1 - F_t)*G_t*D_t) / pow((etaI* bRec.wi.dot(wh_t) + etaT*bRec.wo.dot(wh_t)), 2);

		Color3f result = fr + ft;
		return Color3f(fabsf(result.x()), fabsf(result.y()), fabsf(result.z()));
	}



	virtual float pdf(const BSDFQueryRecord &bRec) const {
		if (bRec.measure != ESolidAngle)
			return 0.0f;

		float texture_alpha = m_alpha;
		if (m_texture != nullptr) {
			Point2f uv(bRec.uv);
			Color3f rgb = m_texture->sample(uv);
			float alpha_intensity = fabsf(0.21 * rgb.x() + 0.72 * rgb.y() + 0.07 * rgb.z());
			texture_alpha = fmax(0, m_alpha * alpha_intensity);
		}

		if (texture_alpha == 0) {
			return 1.0f;
		}

		float etaI = m_extIOR, etaT = m_intIOR;
		float cosThetaI = Frame::cosTheta(bRec.wi);
		if (cosThetaI < 0.0f) {
			cosThetaI = -cosThetaI;
			std::swap(etaI, etaT);
		}
		float fr = fresnel(cosThetaI, etaI, etaT);

		Vector3f wh_r = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		Vector3f wh_t = (-etaI * bRec.wi - etaT * bRec.wo) / (-etaI * bRec.wi - etaT * bRec.wo).norm();

		float pdf_r = Warp::squareToBeckmannPdf(wh_r, texture_alpha);
		float pdf_t = Warp::squareToBeckmannPdf(wh_t, texture_alpha);

		float pdf_r_wo = pdf_r / (4 * wh_r.dot(bRec.wo));
		float pdf_t_wo = pdf_t / (4 * wh_t.dot(bRec.wo));

		return fr * pdf_r_wo + (1 - fr) * pdf_t_wo;
	}



	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		float texture_alpha = m_alpha;
		if (m_texture != nullptr) {
			Point2f uv(bRec.uv);
			Color3f rgb = m_texture->sample(uv);
			float alpha_intensity = fabsf(0.21 * rgb.x() + 0.72 * rgb.y() + 0.07 * rgb.z());
			texture_alpha = fmax(0, m_alpha * alpha_intensity);
		}

		// PBRT p516
		Normal3f n = Normal3f(0, 0, 1);
		float etaI = m_extIOR, etaT = m_intIOR;
		float cosThetaI = Frame::cosTheta(bRec.wi);
		float fr = fresnel(cosThetaI, etaI, etaT);

		/* Swap the indices of refraction if the interaction starts
		at the inside of the object */
		if (cosThetaI < 0.0f) {
			std::swap(etaI, etaT);
			cosThetaI = -cosThetaI;
			n = Normal3f(0, 0, -1);
		}

		/* Using Snell's law, calculate the squared sine of the
		angle between the normal and the transmitted ray */
		float eta = etaI / etaT,
			sinThetaTSqr = eta*eta * (1 - cosThetaI*cosThetaI);
		float cosThetaT = std::sqrt(1.0f - sinThetaTSqr);

		Point2f sample(_sample);
		Vector3f wh = Warp::squareToBeckmann(sample, texture_alpha);
		wh.z() = wh.z() * n.z();

		if (sample.x() < fr) {
			/* reflection */
			// PBRT p526
			bRec.eta = 1.0;
			bRec.wo = 2 * wh * wh.dot(bRec.wi) - bRec.wi;
		}
		else {
			/* refaction */
			// PBRT p531
			bRec.eta = eta;
			bRec.wo = eta * -bRec.wi + (eta * cosThetaI - cosThetaT) * wh;
		}
		bRec.measure = EDiscrete;
		return Color3f(1.0);
	}

	/// Add texture for the albedo
	virtual void addChild(NoriObject *obj) override {
		if (obj->getClassType() == ETexture) {
			m_texture = static_cast<Texture *>(obj);
		}
		else {
			throw NoriException("TextureDiffuse::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}

	virtual std::string toString() const {
		return tfm::format(
			"Microfacet Dielectric[\n"
			"  intIOR = %f,\n"
			"  extIOR = %f\n"
			"]",
			m_intIOR, m_extIOR);
	}
private:
	//Color3f m_kd;
	//float m_ks;
	float m_alpha;
	float m_intIOR;
	float m_extIOR;
	Texture * m_texture = nullptr;
};

NORI_REGISTER_CLASS(MicrofacetDielectric, "micro_dielectric");
NORI_NAMESPACE_END
