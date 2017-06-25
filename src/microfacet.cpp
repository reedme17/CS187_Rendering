/*
A simple BlinnPhong + diffuse blend BSDF skeleton implementation
Copyright (c) 2017 by Wojciech Jarosz
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
	Microfacet(const PropertyList &propList) {

		/* Albedo of the diffuse base material (a.k.a "kd") */
		m_kd = propList.getColor("kd", Color3f(0.5f));

		/* To ensure energy conservation, we must scale the
		specular component by 1-kd.

		While that is not a particularly realistic model of what
		happens in reality, this will greatly simplify the
		implementation. Please see the course staff if you're
		interested in implementing a more realistic version
		of this BRDF. */
		m_ks = 1 - m_kd.maxCoeff();

		/* RMS surface roughness */
		m_alpha = propList.getFloat("alpha", 0.1f);

		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);
	}

	/// Evaluate the BRDF for the given pair of directions
	virtual Color3f eval(const BSDFQueryRecord &bRec) const {
		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		Normal3f n = Normal3f(0, 0, 1);

		float D = expf(-Frame::tanTheta(wh) * Frame::tanTheta(wh) / (m_alpha * m_alpha)) /
			(M_PI * m_alpha * m_alpha * powf(Frame::cosTheta(wh), 4));

		float fre = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);

		float b_wi = powf(m_alpha * Frame::tanTheta(bRec.wi), -1);
		float b_wo = powf(m_alpha * Frame::tanTheta(bRec.wo), -1);
		float c_wi = bRec.wi.dot(wh) / bRec.wi.dot(n);
		float c_wo = bRec.wo.dot(wh) / bRec.wo.dot(n);
		float x_wi = c_wi > 0.0f ? 1.0f : 0.0f;
		float x_wo = c_wo > 0.0f ? 1.0f : 0.0f;
		float shadow_wi = b_wi < 1.6f ? x_wi * (3.535f*b_wi + 2.181f*b_wi*b_wi) / (1.0f + 2.276f*b_wi + 2.577f*b_wi*b_wi) : 1.0f;
		float shadow_wo = b_wo < 1.6f ? x_wo * (3.535f*b_wo + 2.181f*b_wo*b_wo) / (1.0f + 2.276f*b_wo + 2.577f*b_wo*b_wo) : 1.0f;
		float G = shadow_wi * shadow_wo;

		float cosThetaIO = 4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo);

		Color3f result = m_kd * INV_PI + m_ks * D * fre * G / cosThetaIO;
		return result;
	}

	/// Evaluate the sampling density of \ref sample() wrt. solid angles
	virtual float pdf(const BSDFQueryRecord &bRec) const {
		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		Normal3f n = Normal3f(0, 0, 1);
		Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		float wPdf = Warp::squareToBeckmannPdf(wh, m_alpha);

		float sPdf = wPdf / (4 * wh.dot(bRec.wo));
		float dPdf = Warp::squareToCosineHemispherePdf(bRec.wo);

		float result = sPdf * m_ks + dPdf * (1 - m_ks);
		return result;
	}

	/// Sample the BRDF
	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		Point2f sample(_sample);

		if (sample.x() < m_ks) {
			/* Sample the specular component */
			sample.x() /= m_ks;
			Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
			bRec.wo = 2 * wh * wh.dot(bRec.wi) - bRec.wi;
		}

		else {
			/* Sample the diffuse component */
			sample.x() = (sample.x() - m_ks) / (1 - m_ks);
			bRec.wo = Warp::squareToCosineHemisphere(sample);
		}

		/* We sampled in solid angle measure */
		bRec.measure = ESolidAngle;

		/* Relative index of refraction: no change */
		bRec.eta = 1.0f;

		float pdfval = pdf(bRec);
		if (pdfval == 0) {
			/* Reject samples outside of the positive
			hemisphere, which have zero probability */
			return Color3f(0.0f);
		}

		return eval(bRec) * (Frame::cosTheta(bRec.wo) / pdfval);
	}

	virtual std::string toString() const {
		return tfm::format(
			"Microfacet[\n"
			"  alpha = %f,\n"
			"  kd = %s,\n"
			"  ks = %f\n"
			"]",
			m_exponent,
			m_kd.toString(),
			m_ks
		);
	}
private:
	float m_exponent;
	float m_ks;
	float m_alpha;
	float m_intIOR;
	float m_extIOR;
	Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
