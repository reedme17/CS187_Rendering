/*
This file is part of Nori, a simple educational ray tracer

Copyright (c) 2015 by Wenzel Jakob

Nori is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License Version 3
as published by the Free Software Foundation.

Nori is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

/**
* \brief Diffuse / Lambertian BRDF model
*/
class TextureFresnel : public BSDF {
public:
	TextureFresnel(const PropertyList &propList) {
		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);

		m_alpha = propList.getFloat("alpha", 0.1f);
	}

	/// Evaluate the BRDF model
	Color3f eval(const BSDFQueryRecord &bRec) const {
		Point2f uv(bRec.uv);
		Color3f kd = m_albedo->sample(uv);
		float ks = 1 - kd.maxCoeff();

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

		Color3f result = kd * INV_PI + ks * D * fre * G / cosThetaIO;
		return result;
	}

	/// Compute the density of \ref sample() wrt. solid angles
	float pdf(const BSDFQueryRecord &bRec) const {
		Point2f uv(bRec.uv);
		Color3f kd = m_albedo->sample(uv);
		float ks = 1 - kd.maxCoeff();

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

		float result = sPdf * ks + dPdf * (1 - ks);
		return result;
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		Point2f uv(bRec.uv);
		Color3f kd = m_albedo->sample(uv);
		float ks = 1 - kd.maxCoeff();

		Point2f sample(_sample);

		if (sample.x() < ks) {
			/* Sample the specular component */
			sample.x() /= ks;
			Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
			bRec.wo = 2 * wh * wh.dot(bRec.wi) - bRec.wi;
		}

		else {
			/* Sample the diffuse component */
			sample.x() = (sample.x() - ks) / (1 - ks);
			bRec.wo = Warp::squareToCosineHemisphere(sample);
		}

		/* We sampled in solid angle measure */
		bRec.measure = ESolidAngle;

		/* Relative index of refraction: no change */
		bRec.eta = 1.0f;

		float pdfval = pdf(bRec);
		if (pdfval <= 0) {
			/* Reject samples outside of the positive
			hemisphere, which have zero probability */
			return Color3f(0.0f);
		}

		return eval(bRec) * (Frame::cosTheta(bRec.wo) / pdfval);
	}

	bool isDiffuse() const {
		return true;
	}

	/// Add texture for the albedo
	virtual void addChild(NoriObject *obj) override {
		if (obj->getClassType() == ETexture) {
			m_albedo = static_cast<Texture *>(obj);
		}
		else {
			throw NoriException("Texture Fresnel::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"Texture Fresnel[\n"
			"  albedo = %s\n"
			"]", m_albedo->toString());
	}

	EClassType getClassType() const { return EBSDF; }


private:
	Texture * m_albedo;
	float m_intIOR, m_extIOR;
	float m_alpha;
};

NORI_REGISTER_CLASS(TextureFresnel, "fresnel diffuse with texture");
NORI_NAMESPACE_END
