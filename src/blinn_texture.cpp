/*
A simple BlinnPhong + diffuse blend BSDF skeleton implementation
Copyright (c) 2017 by Wojciech Jarosz
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>


NORI_NAMESPACE_BEGIN

class TextureBlinn : public BSDF {
public:
	TextureBlinn(const PropertyList &propList) {
		/* specular exponent */
		m_exponent = propList.getFloat("exponent", 100.f);
		m_specular = propList.getFloat("specular", 0.5f);
	}

	/// Evaluate the BRDF for the given pair of directions
	virtual Color3f eval(const BSDFQueryRecord &bRec) const {
		Point2f uv(bRec.uv);
		Color3f m_kd = m_texture->sample(uv);

		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		/* ############################################### */
		/* TODO: Compute the value of the blinn-phong BRDF */
		/* ############################################### */
		//throw NoriException("BlinnPhong::eval is not implemented!");
		Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		Normal3f n = Normal3f(0, 0, 1);
		Color3f result = m_kd * INV_PI + m_specular * (m_exponent + 2) *INV_TWOPI * powf(wh.dot(n), m_exponent);
		return result;
	}

	/// Evaluate the sampling density of \ref sample() wrt. solid angles
	virtual float pdf(const BSDFQueryRecord &bRec) const {
		Point2f uv(bRec.uv);
		Color3f m_kd = m_texture->sample(uv);

		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle
			|| Frame::cosTheta(bRec.wi) <= 0
			|| Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		/* ############################## */
		/* TODO: Compute the sampling pdf */
		/* ############################## */
		//throw NoriException("BlinnPhong::pdf is not implemented!");
		Normal3f n = Normal3f(0, 0, 1);
		Vector3f wh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		float wPdf = Warp::squareToCosinePowerHemispherePdf(wh, m_exponent);

		float sPdf = wPdf / (4 * wh.dot(bRec.wo));
		float dPdf = Warp::squareToCosineHemispherePdf(bRec.wo);

		float result = sPdf * m_specular + dPdf * (1 - m_specular);
		return result;
	}

	/// Sample the BRDF
	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		Point2f uv(bRec.uv);
		Color3f m_kd = m_texture->sample(uv);

		Point2f sample(_sample);

		if (sample.x() < m_specular) {
			sample.x() /= m_specular;

			/* ################################### */
			/* TODO: Sample the specular component */
			/* ################################### */
			//throw NoriException("BlinnPhong::sample is not fully implemented!");
			Vector3f wh = Warp::squareToCosinePowerHemisphere(sample, m_exponent);
			bRec.wo = 2 * wh * wh.dot(bRec.wi) - bRec.wi;

		}
		else {
			sample.x() = (sample.x() - m_specular) / (1 - m_specular);

			/* ################################## */
			/* TODO: Sample the diffuse component */
			/* ################################## */
			//throw NoriException("BlinnPhong::sample is not fully implemented!");
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

	/// Add texture for the albedo
	virtual void addChild(NoriObject *obj) override {
		if (obj->getClassType() == ETexture) {
			m_texture = static_cast<Texture *>(obj);
		}
		else {
			throw NoriException("Texture Fresnel::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}


	virtual std::string toString() const {
		return tfm::format(
			"Texture Blinn[\n"
			"  alpha = %f \n"
			"]",
			m_exponent
		);
	}
private:
	Texture * m_texture;
	float m_exponent;
	float m_specular;
};

NORI_REGISTER_CLASS(TextureBlinn, "blinn with texture");
NORI_NAMESPACE_END
