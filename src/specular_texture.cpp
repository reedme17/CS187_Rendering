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
class TextureSpecular : public BSDF {
public:
	TextureSpecular(const PropertyList &propList) {
		m_albedo = propList.getColor("albedo", Color3f(0.5f));
		m_specular = propList.getColor("specular", Color3f(0.5f));
		m_exponent = propList.getFloat("exponent", 100.f);
	}

	virtual Color3f eval(const BSDFQueryRecord &bRec) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		if (bRec.measure != ESolidAngle) {
			return m_specular;
		}
		return m_albedo;
	}

	virtual float pdf(const BSDFQueryRecord &bRec) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		if (bRec.measure != ESolidAngle) {
			return 0.0f;
		}

		Point2f uv(bRec.uv);
		float s = m_texture->sample(uv).maxCoeff();

		if (s < 0.5f) {
			return Warp::squareToCosineHemispherePdf(bRec.wo);
		}
		else 
			return Warp::squareToCosinePowerHemispherePdf(bRec.wo, m_exponent);
	}

	/// Draw a a sample from the BRDF model
	Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		Point2f sample(_sample);

		Point2f uv(bRec.uv);
		float s = m_texture->sample(uv).maxCoeff();

		if (s < 0.5f) {
			bRec.measure = ESolidAngle;
			bRec.wo = Warp::squareToCosineHemisphere(sample);
			return m_albedo;
		}

		else {
			Vector3f wh = Warp::squareToCosinePowerHemisphere(sample, m_exponent);
			bRec.wo = 2 * wh * wh.dot(bRec.wi) - bRec.wi;
			bRec.measure = EDiscrete;
			return m_specular;
		}
	}

	/// Add texture for the albedo
	virtual void addChild(NoriObject *obj) override {
		if (obj->getClassType() == ETexture) {
			m_texture = static_cast<Texture *>(obj);
		}
		else {
			throw NoriException("TextureSpecular::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"Texture Specular[\n"
			"  albedo = %s\n"
			"]", m_albedo.toString());
	}

	EClassType getClassType() const { return EBSDF; }
private:
	Texture * m_texture = nullptr;
	Color3f m_albedo;
	Color3f m_specular;
	float m_exponent;
};

NORI_REGISTER_CLASS(TextureSpecular, "specular with texture");
NORI_NAMESPACE_END
