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
class TextureDiffuse : public BSDF {
public:
	TextureDiffuse(const PropertyList &propList) {
		m_default_albedo = propList.getColor("albedo", Color3f(0.5f));
	}

	/// Evaluate the BRDF model
	Color3f eval(const BSDFQueryRecord &bRec) const {
		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle)
			return Color3f(0.0f);

		/* The BRDF is simply the albedo / pi */
		Point2f uv(bRec.uv);
		if (!m_albedo || !m_albedo->sample(uv).x()) {
			return m_default_albedo * INV_PI;
		}
		return m_albedo->sample(uv) * INV_PI;
	}

	/// Compute the density of \ref sample() wrt. solid angles
	float pdf(const BSDFQueryRecord &bRec) const {
		/* This is a smooth BRDF -- return zero if the measure
		is wrong, or when queried for illumination on the backside */
		if (bRec.measure != ESolidAngle)
			return 0.0f;


		/* Importance sampling density wrt. solid angles:
		cos(theta) / pi.

		Note that the directions in 'bRec' are in local coordinates,
		so Frame::cosTheta() actually just returns the 'z' component.
		*/
		return INV_PI * Frame::cosTheta(bRec.wo);
	}

	/// Draw a a sample from the BRDF model
	Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
		bRec.measure = ESolidAngle;

		/* Warp a uniformly distributed sample on [0,1]^2
		to a direction on a cosine-weighted hemisphere */
		bRec.wo = Warp::squareToCosineHemisphere(sample);

		/* Relative index of refraction: no change */
		bRec.eta = 1.0f;

		/* eval() / pdf() * cos(theta) = albedo. There
		is no need to call these functions. */
		if (!m_albedo || !m_albedo->sample(bRec.uv).x()) {
			return m_default_albedo * INV_PI;
		}
		return m_albedo->sample(bRec.uv);
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
			throw NoriException("TextureDiffuse::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"TextureDiffuse[\n"
			"]");
	}

	EClassType getClassType() const { return EBSDF; }
private:
	Color3f m_default_albedo;
	Texture * m_albedo = nullptr;
};

NORI_REGISTER_CLASS(TextureDiffuse, "diffuse with texture");
NORI_NAMESPACE_END
