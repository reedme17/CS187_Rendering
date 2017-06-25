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
class PaperDiffuse : public BSDF {
public:
	PaperDiffuse(const PropertyList &propList) {
		m_albedo = propList.getColor("albedo", Color3f(0.5f));
		m_transp = propList.getFloat("transparency", 0.5f);
	}

	virtual Color3f eval(const BSDFQueryRecord &bRec) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		if (bRec.measure != ESolidAngle) {
			return Color3f(0.0f);
		}
		Point2f uv(bRec.uv);
		if (!m_texture || !m_texture->sample(uv).x()) {
			return m_albedo * INV_PI;
		}
		return m_texture->sample(uv) * INV_PI;
	}

	virtual float pdf(const BSDFQueryRecord &bRec) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		if (bRec.measure != ESolidAngle) {
			return 0.0f;
		}
		return INV_PI * Frame::cosTheta(bRec.wo);
	}

	/// Draw a a sample from the BRDF model
	Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
		Point2f sample(_sample);

		if (sample.x() < (1-m_transp)) {
			sample.x() /= (1 - m_transp);
			bRec.measure = ESolidAngle;

			bRec.wo = Warp::squareToCosineHemisphere(sample);
			bRec.eta = 1.0f;

			float cosThetaI = Frame::cosTheta(bRec.wi);
			Normal3f n = Normal3f(0, 0, 1);
			if (cosThetaI < 0.0f) {
				cosThetaI = -cosThetaI;
				n = Normal3f(0, 0, -1);
				bRec.wo.z() = bRec.wo.z()*-1;
			}

			Point2f uv(bRec.uv);
			if (!m_texture || !m_texture->sample(uv).x()) {
				return m_albedo;
			}
			return m_texture->sample(uv);
		}

		else {
			float cosThetaI = Frame::cosTheta(bRec.wi);
			Normal3f n = Normal3f(0, 0, 1);
			if (cosThetaI < 0.0f) {
				cosThetaI = -cosThetaI;
				n = Normal3f(0, 0, -1);
			}
			float eta = 1,
				sinThetaTSqr = eta*eta * (1 - cosThetaI*cosThetaI);
			float cosThetaT = std::sqrt(1.0f - sinThetaTSqr);
			bRec.wo = eta * -bRec.wi + (eta * cosThetaI - cosThetaT) * n;

			bRec.measure = EDiscrete;
			return Color3f(1.0f);
		}
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

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"Paper Diffuse[\n"
			"  albedo = %s\n"
			"]", m_albedo.toString());
	}

	EClassType getClassType() const { return EBSDF; }
private:
	Texture * m_texture = nullptr;
	Color3f m_albedo;
	float m_transp; // transparency
};

NORI_REGISTER_CLASS(PaperDiffuse, "paper");
NORI_NAMESPACE_END
