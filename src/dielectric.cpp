/*
	Dielectric material skeleton implementation
	Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
	Dielectric(const PropertyList &propList) {
		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);
	}

	virtual Color3f eval(const BSDFQueryRecord &) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		return Color3f(0.0f);
	}

	virtual float pdf(const BSDFQueryRecord &) const {
		/* Discrete BRDFs always evaluate to zero in Nori */
		return 0.0f;
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
		//throw NoriException("Dielectric::sample() is not yet implemented!");

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

		if (sample.x() < fr) {
			/* reflection */
			// PBRT p526
			bRec.eta = 1.0;
			bRec.wo = 2 * n * n.dot(bRec.wi) - bRec.wi;
		}
		else {
			/* refaction */
			// PBRT p531
			bRec.eta = eta;
			bRec.wo = eta * -bRec.wi + (eta * cosThetaI - cosThetaT) * n;
		}

		bRec.measure = EDiscrete;
		return Color3f(1.0);
    }

    virtual std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
