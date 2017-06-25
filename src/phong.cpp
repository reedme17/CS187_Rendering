/*
    Phong + diffuse blend BSDF skeleton implementation
    Copyright (c) 2017 by Wojciech Jarosz
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Phong : public BSDF {
public:
    Phong(const PropertyList &propList) {
        /* specular exponent */
        m_exponent = propList.getFloat("exponent", 100.f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));
        m_ks = propList.getFloat("ks", 1.0f);

        /* To ensure energy conservation, we must scale the
           specular component by 1-kd.

           While that is not a particularly realistic model of what
           happens in reality, this will greatly simplify the
           implementation. Please see the course staff if you're
           interested in implementing a more realistic version
           of this BRDF. */
        m_ks *= 1 - m_kd.maxCoeff();
    }

    /// Evaluate the BRDF for the given pair of directions
    virtual Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
           is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);

        /* ######################################### */
        /* TODO: Compute the value of the phong BRDF */
        /* ######################################### */
        //throw NoriException("Phong::eval is not implemented!");
		Normal3f n = Normal3f(0, 0, 1);
		Vector3f wr = 2 * n * n.dot(bRec.wi) - bRec.wi;
		Color3f result = m_kd * INV_PI + m_ks * (m_exponent + 2) * INV_TWOPI * powf(wr.dot(bRec.wo), m_exponent);
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

        /* ############################## */
        /* TODO: Compute the sampling pdf */
        /* ############################## */
        //throw NoriException("Phong::pdf is not implemented!");
		Normal3f n = Normal3f(0, 0, 1);
		Vector3f wr = 2 * n * n.dot(bRec.wi) - bRec.wi;
		const Vector3f temp = Frame(wr).toLocal(bRec.wo);
		float sPdf = Warp::squareToCosinePowerHemispherePdf(temp, m_exponent);
		float dPdf = Warp::squareToCosineHemispherePdf(bRec.wo);
		float result = sPdf * m_ks + dPdf * (1 - m_ks);
		return result;
    }

    /// Sample the BRDF
    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        Point2f sample(_sample);

        if (sample.x() < m_ks) {
            sample.x() /= m_ks;

            /* ################################### */
            /* TODO: Sample the specular component */
            /* ################################### */
            //throw NoriException("Phong::sample is not fully implemented!");
			bRec.wo = Warp::squareToCosinePowerHemisphere(sample, m_exponent);
			Normal3f n = Normal3f(0, 0, 1);
			Vector3f wr = 2 * n * n.dot(bRec.wi) - bRec.wi;
			bRec.wo = Frame(wr).toWorld(bRec.wo);

        } else {
            sample.x() = (sample.x() - m_ks) / (1-m_ks);
            
            /* ################################## */
            /* TODO: Sample the diffuse component */
            /* ################################## */
            //throw NoriException("Phong::sample is not fully implemented!");
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
            "Phong[\n"
            "  exponent = %f,\n"
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
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Phong, "phong");
NORI_NAMESPACE_END
