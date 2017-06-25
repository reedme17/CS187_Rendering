#include <nori/medium.h>
#include <nori/warp.h>
#include <nori/frame.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

/**
* \brief Diffuse / Lambertian BRDF model
*/
class HomogeneousMedium : public Medium {
public:
	HomogeneousMedium(const PropertyList &propList) {
		m_sigma_a = propList.getColor("absorption", Color3f(0.0f));
		m_sigma_s = propList.getColor("scattering", Color3f(0.0f));
		m_sigma_t = m_sigma_a + m_sigma_s;
		m_albedo = m_sigma_s / m_sigma_t;
		m_g = propList.getFloat("g", 0.0f);
	}

	Color3f eval(const MediumQueryRecord &mRec) const {
		return Color3f(1.0f);
	}

	float pdf(const MediumQueryRecord &mRec) const {
		float pdf = Warp::squareToUniformSpherePdf(mRec.wo);
		return pdf;
	}

	Color3f sample(MediumQueryRecord &mRec, const Point2f &sample) const {
		mRec.sigma_a = m_sigma_a;
		mRec.sigma_s = m_sigma_s;
		mRec.sigma_t = m_sigma_t;
		mRec.albedo = m_albedo;
		mRec.g = m_g;
		mRec.wo = Warp::squareToUniformSphere(sample);
		mRec.pdf = Warp::squareToUniformSpherePdf(mRec.wo);
		mRec.pf = 1 / (4 * M_PI);
		mRec.tr = Color3f(expf(-m_sigma_t.x() * mRec.dist), expf(-m_sigma_t.y() * mRec.dist), expf(-m_sigma_t.z() * mRec.dist));

		float cosThetaI = Frame::cosTheta(mRec.wi);
		if (cosThetaI < 0.0f) {
			mRec.n.z() = mRec.n.z() * -1;
		}

		return Color3f(1.0f);
	}

	float sampleFreePath(const Point2f &sample) const {
		float result = -log(1 - sample.x()) / m_sigma_t.minCoeff();
		return result;
	}

	float pdfFreePath(const MediumQueryRecord &mRec) const {
		float result = m_sigma_t.minCoeff() * expf(-m_sigma_t.minCoeff() * mRec.dist);
		return result;
	}

	bool isHomogeneousMedium() const { return true; }

	bool isInMedium(Point3f o) const {
		return m_mesh->getBoundingBox().contains(o, 0);
	}

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"Homogeneous Medium[\n"
			"  absorption coefficient = %f,\n"
			"  scattering coefficient = %f\n"
			"]",
			m_sigma_a, m_sigma_s);
	}


private:
	Color3f m_sigma_a; // absorption coefficient 
	Color3f m_sigma_s; // scattering coefficient 
	Color3f m_sigma_t; // extinction coefficient 
	Color3f m_albedo;  
	float m_g; // g term for phase function 

};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous");
NORI_NAMESPACE_END
