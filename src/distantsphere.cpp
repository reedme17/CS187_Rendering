#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
* \brief Simple area emitter with uniform emittance
*/
class DistantSphere : public Emitter {
public:
	DistantSphere(const PropertyList &propList) : m_mesh(NULL) {
		/* Emitted radiance */
		m_radiance = propList.getColor("radiance");
		m_angle = propList.getFloat("thetaA", 180) * M_PI / 180;
		m_toWorld = propList.getTransform("toWorld", Transform());
		m_toLocal = m_toWorld.inverse();
	}

	Color3f eval(const EmitterQueryRecord &lRec) const {
		//throw NoriException("DistantSphere::eval is not yet implemented!");
		if ((m_toLocal * lRec.wi).z() > cos(m_angle)) {
			return m_radiance;
		}
		else {
			return 0.0f;
		}
	}

	Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
		//throw NoriException("DistantSphere::sample is not yet implemented!");
		lRec.wi = m_toWorld * Warp::squareToUniformSphereCap(sample, cos(m_angle));
		lRec.pdf = Warp::squareToUniformSphereCapPdf((m_toLocal * lRec.wi), cos(m_angle));
		lRec.dist = INFINITY;
		if (lRec.pdf == 0) {
			return Color3f(0.0f);
		}
		return eval(lRec) / pdf(lRec);
	}

	float pdf(const EmitterQueryRecord &lRec) const {
		//throw NoriException("DistantSphere::pdf is not yet implemented!");
		return Warp::squareToUniformSphereCapPdf((m_toLocal * lRec.wi), cos(m_angle));
	}

	bool isEnvironmentEmitter() const { return true; }

	void setParent(NoriObject *object) {
		if (object->getClassType() != EScene)
			throw NoriException("DistantSphere: must be parented to the scene!");
	}

	std::string toString() const {
		return tfm::format("DistantSphere[radiance=%s]", m_radiance.toString());
	}
private:
	Color3f m_radiance;
	float m_angle;
	Transform m_toWorld;
	Transform m_toLocal;
	Mesh *m_mesh;
};

NORI_REGISTER_CLASS(DistantSphere, "distantsphere");
NORI_NAMESPACE_END
