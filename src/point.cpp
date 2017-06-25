#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN


class PointLight : public Emitter {
public:
	PointLight(const PropertyList &propList) {
		m_power = propList.getColor("power");
		m_position = propList.getPoint3("position");
	}

	/// Evaluate the value of point light emitting 
	Color3f eval(const EmitterQueryRecord &lRec) const {
		Color3f result = m_power / (4 * M_PI * (m_position - lRec.ref).norm() * (m_position - lRec.ref).norm());
		return result;
	}

	/// get the value and parameter from xml and sample the point light emitter
	/// in this case we didn't do sampling so it return the whole value 
	Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
		/* Sampled position on the light source  */
		lRec.p = m_position;
		/* Direction vector from 'ref' to 'p'  */
		lRec.wi = m_position - lRec.ref;
		lRec.wi /= lRec.wi.norm();
		/* Distance between 'ref' and 'p'  */
		lRec.dist = (m_position - lRec.ref).norm();
		/* Solid angle density wrt. 'ref'  */
		lRec.pdf = pdf(lRec);
		return eval(lRec) / pdf(lRec);
	}

	/// Compute the probability of sampling, since we didn't do sampling, it is 1.0
	float pdf(const EmitterQueryRecord &lRec) const {
		return 1.0f;
	}

	/// Return a human-readable summary
	std::string toString() const {
		return tfm::format(
			"PointLight["
			" position %s,"
			" power %s"
			" ]",
			m_position.toString(),
			m_power.toString()
		);
	}

private:
	Color3f m_power;
	Point3f m_position;
};



NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END
