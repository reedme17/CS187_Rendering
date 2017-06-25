/*
    Area emitter skeleton
    Copyright (c) 2015 by Wenzel Jakob
*/

#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/**
 * \brief Simple area emitter with uniform emittance
 */
class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &propList) : m_mesh(NULL) {
        /* Emitted radiance */
        m_radiance = propList.getColor("radiance", Color3f(0.0f));
    }

	Color3f eval(const EmitterQueryRecord &lRec) const {
		//throw NoriException("AreaEmitter::eval is not yet implemented!");
		if (lRec.n.dot(-lRec.wi) < 0) {
			return Color3f(0.0f);
		}
		return m_radiance;
	}

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
        // throw NoriException("AreaEmitter::sample is not yet implemented!");
		m_mesh->samplePosition(sample, lRec.p, lRec.n);
		lRec.wi = lRec.p - lRec.ref;
		lRec.wi /= lRec.wi.norm();
		lRec.dist = (lRec.p - lRec.ref).norm();
		lRec.pdf = pdf(lRec);
		return eval(lRec) / pdf(lRec);
    }

    float pdf(const EmitterQueryRecord &lRec) const {
        //throw NoriException("AreaEmitter::pdf is not yet implemented!");
		return m_mesh->pdf() * lRec.dist * lRec.dist / fabs(lRec.n.dot(-lRec.wi));
    }

	void setParent(NoriObject *object) {
		if (object->getClassType() != EMesh)
			throw NoriException("AreaEmitter: attached to a non-mesh object!");
		m_mesh = static_cast<Mesh *>(object);
	}

    std::string toString() const {
        return tfm::format("AreaEmitter[radiance=%s]", m_radiance.toString());
    }
private:
    Color3f m_radiance;
    Mesh *m_mesh;
};

NORI_REGISTER_CLASS(AreaEmitter, "area");
NORI_NAMESPACE_END
