#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/warp.h>
#include <nori/texture.h>
#include <nori/dpdf.h>

NORI_NAMESPACE_BEGIN

/**
* \brief Simple area emitter with uniform emittance
*/
class EnvironmentalLight : public Emitter {
public:
	EnvironmentalLight(const PropertyList &propList) : m_mesh(NULL) {
		/* Emitted radiance */
		m_radiance = propList.getColor("radiance");
		m_angle = propList.getFloat("thetaA", 180) * M_PI / 180;
		m_toWorld = propList.getTransform("toWorld", Transform());
		m_toLocal = m_toWorld.inverse();
	}

	Color3f eval(const EmitterQueryRecord &lRec) const {
		if ((m_toLocal * lRec.wi).z() > cos(m_angle)) {
			if (m_texture) {
				return m_texture->sample(getPixel(lRec.wi));
			}
			return m_radiance;
		}
		else {
			return 0.0f;
		}
	}


	Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
		//Point2f s = sample;
		//size_t index = distr.sampleReuse(s.x());

		lRec.wi = m_toWorld * Warp::squareToUniformSphereCap(sample, cos(m_angle));
		lRec.pdf = Warp::squareToUniformSphereCapPdf((m_toLocal * lRec.wi), cos(m_angle));
		lRec.dist = INFINITY;
		if (lRec.pdf == 0) {
			return Color3f(0.0f);
		}
		return eval(lRec) / pdf(lRec);
	}



	float pdf(const EmitterQueryRecord &lRec) const {
		return Warp::squareToUniformSphereCapPdf((m_toLocal * lRec.wi), cos(m_angle));
		//return 1.0f / distr.getSum();
	}

	bool isEnvironmentEmitter() const { return true; }

	void setParent(NoriObject *object) {
		if (object->getClassType() != EScene)
			throw NoriException("EnvironmentalLight: must be parented to the scene!");
	}

	virtual void addChild(NoriObject *obj) override {
		if (obj->getClassType() == ETexture) {
			m_texture = static_cast<Texture *>(obj);
			//int width = m_texture->rows();
			//int height = m_texture->cols();
			//Bitmap texture = m_texture->getBitmap();

			//for (int i = 0; i < width; i++) {
			//	for (int j = 0; j < height; j++) {
			//		float weight = texture(i, j).maxCoeff();
			//		distr.append(weight);
			//	}
			//}

			//distr.normalize();
		}
		else {
			throw NoriException("EnvironmentalLight::addChild(<%s>) cannot be done!",
				classTypeName(obj->getClassType()));
		}
	}


	float sign(float x) const {
		return x < 0 ? -1.0f : 1.0f;
	}

	Point2f getPixel(Vector3f dir) const {
		float r = sqrt(dir(2) * dir(2) + dir(0) * dir(0));
		float latitude = (r < abs(dir(1))) ? acos(r / dir.norm()) * sign(dir(1)) : asin(dir(1) / dir.norm());
		float longitude = (dir(2) == 0 && dir(0) == 0) ? 0 : atan2(dir(0), dir(2));

		float uv_x = fmin(1, fmax(0, latitude / - M_PI + 0.5f));
		float uv_y = fmin(1, fmax(0, longitude / (- 2 * M_PI) + 0.5f));

		return Point2f(uv_x, uv_y);
	}



	Vector3f getDirection(Point2f xy) const {
	}




	std::string toString() const {
		return tfm::format("EnvironmentalLight[radiance=%s]", m_radiance.toString());
	}

private:
	Texture * m_texture = nullptr;
	Color3f m_radiance;
	float m_angle;
	Transform m_toWorld;
	Transform m_toLocal;
	Mesh *m_mesh;
	DiscretePDF distr;
};

NORI_REGISTER_CLASS(EnvironmentalLight, "environmental light");
NORI_NAMESPACE_END
