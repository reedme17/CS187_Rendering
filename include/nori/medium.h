#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct MediumQueryRecord {

	/// Pointer to the sampled medium
	const Medium *medium;

	/// Origin point 
	Point3f ref;
	/// Point associated with the point (phase function)
	Point3f p;
	/// Associated surface normal
	Normal3f n;
	/// Solid angle density wrt. 'ref'
	float pdf;
	/// Direction vector from 'ref' to 'p'
	Vector3f wi;
	/// Outgoing direction (in the local frame)
	Vector3f wo;
	/// Distance between 'ref' and 'p'
	float dist;

	Color3f sigma_a; // absorption coefficient 
	Color3f sigma_s; // scattering coefficient 
	Color3f sigma_t; // extinction coefficient 
	Color3f albedo; // extinction coefficient 

	float pf; // phase function 
	float g;  // g value for phase function 
	Color3f tr; // transmittance

	/// Additional information possibly needed by the BSDF
	/// UV associated with the point
	Point2f uv;

	MediumQueryRecord() : medium(nullptr) { }

	MediumQueryRecord(const Point3f &ref) : ref(ref) { }

	MediumQueryRecord(const Medium *medium,
		const Point3f &ref, const Point3f &p,
		const Normal3f &n) : medium(medium), ref(ref), p(p), n(n) {
		wi = p - ref;
		dist = wi.norm();
		wi /= dist;
	}

	MediumQueryRecord(const Medium *medium, const Ray3f &ray) :
		medium(medium), ref(ray.o), p(ray(1)), n(-ray.d), wi(ray.d),
		dist(std::numeric_limits<float>::infinity()) {
	}

	/// Return a human-readable string summary
	std::string toString() const;

};


class Medium : public NoriObject {
public:
	virtual Color3f sample(MediumQueryRecord &mRec, const Point2f &sample) const = 0;

	virtual float sampleFreePath(const Point2f &sample) const = 0;

	virtual Color3f eval(const MediumQueryRecord &mRec) const = 0;

	virtual float pdf(const MediumQueryRecord &mRec) const = 0;

	virtual float pdfFreePath(const MediumQueryRecord &mRec) const = 0;

	virtual bool isHomogeneousMedium() const { return false; }

	virtual bool isInMedium(Point3f o) const { return false; }

	virtual ~Medium() {}

	virtual EClassType getClassType() const { return EMedium; }

	void setMesh(Mesh * mesh) { m_mesh = mesh; }

protected:
	Mesh * m_mesh = nullptr;
};



inline std::string MediumQueryRecord::toString() const {
	return tfm::format(
		"MediumQueryRecord[\n"
		"  medium = \"%s\",\n"
		"  ref = %s,\n"
		"  p = %s,\n"
		"  n = %s,\n"
		"  pdf = %f,\n"
		"  wi = %s,\n"
		"  dist = %f\n"
		"]",
		indent(medium->toString()),
		ref.toString(), p.toString(),
		n.toString(), pdf, wi.toString(), dist
	);
}

NORI_NAMESPACE_END
