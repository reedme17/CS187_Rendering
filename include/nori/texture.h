#pragma once

#include <nori/object.h>
#include <nori/bitmap.h>

NORI_NAMESPACE_BEGIN

class Texture : public NoriObject {
public:

	/**
	* \brief Return the type of object (i.e. Mesh/BSDF/etc.)
	* provided by this instance
	* */
	virtual EClassType getClassType() const { return ETexture; }


	/**
	* return the albedo value at the given UV coordinate
	* */
	virtual Color3f sample(Point2f &uv) const = 0;

	Bitmap getBitmap() {
		return m_texture;
	}

	int cols() {
		return m_texture.cols();
	}

	int rows() {
		return m_texture.rows();
	}

	std::string getType() {
		return m_type;
	}



protected:
	Bitmap m_texture;
	std::string m_type;
};

NORI_NAMESPACE_END
