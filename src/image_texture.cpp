
#include <nori/texture.h>
#include <nori/bitmap.h>
#include <filesystem/resolver.h>
#include <fstream>
#include <stb_image.h>
#include <gl/gl.h>


NORI_NAMESPACE_BEGIN

class ImageTexture : public Texture {
public:
	ImageTexture(const PropertyList &propList) {
		m_type = propList.getString("type", "albedo");

		filesystem::path filename =
			getFileResolver()->resolve(propList.getString("filename"));
		std::ifstream is(filename.str());
		if (is.fail())
			throw NoriException("Unable to open image file \"%s\"!", filename);

		if (filename.extension() == "exr") {
			m_texture = Bitmap(filename.str());
		}

		if (filename.extension() != "exr") {
			// Mip map : PBRT p623
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 80, 80, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			//glGenerateMipmap(GL_TEXTURE_2D);

			int width, height, n;
			data = stbi_load(filename.str().c_str(), &width, &height, &n, 3);
			if (data != NULL){
				m_texture.resize(height, width);
				for (int i = 0; i < height; i++) {
					for (int j = 0; j < width; j++) {
						int offset = (i * width + j) * n;
						Color3f pixel(*(data + offset + 0) / 255.f, 
									  *(data + offset + 1) / 255.f, 
							          *(data + offset + 2) / 255.f);
						m_texture(i, j) = pixel;
					}
				}
				stbi_image_free(data);
			}

			//float pixels[] = {
			//	0.0f, 0.0f, 0.0f,   1.0f, 1.0f, 1.0f,
			//	1.0f, 1.0f, 1.0f,   0.0f, 0.0f, 0.0f
			//};
			//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 2, 0, GL_RGB, GL_FLOAT, pixels);
			// throw NoriException("Unable to open image file of type \"%s\"!", filename.extension());
		}
	}

	Color3f sample(Point2f &uv) const {
		int x = fmax(abs(int(m_texture.rows() * uv.x())) - 1, 0);
		int y = fmax(abs(int(m_texture.cols() * uv.y())) - 1, 0);

		Color3f result = m_texture(x, y);
		return result;
	}

	virtual std::string toString() const {
		return "ImageTexture[]";
	}

private:
	unsigned char* data;
};

NORI_REGISTER_CLASS(ImageTexture, "image_texture");

NORI_NAMESPACE_END
