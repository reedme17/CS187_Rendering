/*
This file is part of Nori, a simple educational ray tracer

Copyright (c) 2015 by Wenzel Jakob

Nori is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License Version 3
as published by the Free Software Foundation.

Nori is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/mesh.h>
#include <nori/timer.h>
#include <filesystem/resolver.h>
#include <unordered_map>
#include <fstream>

#include <nori/bbox.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/dpdf.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

/**
* \brief Loader for Wavefront OBJ triangle meshes
*/
class MoveWavefrontOBJ : public Mesh {
public:
	MoveWavefrontOBJ(const PropertyList &propList) {
		typedef std::unordered_map<OBJVertex, uint32_t, OBJVertexHash> VertexMap;

		filesystem::path filename =
			getFileResolver()->resolve(propList.getString("filename"));

		std::ifstream is(filename.str());
		if (is.fail())
			throw NoriException("Unable to open OBJ file \"%s\"!", filename);
		Transform trafo = propList.getTransform("toWorld", Transform());
		Transform trafo2 = propList.getTransform("toWorld2", trafo);
		m_trans1 = trafo;
		m_trans2 = trafo2;

		cout << "Loading \"" << filename << "\" .. ";
		cout.flush();
		Timer timer;

		std::vector<Vector3f>   positions;
		std::vector<Vector2f>   texcoords;
		std::vector<Vector3f>   normals;
		std::vector<uint32_t>   indices;
		std::vector<OBJVertex>  vertices;
		VertexMap vertexMap;

		std::string line_str;
		while (std::getline(is, line_str)) {
			std::istringstream line(line_str);

			std::string prefix;
			line >> prefix;

			if (prefix == "v") {
				Point3f p;
				line >> p.x() >> p.y() >> p.z();
				Point3f p2;
				p2 = trafo2 * p;
				p = trafo * p;
				m_bbox.expandBy(p);
				m_bbox.expandBy(p2);
				positions.push_back(p);
			}
			else if (prefix == "vt") {
				Point2f tc;
				line >> tc.x() >> tc.y();
				texcoords.push_back(tc);
			}
			else if (prefix == "vn") {
				Normal3f n;
				line >> n.x() >> n.y() >> n.z();
				normals.push_back((trafo * n).normalized());
			}
			else if (prefix == "f") {
				std::string v1, v2, v3, v4;
				line >> v1 >> v2 >> v3 >> v4;
				OBJVertex verts[6];
				int nVertices = 3;

				verts[0] = OBJVertex(v1);
				verts[1] = OBJVertex(v2);
				verts[2] = OBJVertex(v3);

				if (!v4.empty()) {
					/* This is a quad, split into two triangles */
					verts[3] = OBJVertex(v4);
					verts[4] = verts[0];
					verts[5] = verts[2];
					nVertices = 6;
				}
				/* Convert to an indexed vertex list */
				for (int i = 0; i<nVertices; ++i) {
					const OBJVertex &v = verts[i];
					VertexMap::const_iterator it = vertexMap.find(v);
					if (it == vertexMap.end()) {
						vertexMap[v] = (uint32_t)vertices.size();
						indices.push_back((uint32_t)vertices.size());
						vertices.push_back(v);
					}
					else {
						indices.push_back(it->second);
					}
				}
			}
		}

		m_F.resize(3, indices.size() / 3);
		memcpy(m_F.data(), indices.data(), sizeof(uint32_t)*indices.size());

		m_V.resize(3, vertices.size());
		for (uint32_t i = 0; i<vertices.size(); ++i)
			m_V.col(i) = positions.at(vertices[i].p - 1);

		if (!normals.empty()) {
			m_N.resize(3, vertices.size());
			for (uint32_t i = 0; i<vertices.size(); ++i)
				m_N.col(i) = normals.at(vertices[i].n - 1);
		}

		if (!texcoords.empty()) {
			m_UV.resize(2, vertices.size());
			for (uint32_t i = 0; i<vertices.size(); ++i)
				m_UV.col(i) = texcoords.at(vertices[i].uv - 1);
		}

		m_name = filename.str();
		cout << "done. (V=" << m_V.cols() << ", F=" << m_F.cols() << ", took "
			<< timer.elapsedString() << " and "
			<< memString(m_F.size() * sizeof(uint32_t) +
				sizeof(float) * (m_V.size() + m_N.size() + m_UV.size()))
			<< ")" << endl;
	}

	void samplePosition(const Point2f &sample, Point3f &p, Normal3f &n) const {
		//throw NoriException("Mesh::samplePosition is not yet implemented!");
		Point2f s = sample;
		size_t index = distr.sampleReuse(s.x());

		// PBRT 839
		Point2f triSample = Warp::squareToUniformTriangle(s);
		float u = triSample.x(), v = triSample.y(), w = 1 - (u + v);

		const Point3f p0 = m_V.col(m_F(0, index)), p1 = m_V.col(m_F(1, index)), p2 = m_V.col(m_F(2, index));
		p = u*p0 + v*p1 + w*p2;
		//p = m_trans2 * p;

		if (m_N.size() == 0) {
			Vector3f d1 = p1 - p0;
			Vector3f d2 = p2 - p0;
			n = (d1).cross(d2);
			//n = m_trans2 * n;
			n.normalize();
		}
		else {
			const Normal3f n0 = m_N.col(m_F(0, index)), n1 = m_N.col(m_F(1, index)), n2 = m_N.col(m_F(2, index));
			n = u*n0 + v*n1 + w*n2;
			//n = m_trans2 * n;
			n.normalize();
		}
	}

	bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
		uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
		Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

		Transform temp(m_trans2.getMatrix() * m_trans1.getMatrix().inverse());
		float time = ((float)rand() / (RAND_MAX));
		Transform trans_dt = Transform(Eigen::Matrix4f::Identity()).animatedTransform(temp, ray.time);
		p0 = trans_dt * p0;
		p1 = trans_dt * p1;
		p2 = trans_dt * p2;

		/* Find vectors for two edges sharing v[0] */
		Vector3f edge1 = p1 - p0, edge2 = p2 - p0;

		/* Begin calculating determinant - also used to calculate U parameter */
		Vector3f pvec = ray.d.cross(edge2);

		/* If determinant is near zero, ray lies in plane of triangle */
		float det = edge1.dot(pvec);

		if (det > -1e-8f && det < 1e-8f)
			return false;
		float inv_det = 1.0f / det;

		/* Calculate distance from v[0] to ray origin */
		Vector3f tvec = ray.o - p0;

		/* Calculate U parameter and test bounds */
		u = tvec.dot(pvec) * inv_det;
		if (u < 0.0 || u > 1.0)
			return false;

		/* Prepare to test V parameter */
		Vector3f qvec = tvec.cross(edge1);

		/* Calculate V parameter and test bounds */
		v = ray.d.dot(qvec) * inv_det;
		if (v < 0.0 || u + v > 1.0)
			return false;

		/* Ray intersects triangle -> compute t */
		t = edge2.dot(qvec) * inv_det;

		return t >= ray.mint && t <= ray.maxt;
	}

	BoundingBox3f getBoundingBox(uint32_t index) const {
		BoundingBox3f result(m_V.col(m_F(0, index)));
		result.expandBy(m_V.col(m_F(1, index)));
		result.expandBy(m_V.col(m_F(2, index)));

		Transform temp(m_trans2.getMatrix() * m_trans1.getMatrix().inverse());
		uint32_t i0 = m_F(0, index), i1 = m_F(1, index), i2 = m_F(2, index);
		Point3f p0 = m_V.col(i0), p1 = m_V.col(i1), p2 = m_V.col(i2);

		p0 = temp * p0;
		p1 = temp * p1;
		p2 = temp * p2;
		result.expandBy(p0);
		result.expandBy(p1);
		result.expandBy(p2);

		return result;
	}

	Point3f getCentroid(uint32_t index) const {
		Point3f result((1.0f / 3.0f) *
			(m_V.col(m_F(0, index)) +
				m_V.col(m_F(1, index)) +
				m_V.col(m_F(2, index))));
		return result;
	}




protected:
	/// Vertex indices used by the OBJ format
	struct OBJVertex {
		uint32_t p = (uint32_t)-1;
		uint32_t n = (uint32_t)-1;
		uint32_t uv = (uint32_t)-1;

		inline OBJVertex() { }

		inline OBJVertex(const std::string &string) {
			std::vector<std::string> tokens = tokenize(string, "/", true);

			if (tokens.size() < 1 || tokens.size() > 3)
				throw NoriException("Invalid vertex data: \"%s\"", string);

			p = toUInt(tokens[0]);

			if (tokens.size() >= 2 && !tokens[1].empty())
				uv = toUInt(tokens[1]);

			if (tokens.size() >= 3 && !tokens[2].empty())
				n = toUInt(tokens[2]);
		}

		inline bool operator==(const OBJVertex &v) const {
			return v.p == p && v.n == n && v.uv == uv;
		}
	};

	/// Hash function for OBJVertex
	struct OBJVertexHash : std::unary_function<OBJVertex, size_t> {
		std::size_t operator()(const OBJVertex &v) const {
			size_t hash = std::hash<uint32_t>()(v.p);
			hash = hash * 37 + std::hash<uint32_t>()(v.uv);
			hash = hash * 37 + std::hash<uint32_t>()(v.n);
			return hash;
		}
	};
};

NORI_REGISTER_CLASS(MoveWavefrontOBJ, "move_obj");
NORI_NAMESPACE_END
