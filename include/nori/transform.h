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

#pragma once

#include <nori/common.h>
#include <nori/ray.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

NORI_NAMESPACE_BEGIN

/**
 * \brief Homogeneous coordinate transformation
 *
 * This class stores a general homogeneous coordinate tranformation, such as
 * rotation, translation, uniform or non-uniform scaling, and perspective
 * transformations. The inverse of this transformation is also recorded
 * here, since it is required when transforming normal vectors.
 */
struct Transform {
public:
    /// Create the identity transform
    Transform() : 
        m_transform(Eigen::Matrix4f::Identity()),
        m_inverse(Eigen::Matrix4f::Identity()) { }

    /// Create a new transform instance for the given matrix 
    Transform(const Eigen::Matrix4f &trafo);

    /// Create a new transform instance for the given matrix and its inverse
    Transform(const Eigen::Matrix4f &trafo, const Eigen::Matrix4f &inv) 
        : m_transform(trafo), m_inverse(inv) { }

    /// Return the underlying matrix
    const Eigen::Matrix4f &getMatrix() const {
        return m_transform;
    }

    /// Return the inverse of the underlying matrix
    const Eigen::Matrix4f &getInverseMatrix() const {
        return m_inverse;
    }

    /// Return the inverse transformation
    Transform inverse() const {
        return Transform(m_inverse, m_transform);
    }



	Transform animatedTransform(Transform endTransform, float time) const {
		Transform startTransform = m_transform;

		/* handle boundary conditions for matrix interpolation */
		bool actuallyAnimated = startTransform.getMatrix() != endTransform.getMatrix();
		float startTime(0.0f), endTime(1.0f), dt(time);
		if (!actuallyAnimated || time <= startTime) {
			return startTransform;
		}
		if (time >= endTime) {
			return endTransform;
		}

		/* decompose the transformation matrix */
		Vector3f T0, T1; 
		Eigen::Quaternion<float> R0, R1;
		Eigen::Matrix4f S0, S1;
		decompose(startTransform.getMatrix(), &T0, &R0, &S0);
		decompose(endTransform.getMatrix(), &T1, &R1, &S1);

		/* interpolate the transformation matrix*/
		// PBRT p106-107

		bool hasRotation = R0.dot(R1) < 0.9995f;
		if (!hasRotation) {
			//Transform transDiff = startTransform.getMatrix() - endTransform.getMatrix();
			//const Transform result = startTransform.getMatrix() - time * transDiff.getMatrix();
			//return result;
		}

		/* interpolate translation at dt */
		Vector3f trans = (1 - dt) * T0 + dt * T1;

		/* interpolate rotation at dt */
		Eigen::Quaternion<float> rotate = R0.slerp(dt, R1);

		/* interpolate scale at dt */
		Eigen::Matrix4f scale;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				scale(i, j) = lerp(dt, S0(i, j), S1(i, j));
			}
		}

		/* compute interpolated matrix as product of interpolated components */
		Eigen::Matrix4f rotate_Matrix = Eigen::Matrix4f::Identity(); 
		rotate_Matrix.block(0, 0, 3, 3) = rotate.toRotationMatrix();


		rotate_Matrix(0, 3) = trans.x(); rotate_Matrix(1, 3) = trans.y(); rotate_Matrix(2, 3) = trans.z();
		Transform result((rotate_Matrix * scale).normalized());
		return result;
	}

	void decompose(const Eigen::Matrix4f &m, Vector3f *T, Eigen::Quaternion<float> *Rquat, Eigen::Matrix4f *S) const {
		// PBRT p104-105 

		/* extract translation T from transformation matrix */
		*T = Vector3f(m(0, 3), m(1, 3), m(2, 3));

		/* compute new transformation matrix M without translation */
		Eigen::Matrix4f M = m;
		for (int i = 0; i < 3; i++) {
			M(i, 3) = M(3, i) = 0.0f;
		}
		M(3, 3) = 1.0f;

		/* extract rotation R from transformation matrix */
		float norm;
		int count = 0;
		Eigen::Matrix4f R = M;
		do {
			// compute next matrix Rnext in series 
			Eigen::Matrix4f Rnext;
			Eigen::Matrix4f Rit = R.transpose().inverse();
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					Rnext(i, j) = 0.5f * (R(i, j) + Rit(i, j));
				}
			}
			// compute norm of difference between R and Rnext
			norm = 0;
			for (int i = 0; i < 3; i++) {
				float n = fabsf(R(i, 0) - Rnext(i, 0)) +
						  fabsf(R(i, 1) - Rnext(i, 1)) +
						  fabsf(R(i, 2) - Rnext(i, 2));
				norm = fmax(norm, n);
			}
			R = Rnext;
		} while (count++ < 100 && norm > 0.0001);
		Eigen::Matrix3f R_3x3 = R.topLeftCorner<3, 3>();
		*Rquat = R_3x3;

		/* compute scale S using rotation and original matrix */
		*S = Eigen::Matrix4f (R.inverse() * M);

		/* flip R[1] if needed to select shortest path */
		if (R.row(0).dot(R.row(1))) {
			R.row(1) = -R.row(1);
		}
	}


    /// Concatenate with another transform
    Transform operator*(const Transform &t) const;

    /// Apply the homogeneous transformation to a 3D vector
    Vector3f operator*(const Vector3f &v) const {
        return m_transform.topLeftCorner<3,3>() * v;
    }

    /// Apply the homogeneous transformation to a 3D normal
    Normal3f operator*(const Normal3f &n) const {
        return m_inverse.topLeftCorner<3, 3>().transpose() * n;
    }

    /// Transform a point by an arbitrary matrix in homogeneous coordinates
    Point3f operator*(const Point3f &p) const {
        Vector4f result = m_transform * Vector4f(p[0], p[1], p[2], 1.0f);
        return result.head<3>() / result.w();
    }

    /// Apply the homogeneous transformation to a ray
    Ray3f operator*(const Ray3f &r) const {
        return Ray3f(
            operator*(r.o), 
            operator*(r.d), 
            r.mint, r.maxt
        );
    }

    /// Return a string representation
    std::string toString() const;
private:
    Eigen::Matrix4f m_transform;
    Eigen::Matrix4f m_inverse;
};

NORI_NAMESPACE_END
