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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
	// PBRT p777
	float r = std::sqrt(sample.y());
	float theta = 2 * M_PI * sample.x();

	return Point2f(
		r * std::cos(theta),
		r * std::sin(theta));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
	if (p.squaredNorm() < 1.f) {
		return 1 / M_PI;
	}
	else{
		return 0.0;
	}
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
	// PBRT p776
	float z = 1 - 2 * sample.y();
	float r = std::sqrt(fmax(0.0f, 1 - z * z));
	float phi = 2 * M_PI * sample.x();
	return Vector3f(r * std::cos(phi), r*std::sin(phi), z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
	return 1 / (4 * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
	// PBRT p775
	float z = sample.y();
	float r = std::sqrt(fmax(0.0f, 1 - z * z));
	float phi = 2 * M_PI * sample.x();
	return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
	if (v.z() > 0.0f) {
		return 1 / (2 * M_PI);
	}
	else {
		return 0.0;
	}
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
	float cosTheta = (1.0f - sample.y()) + sample.y() * cosThetaMax;
	float sinTheta = std::sqrt(fmax(1.0f - cosTheta * cosTheta, 0.0f));
	float phi = sample.x() * 2 * M_PI;
	return Vector3f(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta, cosTheta);
}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
	if (v.z() > cosThetaMax) {
		return 1 / (2 * M_PI * (1 - cosThetaMax));
	}
	else {
		return 0.0;
	}
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
	// PBRT p780
	Point2f d = squareToUniformDisk(sample);
	float z = std::sqrt(std::fmax(0.0f, 1 - d.x() * d.x() - d.y() * d.y()));
	return Vector3f(d.x(), d.y(), z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
	if (v.z() > 0.0f) {
		return (v.z() / std::sqrt(v.squaredNorm())) / M_PI;
	}
	else {
		return 0.0;
	}
}

Vector3f Warp::squareToCosinePowerHemisphere(const Point2f &sample, float n) {
	float cosTheta = std::pow(sample.y(), 1/(n+1));
	float phi = 2 * M_PI * sample.x();
	float r = std::sqrt(fmax(0.0f, 1.0f - cosTheta * cosTheta));
	return Vector3f(r * std::cos(phi), r*std::sin(phi), cosTheta);
}

float Warp::squareToCosinePowerHemispherePdf(const Vector3f& v, float n) {
	if (v.z() > 0.0f) {
		float cosTheta = (v.z() / std::sqrt(v.squaredNorm()));
		return ((n + 1) / (2 * M_PI)) * std::pow(cosTheta, n);
	}
	else {
		return 0.0;
	}
}

Point2f Warp::squareToUniformTriangle(const Point2f &sample) {
	// PBRT p782
	float su0 = std::sqrt(sample.y());
	return Point2f(1 - su0, sample.x() * su0);
}

float Warp::squareToUniformTrianglePdf(const Point2f &sample) {
	if (sample.y() <= 1-sample.x()) {
		return 2.0f;
	}
	else {
		return 0.0f;
	}
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
	// PBRT p809
	float logSample = log(sample.y());
	if (std::isinf(logSample)) logSample = 0;
	float tan2Theta = -alpha * alpha * logSample;
	float phi = sample.x() * 2 * M_PI;
	float cosTheta = 1 / sqrt(1 + tan2Theta);
	float sinTheta = sqrt(fmax(0.0f, 1.0f - cosTheta * cosTheta));
	return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

Vector3f Warp::squareToBeckmann2(const Point2f &sample, float alphaX, float alphaY) {
	// PBRT p809
	float logSample = log(sample.y());
	if (std::isinf(logSample)) logSample = 0;
	float tan2Theta = -alphaX * alphaY * logSample;
	float phi = sample.x() * 2 * M_PI;
	float cosTheta = 1 / sqrt(1 + tan2Theta);
	float sinTheta = sqrt(fmax(0.0f, 1.0f - cosTheta * cosTheta));
	return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
	if (m.z() > 0.0f) {
		float tan2Theta = Frame::tanTheta(m) * Frame::tanTheta(m);
		if (std::isinf(tan2Theta)) return 0;
		float cos4Theta = Frame::cosTheta(m) * Frame::cosTheta(m) *
						  Frame::cosTheta(m) * Frame::cosTheta(m);
		float D_wh = std::exp(-tan2Theta / (alpha * alpha)) /
						(M_PI * alpha * alpha * cos4Theta);
		return D_wh * fabsf(Frame::cosTheta(m));
	}
	else {
		return 0.0;
	}
}

float Warp::squareToBeckmannPdf2(const Vector3f &m, float alphaX, float alphaY) {
	if (m.z() > 0.0f) {
		float tan2Theta = Frame::tanTheta(m) * Frame::tanTheta(m);
		if (std::isinf(tan2Theta)) return 0;
		float cos4Theta = Frame::cosTheta(m) * Frame::cosTheta(m) *
			Frame::cosTheta(m) * Frame::cosTheta(m);
		float D_wh = std::exp(-tan2Theta / (alphaX * alphaY)) /
			(M_PI * alphaX * alphaY * cos4Theta);
		return D_wh * fabsf(Frame::cosTheta(m));
	}
	else {
		return 0.0;
	}
}



Vector3f Warp::squareToHenyeyGreenstein(const Point2f &sample, float g) {
	// PBRT p899
	float cosTheta;
	if (fabs(g) < 1e-3) {
		cosTheta = 1 - 2 * sample.y();
	}
	else {
		float sqrTerm = (1 - g*g) / (1 - g + 2 * g*sample.y());
		cosTheta = (1 + g*g - sqrTerm*sqrTerm) / (2 * g);
	}
	float sinTheta = std::sqrt(fmax(0.0f, 1.0f - cosTheta * cosTheta));
	float phi = sample.x() * 2 * M_PI;

	return Vector3f(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);
}

float Warp::squareToHenyeyGreensteinPdf(const Vector3f& v, float g) {
	// PBRT p681
	float cosTheta = v.z();
	float pdf = (1 - g*g) / (4 * M_PI * powf((1 + g*g - 2 * g * cosTheta), 1.5));
	return pdf;
}


NORI_NAMESPACE_END
