/*
 * Quaternion.h
 *
 *  Created on: 04/12/2012
 *      Author: Ignacio Mellado-Bataller
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "cvg_types.h"
#include "Vector3.h"
#include "Vector4.h"
#include "RotMatrix3.h"
#include "cvgString.h"
#include <iostream>

class Quaternion : public virtual Vector4
{
public:
	Quaternion();
	Quaternion(const Vector3 &axis, cvg_double angle);
	Quaternion(const RotMatrix3 &rotm);
	Quaternion(cvg_double x, cvg_double y, cvg_double z, cvg_double w);
	Quaternion(const Quaternion &q);
	virtual ~Quaternion();

	void set(const Vector3 &axis, cvg_double angle);
	Vector3 getAxis() const;
	cvg_double getAngle() const;
	RotMatrix3 getRotMatrix() const;

	Quaternion normalize() const;
	Quaternion operator + (const Quaternion &q) const;
	Quaternion operator - (const Quaternion &q) const;
	Quaternion operator - () const;
	cvg_double dot(const Quaternion &q) const;
	Quaternion hProduct(const Quaternion &q) const;	// Hamilton product
	Quaternion exp() const;
	Quaternion log() const;
	Quaternion conj() const;
	Quaternion operator / (cvg_double f) const;
	Quaternion operator * (cvg_double f) const;
	cvg_double operator * (const Quaternion &q) const;
	Quaternion inverse() const;
	cvg_double norm() const;

	cvgString toString() const;
};

inline Quaternion operator * (cvg_double left, const Quaternion &right) { return right * left; }
std::ostream &operator << (std::ostream &out, const Quaternion &q);

#endif /* QUATERNION_H_ */
