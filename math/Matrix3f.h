/*
 * Matrix3f.h
 *
 *  Created on: Jul 12, 2015
 *      Author: adrien
 */

#ifndef MATH_MATRIX3F_H_
#define MATH_MATRIX3F_H_

#include "math/Math.h"

class Matrix3f {
public:

	// Vectors comprising the rows of the matrix
	Vector3f        a, b, c;

	// trivial constructor
	Matrix3f() {
	}

	// setting ctor
	Matrix3f(const Vector3f &a0, const Vector3f &b0, const Vector3f &c0) : a(a0), b(b0), c(c0) {
	}

	// setting ctor
	Matrix3f(const float ax, const float ay, const float az, const float bx, const float by, const float bz, const float cx, const float cy, const float cz) {
		a.x = ax;
		a.y = ay;
		a.z = az;
		b.x = bx;
		b.y = by;
		b.z = bz;
		c.x = cx;
		c.y = cy;
		c.z = cz;
	}

	void from_euler(float roll, float pitch, float yaw)
	{
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		a.x = cp * cy;
		a.y = (sr * sp * cy) - (cr * sy);
		a.z = (cr * sp * cy) + (sr * sy);
		b.x = cp * sy;
		b.y = (sr * sp * sy) + (cr * cy);
		b.z = (cr * sp * sy) - (sr * cy);
		c.x = -sp;
		c.y = sr * cp;
		c.z = cr * cp;
	}

	// function call operator
	void operator        () (const Vector3f &a0, const Vector3f &b0, const Vector3f &c0)
	{
		a = a0; b = b0; c = c0;
	}

	// test for equality
	bool operator        == (const Matrix3f &m)
    		{
		return (a.x == m.a.x
				&& a.y == m.a.y
				&& a.z == m.a.z
				&& b.x == m.b.x
				&& b.y == m.b.y
				&& b.z == m.b.z
				&& c.x == m.b.x
				&& c.y == m.b.y
				&& c.z == m.b.z);
    		}

	// test for inequality
	bool operator        != (const Matrix3f &m)
    		{
		return (a.x != m.a.x
				|| a.y != m.a.y
				|| a.z != m.a.z
				|| b.x != m.b.x
				|| b.y != m.b.y
				|| b.z != m.b.z
				|| c.x != m.b.x
				|| c.y != m.b.y
				|| c.z != m.b.z);
    		}

	// negation
	Matrix3f operator        - (void) const
	{
		return Matrix3f(vect3fMultiply(a, -1.0), vect3fMultiply(b, -1.0), vect3fMultiply(c, -1.0));
	}

	// addition
	Matrix3f operator        + (const Matrix3f &m) const
	{
		return Matrix3f(vect3fAdd(a, m.a), vect3fAdd(b, m.b), vect3fAdd(c, m.c));
	}
	Matrix3f &operator        += (const Matrix3f &m)
    		{
		return *this = *this + m;
    		}

	// subtraction
	Matrix3f operator        - (const Matrix3f &m) const
	{
		return Matrix3f(vect3fSubstract(a, m.a), vect3fSubstract(b, m.b), vect3fSubstract(c, m.c));
	}
	Matrix3f &operator        -= (const Matrix3f &m)
    		{
		return *this = *this - m;
    		}

	// uniform scaling
	Matrix3f operator        * (const float num) const
    		{
		return Matrix3f(vect3fMultiply(a, num), vect3fMultiply(b, num), vect3fMultiply(c, num));
    		}
	Matrix3f &operator        *= (const float num)
    		{
		return *this = *this * num;
    		}
	Matrix3f operator        / (const float num) const
    		{
		return Matrix3f(vect3fDivide(a, num), vect3fDivide(b, num), vect3fDivide(c, num));
    		}
	Matrix3f &operator        /= (const float num)
    		{
		return *this = *this / num;
    		}

	// allow a Matrix3 to be used as an array of vectors, 0 indexed
	Vector3f & operator[](uint8_t i) {
		Vector3f *_v = &a;
		return _v[i];
	}

	const Vector3f & operator[](uint8_t i) const {
		const Vector3f *_v = &a;
		return _v[i];
	}

	// multiplication by a vector
	Vector3f operator         *(const Vector3f &v) const;

	// multiplication of transpose by a vector
	Vector3f                  mul_transpose(const Vector3f &v) const;


	// extract x column
	Vector3f                  colx(void) const
	{
		return vect3fInstance(a.x, b.x, c.x);
	}

	// extract y column
	Vector3f        coly(void) const
	{
		return vect3fInstance(a.y, b.y, c.y);
	}

	// extract z column
	Vector3f        colz(void) const
	{
		return vect3fInstance(a.z, b.z, c.z);
	}

	// multiplication by another Matrix3f
	Matrix3f operator *(const Matrix3f &m) const;

	Matrix3f &operator        *=(const Matrix3f &m)
    		{
		return *this = *this * m;
    		}

	// transpose the matrix
	Matrix3f          transposed(void) const;

	void transpose(void)
	{
		*this = transposed();
	}

	// zero the matrix
	void        zero(void);

	// setup the identity matrix
	void        identity(void) {
		a.x = b.y = c.z = 1;
		a.y = a.z = 0;
		b.x = b.z = 0;
		c.x = c.y = 0;
	}

	// check if any elements are NAN
	bool        is_nan(void)
	{
		// TODO check vector is_nan function
		return false;
	}

	// create a rotation matrix from Euler angles
	void        from_euler(float roll, float pitch, float yaw);

	// create eulers from a rotation matrix
	void        to_euler(float *roll, float *pitch, float *yaw) const;

	// apply an additional rotation from a body frame gyro vector
	// to a rotation matrix.
	void        rotate(const Vector3f &g);

	// apply an additional rotation from a body frame gyro vector
	// to a rotation matrix but only use X, Y elements from gyro vector
	void        rotateXY(const Vector3f &g);

	// apply an additional inverse rotation to a rotation matrix but
	// only use X, Y elements from rotation vector
	void        rotateXYinv(const Vector3f &g);

	// normalize a rotation matrix
	void        normalize(void);
};


#endif /* MATH_MATRIX3F_H_ */
