/*
 * Matrix.h
 *
 *  Created on: May 26, 2015
 *      Author: adrien
 */

#ifndef MATH_MATRIX_H_
#define MATH_MATRIX_H_

#include "vector3.h"

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3 {
public:

    // Vectors comprising the rows of the matrix
    Vector3<T>        a, b, c;

    // trivial ctor
    // note that the Vector3 ctor will zero the vector elements
    Matrix3<T>() {
    }

    // setting ctor
    Matrix3<T>(const Vector3<T> &a0, const Vector3<T> &b0, const Vector3<T> &c0) : a(a0), b(b0), c(c0) {
    }

    // setting ctor
    Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz) : a(ax,ay,az), b(bx,by,bz), c(cx,cy,cz) {
    }

    // function call operator
    void operator        () (const Vector3<T> &a0, const Vector3<T> &b0, const Vector3<T> &c0)
    {
        a = a0; b = b0; c = c0;
    }

    // test for equality
    bool operator        == (const Matrix3<T> &m)
    {
        return (a==m.a && b==m.b && c==m.c);
    }

    // test for inequality
    bool operator        != (const Matrix3<T> &m)
    {
        return (a!=m.a || b!=m.b || c!=m.c);
    }

    // negation
    Matrix3<T> operator        - (void) const
    {
        return Matrix3<T>(-a,-b,-c);
    }

    // addition
    Matrix3<T> operator        + (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a+m.a, b+m.b, c+m.c);
    }
    Matrix3<T> &operator        += (const Matrix3<T> &m)
    {
        return *this = *this + m;
    }

    // subtraction
    Matrix3<T> operator        - (const Matrix3<T> &m) const
    {
        return Matrix3<T>(a-m.a, b-m.b, c-m.c);
    }
    Matrix3<T> &operator        -= (const Matrix3<T> &m)
    {
        return *this = *this - m;
    }

    // uniform scaling
    Matrix3<T> operator        * (const T num) const
    {
        return Matrix3<T>(a*num, b*num, c*num);
    }
    Matrix3<T> &operator        *= (const T num)
    {
        return *this = *this * num;
    }
    Matrix3<T> operator        / (const T num) const
    {
        return Matrix3<T>(a/num, b/num, c/num);
    }
    Matrix3<T> &operator        /= (const T num)
    {
        return *this = *this / num;
    }

    // allow a Matrix3 to be used as an array of vectors, 0 indexed
    Vector3<T> & operator[](uint8_t i) {
        Vector3<T> *_v = &a;
#if defined(MATH_CHECK_INDEXES) && (MATH_CHECK_INDEXES == 1)
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const Vector3<T> & operator[](uint8_t i) const {
        const Vector3<T> *_v = &a;
#if defined(MATH_CHECK_INDEXES) && (MATH_CHECK_INDEXES == 1)
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // multiplication by a vector
    Vector3<T> operator         *(const Vector3<T> &v) const;

    // multiplication of transpose by a vector
    Vector3<T>                  mul_transpose(const Vector3<T> &v) const;

    // multiplication by a vector giving a V2 result (XY components)
    V2<T> mulXY(const Vector3<T> &v) const;

    // extract x column
    Vector3<T>                  colx(void) const
    {
        return Vector3<T>(a.x, b.x, c.x);
    }

    // extract y column
    Vector3<T>        coly(void) const
    {
        return Vector3<T>(a.y, b.y, c.y);
    }

    // extract z column
    Vector3<T>        colz(void) const
    {
        return Vector3<T>(a.z, b.z, c.z);
    }

    // multiplication by another Matrix3<T>
    Matrix3<T> operator *(const Matrix3<T> &m) const;

    Matrix3<T> &operator        *=(const Matrix3<T> &m)
    {
        return *this = *this * m;
    }

    // transpose the matrix
    Matrix3<T>          transposed(void) const;

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
        return a.is_nan() || b.is_nan() || c.is_nan();
    }

    // create a rotation matrix from Euler angles
    void        from_euler(float roll, float pitch, float yaw);

    // create eulers from a rotation matrix
    void        to_euler(float *roll, float *pitch, float *yaw) const;

    // apply an additional rotation from a body frame gyro vector
    // to a rotation matrix.
    void        rotate(const Vector3<T> &g);

    // apply an additional rotation from a body frame gyro vector
    // to a rotation matrix but only use X, Y elements from gyro vector
    void        rotateXY(const Vector3<T> &g);

    // apply an additional inverse rotation to a rotation matrix but
    // only use X, Y elements from rotation vector
    void        rotateXYinv(const Vector3<T> &g);

    // normalize a rotation matrix
    void        normalize(void);
};

typedef Matrix3<int16_t>                Matrix3i;
typedef Matrix3<uint16_t>               Matrix3ui;
typedef Matrix3<int32_t>                Matrix3l;
typedef Matrix3<uint32_t>               Matrix3ul;
typedef Matrix3<float>                  Matrix3f;



// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::from_euler(float roll, float pitch, float yaw)
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

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::to_euler(float *roll, float *pitch, float *yaw) const
{
    if (pitch != NULL) {
        *pitch = -asin(c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(c.y, c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(b.x, a.x);
    }
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
template <typename T>
void Matrix3<T>::rotate(const Vector3<T> &g)
{
    Matrix3<T> temp_matrix;
    temp_matrix.a.x = a.y * g.z - a.z * g.y;
    temp_matrix.a.y = a.z * g.x - a.x * g.z;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
    temp_matrix.b.x = b.y * g.z - b.z * g.y;
    temp_matrix.b.y = b.z * g.x - b.x * g.z;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
    temp_matrix.c.x = c.y * g.z - c.z * g.y;
    temp_matrix.c.y = c.z * g.x - c.x * g.z;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
template <typename T>
void Matrix3<T>::rotateXY(const Vector3<T> &g)
{
    Matrix3<T> temp_matrix;
    temp_matrix.a.x = -a.z * g.y;
    temp_matrix.a.y = a.z * g.x;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
    temp_matrix.b.x = -b.z * g.y;
    temp_matrix.b.y = b.z * g.x;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
    temp_matrix.c.x = -c.z * g.y;
    temp_matrix.c.y = c.z * g.x;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}

// apply an additional inverse rotation to a rotation matrix but
// only use X, Y elements from rotation vector
template <typename T>
void Matrix3<T>::rotateXYinv(const Vector3<T> &g)
{
    Matrix3<T> temp_matrix;
    temp_matrix.a.x =   a.z * g.y;
    temp_matrix.a.y = - a.z * g.x;
    temp_matrix.a.z = - a.x * g.y + a.y * g.x;
    temp_matrix.b.x =   b.z * g.y;
    temp_matrix.b.y = - b.z * g.x;
    temp_matrix.b.z = - b.x * g.y + b.y * g.x;
    temp_matrix.c.x =   c.z * g.y;
    temp_matrix.c.y = - c.z * g.x;
    temp_matrix.c.z = - c.x * g.y + c.y * g.x;

    (*this) += temp_matrix;
}

/*
  re-normalise a rotation matrix
*/
template <typename T>
void Matrix3<T>::normalize(void)
{
    float error = a * b;
    Vector3<T> t0 = a - (b * (0.5f * error));
    Vector3<T> t1 = b - (a * (0.5f * error));
    Vector3<T> t2 = t0 % t1;
    a = t0 * (1.0f / t0.length());
    b = t1 * (1.0f / t1.length());
    c = t2 * (1.0f / t2.length());
}

// multiplication by a vector
template <typename T>
Vector3<T> Matrix3<T>::operator *(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z,
                      c.x * v.x + c.y * v.y + c.z * v.z);
}

// multiplication by a vector, extracting only the xy components
template <typename T>
V2<T> Matrix3<T>::mulXY(const Vector3<T> &v) const
{
    return V2<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z);
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Matrix3<T>::mul_transpose(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + b.x * v.y + c.x * v.z,
                      a.y * v.x + b.y * v.y + c.y * v.z,
                      a.z * v.x + b.z * v.y + c.z * v.z);
}

// multiplication by another Matrix3<T>
template <typename T>
Matrix3<T> Matrix3<T>::operator *(const Matrix3<T> &m) const
{
    Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
                                a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
                                a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
                     Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
                                b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
                                b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
                     Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
                                c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
                                c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
    return temp;
}

template <typename T>
Matrix3<T> Matrix3<T>::transposed(void) const
{
    return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
                      Vector3<T>(a.y, b.y, c.y),
                      Vector3<T>(a.z, b.z, c.z));
}

template <typename T>
void Matrix3<T>::zero(void)
{
    a.x = a.y = a.z = 0;
    b.x = b.y = b.z = 0;
    c.x = c.y = c.z = 0;
}


// only define for float
template void Matrix3<float>::zero(void);
template void Matrix3<float>::rotate(const Vector3<float> &g);
template void Matrix3<float>::rotateXY(const Vector3<float> &g);
template void Matrix3<float>::rotateXYinv(const Vector3<float> &g);
template void Matrix3<float>::normalize(void);
template void Matrix3<float>::from_euler(float roll, float pitch, float yaw);
template void Matrix3<float>::to_euler(float *roll, float *pitch, float *yaw) const;
template Vector3<float> Matrix3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Matrix3<float>::mul_transpose(const Vector3<float> &v) const;
template Matrix3<float> Matrix3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Matrix3<float>::transposed(void) const;
template V2<float> Matrix3<float>::mulXY(const Vector3<float> &v) const;



#endif /* MATH_MATRIX_H_ */
