/*
 * vector3.h
 *
 *  Created on: May 26, 2015
 *      Author: adrien
 */

#ifndef MATH_VECTOR3_H_
#define MATH_VECTOR3_H_


#include <math.h>
#include <float.h>
#include <string.h>


#if defined(MATH_CHECK_INDEXES) && (MATH_CHECK_INDEXES == 1)
#include <assert.h>
#endif

// are two floats equal
static inline bool is_equal(const float fVal1, const float fVal2) { return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; }

// is a float is zero
static inline bool is_zero(const float fVal1) { return fabsf(fVal1) < FLT_EPSILON ? true : false; }


template <typename T>
class Matrix3;

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    Vector3<T>() {
        x = y = z = 0;
    }

    // setting ctor
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0, const T z0)
    {
        x= x0; y= y0; z= z0;
    }

    // test for equality
    bool operator ==(const Vector3<T> &v) const;

    // test for inequality
    bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -(void) const;

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const;

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const;

    // uniform scaling
    Vector3<T> operator *(const T num) const;

    // uniform scaling
    Vector3<T> operator  /(const T num) const;

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v);

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v);

    // uniform scaling
    Vector3<T> &operator *=(const T num);

    // uniform scaling
    Vector3<T> &operator /=(const T num);

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if defined(MATH_CHECK_INDEXES) && (MATH_CHECK_INDEXES == 1)
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if defined(MATH_CHECK_INDEXES) && (MATH_CHECK_INDEXES == 1)
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const;

    // multiply a row vector by a matrix, to give a row vector
    Vector3<T> operator *(const Matrix3<T> &m) const;

    // multiply a column vector by a row vector, returning a 3x3 matrix
    Matrix3<T> mul_rowcol(const Vector3<T> &v) const;

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // computes the angle between this vector and another vector
    float angle(const Vector3<T> &v2) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // check if all elements are zero
    bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON) && (fabsf(z) < FLT_EPSILON); }


    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float length(void) const;

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void  reflect(const Vector3<T> &n)
    {
        Vector3<T>        orig(*this);
        project(n);
        *this = *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v) const
    {
        return v * (*this * v)/(v*v);
    }


};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vect3f;


#define HALF_SQRT_2 0.70710678118654757f


// vector cross product
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    return temp;
}

// dot product
template <typename T>
T Vector3<T>::operator *(const Vector3<T> &v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
float Vector3<T>::length(void) const
{
    return pythagorous3(x, y, z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator *=(const T num)
{
    x*=num; y*=num; z*=num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator /=(const T num)
{
    x /= num; y /= num; z /= num;
    return *this;
}

template <typename T>
Vector3<T> &Vector3<T>::operator -=(const Vector3<T> &v)
{
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
}

template <typename T>
bool Vector3<T>::is_nan(void) const
{
    return isnan(x) || isnan(y) || isnan(z);
}

template <typename T>
bool Vector3<T>::is_inf(void) const
{
    return isinf(x) || isinf(y) || isinf(z);
}

template <typename T>
Vector3<T> &Vector3<T>::operator +=(const Vector3<T> &v)
{
    x+=v.x; y+=v.y; z+=v.z;
    return *this;
}

template <typename T>
Vector3<T> Vector3<T>::operator /(const T num) const
{
    return Vector3<T>(x/num, y/num, z/num);
}

template <typename T>
Vector3<T> Vector3<T>::operator *(const T num) const
{
    return Vector3<T>(x*num, y*num, z*num);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(const Vector3<T> &v) const
{
    return Vector3<T>(x-v.x, y-v.y, z-v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator +(const Vector3<T> &v) const
{
    return Vector3<T>(x+v.x, y+v.y, z+v.z);
}

template <typename T>
Vector3<T> Vector3<T>::operator -(void) const
{
    return Vector3<T>(-x,-y,-z);
}

template <typename T>
bool Vector3<T>::operator ==(const Vector3<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y) && is_equal(z,v.z));
}

template <typename T>
bool Vector3<T>::operator !=(const Vector3<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y) || !is_equal(z,v.z));
}

template <typename T>
float Vector3<T>::angle(const Vector3<T> &v2) const
{
    return acosf((*this)*v2) / (float)((this->length()*v2.length()));
}

// multiplication of transpose by a vector
template <typename T>
Vector3<T> Vector3<T>::operator *(const Matrix3<T> &m) const
{
    return Vector3<T>(*this * m.colx(),
                      *this * m.coly(),
                      *this * m.colz());
}

// multiply a column vector by a row vector, returning a 3x3 matrix
template <typename T>
Matrix3<T> Vector3<T>::mul_rowcol(const Vector3<T> &v2) const
{
    const Vector3<T> v1 = *this;
    return Matrix3<T>(v1.x * v2.x, v1.x * v2.y, v1.x * v2.z,
                      v1.y * v2.x, v1.y * v2.y, v1.y * v2.z,
                      v1.z * v2.x, v1.z * v2.y, v1.z * v2.z);
}

// only define for float
template float Vector3<float>::length(void) const;
template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
template float Vector3<float>::operator *(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Vector3<float>::mul_rowcol(const Vector3<float> &v) const;
template Vector3<float> &Vector3<float>::operator *=(const float num);
template Vector3<float> &Vector3<float>::operator /=(const float num);
template Vector3<float> &Vector3<float>::operator -=(const Vector3<float> &v);
template Vector3<float> &Vector3<float>::operator +=(const Vector3<float> &v);
template Vector3<float> Vector3<float>::operator /(const float num) const;
template Vector3<float> Vector3<float>::operator *(const float num) const;
template Vector3<float> Vector3<float>::operator +(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(const Vector3<float> &v) const;
template Vector3<float> Vector3<float>::operator -(void) const;
template bool Vector3<float>::operator ==(const Vector3<float> &v) const;
template bool Vector3<float>::operator !=(const Vector3<float> &v) const;
template bool Vector3<float>::is_nan(void) const;
template bool Vector3<float>::is_inf(void) const;
template float Vector3<float>::angle(const Vector3<float> &v) const;












template <typename T>
struct V2
{
    T x, y;

    // trivial ctor
    V2<T>() {
        x = y = 0;
    }

    // setting ctor
    V2<T>(const T x0, const T y0) : x(x0), y(y0) {
    }

    // function call operator
    void operator ()(const T x0, const T y0)
    {
        x= x0; y= y0;
    }

    // test for equality
    bool operator ==(const V2<T> &v) const;

    // test for inequality
    bool operator !=(const V2<T> &v) const;

    // negation
    V2<T> operator -(void) const;

    // addition
    V2<T> operator +(const V2<T> &v) const;

    // subtraction
    V2<T> operator -(const V2<T> &v) const;

    // uniform scaling
    V2<T> operator *(const T num) const;

    // uniform scaling
    V2<T> operator  /(const T num) const;

    // addition
    V2<T> &operator +=(const V2<T> &v);

    // subtraction
    V2<T> &operator -=(const V2<T> &v);

    // uniform scaling
    V2<T> &operator *=(const T num);

    // uniform scaling
    V2<T> &operator /=(const T num);

    // dot product
    T operator *(const V2<T> &v) const;

    // cross product
    T operator %(const V2<T> &v) const;

    // computes the angle between this vector and another vector
    float angle(const V2<T> &v2) const;

    // computes the angle in radians between the origin and this vector
    T angle(void) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    T   length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float           length(void) const;

    // normalizes this vector
    void    normalize()
    {
        *this/=length();
    }

    // returns the normalized vector
    V2<T>  normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void    reflect(const V2<T> &n)
    {
        V2<T>        orig(*this);
        project(n);
        *this= *this*2 - orig;
    }

    // projects this vector onto v
    void    project(const V2<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    V2<T>  projected(const V2<T> &v)
    {
        return v * (*this * v)/(v*v);
    }
};

typedef V2<int16_t>        V2i;
typedef V2<uint16_t>       V2ui;
typedef V2<int32_t>        V2l;
typedef V2<uint32_t>       V2ul;
typedef V2<float>          V2f;



template <typename T>
float V2<T>::length(void) const
{
	return pythagorous2(x, y);
}


// dot product
template <typename T>
T V2<T>::operator *(const V2<T> &v) const
{
    return x*v.x + y*v.y;
}

// cross product
template <typename T>
T V2<T>::operator %(const V2<T> &v) const
{
    return x*v.y - y*v.x;
}

template <typename T>
V2<T> &V2<T>::operator *=(const T num)
{
    x*=num; y*=num;
    return *this;
}

template <typename T>
V2<T> &V2<T>::operator /=(const T num)
{
    x /= num; y /= num;
    return *this;
}

template <typename T>
V2<T> &V2<T>::operator -=(const V2<T> &v)
{
    x -= v.x; y -= v.y;
    return *this;
}

template <typename T>
bool V2<T>::is_nan(void) const
{
    return isnan(x) || isnan(y);
}

template <typename T>
bool V2<T>::is_inf(void) const
{
    return isinf(x) || isinf(y);
}

template <typename T>
V2<T> &V2<T>::operator +=(const V2<T> &v)
{
    x+=v.x; y+=v.y;
    return *this;
}

template <typename T>
V2<T> V2<T>::operator /(const T num) const
{
    return V2<T>(x/num, y/num);
}

template <typename T>
V2<T> V2<T>::operator *(const T num) const
{
    return V2<T>(x*num, y*num);
}

template <typename T>
V2<T> V2<T>::operator -(const V2<T> &v) const
{
    return V2<T>(x-v.x, y-v.y);
}

template <typename T>
V2<T> V2<T>::operator +(const V2<T> &v) const
{
    return V2<T>(x+v.x, y+v.y);
}

template <typename T>
V2<T> V2<T>::operator -(void) const
{
    return V2<T>(-x,-y);
}

template <typename T>
bool V2<T>::operator ==(const V2<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y));
}

template <typename T>
bool V2<T>::operator !=(const V2<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y));
}

template <typename T>
float V2<T>::angle(const V2<T> &v2) const
{
    float len = this->length() * v2.length();
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = ((*this)*v2) / len;
    if (fabsf(cosv) >= 1) {
        return 0.0f;
    }
    return acosf(cosv);
}

// only define for float
template float V2<float>::length(void) const;
template float V2<float>::operator *(const V2<float> &v) const;
template float V2<float>::operator %(const V2<float> &v) const;
template V2<float> &V2<float>::operator *=(const float num);
template V2<float> &V2<float>::operator /=(const float num);
template V2<float> &V2<float>::operator -=(const V2<float> &v);
template V2<float> &V2<float>::operator +=(const V2<float> &v);
template V2<float> V2<float>::operator /(const float num) const;
template V2<float> V2<float>::operator *(const float num) const;
template V2<float> V2<float>::operator +(const V2<float> &v) const;
template V2<float> V2<float>::operator -(const V2<float> &v) const;
template V2<float> V2<float>::operator -(void) const;
template bool V2<float>::operator ==(const V2<float> &v) const;
template bool V2<float>::operator !=(const V2<float> &v) const;
template bool V2<float>::is_nan(void) const;
template bool V2<float>::is_inf(void) const;
template float V2<float>::angle(const V2<float> &v) const;


#endif /* MATH_VECTOR3_H_ */
