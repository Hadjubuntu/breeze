/*
 * Math.h
 *
 *  Created on: 6 sept. 2014
 *      Author: hadjmoody
 */


#include <math.h>
#include "Common.h"

#ifndef MATH_H_
#define MATH_H_

#ifndef abs
#define abs(val) ((val) < 0 ? -(val) : (val))
#endif

// Already defined in Arduino.h  #define PI     3.14159265f

// Earth gravity (m/s^2)
#define G_MASS 9.81

// Earth radius in meters near 45° nord
#define R 6378100.0f
#define GPS_AROUND_LATITUDE 2.1 // Latitude around we are flying in degrees

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

#define RAD2DEG 57.29578
#define DEG2RAD 0.01745329

//------------------------------------------------
// Structures


/**
 * Arduino int is 8bits, double precision has no more than 6-7 digits,
 * therefore we use int 32 bits for precision in latitude and longitude
 */
typedef struct T_GEOPOSITION {
	int32_t lat, lon; // in degrees with 10^7 multiplier
	double alt ; // in meters
	long time; // Time in micros
} GeoPosition ;

GeoPosition geoPositionCopy(GeoPosition origin) {
	GeoPosition copy;
	copy.lat = origin.lat;
	copy.lon = origin.lon;
	copy.alt = origin.alt;
	copy.time = origin.time;

	return copy;
}

double pow2(double a) {
	return a*a;
}

// 2D vector length
float pythagorous2(float a, float b) {
	return sqrt(pow2(a)+pow2(b));
}


double toDeg(double v) {
	return (v * 180.0 / M_PI);
}

double toRad(double v) {
	return (v * M_PI / 180.0);
}


// Earth projection near 2.1 latitude degree
double _lonToX(int32_t lon, int32_t latitude) {
	double lonDouble = (double)(lon * 1.0e-7f);
	double latDouble = (double)(latitude * 1.0e-7f);
	return toRad(R*cos(toRad(latDouble)) *(lonDouble));
}
double _latToY(int32_t lat) {
	double latDouble = (double)(lat*1.0e-7f);
	return toRad(R*(latDouble));
}



typedef struct T_VECTOR2 {
	double x;
	double y;
} Vector2;

Vector2 vCopy(Vector2 C) {
	Vector2 P;
	P.x = C.x;
	P.y = C.y;

	return P;
}


Vector2 geoPositionToVector2(GeoPosition loc) {
	Vector2 pos;
	pos.x = _lonToX(loc.lon, loc.lat);
	pos.y = _latToY(loc.lat);
	return pos;
}

Vector2 vect2Diff(Vector2 A, Vector2 B) {
	Vector2 C;
	C.x = B.x - A.x;
	C.y = B.y - A.y;
	return C;
}
//Vector2 getVector2(double x, double y) {
//	Vector2 v;
//	v.x = x;
//	v.y = y;
//	return v;
//}
Vector2 vect2Add(Vector2 A, Vector2 B) {
	Vector2 C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	return C;
}
double vect2CrossProduct(Vector2 A, Vector2 B) {
	return A.x * B.y - B.x * A.y;
}
double vect2DotProduct(Vector2 A, Vector2 B) {
	return A.x*B.x + A.y*B.y;
}

double vect2Norm(Vector2 A) {
	return pythagorous2(A.x, A.y);
}

void normalize(Vector2 *A) {
	double norm = vect2Norm(*A);
	if (norm > 0) {
		(*A).x = (*A).x / norm;
		(*A).y = (*A).y / norm;
	}
}

Vector2 getNormalized(Vector2 A) {
	Vector2 C = vCopy(A);
	normalize(&C);
	return C;
}


// Return angle directed from B to A (+)
double getAngle(Vector2 A, Vector2 B) {
	Vector2 An = vCopy(A);
	Vector2 Bn = vCopy(B);
	normalize(&An);
	normalize(&Bn);
	return toDeg(atan2(vect2CrossProduct(An,Bn), vect2DotProduct(An,Bn)));
}

Vector2 vect2ScalarMultiply(double K, Vector2 v) {
	Vector2 C = vCopy(v);
	C.x = C.x * K;
	C.y = C.y * K;
	return C;
}


void integerToArray(int *output, int nbElements, long value) {
	for (int i = nbElements-1; i >= 0; i--) {
		output[i] = value % 10;
		value /= 10;
	}
}

// Return the load factor corresponding to current roll in degrees
double loadFactor(double rollDeg) {
	double cosRoll = cos(toRad(rollDeg));
	if (abs(cosRoll) < 0.1) {
		cosRoll = 0.1;
	}

	return 1.0/cosRoll;
}


//-------------------------------------------
// Angle between vect(dx,dy) and north geometrically
// Its means vect dy = 1, dx = 0, angle = 0
double angleBearingToNorthDeg(double dy, double dx) {
	double angleRad;

	if (dy < 0.0 && dx < 0.0) {
		angleRad = atan2(-dy, -dx) + M_PI/2.0;
	}
	else {
		angleRad = atan2(dy, dx);
		angleRad = angleRad - M_PI / 2.0;
	}

	return angleRad * 180.0 / M_PI;
}


//-------------------------------------------
// Converts any double data into centi-int
// Used for data compression with consistency
int toCenti(double v) {
	return (int)(v*100);
}

long toPow6(double input) {
	return (long)(input * 1000000L);
}

int isign(int a) {
	if (a > 0) {
		return 1;
	}
	else {
		return -1;
	}
}

#define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
// fast_atan2 - faster version of atan2
//      126 us on AVR cpu vs 199 for regular atan2
//      absolute error is < 0.005 radians or 0.28 degrees
//      origin source: https://gist.github.com/volkansalma/2972237/raw/
float fast_atan2(float y, float x)
{
	if (x == 0.0f) {
		if (y > 0.0f) {
			return FAST_ATAN2_PIBY2_FLOAT;
		}
		if (y == 0.0f) {
			return 0.0f;
		}
		return -FAST_ATAN2_PIBY2_FLOAT;
	}
	float atan;
	float z = y/x;
	if (fabs( z ) < 1.0f) {
		atan = z / (1.0f + 0.28f * z * z);
		if (x < 0.0f) {
			if (y < 0.0f) {
				return atan - PI;
			}
			return atan + PI;
		}
	} else {
		atan = FAST_ATAN2_PIBY2_FLOAT - (z / (z * z + 0.28f));
		if (y < 0.0f) {
			return atan - PI;
		}
	}
	return atan;
}

#define SQRT_MAGIC_F 0x5f3759df
float  fast_sqrt2(const float x)
{
	const float xhalf = 0.5f*x;

	union // get bits for floating value
	{
		float x;
		int i;
	} u;
	u.x = x;
	u.i = SQRT_MAGIC_F - (u.i >> 1);  // gives initial guess y0
	return x*u.x*(1.5f - xhalf*u.x*u.x);// Newton step, repeating increases accuracy
}


// Vector3
//-------------------------------------------------------
typedef struct T_VECTOR3 {
	float x;
	float y;
	float z;
} Vector3f;

float approx(float v) {
	float k = 100.0;
	return roundf(v * k) / k;
}

Vector3f vect3fInstance(float x, float y, float z) {
	Vector3f res;
	res.x = x;
	res.y = y;
	res.z = z;

	return res;
}

Vector3f vectAbsBounded(Vector3f inputVector, float max_x, float max_y, float max_z) {
	Vector3f output;

	output.x = constrain(inputVector.x, -max_x, max_x);
	output.y = constrain(inputVector.y, -max_y, max_y);
	output.z = constrain(inputVector.z, -max_z, max_z);

	return output;
}

// Skeleton rotation frame function (internal sub-routine)
Vector3f _rot_frame_efbf_bfef(bool ef_bf, Vector3f frame_vector, Attitude *att);

/**
 * Transforms vector in earth-frame into body-frame vector
 * Angles are inversed to make the rotation inverse.
 *
 * See right-rule : https://www.evl.uic.edu/ralph/508S98/coordinates.html
 */
Vector3f rot_ef_bf(Vector3f ef_vector, Attitude *att) {
	Vector3f output = _rot_frame_efbf_bfef(true, ef_vector, att);
	return output;
}

Vector3f rot_bf_ef(Vector3f bf_vector, Attitude *att) {
	Vector3f output = _rot_frame_efbf_bfef(false, bf_vector, att);
	return output;
}

Vector3f _rot_frame_efbf_bfef(bool ef_bf, Vector3f frame_vector, Attitude *att) {
	Vector3f output;

	// Earth-frame to body-frame, rotation normal, otherwise
	// inversed rotation
	int sign = 1;
	if (ef_bf) {
		sign = 1;
	}
	else {
		sign = -1;
	}

	float cos_p = cos(sign * toRad(att->pitch));
	float cos_r = cos(sign * (-toRad(att->roll)));
	float sin_p = sin(sign * toRad(att->pitch));
	float sin_r = sin(sign * (-toRad(att->roll)));

	output.x = cos_p * frame_vector.x + sin_p * sin_r * frame_vector.y + cos_r*sin_p * frame_vector.z;
	output.y = cos_r  * frame_vector.y - sin_r * frame_vector.z;
	output.z = -sin_p * frame_vector.x + cos_p * sin_r * frame_vector.y + cos_p * cos_r * frame_vector.z;

	return output;
}

/**
 * Fast sinus function
 */
float fast_sin(float x) {
	const float B = 4/PI;
	const float C = -4/(PI*PI);

	float y = B * x + C * x * abs(x);

	const float P = 0.225;

	y = P * (y * abs(y) - y) + y;   // Q * y + P * y * abs(y)

	return y;
}

float fast_cos(float x) {
	return fast_sin(x + PI/2.0);
}


// TODO Use fast_atan2 if not enough fast for CPU ?
// Fast vector acceleration to roll conversion
float vectAccelToRoll(Vector3f acc3f) {
	return atan2(acc3f.y, acc3f.z);
}

// Fast vector acceleration to pitch conversion
float vectAccelToPitch(Vector3f acc3f) {
	return atan2(acc3f.x, pythagorous2(acc3f.z, acc3f.y));
}

#endif /* MATH_H_ */
