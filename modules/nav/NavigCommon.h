/*
 * NavigCommon.h
 *
 *  Created on: Jan 26, 2015
 *      Author: adrien
 */

#ifndef NAVIGCOMMON_H_
#define NAVIGCOMMON_H_

#include "math/Math.h"

#define DIST_TO_WAYPOINT_FOR_VALIDATION 6 // 2 meters for ground plan, 10 meters for real flight

double geoDistance(GeoPosition pos1, GeoPosition pos2) {
	double dlon = (pos2.lon - pos1.lon)*1e-7;
	double dlat = (pos2.lat - pos1.lat)*1e-7;
	double a = pow2(sin(toRad(dlat/2))) + cos(toRad(pos1.lat*1e-7)) * cos(toRad(pos2.lat*1e-7)) * pow2(sin(toRad(dlon/2)));
	double c = 2 * atan2( sqrt(a), sqrt(1-a) );
	return R * c;
}


#endif /* NAVIGCOMMON_H_ */
