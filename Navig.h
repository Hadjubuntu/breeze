#include "Math.h"
#include "Common.h"
#include "FilterAverage.h"

#ifndef NAVIG_H_
#define NAVIG_H_



//------------------------------------------------
// Variables
long lastGoodUpdate = 0;

#define DIST_TO_WAYPOINT_FOR_VALIDATION 6 // 2 meters for ground plan, 10 meters for real flight

int NB_WAYPOINTS = 1;
GeoPosition wps[] = {{489183946, 21388276, 0.5, 0},
};

GeoPosition landingPosition = {489183946, 21388276, 0.5, 0};



// GPS states
#define GPS_STATE_ON_TRACK 1
#define GPS_STATE_LOST 2
#define GPS_STATE_LANDING 3

// Naviguation configuration
#define GPS_TIME_OUT 10000000L // 10 seconds of no new data
#define GPS_MAX_DISTANCE 5000 // If a waypoint is 5 km, mission aborted
#define _L1_damping 0.70f
#define _L1_period 25
#define MIN_ALTITUDE_TOLERANCE 20
#define ALTITUDE_TOLERANCE 4 // At altitude tolerance, altitude acceptable tolerance
#define HOLD_HEADING_IN_TAKEOFF_AND_LANDING 0

// Parameters
//--------------------------------------------

// Method navigation (0 means L1 navigation, 1 means using heading cap GPS)
int NAVIG_METHOD_ANGLE_DIFF = 1;


int currentWP = 0 ;
bool WP_on_target = false;
double distance = 0.0;
double previousDistance = 0.0;
int GPSState = GPS_STATE_ON_TRACK;
bool MissionDone = false;
bool modeGoHome = false;
bool takeoffPositionSet = false;
long lastUpdateHeading = 0;
long gpsLastUpdateFixtime = 0;
GeoPosition takeoffPosition;
GeoPosition currentPosition;
GeoPosition previousPosition;
GeoPosition estimatedPosition;

// Heading, current heading and heading degrees to hold
// in takeoff and landing phase
double currentHeading = 0.0; // Current angle heading to north (degrees)
double landingHoldHeading;
double takeoffHoldHeading;

// Angle between direction and waypoint
double angleDiff = 0.0;
double gpsRollDesired = 0.0;
double gpsPitchDesired = 0.0;
int glitchIter = 0 ;
int iterInsertGPSData = 0;
// Method 2 bearing
double angleBearing = 0.0;
double gpsRollDesired2 = 0.0;
double gpsPitchDesired2 = 0.0;

// Functions skeleton
//-----------------------------------------
void resetWPdata();
void updateHeading(FilterAverage *);
void updateDistance();
double geoDistance(GeoPosition, GeoPosition);


// Initialize naviguation
//------------------------------------------
void initNavig() {
	if (HOLD_HEADING_IN_TAKEOFF_AND_LANDING) {
		// Evaluate angle to hold when landing
		GeoPosition beforeLandingPosition = wps[NB_WAYPOINTS-1];

		// Check if landing position and before landing position are apart (more than 3 meters)
		if (geoDistance(beforeLandingPosition, landingPosition) < 3.0) {
			if (NB_WAYPOINTS >= 2) {
				beforeLandingPosition = wps[NB_WAYPOINTS-2];
			}
			else {
				Serial.println("Error, not enough waypoints to have a descent before landing position.");
			}
		}


		Vector2 vect2LandingPos = geoPositionToVector2(landingPosition);
		Vector2 vect2BeforeLandingPos = geoPositionToVector2(beforeLandingPosition);
		landingHoldHeading = angleBearingToNorthDeg(vect2LandingPos.y - vect2BeforeLandingPos.y, vect2LandingPos.x - vect2BeforeLandingPos.x);

		Serial.print("To land, UAV will maintain ");
		Serial.print(landingHoldHeading);
		Serial.println(" degrees heading");
	}
}

bool isGPSDataRelevant(long cTime) {
	return ((cTime - currentPosition.time) < GPS_TIME_OUT);
}

double geoDistance(GeoPosition pos1, GeoPosition pos2) {
	double dlon = (pos2.lon - pos1.lon)*1e-7;
	double dlat = (pos2.lat - pos1.lat)*1e-7;
	double a = pow2(sin(toRad(dlat/2))) + cos(toRad(pos1.lat*1e-7)) * cos(toRad(pos2.lat*1e-7)) * pow2(sin(toRad(dlon/2)));
	double c = 2 * atan2( sqrt(a), sqrt(1-a) );
	return R * c;
}


GeoPosition getGeoPositionEstimation(long cTime, bool *hasBeenUpdate) {
	(*hasBeenUpdate) = false;
	if (iterInsertGPSData < 2 || (currentPosition.time - previousPosition.time) <= 0.0) {
		return currentPosition;
	}
	else {
		(*hasBeenUpdate) = true;

		GeoPosition estim;

		int32_t dlat = currentPosition.lat-previousPosition.lat;
		int32_t dlon = currentPosition.lon-previousPosition.lon;
		double dz = currentPosition.alt-previousPosition.alt;

		double dtFactor = (cTime-currentPosition.time) / (currentPosition.time - previousPosition.time);
		estim.lat = (int32_t) (currentPosition.lat + dlat * dtFactor) ;
		estim.lon = (int32_t) (currentPosition.lon + dlon * dtFactor);
		estim.alt = currentPosition.alt + dz * dtFactor;
		estim.time = cTime ;

		return estim;
	}
}

Vector2 getGroundSpeedVect2() {
	Vector2 cPos = geoPositionToVector2(currentPosition);
	Vector2 pPos = geoPositionToVector2(previousPosition);

	return vect2ScalarMultiply(1.0/((currentPosition.time-previousPosition.time)/S_TO_US) , vect2Diff(pPos, cPos));
}

// Procedure to define takeoff as the last waypoint and then land
void returnHome() {
	currentWP = NB_WAYPOINTS - 1;
	if (landingPosition.lat == 0.0) {
		wps[currentWP].lat = takeoffPosition.lat;
		wps[currentWP].lon = takeoffPosition.lon;
	}
	else {
		wps[currentWP].lat = landingPosition.lat;
		wps[currentWP].lon = landingPosition.lon;
	}
	wps[currentWP].alt = 1;

	modeGoHome = true;
	resetWPdata();
	updateDistance();
}

bool checkGPSPosition(long cTime, double pLat, double pLon, double pAlt) {
	// If we use the protection against glitch gps data
	if (GPS_GLITCH_PROTECTION) {
		bool allOk = false;
		// If first position, then accept it
		if (takeoffPositionSet == false) {
			allOk = true;
		}
		// Accept also accept new position if we haven't update any since 1 second
		else if ((cTime - lastGoodUpdate) > 1*S_TO_US) {
			allOk = true;
		}
		else {
			bool hasUpdate = false;
			GeoPosition newProposalPosition;
			newProposalPosition.lat = pLat;
			newProposalPosition.lon = pLon;
			newProposalPosition.alt = pAlt;
			newProposalPosition.time = cTime;

			GeoPosition estimationPos = getGeoPositionEstimation(cTime, &hasUpdate);

			double distMetersPreviousPosToNewPos = geoDistance(estimationPos, newProposalPosition);

			allOk = true;

			// Remove position if too far
			if (distMetersPreviousPosToNewPos > MAX_DISTANCE_BETWEEN_ESTIM_AND_POS) {
				Serial.println("GPS position deleted, seems to be a glitch");
				glitchIter ++;
				allOk = false;
			}

		}

		if (allOk) {
			glitchIter = 0;
			lastGoodUpdate = cTime;
		}
		if (glitchIter > NB_GPS_GLITCHES_MAX) {
			Serial.println("Too many glitches, return home procedure");
			returnHome();
		}

		return allOk ;
	}
	else {
		return true;
	}
}


// Update data received by GPS
void updateGPSData(double pLat, double pLon, double pAlt, double pVms, int pCourseDegrees, long pFixtime) {

	long ctime = micros();
	// Update GPS data each 100ms to have  10Hz update
	if (gpsLastUpdateFixtime != pFixtime) {
		// Protect from negative altitude in meters
		if (pLat < 0) {
			pLat = 0;
		}

		// Check if the new position is "not so far" to the previous or the estimated position
		if (checkGPSPosition(ctime, pLat, pLon, pAlt)) {
			previousPosition = currentPosition;
			currentPosition.time = ctime;
			currentPosition.alt = pAlt;
			currentPosition.lat = pLat;
			currentPosition.lon = pLon;

			currentHeading = pCourseDegrees;

			GPSState = GPS_STATE_ON_TRACK;

			/** MOCK
			pVms = 10;
			currentPosition.time = micros()+5;
			currentPosition.alt = 0.5;
			currentPosition.lat =  489240260;
			currentPosition.lon = 21603880;
			previousPosition = currentPosition;
			previousPosition.lat =  489240340;
			previousPosition.lon = 21628080;
			previousPosition.time = micros()-150000;*/

			// @DEPRECATED because doesn't work if few gps precision
			// To have more precision, we work on 1e7 lat lon data
			// And we update only when UAV has speed
			/*if (pVms > 1.0) {
				currentHeading = angleBearingToNorthDeg((double)(currentPosition.lat-previousPosition.lat), (double)(currentPosition.lon-previousPosition.lon));
				Serial.print("heading=");
				Serial.print(currentHeading);
				Serial.println("\t");
				Serial.print(currentPosition.lat-previousPosition.lat);
				Serial.print("\t");
				Serial.println(pVms);
			}*/

			if (takeoffPositionSet == false) {
				takeoffPosition = currentPosition;
				takeoffPositionSet = true;
			}

			if (iterInsertGPSData < 100) {
				iterInsertGPSData ++ ;
			}
		}

		gpsLastUpdateFixtime = pFixtime;
	}
}



double simplePID(double pAngleDiff) {
	double KpGps2Roll = 0.5;

	double roll = KpGps2Roll * pAngleDiff;
	roll = constrain(roll, -FLIGHT_BY_GPS_MAX_ROLL, FLIGHT_BY_GPS_MAX_ROLL);
	return roll ;
}

void resetWPdata() {
	WP_on_target = false;
	previousDistance = 0.0;
	distance = 0.0;
}


void goNextWaypoint(FilterAverage *altitudeBarometer) {
	if (currentWP < NB_WAYPOINTS) {
		currentWP ++;
	}
	else {
		MissionDone = true;
		returnHome();
	}
	resetWPdata();
	updateHeading(altitudeBarometer);
}

void updateDistance() {
	distance = geoDistance(currentPosition, wps[currentWP]);
}

double angleCapVect(double dy, double dx) {
	double angle = -atan2(dy, dx)*180.0/M_PI + 90.0;
	if (dx < 0.0 && dy > 0.0) {
		angle = 360.0 + angle;
	}
	return angle;
}

// Returns angle in degree to have to head to the new targeted cap heading
// Right is positive, left negative
double diffAngleUsingCapDegrees(double currentCapHeadingDeg, double targetedCapDeg) {
	double diff = targetedCapDeg - currentCapHeadingDeg;
	if (diff > 180.0) {
		diff =   diff - 360.0 ;
	}
	else if (diff < -180.0) {
		diff = 360.0 + diff;
	}

	return diff;
}


// This function computes all navigation parameter
// Takes 5ms on Atmega 2560
void updateHeading(FilterAverage *altitudeBarometer) {
	long cTime = micros();
	if (currentPosition.time > cTime) {
		cTime = currentPosition.time;
	}

	// Forget GPS heading if lost track
	if (isGPSDataRelevant(cTime) == false) {
		gpsRollDesired = 0;
		gpsPitchDesired = 0;
		GPSState = GPS_STATE_LOST;
		return ;
	}

	// Update distance to waypoint
	updateDistance();

	if (modeGoHome == false && MissionDone == false) {
		// In mission, go to next WP if the mission is not done and distance radius near WP
		if (distance < DIST_TO_WAYPOINT_FOR_VALIDATION) {
			goNextWaypoint(altitudeBarometer);
			return;
		}
		// If the waypoint is impossible to reach, go home
		else if (distance > GPS_MAX_DISTANCE) {
			Serial.print("Waypoint to far (");
			Serial.print(distance);
			Serial.println(" m)");
			returnHome();
		}

		// If we finished to travel through all waypoints then start landing
		if (currentWP >= NB_WAYPOINTS) {
			MissionDone = true;

			// Return home : try to land where we tookoff
			returnHome();
		}
	}

	// Always update current position
	bool hasBeenUpdate = true;
	estimatedPosition = currentPosition;
	estimatedPosition = getGeoPositionEstimation(cTime, &hasBeenUpdate);


	// Mercator project
	Vector2 currentPos2D = geoPositionToVector2(estimatedPosition);
	Vector2 targetPos2D = geoPositionToVector2(wps[currentWP]);


	//--------------------------------------------
	// ROLL GPS HEADING
	bool hasUpdate;
	GeoPosition posFuture = getGeoPositionEstimation(estimatedPosition.time+((long)S_TO_US)*10, &hasUpdate); // Where the uav gonna be in ten second
	Vector2 posFuture2D = geoPositionToVector2(posFuture);

	if (NAVIG_METHOD_ANGLE_DIFF == 0) {
		angleDiff = getAngle(vect2Diff(currentPos2D, targetPos2D), vect2Diff(currentPos2D, posFuture2D));
	}
	else if (NAVIG_METHOD_ANGLE_DIFF == 1) {
		double capTargetDeg = angleCapVect(targetPos2D.y-currentPos2D.y, targetPos2D.x-currentPos2D.x);
		angleDiff = diffAngleUsingCapDegrees(currentHeading, capTargetDeg);
	}

	angleDiff = constrain(angleDiff, -180.0, 180.0);

	// Method 1 roll depends on angle diff directly as P controller
	gpsRollDesired = simplePID(angleDiff);

	if (WP_on_target == false) {
		if (distance < previousDistance && abs(angleDiff) <= 90.0) {
			WP_on_target = true;
		}
	}
	else {
		// If we were in target to WP, but distance become bigger and WP seems behind, then go to next WP
		// Allow large distance 10 times the normal distance for validation
		if (distance > previousDistance && abs(angleDiff) > 90.0 && distance < 3 * DIST_TO_WAYPOINT_FOR_VALIDATION) {
			WP_on_target = false;
			goNextWaypoint(altitudeBarometer);
			return;
		}
	}

	//---------------------------------------------
	// Roll method L1 (Straight line tracking)
	double L1 = 0.0;

	Vector2 prevWP;
	if (currentWP > 0) {
		prevWP = geoPositionToVector2(wps[currentWP-1]);
	}
	else {
		prevWP = geoPositionToVector2(takeoffPosition);
	}
	Vector2 prevWP_Puav = vect2Diff(prevWP, currentPos2D);
	Vector2 prevWP_currentWP = vect2Diff(prevWP, targetPos2D);
	Vector2 prevWP_E = vect2ScalarMultiply(vect2DotProduct(prevWP_Puav, prevWP_currentWP) / pow2(vect2Norm(prevWP_currentWP)), prevWP_currentWP);

	Vector2 E = vect2Add(prevWP, prevWP_E);
	Vector2 Puav_E = vect2Diff(currentPos2D, E);
	double norm_En = vect2Norm(Puav_E);


	Vector2 E_currentWP = vect2Diff(E, targetPos2D);
	L1 = vect2Norm(E_currentWP);

	// If enough distance, make L1 shorter
	if (L1 > 15) {
		L1 = _L1_damping * L1;
	}

	double Ddt = sqrt(pow2(L1) - pow2(norm_En));

	Vector2 N = getNormalized(Puav_E); // warning : from uav to E and not the inverse
	Vector2 T = getNormalized(prevWP_currentWP);
	Vector2 Puav_L1 = vect2Add(vect2ScalarMultiply(Ddt, T), vect2ScalarMultiply(norm_En, N));

	Vector2 Vg = getGroundSpeedVect2();
	double norm_Vg = vect2Norm(Vg);
	double sin_Nu = vect2CrossProduct(Vg, Puav_L1) / (norm_Vg * L1);
	sin_Nu = constrain(sin_Nu, -0.7071, 0.7071);
	//	double Nu = asin(sin_Nu); // Not used
	double K_acmd = 1.0;
	double Acmd = 2*K_acmd*pow2(norm_Vg) * sin_Nu / L1;

	// Because we are working on lat/lon to x/y dimension, we need to use the coefficient
	// R / 350.0 to have reasonnable roll desired
	gpsRollDesired2 = constrain(toDeg(atan(-Acmd/9.81)), -FLIGHT_BY_GPS_MAX_ROLL, FLIGHT_BY_GPS_MAX_ROLL);
	// To prevent initial rolling
	if (gpsRollDesired == 0.0) {
		gpsRollDesired2 = 0.0;
	}

	//---------------------------------------------
	// PITCH GPS HEADING
	// Define pitch which depends on diff altitude and distance
	double currentAltitude = currentPosition.alt;

	if (altitudeBarometer->areDataRelevant(cTime)) {
		currentAltitude = (int)(altitudeBarometer->getAverage() / 100.0f); // Convert into meter and smooth the output
	}

	double dzMeters = wps[currentWP].alt - currentAltitude;


	// To limit gps altitude imprecision variation,
	// At a defined altitude, we limit the difference of altitude evaluated
	if (currentAltitude > MIN_ALTITUDE_TOLERANCE && wps[currentWP].alt > MIN_ALTITUDE_TOLERANCE) {
		if (abs(dzMeters) < ALTITUDE_TOLERANCE) {
			dzMeters = 0.2f * dzMeters;
		}
	}

	double tmpPitchDesired = toDeg(atan(dzMeters / distance));
	gpsPitchDesired = constrain(tmpPitchDesired, FLIGHT_BY_GPS_MIN_PITCH, FLIGHT_BY_GPS_MAX_PITCH);



	// Store last update heading
	lastUpdateHeading = cTime;
	previousDistance = distance;
}




#endif
