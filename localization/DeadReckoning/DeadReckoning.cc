/*
 * DeadReckoning.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#include <DeadReckoning/DeadReckoning.h>

namespace Localization {

DeadReckoning::DeadReckoning() {
    // TODO Auto-generated constructor stub

}

DeadReckoning::~DeadReckoning() {
    // TODO Auto-generated destructor stub
}

void DeadReckoning::getPosition(LonLat *lastGDRPos, LonLat *lastSUMOPos, LonLat *atualSUMOPos){
    LatLon deadReckPos; // reckognized DR
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    double lat, lon; //coordenadas DR
    double s_12; //odometer
    double azi_1, azi_2; //distance and azimuths

    //gyro and odometer...
    geod.Inverse(lastSUMOPos->lat, lastSUMOPos->lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12, azi_1, azi_2);

    //calc new GDR position
    //geod.Direct(lastGDRPos.lat, lastGDRPoslon, azi_1, s_12, lat, lon);

    lastGDRPos->lat = lat;
    lastGDRPos->lon = lon;
}

void DeadReckoning::getError(double *errorGDR, LonLat *lastGDRPos, LonLat *atualSUMOPos){
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    //geod.Inverse(lastGDRPos->lat, lastGDRPos->lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12, errorGDR);
}

} /* namespace Localization */
