/*
 * DeadReckoning.h
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#ifndef LOCALIZATION_DEADRECKONING_DEADRECKONING_H_
#define LOCALIZATION_DEADRECKONING_DEADRECKONING_H_


//Geodesic Library
#include "Geodesic.hpp"
#include <Constants.hpp>
#include <Geocentric.hpp>
#include <LocalCartesian.hpp>

struct t_LatLon{
    double lat;
    double lon;
};typedef struct t_LatLon LatLon;

using namespace GeographicLib;

class DeadReckoning {
public:
    DeadReckoning();
    virtual ~DeadReckoning();
    static void getPosition(LatLon *lastGDRPos, LatLon *lastSUMOPos, LatLon *atualSUMOPos);
    static void getError(double *errorGDR, LatLon *lastGDRPos, LatLon *atualSUMOPos);
};

#endif /* LOCALIZATION_DEADRECKONING_DEADRECKONING_H_ */
