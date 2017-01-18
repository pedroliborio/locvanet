/*
 * Types.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef TYPES_TYPES_H_
#define TYPES_TYPES_H_


#include "veins/base/utils/Coord.h"

struct t_LonLat{
    double lon;
    double lat;
};typedef struct t_LonLat LonLat;

struct AnchorNode_t{
            int vehID; //sender vehicle ID
            Coord realPos; //vehicle sender real position
            Coord deadReckPos;
            Coord gpsPos;
            double realDist; //real distance (euclidean)
            double deadReckDist;
            double gpsDist;
            double rssiDistFS; //RSSI Distance FS
            double rssiDistTRGI;//RSSI Distance TRGI
            double rssiFS; //RSSI FS;
            double rssiTRGI; //RSSI TRGI
            double rssiDistAvgFilterFS = .0; //RSSI Distance FS Avg Filter
            double rssiDistAvgFilterTRGI = .0; //RSSI Distance TRGI Avg Filter
            simtime_t timestamp; // timestamp of the beacon received
            int k = 0; //Actual Iteration for the Avg Filter
};typedef struct AnchorNode_t AnchorNode;

namespace Localization {

class Types {
public:
    Types();
    virtual ~Types();
};

} /* namespace Localization */

#endif /* TYPES_TYPES_H_ */
