/*
 * Projection.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: liborio
 */

#include <Projections/Projection.h>

namespace Projection {

Projection::Projection() {
    // TODO Auto-generated constructor stub

}

Projection::~Projection() {
    // TODO Auto-generated destructor stub
}

void Projection::fromUTMToLonLat(Coord *coord, Coord *netOffset, std::string projString){
    projPJ pj_utm;
    projLP geoCoord;
    projXY coordUTM;

    coordUTM.u = coord->x - netOffset->x;
    coordUTM.v = coord->y - netOffset->y;

    pj_utm = pj_init_plus("+proj=utm +zone=10 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");

    geoCoord = pj_inv(coordUTM,pj_utm);

    geoCoord.u*= RAD_TO_DEG;
    geoCoord.v*= RAD_TO_DEG;

    coord->x = geoCoord.u;
    coord->y = geoCoord.v;
}

void Projection::fromLonLatToUTM(Coord *coord, Coord *netOffset, std::string projString){
    projPJ pj_utm;
    projLP geoCoord;
    projXY coordUTM;

    geoCoord.u = coord.y;
    geoCoord.v = coord.x;

    geoCoord.u*= DEG_TO_RAD;
    geoCoord.v*= DEG_TO_RAD;

    pj_utm = pj_init_plus("+proj=utm +zone=10 +ellps=WGS84 +datum=WGS84 +units=m +no_defs");

    coordUTM = pj_inv(geoCoord,pj_utm);

    coord->x = coordUTM.u;
    coord->y = coordUTM.v;
}

} /* namespace Projection */
