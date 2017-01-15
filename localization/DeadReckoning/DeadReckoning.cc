/*
 * DeadReckoning.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#include <DeadReckoning/DeadReckoning.h>

DeadReckoning::DeadReckoning() {
    // TODO Auto-generated constructor stub

}

DeadReckoning::~DeadReckoning() {
    // TODO Auto-generated destructor stub
}

void DeadReckoning::getPosition(LatLon *lastGDRPos, LatLon *lastSUMOPos, LatLon *atualSUMOPos){
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

void DeadReckoning::getError(double *errorGDR, LatLon *lastGDRPos, LatLon *atualSUMOPos){
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());// Geodesic
    //geod.Inverse(lastGDRPos->lat, lastGDRPos->lon, atualSUMOPos->lat, atualSUMOPos->lon, s_12, errorGDR);
}






/*
void LocAppCom::GeodesicDRModule(void){
    //Here we need to get the local coordinates position
    //Convert to lat lon points and pass to GDR algorithm.
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    double latGDR, lonGDR;
    geod.Direct(this->lastGDRPos.x,this->lastGDRPos.y,bearing,distance,latGDR,lonGDR);
    //need put the trace in some output
}

Convert Between Local Coordinates to Geodesic Coordinates and
//calculates bearing and distance like and odometer and a gyroscope
void LocAppCom::VehicleKinematicsModule(void){

    //Abou Geocentric Coordinates:
    *
      * \brief %Geocentric coordinates
      *
      * Convert between geodetic coordinates latitude = \e lat, longitude = \e
      * lon, height = \e h (measured vertically from the surface of the ellipsoid)
      * to geocentric coordinates (\e X, \e Y, \e Z).  The origin of geocentric
      * coordinates is at the center of the earth.  The \e Z axis goes thru the
      * north pole, \e lat = 90&deg;.  The \e X axis goes thru \e lat = 0,
      * \e lon = 0.  %Geocentric coordinates are also known as earth centered,
      * earth fixed (ECEF) coordinates.
      *
      * The conversion from geographic to geocentric coordinates is
      * straightforward.  For the reverse transformation we use
      * - H. Vermeille,
      *   <a href="https://dx.doi.org/10.1007/s00190-002-0273-6"> Direct
      *   transformation from geocentric coordinates to geodetic coordinates</a>,
      *   J. Geodesy 76, 451--454 (2002).
      * .
      * Several changes have been made to ensure that the method returns accurate
      * results for all finite inputs (even if \e h is infinite).  The changes are
      * described in Appendix B of
      * - C. F. F. Karney,
      *   <a href="http://arxiv.org/abs/1102.1215v1">Geodesics
      *   on an ellipsoid of revolution</a>,
      *   Feb. 2011;
      *   preprint
      *   <a href="http://arxiv.org/abs/1102.1215v1">arxiv:1102.1215v1</a>.
      * .
      * Vermeille similarly updated his method in
      * - H. Vermeille,
      *   <a href="https://dx.doi.org/10.1007/s00190-010-0419-x">
      *   An analytical method to transform geocentric into
      *   geodetic coordinates</a>, J. Geodesy 85, 105--117 (2011).
      * .
      * See \ref geocentric for more information.
      *
      * The errors in these routines are close to round-off.  Specifically, for
      * points within 5000 km of the surface of the ellipsoid (either inside or
      * outside the ellipsoid), the error is bounded by 7 nm (7 nanometers) for
      * the WGS84 ellipsoid.  See \ref geocentric for further information on the
      * errors.
      *
      * Example of use:
      * \include example-Geocentric.cpp
      *
      * <a href="CartConvert.1.html">CartConvert</a> is a command-line utility
      * providing access to the functionality of Geocentric and LocalCartesian.
      *********************************************************************



    //Coordinates of the sumo.net boundingbox
    double lat0 = -22.910044, lon0 = -43.207808;
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    LocalCartesian proj(lat0, lon0, 0, earth);
    Coord pos = this->mobility->getCurrentPosition();
    double x = pos.x, y = pos.y, z = 0;
    double lat, lon, h, s12, azi1, azi2;
    //Obtain Lat Lon Coordinates
    proj.Reverse(x, y, z, lat, lon, h);
    //Update the bearing and distance with GeographicLib...
    geod.Inverse(this->lastSUMOPos.x, this->lastSUMOPos.y,lat,lon,s12,azi1,azi2);
    this->distance = s12;
    this->bearing = azi1;
    //geod.Inverse(
    //Forward = between lat lon to x, y
    //double lat = 50.9, lon = 1.8, h = 0; // Calais
    //double x, y, z;
    //proj.Forward(lat, lon, h, x, y, z);

}*/
