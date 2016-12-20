/*
 * TwoRayInterference.h
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_TWORAYINTERFERENCE_HPP_
#define LOCALIZATION_RSSI_TWORAYINTERFERENCE_HPP_

#include <Math.hpp>

class TwoRayInterference {
private:
    /*double constVelLight = 299792458.0; //m/s
    double lambda = 0.051; //wave length for CCH frequency
    double frequencyCCH = 5.890; //GHz
    double pTx = 20.0; //milliWatts
    double alpha = 2.0; // pathloss exponent
    double epsilonR = 1.02; //dieletric constant
    double ht = 1.895; //height of antenn transmitter
    double hr = 1.895; //height of antenna receiver*/
public:
    TwoRayInterference();
    virtual ~TwoRayInterference();
    static double getRSSI(double distance, double pTx, double lambda, double ht, double hr, double epsilonR);
    static double getDistance(double rssi, double distance, double pTx, double lambda, double ht, double hr, double epsilonR);

};

#endif /* LOCALIZATION_RSSI_TWORAYINTERFERENCE_HPP_ */
