/*
 * FreeSpaceModel.h
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_FREESPACEMODEL_HPP_
#define LOCALIZATION_RSSI_FREESPACEMODEL_HPP_

#include<Math.hpp>

class FreeSpaceModel {
private:
    /*double alpha = 2.0; // pathloss exponent
    double pTx = 20.0; //Transmission Potency
    double frequencyCCH = 5.890; //frequency of the cch
    double lambda = 0.051; //wave length for CCH frequency
    double constVelLight = 299792458.0; //m/s*/
public:
    FreeSpaceModel();
    virtual ~FreeSpaceModel();

    static double getRSSI(double distance, double pTx, double alpha, double lambda);
    static double getDistance(double rssi, double pTx, double alpha, double lambda);

};

#endif /* LOCALIZATION_RSSI_FREESPACEMODEL_HPP_ */
