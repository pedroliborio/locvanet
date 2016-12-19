/*
 * FreeSpaceModel.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#include <FreeSpaceModel.hpp>

FreeSpaceModel::FreeSpaceModel() {
    // TODO Auto-generated constructor stub

}

FreeSpaceModel::~FreeSpaceModel() {
    // TODO Auto-generated destructor stub
}

double FreeSpaceModel::getRSSI(double distance){
    double rssi;
    rssi = 10*log10(pTx) - 10 * log10((16* M_PI * M_PI * pow(distance, alpha)) / pow(lambda,alpha));
    return rssi;
}

double FreeSpaceModel::getDistance(double rssi){
    double distance;
    distance = (lambda / (pow((4 * M_PI),(2/alpha)) ) ) * (pow(pTx, (1/alpha))) * (pow(10,( - (rssi / (10*alpha) ) )));
    return distance;
}

