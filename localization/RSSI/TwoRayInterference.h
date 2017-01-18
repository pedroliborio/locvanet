/*
 * TwoRayInterference.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_TWORAYINTERFERENCE_H_
#define LOCALIZATION_RSSI_TWORAYINTERFERENCE_H_

namespace Localization {

class TwoRayInterference {
private:
    double rssi;
    double distance;
public:
    TwoRayInterference();
    virtual ~TwoRayInterference();
    double setRSSI(double distance, double pTx, double lambda, double ht, double hr, double epsilonR);
    double setDistance(double rssi, double distance, double pTx, double lambda, double ht, double hr, double epsilonR);
};

} /* namespace Localization */

#endif /* LOCALIZATION_RSSI_TWORAYINTERFERENCE_H_ */
