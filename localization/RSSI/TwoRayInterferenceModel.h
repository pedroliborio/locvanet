/*
 * TwoRayInterferenceModel.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_TWORAYINTERFERENCEMODEL_H_
#define LOCALIZATION_RSSI_TWORAYINTERFERENCEMODEL_H_

namespace Localization {

class TwoRayInterferenceModel {
private:
    double rssi;
    double distance;
public:
    TwoRayInterferenceModel();
    virtual ~TwoRayInterferenceModel();
    void setRSSI(double distance, double pTx, double lambda, double ht, double hr, double epsilonR);
    void setDistance(double rssi, double distance, double pTx, double lambda, double ht, double hr, double epsilonR);
};

} /* namespace Localization */

#endif /* LOCALIZATION_RSSI_TWORAYINTERFERENCEMODEL_H_ */
