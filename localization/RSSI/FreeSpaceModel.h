/*
 * FreeSpaceModel.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_FREESPACEMODEL_H_
#define LOCALIZATION_RSSI_FREESPACEMODEL_H_

namespace Localization {

class FreeSpaceModel {
private:
    double rssi;
    double distance;
public:
    FreeSpaceModel();
    virtual ~FreeSpaceModel();
    void setRSSI(double distance, double pTx, double alpha, double lambda);
    void setDistance(double rssi, double pTx, double alpha, double lambda);
};

} /* namespace Localization */

#endif /* LOCALIZATION_RSSI_FREESPACEMODEL_H_ */
