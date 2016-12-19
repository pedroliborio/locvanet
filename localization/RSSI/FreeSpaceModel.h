/*
 * FreeSpaceModel.h
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#ifndef LOCALIZATION_RSSI_FREESPACEMODEL_H_
#define LOCALIZATION_RSSI_FREESPACEMODEL_H_

class FreeSpaceModel {
private:
    double alpha = 2.0; // pathloss exponent
    double pTx = 20.0; //Transmission Potency
    double frequencyCCH = 5.890; //frequency of the cch
    double lambda = 0.051; //wave length for CCH frequency
    double constVelLight = 299792458.0; //m/s
public:
    FreeSpaceModel();
    virtual ~FreeSpaceModel();

    double getRSSI(double distance);
    double getDistance(double rssi);

    double getAlpha() const {
        return alpha;
    }

    void setAlpha(double alpha = 2.0) {
        this->alpha = alpha;
    }

    double getConstVelLight() const {
        return constVelLight;
    }

    void setConstVelLight(double constVelLight = 299792458.0) {
        this->constVelLight = constVelLight;
    }

    double getFrequencyCch() const {
        return frequencyCCH;
    }

    void setFrequencyCch(double frequencyCch = 5.890) {
        frequencyCCH = frequencyCch;
    }

    double getLambda() const {
        return lambda;
    }

    void setLambda(double lambda = 0.051) {
        this->lambda = lambda;
    }

    double getTx() const {
        return pTx;
    }

    void setTx(double tx = 20.0) {
        pTx = tx;
    }
};

#endif /* LOCALIZATION_RSSI_FREESPACEMODEL_H_ */
