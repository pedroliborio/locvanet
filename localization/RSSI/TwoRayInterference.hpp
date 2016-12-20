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
    double constVelLight = 299792458.0; //m/s
    double lambda = 0.051; //wave length for CCH frequency
    double frequencyCCH = 5.890; //GHz
    double pTx = 20.0; //milliWatts
    double alpha = 2.0; // pathloss exponent
    double epsilonR = 1.02; //dieletric constant
    double ht = 1.895; //height of antenn transmitter
    double hr = 1.895; //height of antenna receiver
public:
    TwoRayInterference();
    virtual ~TwoRayInterference();
    static double getRSSI(double distance);
    static double getDistance(double rssi, double distance);

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

    double getEpsilonR() const {
        return epsilonR;
    }

    void setEpsilonR(double epsilonR = 1.02) {
        this->epsilonR = epsilonR;
    }

    double getFrequencyCch() const {
        return frequencyCCH;
    }

    void setFrequencyCch(double frequencyCch = 5.890) {
        frequencyCCH = frequencyCch;
    }

    double getHr() const {
        return hr;
    }

    void setHr(double hr = 1.895) {
        this->hr = hr;
    }

    double getHt() const {
        return ht;
    }

    void setHt(double ht = 1.895) {
        this->ht = ht;
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

#endif /* LOCALIZATION_RSSI_TWORAYINTERFERENCE_HPP_ */
