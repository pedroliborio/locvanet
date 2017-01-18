/*
 * DeadReckoning.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_DEADRECKONING_DEADRECKONING_H_
#define LOCALIZATION_DEADRECKONING_DEADRECKONING_H_

#include <Types.h>

namespace Localization {

class DeadReckoning {
private:
    LonLat lastKnowPos;
    double error;
public:
    DeadReckoning();
    virtual ~DeadReckoning();
    void getPosition(LonLat *lastGDRPos, LonLat *lastSUMOPos, LonLat *atualSUMOPos);
    void getError(double *errorGDR, LonLat *lastGDRPos, LonLat *atualSUMOPos);
};

} /* namespace Localization */

#endif /* LOCALIZATION_DEADRECKONING_DEADRECKONING_H_ */
