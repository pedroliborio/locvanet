/*
 * Multilateration.h
 *
 *  Created on: Jan 18, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_MULTILATERATION_MULTILATERATION_H_
#define LOCALIZATION_MULTILATERATION_MULTILATERATION_H_

namespace Localization {

class Multilateration {
public:
    const int REAL_POS = 0;
    const int DR_POS = 1;
    const int GPS_POS = 2;
    const int REAL_DIST = 0;
    const int DR_DIST = 1;
    const int GPS_DIST = 2;
    const int FS_DIST = 3;
    const int TRGI_DIST = 4;

    Multilateration();
    virtual ~Multilateration();
    void DoMultilateration(std::list<AnchorNode> *anchorNodes, const int POS_TYPE, const int DIST_TYPE);
    void LeastSquares(void);
    void getDistList(std::list<AnchorNode> *anchorNodes, const int DIST_TYPE);
    void getPosList(std::list<AnchorNode> *anchorNodes, const int POS_TYPE);
};

} /* namespace Localization */

#endif /* LOCALIZATION_MULTILATERATION_MULTILATERATION_H_ */
