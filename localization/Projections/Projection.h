/*
 * Projection.h
 *
 *  Created on: Jan 17, 2017
 *      Author: liborio
 */

#ifndef LOCALIZATION_PROJECTIONS_PROJECTION_H_
#define LOCALIZATION_PROJECTIONS_PROJECTION_H_

#include <proj_api.h>

namespace Projection {

class Projection {
private:
    Coord netoffset;

public:
    Projection();
    virtual ~Projection();
    void fromUTMToLatLon(Coord *coord, Coord *netOffset, std::string projString);
};

} /* namespace Projection */

#endif /* LOCALIZATION_PROJECTIONS_PROJECTION_H_ */
