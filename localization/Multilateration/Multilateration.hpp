/*
 * Multilateration.h
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#ifndef LOCALIZATION_MULTILATERATION_MULTILATERATION_HPP_
#define LOCALIZATION_MULTILATERATION_MULTILATERATION_HPP_

//TNT and JAMA libraries
#include "tnt_array2d.h"
#include "jama_qr.h"
#include "veins/base/utils/Coord.h"

struct AnchorNode_t{
            int vehID; //sender vehicle ID
            Coord realPosition; //vehicle sender real position
            double realDistance; //real distance (euclidean)
            double rssiDistanceFS; //Distance RSSI
            double rssiDistanceTRGI;
            double rssiFS; //RSSI;
            double rssiTRGI;
            simtime_t timestamp; // timestamp of the message
};typedef struct AnchorNode_t AnchorNode;



class Multilateration {
public:
    Multilateration();
    virtual ~Multilateration();
    static Coord LeastSquares(Coord *positions, double *distances, int size);
    static void InitializePosDist(std::list<AnchorNode> *anchorNodes, Coord *positions, double *distances, std::string model);
};

#endif /* LOCALIZATION_MULTILATERATION_MULTILATERATION_HPP_ */
