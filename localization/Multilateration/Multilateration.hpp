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
    Coord LeastSquares(std::vector<Coord> *positions, std::vector<double> *distances);
    std::vector<Coord> InitializePositions(std::list<AnchorNode> *anchorNodes);
    std::vector<double> InitializeDistFS(std::list<AnchorNode> *anchorNodes);
    std::vector<double> InitializeDistTRGI(std::list<AnchorNode> *anchorNodes);
};

#endif /* LOCALIZATION_MULTILATERATION_MULTILATERATION_HPP_ */
