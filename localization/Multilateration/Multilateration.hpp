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
            double rssiDistanceFS; //RSSI Distance FS
            double rssiDistanceTRGI;//RSSI Distance TRGI
            double rssiFS; //RSSI FS;
            double rssiTRGI; //RSSI TRGI
            double rssiDistAvgFilterFS = .0; //RSSI Distance FS Avg Filter
            double rssiDistAvgFilterTRGI = .0; //RSSI Distance TRGI Avg Filter
            simtime_t timestamp; // timestamp of the beacon received
            int k = 0; //Actual Iteration for the Avg Filter
};typedef struct AnchorNode_t AnchorNode;



class Multilateration {
public:
    Multilateration();
    virtual ~Multilateration();
    static Coord LeastSquares(std::vector<Coord> *positions, std::vector<double> *distances, int size);
    static void InitializePosDist(std::list<AnchorNode> *anchorNodes, std::vector<Coord> *positions, std::vector<double> *distances, std::string model);
};

#endif /* LOCALIZATION_MULTILATERATION_MULTILATERATION_HPP_ */
