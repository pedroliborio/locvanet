//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __LOCVANET_LOCAPPCOM_H_
#define __LOCVANET_LOCAPPCOM_H_

//Veins libraries
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
//Geodesic Library
#include <localization/GeographicLib/include/GeographicLib/Geodesic.hpp>
#include <localization/GeographicLib/include/GeographicLib/Constants.hpp>
#include <localization/GeographicLib/include/GeographicLib/Geocentric.hpp>
#include <localization/GeographicLib/include/GeographicLib/LocalCartesian.hpp>
//Utils libraries
#include <exception>
#include <cmath>

//TNT and JAMA libraries
#include <tnt_array2d.h>
#include <jama_qr.h>

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;

using Veins::AnnotationManager;

using namespace GeographicLib;

/**
 * Communication approach for localization based on beaconing
 * Using BaseWaveApplayer of Veins
 */
class LocAppCom : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
        TraCIMobility* mobility;
        TraCICommandInterface* traci;

        struct NeighborNode{
            Coord realPosition;
            double realDistance;
            //double rssiDistance;
            //simtime_t timestamp;
        };
    protected:
        AnnotationManager* annotations;
        static const simsignalwrap_t mobilityStateChangedSignal;
        //Coord gDRPosition;
        bool gpsOutage;// Is GPS In Outage?
        Coord lastGPSPos;  //Last GPS Know Position
        Coord lastGDRPos;  //Last GDR Know Position
        Coord lastSUMOPos; //Last SUMO Know Position used to compute bearing and distance traveled
        Coord coopPosPos;
        double bearing; //Bearing given by Geodesic Inverse (Giroscope)
        double distance; //Distance given by Geodesic Direct (Odometer)
        std::list<NeighborNode> listNeighbors;//list of neighbors vehicles

    protected:
        //This method will manipulates the information received from a message
        virtual void onData(WaveShortMessage* wsm);
        //This method will manipulates the information received on beacon
        virtual void onBeacon(WaveShortMessage* wsm);
        //This method crate a beacon with vehicle kinematics information
        virtual void handleSelfMsg(cMessage* msg);
        void GeodesicDRModule(void);
        void VehicleKinematicsModule(void);
        void LeastSquares(std::list<NeighborNode>* listNeighbor);
};

#endif
