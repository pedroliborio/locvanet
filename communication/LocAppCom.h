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
#include <ctime>

//TNT and JAMA libraries
#include <tnt_array2d.h>
#include <jama_qr.h>

#include <fstream>

#include <localization/Multilateration/Multilateration.hpp>
#include <localization/RSSI/FreeSpaceModel.hpp>
#include <localization/RSSI/TwoRayInterference.hpp>

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;

using Veins::AnnotationManager;

using namespace GeographicLib;

/**
 * Communication approach for localization based on beaconing
 * Using BaseWaveApplayer of Veins
 */
class LocAppCom : public BaseWaveApplLayer {
    private:
        std::list<AnchorNode> anchorNodes;//list of neighbor vehicles
        Coord coopPosFS; //cooperative Positioning Estimation
        Coord coopPosTRGI; //cooperative Positioning Estimation
        double constVelLight = 299792458.0; //m/s
        double lambda = 0.051; //wave length for CCH frequency
        double frequencyCCH = 5.890; //GHz
        double pTx = 20.0; //milliWatts
        double alpha = 2.0; // pathloss exponent
        double epsilonR = 1.02; //dieletric constant
        double ht = 1.895; //height of antenn transmitter
        double hr = 1.895; //height of antenna receiver
    public:
        virtual void initialize(int stage);
        virtual void finish();
        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        time_t timeSeed;
        //Struct with the attributes of a neighbor node
    protected:
        AnnotationManager* annotations;
        static const simsignalwrap_t mobilityStateChangedSignal;
        //Coord gDRPosition;
        //bool gpsOutage;// Is GPS In Outage?
        //Coord lastGPSPos;  //Last GPS Know Position
        //Coord lastGDRPos;  //Last GDR Know Position
        //Coord lastSUMOPos; //Last SUMO Know Position used to compute bearing and distance traveled
        //double bearing; //Bearing given by Geodesic Inverse (Giroscope)
        //double distance; //Distance given by Geodesic Direct (Odometer)
    protected:
        //This method will manipulates the information received from a message
        virtual void onData(WaveShortMessage* wsm);
        //This method will manipulates the information received on beacon
        virtual void onBeacon(WaveShortMessage* wsm);
        //This method crate a beacon with vehicle kinematics information
        virtual void handleSelfMsg(cMessage* msg);
        void UpdateNeighborList(AnchorNode anchorNode);
        void UpdateNeighborListDistances(void);
        void PrintNeighborList(void);
        void GeodesicDRModule(void);
        void VehicleKinematicsModule(void);

       // void UpdateNeighborsList(void);
};

#endif
