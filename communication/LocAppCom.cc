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

#include "LocAppCom.h"

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t LocAppCom::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

Define_Module(LocAppCom);

void LocAppCom::initialize(int stage){
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        mobility = TraCIMobilityAccess().get(getParentModule());
        Coord vehCoord = mobility->getCurrentPosition();
        //this->lastSUMOPos.x = vehCoord.x;
        //this->lastSUMOPos.y = vehCoord.y;
        traci = mobility->getCommandInterface();
        timeSeed = time(0);
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

    }

}

void LocAppCom::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            //Update the distances in the neighbors list because the vehicle is moving.
            //Also call Least Squares to update self position estimation
            //UpdateNeighborListDistances();
            //TODO Update Vehicle Kinematics Information
                //VehicleKinematicsModule(void);
                //Update GDR Information//
            //.....
            //Update GPS Information
            //....
            //Create a beacon...
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
            //Put current position in the beacon
            wsm->setSenderPos(mobility->getCurrentPosition());
            //Write Output (just for debug)
            std::cout << "Function HandleSelfMessage - Vehicle " << wsm->getSenderAddress() << "Position Sent:" << wsm->getSenderPos()<<"\n";
            //send the message
            sendWSM(wsm);
            //Draw annotation
            //findHost()->getDisplayString().updateWith("r=16,blue");
            //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"green"));
            //Schedule the beacon to sent
            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
            break;
        }
        default: {
            if (msg)
                DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

void  LocAppCom::onBeacon(WaveShortMessage* wsm){
    /***
    //Draw annotation
    //findHost()->getDisplayString().updateWith("r=16,blue");
    //annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
    //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"blue"));
    ***/

    //TODO GetRSSI EV << "Vehicle:" << wsm->getSenderAddress() << "Received Power: " << wsm->getRxPower()<<"\n";
    //Here the vehicle need to maintain a vector with the position of neighbors
    AnchorNode anchorNode;
    anchorNode.timestamp = wsm->getTimestamp();
    anchorNode.realPosition = wsm->getSenderPos();
    anchorNode.realDistance = wsm->getSenderPos().distance(mobility->getCurrentPosition());
    anchorNode.vehID = wsm->getSenderAddress();
    anchorNode.rssiFS = FreeSpaceModel::getRSSI(anchorNode.realDistance);
    anchorNode.rssiDistanceFS = FreeSpaceModel::getDistance(anchorNode.rssiFS);
    anchorNode.rssiTRGI = TwoRayInterference::getRSSI(anchorNode.realDistance);
    anchorNode.rssiDistanceTRGI = TwoRayInterference::getDistance(anchorNode.rssiTRGI, anchorNode.realDistance);

    //TODO Calc the RSSI Distance
    //Update the list of neighbors vehicles
    UpdateNeighborList(anchorNode);



    /*switch(lossModel){
    case 'F':
        printf("FSPM");
        anchorNode.rssi = FreeSpaceRSSI(anchorNode.realDistance);
        anchorNode.rssiDistance = FreeSpaceRSSIDist(anchorNode.rssi);
        break;
    case 'T':
        printf("TRGI");
        anchorNode.rssi = TwoRayInterferenceRSSI(anchorNode.realDistance);
        anchorNode.rssiDistance = TwoRayInterferenceRSSIDist(anchorNode.rssi,anchorNode.realDistance);
        break;
    default:
        //Use real distance
        printf("Real Distance");
        anchorNode.rssi = 0;
        anchorNode.rssiDistance = anchorNode.realDistance;
    }*/

    //std::cout << "List of Neighbor Vehicles Before Update\n";
    //PrintNeighborList();



    //Update distances by radio ranging (precisa do rssi tem que ser passivo)
    //UpdateNeighborListDistances();

    std::cout << "List of Neighbor Vehicles Updated\n";
    PrintNeighborList();

    //FIXME IMplement a mechanism to discard a beacon after some round
    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 4 or more anchor nodes call multilateration method
    if(anchorNodes.size() > 3){
        //TODO Call Multilateration Method
        std::cout << "Function On Beacon - My real position " << mobility->getCurrentPosition() << "\n\n";
        std::vector<Coord> positions;
        positionsMultilateration::InitializePositions(&anchorNodes);
        std::vector<double> distancesFS = Multilateration::InitializeDistFS(&anchorNodes);
        std::vector<double> distancesTRGI = Multilateration::InitializeDistTRGI(&anchorNodes);
        coopPosFS = Multilateration::LeastSquares(&positions, &distancesFS);
        coopPosTRGI = Multilateration::LeastSquares(&positions, &distancesTRGI);
    }
    else{
        coopPosFS.x = coopPosFS.y = coopPosFS.z =  0;
        coopPosTRGI.x = coopPosTRGI.y = coopPosTRGI.z =  0;
    }

    //Log File with results of CP Approach
    //MyRealPosition (SUMO) | My Estimated Position (Via CP) | vehID (Neighbor) | Timestamp | Neighbor Position (SUMO) |  Real Distance | Est. RSSI Dist | RSSI

    /*std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
    beaconLogFile << mobility->getCurrentPosition().x <<'\t'<< mobility->getCurrentPosition().y <<'\t'<< mobility->getCurrentPosition().z
                <<'\t'<< coopPos.x << '\t'<< coopPos.y << '\t'<< coopPos.z <<'\t'<<  anchorNode.vehID <<'\t'<< anchorNode.timestamp
                <<'\t'<< anchorNode.realPosition.x <<'\t'<< anchorNode.realPosition.y <<'\t'<< anchorNode.realPosition.z
                <<'\t'<< anchorNode.realDistance <<'\t'<< anchorNode.rssiDistance <<'\t'<< anchorNode.rssi <<'\t' <<endl;
    beaconLogFile.close();*/

    //The begin of Cooperative Positioning Approach
}

//Update the position of a neighbor vehicle in the list
void LocAppCom::UpdateNeighborList(AnchorNode anchorNode){
    //Verify if anchor node already exists...
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == anchorNode.vehID){
            it->timestamp = anchorNode.timestamp;
            it->realPosition = anchorNode.realPosition;
            it->realDistance = anchorNode.realDistance;
            it->rssiFS = anchorNode.rssiFS;
            it->rssiDistanceFS = anchorNode.rssiDistanceFS;
            it->rssiFS = anchorNode.rssiTRGI;
            it->rssiDistanceFS = anchorNode.rssiDistanceTRGI;
            return;
        }
    }
    //If anchor node not exists add this one
    anchorNodes.push_back(anchorNode);
}

/*Update distances from ego vehicle to another vehicles...
* As the ego vehicle need reiceive positions from neighborhood
* to measure his own position we cannot actualize the position in a active form
* only in a passive way is possible, in another words the vehicle needs to wait
* for new positions for anchor nodes this can be improved by the use of an rsu.
*/
void LocAppCom::UpdateNeighborListDistances(){
    if(anchorNodes.size() > 0){
        for(std::list<AnchorNode>::iterator it= anchorNodes.begin(); it!= anchorNodes.end(); ++it){
            it->realDistance = it->realPosition.distance(mobility->getCurrentPosition());

        }
    }
}

void LocAppCom::PrintNeighborList(){
    for(std::list<AnchorNode>::iterator it = anchorNodes.begin(); it!= anchorNodes.end(); ++it){
            std::cout << "Vehicle "<< it->vehID <<" Pos: "<<it->realPosition << "Dist: " <<it->realDistance <<"\n";
    }
}

//We not using data messages in our approach :)
void LocAppCom::onData(WaveShortMessage* wsm){

}

void LocAppCom::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details){
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}




void LocAppCom::finish(){
    BaseWaveApplLayer::finish();

}



