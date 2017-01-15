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
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        timeSeed = time(0);
        this->distOutage = DBL_MAX;
        this->lastSUMOPos.lat = 0;
        this->lastSUMOPos.lon = 0;
        this->atualSUMOPos.lat = 0;
        this->atualSUMOPos.lon = 0;
        this->lastGDRPos.lat = 0;//initialize DR variables
        this->lastGDRPos.lon = 0;
        this->isInOutage = false;
        GetGPSOutageCoordinates();//Take information of outage from dataset.




        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

    }

}

void LocAppCom::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            /*std::pair<double,double> lonlat = traci->getLonLat(mobility->getCurrentPosition());

            if(lastSUMOPos.lat == 0){
                //THis is just to begin the process at this first time I don't send any beacon
                //only update the last positioning information
                lastSUMOPos.lat = lonlat.second;
                lastSUMOPos.lon = lonlat.first;
                return;
            }

            //take atual position
            atualSUMOPos.lat = lonlat.second;
            atualSUMOPos.lon = lonlat.first;
            this->isInOutage = RecognizeOutage();
            this->isInRecover = RecognizeRecover();


            if(this->isInOutage){
                //call Dead Reckoning...

                //put information on GDR file

            }
            else{

            }

            if(this->isInRecover){

            }*/

            //detect one outage...

            //call DR only when outage occurs
            //

            //std::cout << "************" << endl;
            //std::pair<double,double> lonlat = traci->getLonLat(wsm->getSenderPos());
            //traci->get
            //std::cout <<  lonlat.first <<" "<< lonlat.second << endl;
            //std::cout << wsm->getSenderPos() << endl;
            //std::cout << "************" << endl;
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
    //Draw annotation"
    //findHost()->getDisplayString().updateWith("r=16,blue");
    //annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
    //annotations->scheduleErase(1,annotations->drawLine(wsm->getSenderPos(), mobility->getCurrentPosition(),"blue"));
    ***/


    AnchorNode anchorNode;
    //If the anchorNode already exists it will be get to be update
    //otherwise the new values will gathered and push to the list
    getAnchorNode(wsm->getSenderAddress(), &anchorNode);

    anchorNode.timestamp = wsm->getTimestamp();
    anchorNode.realPosition = wsm->getSenderPos();
    anchorNode.realDistance = wsm->getSenderPos().distance(mobility->getCurrentPosition());
    anchorNode.vehID = wsm->getSenderAddress();

    //Calculating RSSI
    anchorNode.rssiFS = FreeSpaceModel::getRSSI(anchorNode.realDistance, this->pTx, this->alpha, this->lambda);
    anchorNode.rssiDistanceFS = FreeSpaceModel::getDistance(anchorNode.rssiFS, this->pTx, this->alpha, this->lambda);
    anchorNode.rssiTRGI = TwoRayInterference::getRSSI(anchorNode.realDistance, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);
    anchorNode.rssiDistanceTRGI = TwoRayInterference::getDistance(anchorNode.rssiTRGI, anchorNode.realDistance, this->pTx, this->lambda, this->ht, this->hr, this->epsilonR);

    //Improving measurements with Avg Filter
    anchorNode.k++;//Increment k_th iteration
    anchorNode.rssiDistAvgFilterFS = Filters::AverageFilter(anchorNode.k, anchorNode.rssiDistAvgFilterFS, anchorNode.rssiDistanceFS);
    anchorNode.rssiDistAvgFilterTRGI = Filters::AverageFilter(anchorNode.k, anchorNode.rssiDistAvgFilterTRGI, anchorNode.rssiDistanceTRGI);

    //Update new values at the list
    UpdateNeighborList(&anchorNode);

    std::cout << "List of Neighbor Vehicles Updated\n";
    PrintNeighborList();

    //FIXME IMplement a mechanism to discard a beacon after some round
    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 4 or more anchor nodes call multilateration method
    if(anchorNodes.size() > 3){
        //TODO Call Multilateration Method
        std::vector<Coord> positions (anchorNodes.size());
        std::vector<double> distances (anchorNodes.size());
        Multilateration::InitializePosDist(&anchorNodes, &positions, &distances, "FREE_SPACE");
        coopPosFS = Multilateration::LeastSquares(&positions, &distances, anchorNodes.size());
        coopPosFS.z = mobility->getCurrentPosition().z;
        Multilateration::InitializePosDist(&anchorNodes, &positions, &distances, "TWO_RAY_GROUND_INTERFERENCE");
        coopPosTRGI = Multilateration::LeastSquares(&positions, &distances, anchorNodes.size());
        coopPosTRGI.z = mobility->getCurrentPosition().z;
        positions.clear();
        distances.clear();
    }
    else{
        coopPosFS.x = coopPosFS.y = coopPosFS.z =  0;
        coopPosTRGI.x = coopPosTRGI.y = coopPosTRGI.z =  0;
    }

    /****************Log File with results of CP Approach
    **vehID (Neighbor) | Timestamp | MyRealPosition (SUMO) |
    **Neighbor Position (SUMO) |  Real Distance | Est. RSSI Dist FS | RSSI FS |
    **Est. RSSI Dist TRGI | RSSI TRGI | My Estimated Position (Via CP FSpace) | My Estimated Position (Via CP TRGI) |
    * */
    //fixme ONLY FOR DEBUG OF POSITION AND PROJECTIONS
    std::pair<double,double> lonlat = traci->getLonLat(mobility->getCurrentPosition());
    std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
    beaconLogFile << anchorNode.vehID
            <<'\t'<< anchorNode.timestamp
            <<'\t'<< std::setprecision(10) << lonlat.second
            <<'\t'<< std::setprecision(10) << lonlat.first
            <<'\t'<< std::setprecision(10) << mobility->getCurrentPosition().x
            <<'\t'<< std::setprecision(10) << mobility->getCurrentPosition().y
            <<'\t'<< std::setprecision(10) << mobility->getCurrentPosition().z
            <<'\t'<< std::setprecision(10) << anchorNode.realDistance
            <<'\t'<< std::setprecision(10) << anchorNode.realPosition.x
            <<'\t'<< std::setprecision(10) << anchorNode.realPosition.y
            <<'\t'<< std::setprecision(10) << anchorNode.realPosition.z
            <<'\t'<< std::setprecision(10) << anchorNode.rssiDistanceFS
            <<'\t'<< std::setprecision(10) << anchorNode.rssiFS
            <<'\t'<< std::setprecision(10) << anchorNode.rssiDistanceTRGI
            <<'\t'<< std::setprecision(10) << anchorNode.rssiTRGI
            <<'\t'<< std::setprecision(10) << anchorNode.rssiDistAvgFilterFS
            <<'\t'<< std::setprecision(10) << anchorNode.rssiDistAvgFilterTRGI
            <<'\t'<< std::setprecision(10) << coopPosFS.x
            <<'\t'<< std::setprecision(10) << coopPosFS.y
            <<'\t'<< std::setprecision(10) << coopPosFS.z
            <<'\t'<< std::setprecision(10) << coopPosTRGI.x
            <<'\t'<< std::setprecision(10) << coopPosTRGI.y
            <<'\t'<< std::setprecision(10) << coopPosTRGI.z
            << endl;
    beaconLogFile.close();

    //The begin of Cooperative Positioning Approach
}

//Update the position of a neighbor vehicle in the list
void LocAppCom::UpdateNeighborList(AnchorNode *anchorNode){
    //Verify if anchor node already exists...
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == anchorNode->vehID){
            it->vehID = anchorNode->vehID;
            it->timestamp = anchorNode->timestamp;
            it->realDistance = anchorNode->realDistance;
            it->realPosition = anchorNode->realPosition;
            it->rssiDistanceFS = anchorNode->rssiDistanceFS;
            it->rssiFS = anchorNode->rssiFS;
            it->rssiDistanceTRGI = anchorNode->rssiDistanceTRGI;
            it->rssiTRGI = anchorNode->rssiTRGI;
            it->rssiDistAvgFilterFS = anchorNode->rssiDistAvgFilterFS;
            it->rssiDistAvgFilterTRGI = anchorNode->rssiDistAvgFilterTRGI;
            it->k = anchorNode->k;
            return;
        }
    }
    anchorNodes.push_back(*anchorNode);
}


/*
 *Return size of the list of anchor nodes if the id not exists
 *or return the index in the list if the id of an existing anchor node exists
*/
void LocAppCom::getAnchorNode(int id, AnchorNode *anchorNode){
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == id){
            anchorNode->vehID = it->vehID;
            anchorNode->timestamp = it->timestamp;
            anchorNode->realDistance = it->realDistance;
            anchorNode->realPosition = it->realPosition;
            anchorNode->rssiDistanceFS = it->rssiDistanceFS;
            anchorNode->rssiFS = it->rssiFS;
            anchorNode->rssiDistanceTRGI = it->rssiDistanceTRGI;
            anchorNode->rssiTRGI = it->rssiTRGI;
            anchorNode->rssiDistAvgFilterFS = it->rssiDistAvgFilterFS;
            anchorNode->rssiDistAvgFilterTRGI = it->rssiDistAvgFilterTRGI;
            anchorNode->k = it->k;
            break;
        }
    }

}



/*Update distances from ego vehicle to another vehicles...
* As the ego vehicle need receive positions from neighborhood
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

void LocAppCom::GetGPSOutageCoordinates(){
    std::string myRoute;
    myRoute = traciVehicle->getRouteId();
    GetOutageDataFromFile(myRoute);
    std::cout << gpsOutPos.lat << gpsOutPos.lon << errorGPSOut;
    std::cout << gpsRecPos.lat << gpsRecPos.lon << errorGPSRec;
}

void LocAppCom::GetOutageDataFromFile(std::string path){
    std::string date, time, line;
    path = "../outages/"+path+".txt";
    std::fstream file(path);
    std::fstream fileOut("temp.txt");


    /*std::string date, time, line;
    double lat, lon, error;

    std::fstream file("DMATEntranceExit.txt");
    std::fstream fileOut("temp.txt", ios::out);*/

    getline(file, line); // cabecalho

    file >> date >> time >> gpsOutPos.lat >> gpsOutPos.lon >> errorGPSOut;

    file >> date >> time >> gpsRecPos.lat >> gpsRecPos.lon >> errorGPSRec;

    //cabecalho
    fileOut << line;
    //outras linhas
    while (getline(file,line)){
        fileOut << line << '\n';
    }

    fileOut.close();
    file.close();
    //std::remove(path);
    //std::rename("temp.txt", path);

}

bool LocAppCom::RecognizeOutage(){
    double prevDist;
    prevDist = distOutage;
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    geod.Inverse(this->atualSUMOPos.lat, this->atualSUMOPos.lon, this->gpsOutPos.lat,this->gpsOutPos.lon,this->distOutage);
    if(prevDist < distOutage){
        distOutage = prevDist;
        //errorDR = distOutage;
        lastGDRPos = gpsOutPos;
        errorGDR = errorGPSOut;
        ////TODO put information on GDR file
        return true;
    }
    return false;
}

bool LocAppCom::RecognizeRecover(){
    double prevDist;
    prevDist = distOutage;
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    geod.Inverse(this->atualSUMOPos.lat, this->atualSUMOPos.lon, this->gpsRecPos.lat,this->gpsRecPos.lon,this->distOutage);
    if(prevDist < distOutage){
        distOutage = prevDist;
        //errorDR = distOutage;
        lastGDRPos = gpsRecPos;
        errorGDR = errorGPSRec;
        ////TODO put information on GDR file
        DeadReckoning::getPosition(&lastGDRPos, &lastSUMOPos, &atualSUMOPos);
        DeadReckoning::getError(&errorGDR, &lastGDRPos, &atualSUMOPos);
        //put atual dr in file
        return true;
    }
    return false;


}



//bool IsInOutage(double PrevDist){
    //if(atualDist < prevDist){
    //  prevDist = atualDist;
    //  return false
    //}
    //else{
    //  return true
    //}
//}

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



