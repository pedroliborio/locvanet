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
        this->lastSUMOPos.x = vehCoord.x;
        this->lastSUMOPos.y = vehCoord.y;
        traci = mobility->getCommandInterface();

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

    }

}

void LocAppCom::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            //Update Vehicle Kinematics Information
            //VehicleKinematicsModule(void);
            //Update GDR Information//
            //.....
            //Update GPS Information
            //....
            //Create a beacon
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

    //Beacon Log File...
    std::cout << "Function On Beacon - Vehicle " << myId << "received a beacon from Vehicle " << wsm->getSenderAddress() << " Position Received " << wsm->getSenderPos()<<"\n";
    std::fstream beaconLogFile(std::to_string(myId)+".txt", std::fstream::app);
    beaconLogFile << wsm->getSenderAddress() << '\t' << wsm->getSenderPos() << '\t' << wsm->getTimestamp() << '\n';
    beaconLogFile.close();


    //TODO GetRSSI EV << "Vehicle:" << wsm->getSenderAddress() << "Received Power: " << wsm->getRxPower()<<"\n";
    //Here the vehicle need to maintain a vector with the position of neighbors
    AnchorNode anchorNode;
    //NeighborNode Position
    anchorNode.realPosition = wsm->getSenderPos();
    //NeighborNode Euclidean "Real" Distance
    //neighborNode.realDistance = mobility->getCurrentPosition().sqrdist(neighborNode.position);
    //TODO Calc the RSSI Distance
    //https://groups.google.com/forum/#!topic/omnetpp/2ZqWow5QGS0
    //
    //actual size of the vector of anchor nodes
    std::size_t totalAnchorNodes = anchorNodes.size();

    //If the anchor node already exists only actualize their value

    UpdateNeighborList();

    if{
        anchorNodes[wsm->getSenderAddress()] = anchorNode;
    }
    else{// New anchor node found, increase the vector and storage new anchor node information
        anchorNodes.push_back (anchorNode);
    }
    std::cout << "List of Neighbor Vehicles\n";
    for (int i=0; i < (int)totalAnchorNodes; i++){
        std::cout << "Vehicle "<< std::to_string(i) <<' '<<anchorNodes[i].realPosition <<"\n\n";
    }

    /*//TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 3 or more anchor nodes call multilateration method
    if(totalAnchorNodes > 3){
        //TODO Call Multilateration Method
        std::cout << "Function On Beacon - My real position " << mobility->getCurrentPosition() << "\n\n";
        LeastSquares();
    }*/

    //The begin of Cooperative Positioning Approach
}

void LocAppCom::UpdateNeighborList(AnchorNode anchorNode){
    //Verify if anchor node already exists...
    for(std::list<AnchorNode>::iterator it; it!= anchorNodes.end(); ++it){
        if(it->vehID == anchorNode.vehID){
            it->realPosition = anchorNode.realPosition;
        }
        else{
            anchorNodes.push_back(anchorNode);
        }
    }
}

void LocAppCom::onData(WaveShortMessage* wsm){

}

void LocAppCom::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details){
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}

void LocAppCom::GeodesicDRModule(void){
    //Here we need to get the local coordinates position
    //Convert to lat lon points and pass to GDR algorithm.
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    double latGDR, lonGDR;
    geod.Direct(this->lastGDRPos.x,this->lastGDRPos.y,bearing,distance,latGDR,lonGDR);
    //need put the trace in some output
}

/*Convert Between Local Coordinates to Geodesic Coordinates and
//calculates bearing and distance like and odometer and a gyroscope*/

void LocAppCom::VehicleKinematicsModule(void){
    //Coordinates of the sumo.net boundingbox
    double lat0 = -22.910044, lon0 = -43.207808;
    // Alternatively: const Geocentric& earth = Geocentric::WGS84();
    Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
    LocalCartesian proj(lat0, lon0, 0, earth);
    Coord pos = this->mobility->getCurrentPosition();
    double x = pos.x, y = pos.y, z = 0;
    double lat, lon, h, s12, azi1, azi2;
    //Obtain Lat Lon Coordinates
    proj.Reverse(x, y, z, lat, lon, h);
    //Update the bearing and distance with GeographicLib...
    geod.Inverse(this->lastSUMOPos.x, this->lastSUMOPos.y,lat,lon,s12,azi1,azi2);
    this->distance = s12;
    this->bearing = azi1;
    //geod.Inverse(
    //Forward = between lat lon to x, y
    //double lat = 50.9, lon = 1.8, h = 0; // Calais
    //double x, y, z;
    //proj.Forward(lat, lon, h, x, y, z);

}

void LocAppCom::LeastSquares(void){
    std::cout << "Function Least Squares - Vehicle" << myId << '\n';
    std::size_t i, j;
    //double distIesimoNode;

    //total of anchor nodes
    //we using totalAnchorNodes-1 because the last node note entry in the equation (will be subtracted by others);
    std::size_t totalAnchorNodes = anchorNodes.size() - 1;

    //Create matrixes using the TNT library
    //Composing the Linear Equation Ax - b to be solved by LeastSquares
    TNT::Array2D<double> A(totalAnchorNodes,2);
    TNT::Array1D<double> b(totalAnchorNodes);
    TNT::Array1D<double> x(totalAnchorNodes);

    //Position of ego vehicle (that want discover your own position by the network)
    Coord unknowNode = mobility->getCurrentPosition();

    //Subtract the position of one anchorNode by the others
    //We using the last node
    //TODO After, we will improve this distance needed to be calculated by RSSI
    AnchorNode nodeToSubtract;
    nodeToSubtract.realPosition = anchorNodes[totalAnchorNodes].realPosition;
    nodeToSubtract.realDistance = nodeToSubtract.realPosition.distance(unknowNode);
    std::cout << "Function Least Squares - Distance to NodeSubtract " << nodeToSubtract.realDistance << '\n';

    for (i=0; i < totalAnchorNodes; i++){
        //Calculate the distance from the Neighbor node to the unknowNode... Will be by RSSI after...
        anchorNodes[i].realDistance = anchorNodes[i].realPosition.distance(unknowNode);
        std::cout << "Function Least Squares - Distance to AnchorNode "<< i <<" "<< anchorNodes[i].realDistance << '\n';
        //Filling the Matrix A
        A[i][0] =  2.0 * (anchorNodes[i].realPosition.x - nodeToSubtract.realPosition.x);
        A[i][0] =  2.0 * (anchorNodes[i].realPosition.y - nodeToSubtract.realPosition.y);

        //Filling Matrix b
        b[i] = pow(nodeToSubtract.realDistance,2) - pow(anchorNodes[i].realDistance,2) +
               pow(anchorNodes[i].realPosition.x,2) - pow(nodeToSubtract.realPosition.x,2) +
               pow(anchorNodes[i].realPosition.y,2) - pow(nodeToSubtract.realPosition.y,2);
    }

    JAMA::QR<double> qrFact(A);
    x = qrFact.solve(b);

    //Debugging values
    std::cout << "Function Least Squares - Matrix A:"<<"\n";
    for(i=0; i < totalAnchorNodes; i++){
        std::cout << A[i][0] << " - " << A[i][1] << "\n";
    }

    std::cout << "Function Least Squares - Matrix b:"<<"\n";
    for(i=0; i < totalAnchorNodes; i++){
        std::cout << b[i] << "\n";
    }

    std::cout << "Function Least Squares - Matrix X:"<<"\n";
    j  = x.dim1();
    std::cout << j<< '\n';
    for(i=0; i < j; i++){
        std::cout << x[i] << "\n";
    }

    std::cout << "Function Least Squares - My real position " << mobility->getCurrentPosition() << "\n\n";

    //Verify the problem with 3D position affecting the calculation...
}


