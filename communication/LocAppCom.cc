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
            EV << "Vehicle:" << wsm->getSenderAddress() << "Position Sent:" << wsm->getSenderPos()<<"\n";
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

    EV << "Vehicle:" << wsm->getSenderAddress() << "Position Received: " << wsm->getSenderPos()<<"\n";

    //Here the vehicle need to maintain a vector with the position of neighbors
    NeighborNode neighborNode;
    //NeighborNode Position
    neighborNode.realPosition = wsm->getSenderPos();
    //NeighborNode Euclidean "Real" Distance
    //neighborNode.realDistance = mobility->getCurrentPosition().sqrdist(neighborNode.position);
    //TODO Calc the RSSI Distance
    //https://groups.google.com/forum/#!topic/omnetpp/2ZqWow5QGS0
    //
    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time


    listNeighbors.push_front(neighborNode);

    if(listNeighbors.size() > 3){
        //TODO Call Multitrilateration Method
        EV << "My real position out LS" << mobility->getCurrentPosition() << "\n";
        LeastSquares();
    }

    //The begin of Cooperative Positioning Approach
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
    //double lVillasat = 50.9, lon = 1.8, h = 0; // Calais
    //double x, y, z;
    //proj.Forward(lat, lon, h, x, y, z);

}

void LocAppCom::LeastSquares(void){
    int i = 0, j;

    //Capture the total of neighbors minus the node to subtract
    //in the overdetermined system of equations (-1)
    int totalAnchorNodes = listNeighbors.size() - 1;

    //Create matrixes using the TNT library
    //Composing the Linear Equation Ax - b to be solved by LeastSquares
    TNT::Array2D<double> A(totalAnchorNodes,2);
    TNT::Array1D<double> b(totalAnchorNodes);
    TNT::Array1D<double> x(totalAnchorNodes);

    //Position of ego vehicle (that want discover your own position by the network)
    Coord unknowNode = mobility->getCurrentPosition();

    //Subtract on equation by the others...
    //We using the first coordinate
    NeighborNode nodeToSubtract;
    nodeToSubtract.realPosition = listNeighbors.begin()->realPosition;
    nodeToSubtract.realDistance = nodeToSubtract.realPosition.distance(unknowNode);

    //Beginning from the second position (advance one position in the list)
    std::list<NeighborNode>::iterator it = listNeighbors.begin();
    std::advance(it, 1);

    for (; it != listNeighbors.end(); it++){
        //Calculate de distance from the Neighbor node to the unknowNode...
        it->realDistance = it->realPosition.distance(unknowNode);
        //Filling the Matrix A
        A[i][0] =  2.0 * (it->realPosition.x - nodeToSubtract.realPosition.x);
        A[i][0] =  2.0 * (it->realPosition.y - nodeToSubtract.realPosition.y);
        //Filling Matrix b
        b[i] = pow(nodeToSubtract.realDistance,2) - pow(it->realDistance,2) +
               pow(it->realPosition.x,2) - pow(nodeToSubtract.realPosition.x,2) +
               pow(it->realPosition.y,2) - pow(nodeToSubtract.realPosition.y,2);
        i++;
    }

    JAMA::QR<double> qrFact(A);
    X = qrFact.solve(b);

    //Debugging values
    EV << "Matrix A:"<<"\n";
    for(i=0; i < totalAnchorNodes; i++){
        EV << A[i][0] << " - " << A[i][1] << "\n";
    }

    EV << "Matrix b:"<<"\n";
    for(i=0; i < totalAnchorNodes; i++){
        EV << b[i][0] << "\n";
    }

    EV << "Matrix X:"<<"\n";
    j  = X.dim1();
    for(i=0; i < j; i++){
        EV << X[i][0] << "\n";
    }

    EV << "My real position in LS" << mobility->getCurrentPosition() << "\n";

    //Verify the problem with 3D position affecting the calculation...
}

/*void LeastSquares(){
 * // subtract one equation by the others... We using teh first equation
    int i = 0, j;
    double dIesimo, dNodeToSubtract;
    //Create an matrix
    TNT::Array2D<double> A(TAM-1,2);
    TNT::Array1D<double> b(TAM-1);
    TNT::Array1D<double> x(TAM-1);

    Coord positions[TAM];
    Coord unknowNode;

    unknowNode.x = 407.30;
    unknowNode.y = 209.28;

    Coord nodeToSubtract;

    nodeToSubtract = positions[TAM-1];
    cout << nodeToSubtract.x << ' ' << nodeToSubtract.y << " Node To subtract\n\n";

    cout << unknowNode.x << ' ' << unknowNode.y << "Unknow Node\n\n";

    cout << "List of nodes\n\n";
    for(i=0; i < TAM-1; i++){
        cout << positions[i].x << ' ' << positions[i].y << '\n';
    }

    for (i=0; i < TAM-1; i++){

        A[i][0] =  2.0 * (positions[i].x - nodeToSubtract.x);
        A[i][1] =  2.0 * (positions[i].y - nodeToSubtract.y);

        dIesimo = Distance(unknowNode,positions[i]);
        dNodeToSubtract = Distance(unknowNode,nodeToSubtract);

        cout << "DIesimo:" << dIesimo << '\n';
        cout << "DNodeToSubtract:" << dNodeToSubtract << '\n';

        b[i] =  ( pow(dNodeToSubtract,2) - pow(dIesimo,2) ) +
                ( pow(positions[i].x,2) - pow(nodeToSubtract.x,2) ) +
                ( pow(positions[i].y,2) - pow(nodeToSubtract.y,2) );
    }

    JAMA::QR<double> qrFact(A);
    x = qrFact.solve(b);

    //Debugging values
    cout << "Matrix A:"<<"\n";
    for(i=0; i < TAM-1; i++){
        cout << A[i][0] << " - " << A[i][1] << "\n";
    }

    std::cout << "Matrix b:"<<"\n";
    for(i=0; i < TAM-1; i++){
        cout << b[i] << "\n";
    }

    std::cout << "Final Position:"<<"\n";
    j  = x.dim1();
    for(i=0; i < j; i++){
        cout << x[i] << "\n";
    }

}*/



