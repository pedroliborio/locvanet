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

    //Beacon Log File...
    std::cout << "Function On Beacon - Vehicle " << myId << "received a beacon from Vehicle " << wsm->getSenderAddress()
            << " Position Received "<< wsm->getSenderPos()
            << " Potency received" << wsm->getRxPower() << "\n";

    //Estimated distance by RSSI..
    //double distRSSI = 0.00404913 * pow(10,(-wsm->getRxPower()/20)) * sqrt(20);

    std::fstream beaconLogFile(std::to_string(myId)+".txt", std::fstream::app);
    beaconLogFile << wsm->getTimestamp() << '\t' << wsm->getSenderAddress() << '\t' << wsm->getSenderPos() << '\t' << wsm->getSenderPos().distance(mobility->getCurrentPosition()) << '\t' << wsm->getRxPower() << endl;
    beaconLogFile.close();


    //std::fstream rssiLogFile(std::to_string(myId)+"-RSSI-DIST.txt", std::fstream::app);
    //rssiLogFile <<




    //TODO GetRSSI EV << "Vehicle:" << wsm->getSenderAddress() << "Received Power: " << wsm->getRxPower()<<"\n";
    //Here the vehicle need to maintain a vector with the position of neighbors
    AnchorNode anchorNode;
    anchorNode.realPosition = wsm->getSenderPos();
    anchorNode.vehID = wsm->getSenderAddress();
    //neighborNode.realDistance = mobility->getCurrentPosition().sqrdist(neighborNode.position);
    //TODO Calc the RSSI Distancet
    //https://groups.google.com/forum/#!topic/omnetpp/2ZqWow5QGS0
    std::cout << "List of Neighbor Vehicles Before Update\n";
    PrintNeighborList();

    //Update the list of neighbors vehicles
    UpdateNeighborList(anchorNode);
    //Update distances by radio ranging
    UpdateNeighborListDistances();

    std::cout << "List of Neighbor Vehicles Updated\n";
    PrintNeighborList();

    //TODO Timestamp for compute the ttl of the beacon and use it for discard after some time
    //TODO Discard anchor node information with timestamp > than a determined threshold (maybe 100ms)...
    //If there are 3 or more anchor nodes call multilateration method
    if(anchorNodes.size() > 3){
        //TODO Call Multilateration Method
        std::cout << "Function On Beacon - My real position " << mobility->getCurrentPosition() << "\n\n";
        LeastSquares();
    }

    //The begin of Cooperative Positioning Approach
}

//Update thea position of a neighbor vehicle in the list
void LocAppCom::UpdateNeighborList(AnchorNode anchorNode){

    //Verify if anchor node already exists...
    for(std::list<AnchorNode>::iterator it=anchorNodes.begin(); it!= anchorNodes.end(); ++it){
        if(it->vehID == anchorNode.vehID){
            it->realPosition = anchorNode.realPosition;
            return;
        }
    }
    anchorNodes.push_back(anchorNode);
}

//Update distances from ego vehicle to another vehicles...
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

void LocAppCom::CalcDistRSSI(void){




}


//Calculate the distance based on free space model for different alpha values
double LocAppCom::FreeSpace(double alpha, double c, double f, double lambda, double pTx, double d){
    double rssi;
    double distBasRSSI;

    /*rssi = 10*log10(pTx) - 10 * log10((16* M_PI * M_PI * pow(d, alpha)) / pow(lambda,alpha));
    print("Distance Input:", d, "\n")*/

    distBasRSSI = (lambda / (pow((4 * M_PI),(2/alpha)) ) ) * (pow(pTx, (1/alpha))) * (pow(10,( - (rssi / (10*alpha) ) )));

    //print("Distance retrieved from RSSI:", distBasRSSI, "\n")

    return rssi;
}

//Calculate the distance based on Two Ray Ground Interference for different epsilon values
double LocAppCom::TwoRayInterferenceModel(double c, double f, double lambda, double pTx, double d, double ht, double hr, double epsilonR){
    //Simplified model...
    //rssi = 10*math.log10(20) - 20 * math.log10((d*d)/(1.895 * 1.895))
    //return rssi
    //Complete model...
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi, attenuation;
    double rssi;
    double distBasRSSI;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)
    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path
    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta
    cosTheta = d/distRef; //cos of the angle of incidence
    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection
    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays
    attenuation = pow(4 * M_PI * (d/lambda) * 1/(sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) )), 2); //# mw

    /*rssi = 10*math.log10(pTx) - 10 * math.log10(attenuation);

    //print("Distance Input:", d, "\n")*/

    distBasRSSI = sqrt(pTx) *  (lambda / 4 * M_PI) * (pow(10,( (-20 -(rssi)) / 20)))  * (sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) ));

    //print("Distance retrieved from RSSI:", distBasRSSI, "\n")
    return rssi;
}

//Least Squares Method to Trilateration
void LocAppCom::LeastSquares(void){
    std::cout << "Function Least Squares - Vehicle" << myId << '\n';
    int i, j;

    //Minus one because the last line of the matrix will be subtracted by the other
    int totalAnchorNodes = anchorNodes.size() - 1;

    //Create matrixes using the TNT library
    //Composing the Linear Equation Ax - b to be solved by LeastSquares
    TNT::Array2D<double> A(totalAnchorNodes,2);
    TNT::Array1D<double> b(totalAnchorNodes);
    TNT::Array1D<double> x(totalAnchorNodes);

    //Position of ego vehicle (that want discover your own position by the network)
    Coord unknowNode = mobility->getCurrentPosition();

    //Subtract the position of the last anchorNode by the others in system of equations
    std::list<AnchorNode>::iterator nodeToSubtract = anchorNodes.begin();
    std::advance(nodeToSubtract, anchorNodes.size()-1);
    std::cout << "Function Least Squares - NodeToSubtract: " << nodeToSubtract->realPosition <<' '<< nodeToSubtract->realDistance <<  '\n';
    std::list<AnchorNode>::iterator lastIter = anchorNodes.end();
    std::advance(lastIter, -1);

    PrintNeighborList();

    i=0;
    for(std::list<AnchorNode>::iterator it = anchorNodes.begin(); it!= lastIter; ++it){
        //Filling the Matrix A
        std::cout << "node on list"<< it->realPosition <<' '<< it->realDistance << endl;
        std::cout << "node to subtract"<< nodeToSubtract->realPosition <<' '<< nodeToSubtract->realDistance << endl;
        //std::cout << "node to subtract"<< anchorNodes.end()->realPosition <<' '<< anchorNodes.end()->realDistance << endl;

        A[i][0] =  2.0 * (it->realPosition.x - nodeToSubtract->realPosition.x);
        A[i][1] =  2.0 * (it->realPosition.y - nodeToSubtract->realPosition.y);

        //Filling Matrix b
        b[i] = pow(nodeToSubtract->realDistance,2) - pow(it->realDistance,2) +
               pow(it->realPosition.x,2) - pow(nodeToSubtract->realPosition.x,2) +
               pow(it->realPosition.y,2) - pow(nodeToSubtract->realPosition.y,2);
        i++;
    }
    std::cout << i << endl;

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


