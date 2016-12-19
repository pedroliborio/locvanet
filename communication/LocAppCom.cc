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
    //TODO Calc the RSSI Distance
    switch(lossModel){
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
    }

    //std::cout << "List of Neighbor Vehicles Before Update\n";
    //PrintNeighborList();

    //Update the list of neighbors vehicles
    UpdateNeighborList(anchorNode);

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
        LeastSquares();
    }
    else{
        coopPos.x = coopPos.y = coopPos.z =  0;
    }

    //Log File with results of CP Approach
    //MyRealPosition (SUMO) | My Estimated Position (Via CP) | vehID (Neighbor) | Timestamp | Neighbor Position (SUMO) |  Real Distance | Est. RSSI Dist | RSSI

    std::fstream beaconLogFile(std::to_string(myId)+'-'+std::to_string(timeSeed)+".txt", std::fstream::app);
    beaconLogFile << mobility->getCurrentPosition().x <<'\t'<< mobility->getCurrentPosition().y <<'\t'<< mobility->getCurrentPosition().z
                <<'\t'<< coopPos.x << '\t'<< coopPos.y << '\t'<< coopPos.z <<'\t'<<  anchorNode.vehID <<'\t'<< anchorNode.timestamp
                <<'\t'<< anchorNode.realPosition.x <<'\t'<< anchorNode.realPosition.y <<'\t'<< anchorNode.realPosition.z
                <<'\t'<< anchorNode.realDistance <<'\t'<< anchorNode.rssiDistance <<'\t'<< anchorNode.rssi <<'\t' <<endl;
    beaconLogFile.close();

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
            it->rssi = anchorNode.rssi;
            it->rssiDistance = anchorNode.rssiDistance;
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

    //Abou Geocentric Coordinates:
    /**
      * \brief %Geocentric coordinates
      *
      * Convert between geodetic coordinates latitude = \e lat, longitude = \e
      * lon, height = \e h (measured vertically from the surface of the ellipsoid)
      * to geocentric coordinates (\e X, \e Y, \e Z).  The origin of geocentric
      * coordinates is at the center of the earth.  The \e Z axis goes thru the
      * north pole, \e lat = 90&deg;.  The \e X axis goes thru \e lat = 0,
      * \e lon = 0.  %Geocentric coordinates are also known as earth centered,
      * earth fixed (ECEF) coordinates.
      *
      * The conversion from geographic to geocentric coordinates is
      * straightforward.  For the reverse transformation we use
      * - H. Vermeille,
      *   <a href="https://dx.doi.org/10.1007/s00190-002-0273-6"> Direct
      *   transformation from geocentric coordinates to geodetic coordinates</a>,
      *   J. Geodesy 76, 451--454 (2002).
      * .
      * Several changes have been made to ensure that the method returns accurate
      * results for all finite inputs (even if \e h is infinite).  The changes are
      * described in Appendix B of
      * - C. F. F. Karney,
      *   <a href="http://arxiv.org/abs/1102.1215v1">Geodesics
      *   on an ellipsoid of revolution</a>,
      *   Feb. 2011;
      *   preprint
      *   <a href="http://arxiv.org/abs/1102.1215v1">arxiv:1102.1215v1</a>.
      * .
      * Vermeille similarly updated his method in
      * - H. Vermeille,
      *   <a href="https://dx.doi.org/10.1007/s00190-010-0419-x">
      *   An analytical method to transform geocentric into
      *   geodetic coordinates</a>, J. Geodesy 85, 105--117 (2011).
      * .
      * See \ref geocentric for more information.
      *
      * The errors in these routines are close to round-off.  Specifically, for
      * points within 5000 km of the surface of the ellipsoid (either inside or
      * outside the ellipsoid), the error is bounded by 7 nm (7 nanometers) for
      * the WGS84 ellipsoid.  See \ref geocentric for further information on the
      * errors.
      *
      * Example of use:
      * \include example-Geocentric.cpp
      *
      * <a href="CartConvert.1.html">CartConvert</a> is a command-line utility
      * providing access to the functionality of Geocentric and LocalCartesian.
      **********************************************************************/



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

double LocAppCom::FreeSpaceRSSI(double d){
    double rssi;
    //double distBasRSSI;

    rssi = 10*log10(pTx) - 10 * log10((16* M_PI * M_PI * pow(d, alpha)) / pow(lambda,alpha));
    //print("Distance Input:", d, "\n")

    //distBasRSSI = (lambda / (pow((4 * M_PI),(2/alpha)) ) ) * (pow(pTx, (1/alpha))) * (pow(10,( - (rssi / (10*alpha) ) )));

    //print("Distance retrieved from RSSI:", distBasRSSI, "\n")

    return rssi;
}

double LocAppCom::FreeSpaceRSSIDist(double rssi){
    double distRSSI;
    distRSSI = (lambda / (pow((4 * M_PI),(2/alpha)) ) ) * (pow(pTx, (1/alpha))) * (pow(10,( - (rssi / (10*alpha) ) )));
    return distRSSI;
}

double LocAppCom::TwoRayInterferenceRSSI(double d){
    //Simplified model...
    //rssi = 10*math.log10(20) - 20 * math.log10((d*d)/(1.895 * 1.895))
    //return rssi
    //Complete model...
    double ht, hr;
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi, attenuation;
    double rssi;
    //double distBasRSSI;

    ht = hr = 1.895;
    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)
    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path
    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta
    cosTheta = d/distRef; //cos of the angle of incidence
    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection
    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays
    attenuation = pow(4 * M_PI * (d/lambda) * 1/(sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) )), 2); //# mw

    rssi = 10*log10(pTx) - 10 * log10(attenuation);

    //print("Distance Input:", d, "\n")*/
    //distBasRSSI = sqrt(pTx) *  (lambda / 4 * M_PI) * (pow(10,( (-20 -(rssi)) / 20)))  * (sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) ));
    //print("Distance retrieved from RSSI:", distBasRSSI, "\n")
    return rssi;
}

double LocAppCom::TwoRayInterferenceRSSIDist(double rssi, double d){
    //Simplified model...
    //rssi = 10*math.log10(20) - 20 * math.log10((d*d)/(1.895 * 1.895))
    //return rssi
    //Complete model...
    double ht, hr;
    ht = hr = 1.895;
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi;
    //double rssi;
    double distBasRSSI;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)
    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path
    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta
    cosTheta = d/distRef; //cos of the angle of incidence
    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection
    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays
    //attenuation = pow(4 * M_PI * (d/lambda) * 1/(sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) )), 2); //# mw

    //rssi = 10*log10(pTx) - 10 * log10(attenuation);

    //print("Distance Input:", d, "\n")*/

    distBasRSSI = sqrt(pTx) * (lambda / 4 * M_PI) * (pow(10,( (-20 -(rssi)) / 20)))  * (sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) ));

    //print("Distance retrieved from RSSI:", distBasRSSI, "\n")
    return distBasRSSI;
}

//Least Squares Method to Multilateration
void LocAppCom::LeastSquares(void){
    //std::cout << "Function Least Squares - Vehicle" << myId << '\n';
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
    //std::cout << "Function Least Squares - NodeToSubtract: " << nodeToSubtract->realPosition <<' '<< nodeToSubtract->realDistance <<  '\n';
    std::list<AnchorNode>::iterator lastIter = anchorNodes.end();
    std::advance(lastIter, -1);

    PrintNeighborList();

    i=0;
    for(std::list<AnchorNode>::iterator it = anchorNodes.begin(); it!= lastIter; ++it){
        //Filling the Matrix A
        //std::cout << "node on list"<< it->realPosition <<' '<< it->realDistance << endl;
        //std::cout << "node to subtract"<< nodeToSubtract->realPosition <<' '<< nodeToSubtract->realDistance << endl;
        //std::cout << "node to subtract"<< anchorNodes.end()->realPosition <<' '<< anchorNodes.end()->realDistance << endl;

        A[i][0] =  2.0 * (it->realPosition.x - nodeToSubtract->realPosition.x);
        A[i][1] =  2.0 * (it->realPosition.y - nodeToSubtract->realPosition.y);

        //Filling Matrix b
        b[i] = pow(nodeToSubtract->rssiDistance,2) - pow(it->rssiDistance,2) +
               pow(it->realPosition.x,2) - pow(nodeToSubtract->realPosition.x,2) +
               pow(it->realPosition.y,2) - pow(nodeToSubtract->realPosition.y,2);
        i++;
    }
    //std::cout << i << endl;

    JAMA::QR<double> qrFact(A);
    x = qrFact.solve(b);

    //Debugging values
    //std::cout << "Function Least Squares - Matrix A:"<<"\n";
    //for(i=0; i < totalAnchorNodes; i++){
        //std::cout << A[i][0] << " - " << A[i][1] << "\n";
    //}

    //std::cout << "Function Least Squares - Matrix b:"<<"\n";
    //for(i=0; i < totalAnchorNodes; i++){
        //std::cout << b[i] << "\n";
    //}

    //std::cout << "Function Least Squares - Matrix X:"<<"\n";
    //j  = x.dim1();
    //std::cout << j<< '\n';
    //for(i=0; i < j; i++){
        //std::cout << x[i] << "\n";
    //}

    //std::cout << "Function Least Squares - My real position " << mobility->getCurrentPosition() << "\n\n";

    coopPos.x = x[0];
    coopPos.y = x[1];
    coopPos.z = mobility->getCurrentPosition().z;
}


void LocAppCom::finish(){
    BaseWaveApplLayer::finish();

}



