/*
 * TwoRayInterference.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#include <RSSI/TwoRayInterference.h>

TwoRayInterference::TwoRayInterference() {
    // TODO Auto-generated constructor stub

}

TwoRayInterference::~TwoRayInterference() {
    // TODO Auto-generated destructor stub
}

TwoRayInterference::getRSSI(double d){
    double rssi;
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi, attenuation;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)

    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path

    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta

    cosTheta = d/distRef; //cos of the angle of incidence

    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection

    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays

    attenuation = pow(4 * M_PI * (d/lambda) * 1/(sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) )), 2); //# mw

    rssi = 10*log10(pTx) - 10 * log10(attenuation);

    return rssi;
}

TwoRayInterference::getDistance(double rssi, double d){
    double distLOS, distRef, sinTheta, cosTheta, gamma, phi;

    distLOS = sqrt( pow (d,2) + pow((ht - hr),2) ); //distance in the LOS (Line Of sight)
    distRef = sqrt( pow (d,2) + pow((ht + hr),2) ); //distance in the reflection path
    sinTheta = (ht + hr) / distRef; //sin of the incidence angle theta
    cosTheta = d/distRef; //cos of the angle of incidence
    gamma = (sinTheta - sqrt(epsilonR - pow(cosTheta,2))) / (sinTheta + sqrt(epsilonR - pow(cosTheta,2))); //Coeficiente of reflection
    phi = (2*M_PI/lambda * (distLOS - distRef)); //Phase difference of two interfereing rays

    distance =  sqrt(pTx) * (lambda / 4 * M_PI) * (pow(10,( (-20 -(rssi)) / 20)))  *
                (sqrt( (pow((1 + gamma * cos(phi)),2) + pow(gamma,2) * pow(sin(phi),2)) ));

    return distance;
}
