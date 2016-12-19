/*
 * Multilateration.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: liborio
 */

#include <Multilateration.hpp>

Multilateration::Multilateration() {
    // TODO Auto-generated constructor stub

}

Multilateration::~Multilateration() {
    // TODO Auto-generated destructor stub
}

Coord Multilateration::LeastSquares( std::vector<Coord> *positions, std::vector<double> *distances){
        Coord estimatedPosition;
        Coord *posToSubtract;
        double *distToSubtract;
        int i, j;
        //Minus one because the last line of the matrix will be subtracted by the others lines
        int size =  positions.size() - 1;

        //Create matrixes using the TNT library
        //Composing the Linear Equation Ax - b to be solved by LeastSquares
        TNT::Array2D<double> A(size,2);
        TNT::Array1D<double> b(size);
        TNT::Array1D<double> x(size);

        //posToSubtract = positions[size];
        //distToSubtract = (double) distances->at(size);

        for(i=0; i < size; i++){
            A[i][0] =  2.0 * (positions[i].x - positions[size].x);
            A[i][1] =  2.0 * (positions[i].y - positions[size].y);

            b[i] = pow(distances[size],2) - pow(distances[i],2) +
                   pow(positions[i].x,2) - pow(positions[size].x,2) +
                   pow(positions[i].y,2) - pow(positions[size].y,2);
        }

        JAMA::QR<double> qrFact(A);

        x = qrFact.solve(b);

        estimatedPosition.x = x[0];
        estimatedPosition.y = x[1];
        estimatedPosition.z = 0;

        return estimatedPosition;
}

std::vector<Coord> Multilateration::InitializePositions(std::list<AnchorNode> *anchorNodes){
    std::vector<Coord> positions;
    for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
        positions.push_back(it->realPosition);
    }
    return positions;
}

std::vector<double> Multilateration::InitializeDistFS(std::list<AnchorNode> *anchorNodes){
    std::vector<double> distFS;
    for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
        distFS.push_back(it->rssiDistanceFS);
    }
    return distFS;
}

std::vector<double> Multilateration::InitializeDistTRGI(std::list<AnchorNode> *anchorNodes){
    std::vector<double> distTRGI;
    for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
        distTRGI.push_back(it->rssiDistanceTRGI);
    }
    return distTRGI;
}



/*Coord Multilateration::LeastSquares(std::list<AnchorNode> *anchorNodes){
        Coord estimatedPosition;
        //std::cout << "Function Least Squares - Vehicle" << myId << '\n';
        int i, j;

        //Minus one because the last line of the matrix will be subtracted by the other
        int totalAnchorNodes = anchorNodes->size() - 1;

        //Create matrixes using the TNT library
        //Composing the Linear Equation Ax - b to be solved by LeastSquares
        TNT::Array2D<double> A(totalAnchorNodes,2);
        TNT::Array1D<double> b(totalAnchorNodes);
        TNT::Array1D<double> x(totalAnchorNodes);

        //Position of ego vehicle (that want discover your own position by the network)
        //Coord unknowNode = mobility->getCurrentPosition();

        //Subtract the position of the last anchorNode by the others in system of equations
        std::list<AnchorNode>::iterator nodeToSubtract = anchorNodes->begin();
        std::advance(nodeToSubtract, anchorNodes->size()-1);
        //std::cout << "Function Least Squares - NodeToSubtract: " << nodeToSubtract->realPosition <<' '<< nodeToSubtract->realDistance <<  '\n';
        std::list<AnchorNode>::iterator lastIter = anchorNodes->end();
        std::advance(lastIter, -1);

        //PrintNeighborList();

        i=0;
        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= lastIter; ++it){
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

        estimatedPosition.x = x[0];
        estimatedPosition.y = x[1];
        estimatedPosition.z = 0;

        return estimatedPosition;
}*/
