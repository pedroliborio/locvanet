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

Coord Multilateration::LeastSquares(std::vector<Coord> *positions, std::vector<double> *distances, int size){
        Coord estimatedPosition(0,0,0);
        int i;
        //Minus one because the last line of the matrix will be subtracted by the others lines
        size--;

        //Create matrixes using the TNT library
        //Composing the Linear Equation Ax - b to be solved by LeastSquares
        TNT::Array2D<double> A(size,2);
        TNT::Array1D<double> b(size);
        TNT::Array1D<double> x(size);

        //posToSubtract = positions[size];
        //distToSubtract = (double) distances->at(size);

        for(i=0; i < size; i++){
            std::cout << positions->at(i) << "\t" << distances->at(i) << endl;
            A[i][0] =  2.0 * (positions->at(i).x - positions->at(size).x);
            A[i][1] =  2.0 * (positions->at(i).y - positions->at(size).y);

            b[i] = pow(distances->at(size),2) - pow(distances->at(i),2) +
                   pow(positions->at(i).x,2) - pow(positions->at(size).x,2) +
                   pow(positions->at(i).y,2) - pow(positions->at(size).y,2);
        }

        JAMA::QR<double> qrFact(A);

        x = qrFact.solve(b);

        if(x.dim1()> 0){
            estimatedPosition.x = x[0];
            estimatedPosition.y = x[1];
            estimatedPosition.z = 0;
        }

        return estimatedPosition;
}

 void Multilateration::InitializePosDist(std::list<AnchorNode> *anchorNodes, std::vector<Coord> *positions, std::vector<double> *distances, std::string model){
    int i;
    i=0;
    for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
        positions->at(i) = it->realPosition;
        i++;
    }

    i=0;
    if(model == "FREE_SPACE"){//FreeSpace
        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
            distances->at(i) =it->rssiDistanceFS;
            i++;
        }
    }
    else{//Two ray Interference
        for(std::list<AnchorNode>::iterator it = anchorNodes->begin(); it!= anchorNodes->end(); ++it){
            distances->at(i) =it->rssiDistanceTRGI;
            i++;
        }
    }
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

