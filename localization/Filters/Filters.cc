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

#include <Filters/Filters.h>

Filters::Filters() {
    // TODO Auto-generated constructor stub

}

Filters::~Filters() {
    // TODO Auto-generated destructor stub
}

//void AverageFilter(int k, int mean, double newValue, double *newMean){
/********Average Filter
 * Is a recursive function that receives the actual k_th element and
 * compute the actual mean_k based only in the previous mean_k-1 and the
 * new value x_k of the dataset that has been measured.
 *
 */
double Filters::AverageFilter(int k, double mean, double x_k){
    double newMean;
    double alpha = (k-1)/(double)k;
    newMean = (alpha * mean) + (1-alpha) * x_k;
}
