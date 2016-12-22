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

#ifndef LOCALIZATION_FILTERS_FILTERS_H_
#define LOCALIZATION_FILTERS_FILTERS_H_

class Filters {
public:
    Filters();
    virtual ~Filters();
    static double AverageFilter(int k, double mean, double x_k);
};

#endif /* LOCALIZATION_FILTERS_FILTERS_H_ */
