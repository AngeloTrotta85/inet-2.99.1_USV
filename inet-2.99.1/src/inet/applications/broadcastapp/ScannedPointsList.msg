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



cplusplus {{
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"


}}

namespace inet;

cplusplus {{    

struct ScannedPoint {
    	unsigned int scanID;
        Coord position;
        simtime_t timestamp;
       	double watt_read;
        bool decisionMade;
        int scannerAddr;
        //double weight;
        //double decade_factor;
};

}}    

struct ScannedPoint;
class Coord;

packet ScannedPointsList
{
    int nodeAddr;
    Coord nodePosition;
    ScannedPoint scanPoints[];
    ScannedPoint scanPoints_fromOthers[];
}
