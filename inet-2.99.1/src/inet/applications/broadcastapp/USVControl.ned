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

package inet.applications.broadcastapp;

//
// TODO auto-generated type
//
simple USVControl
{
    double defaultRepulsiveWeigth = default(10); 	// define the weigth parameter in the repulsive force calculation
    double desiredWeigthRatio = default(0.05);		// define the desired ratio of the weigth to calculate the distance from 2 points
    bool pathLossMapAvailable = default(true);		// define if the control module has the pathloss map available
    double checkScanTimeStep @unit(s) = default (3s);	// time to check the scan method
    
    @display("i=misc/joystick");
}