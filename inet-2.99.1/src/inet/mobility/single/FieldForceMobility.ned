//
// Author: Emin Ilker Cetinbas (niw3_at_yahoo_d0t_com)
// Copyright (C) 2005 Emin Ilker Cetinbas
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

package inet.mobility.single;

import inet.mobility.base.MovingMobilityBase;

//
// This is a mobility driven by internal and external forces
//
// @author Angelo Trotta
//
simple FieldForceMobility extends MovingMobilityBase
{
    parameters:
        double initialX @unit(m) = default(uniform(this.constraintAreaMinX, this.constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(this.constraintAreaMinY, this.constraintAreaMaxY));
        double initialZ @unit(m) = default(uniform(this.constraintAreaMinZ, this.constraintAreaMaxZ));
        bool initFromDisplayString = default(true);
        
        bool randomMovement = default (false);			// random movement
        volatile double randomStepTimeChange @unit(s) = default(20s); // frequency of changing force angle (can be random)
        double weigthRandomMovement = default (10);		// weigth of the movement force
        
        int debugRandomRepulsivePoints = default(0);	// define a random number of repulsive force (for debug)
        
        double volatileTimeExponentialDecay = default(0.001);
        double repulsiveForceWeight = default(10);
        double defaultRepulsiveDecay = default(0.005);
        
        double maxspeed @unit(mps) = default(5mps); // max speed of the host
        double maxacceleration = default(1); // max acceleration of linear motion (m/s2)
        @class(FieldForceMobility);
}
