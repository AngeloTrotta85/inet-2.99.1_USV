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

#include "inet/mobility/single/FieldForceMobility.h"
#include "inet/common/INETMath.h"

namespace inet {

Define_Module(FieldForceMobility);

FieldForceMobility::FieldForceMobility()
{
    speed = 0;
    maxspeed = 0;
    angle = 0;
    maxacceleration = 0;
    force = Coord::ZERO;
}

void FieldForceMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);

    EV_TRACE << "initializing FieldForceMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {
        maxspeed = par("maxspeed");
        maxacceleration = par("maxacceleration");
        stationary = (speed == 0) && (maxacceleration == 0.0);

        WATCH(force);
        WATCH_RW(force.x);
        WATCH_RW(force.y);
    }
}

void FieldForceMobility::move()
{
    updateFieldForce();

    if (force != Coord::ZERO) {

        if (force.length() > maxacceleration) {
            force.normalize();
            force *= maxacceleration;
        }

        angle = acos(force.x);

        Coord direction(force);
        direction.normalize();
        lastSpeed = direction * speed;
        double elapsedTime = (simTime() - lastUpdate).dbl();
        lastPosition += lastSpeed * elapsedTime;

        // do something if we reach the wall
        Coord dummy;
        double angle_degree = 180.0*(angle/PI);
        handleIfOutside(REFLECT, dummy, dummy, angle_degree);

        // accelerate
        speed += force.length() * elapsedTime;
        if (speed <= 0) {
            speed = 0;
            //stationary = true;
        }
    }
    else {
        speed = 0;
    }
}

void FieldForceMobility::updateFieldForce(void) {
    if (dblrand() < 0.05) {
        //force = Coord((dblrand()*2.0) - 1.0, (dblrand()*2.0) - 1.0, 0);
    }
}

} /* namespace inet */
