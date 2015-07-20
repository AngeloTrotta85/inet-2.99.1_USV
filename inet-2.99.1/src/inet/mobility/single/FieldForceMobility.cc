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
    exploringForce = Coord::ZERO;
    debugRandomRepulsivePoints = 0;

    idxRPL = 1000;
}

void FieldForceMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);

    EV_TRACE << "initializing FieldForceMobility stage " << stage << endl;
    if (stage == INITSTAGE_LOCAL) {

        maxspeed = par("maxspeed");
        maxacceleration = par("maxacceleration");
        stationary = (speed == 0) && (maxacceleration == 0.0);

        defaultVolatileTimeDecay = par("volatileTimeExponentialDecay").doubleValue();
        defaultRepulsiveForceWeight = par("repulsiveForceWeight").doubleValue();
        defaultRepulsiveDecay = par("repulsiveDecay").doubleValue();

        randomMovement = par("randomMovement").boolValue();
        weigthRandomMovement = par("weigthRandomMovement").doubleValue();
        stepRandomChangeTime = par("randomStepTimeChange");
        nextRandomChangeTime = simTime() + (stepRandomChangeTime * dblrand());

        debugRandomRepulsivePoints = par("debugRandomRepulsivePoints");

        //WATCH_LIST(repulsivePointsList);
        WATCH(exploringForce);
        WATCH(force);
    }
    else if (stage == INITSTAGE_LAST) {

        for (int i = 0; i < debugRandomRepulsivePoints; i++) {
            repulsive_point_t rp;

            rp.timestamp = simTime();
            rp.decade_factor = 0.001;//0.005;
            rp.weight = 10;

            rp.position.z = 0;
            rp.position.x = dblrand() * ((constraintAreaMax.x - constraintAreaMin.x)) + constraintAreaMin.x;
            rp.position.y = dblrand() * ((constraintAreaMax.y - constraintAreaMin.y)) + constraintAreaMin.y;

            EV << "Repulsive point " << i << ": " << rp.position << endl;

            repulsivePointsList[idxRPL++] = rp;
        }
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

    //Reset the force
    force = Coord::ZERO;

    //Add node exploring-force ************************************************************
    if (randomMovement) {
        if (nextRandomChangeTime <= simTime()) {
            do {
                exploringForce.x = (dblrand() * 2.0) - 1.0;
                exploringForce.y = (dblrand() * 2.0) - 1.0;
            } while(exploringForce.length() < 0.1);     // to avoid math problems
            exploringForce.normalize();
            exploringForce *= weigthRandomMovement;

            speed = 0;

            EV << "NEW exploring force: " << exploringForce << endl;

            nextRandomChangeTime += stepRandomChangeTime + dblrand();
        }
    }
    force += exploringForce;
    //*********************************************************************************************
    //END Add node exploring-force ************************************************************



    //Add volatile repulsive-force ************************************************************
    Coord volRepulsiveSum = Coord::ZERO;

    for (std::map<unsigned int, repulsive_point_t>::iterator it = volatileRepulsivePointsList.begin(); it != volatileRepulsivePointsList.end(); it++) {
        repulsive_point_t *actP = &(it->second);

        Coord actForce = calcRepulsiveForceTimeDecade(actP, defaultVolatileTimeDecay);

        volRepulsiveSum += actForce;
    }

    EV << "Adding volatile repulsive forces sum: " << volRepulsiveSum << endl;
    force += volRepulsiveSum;
    //*********************************************************************************************
    //END Add volatile repulsive-force ************************************************************


    // Add repulsive forces ************************************************************
    Coord repulsiveSum = Coord::ZERO;

    for (std::map<unsigned int, repulsive_point_t>::iterator it = repulsivePointsList.begin(); it != repulsivePointsList.end(); it++) {
        repulsive_point_t *actP = &(it->second);

        Coord actForce = calcRepulsiveForce(actP);

        //EV << "Repulsive force from " << actP->position << " to me at " << lastPosition << " is " << actForce << endl;

        repulsiveSum += actForce;
    }
    EV << "Adding repulsive forces sum: " << repulsiveSum << endl;

    force += repulsiveSum;
    //*********************************************************************************************
    // END Add repulsive forces ************************************************************
}

Coord FieldForceMobility::calcRepulsiveForce(repulsive_point_t *rp) {
    Coord ris = lastPosition - rp->position;
    double field;

    //double field = rp->weight * exp(-(rp->decade_factor * lastPosition.distance(rp->position)));
    //double sq_distance = pow(lastPosition.x - rp->position.x, 2.0) + pow(lastPosition.y - rp->position.y, 2.0);
    if (lastPosition == rp->position) {
        ris = Coord(dblrand() - 0.5, dblrand() - 0.5);
        field = rp->weight;
    }
    else {
        double sq_distance = lastPosition.sqrdist(rp->position);
        field = rp->weight * exp(-(rp->decade_factor * sq_distance));
    }

    ris.normalize();
    ris *= field;

    return ris;
}

Coord FieldForceMobility::calcRepulsiveForceTimeDecade(repulsive_point_t *rp, double timeDecateFac) {
    Coord ris = calcRepulsiveForce(rp);

    //EV << "Repulsive volatile force " << ris << " - distance " << rp->position.distance(lastPosition)
    //        << " - time passed " << (simTime() > rp->timestamp) << " [" << (simTime() - rp->timestamp).dbl() << "]" << endl;

    if (simTime() > rp->timestamp) {
        ris *= exp(-((simTime() - rp->timestamp).dbl() * timeDecateFac));
    }

    //EV << "FINAL volatile force " << ris << endl;

    return ris;
}

void FieldForceMobility::addPersistentRepulsiveForce(unsigned int id, const Coord& pos, double weight, double decade_factor) {
    repulsive_point_t rp;

    rp.timestamp = simTime();
    rp.decade_factor = decade_factor;
    rp.weight = weight;
    rp.position = pos;

    repulsivePointsList[id] = rp;
}

void FieldForceMobility::addPersistentRepulsiveForce(unsigned int id, const Coord& pos) {
    addPersistentRepulsiveForce(id, pos, defaultRepulsiveForceWeight, defaultRepulsiveDecay);
}

void FieldForceMobility::setVolatileRepulsiveForce(unsigned int addr, const Coord& pos, double weight, double decade_factor) {
    repulsive_point_t rp;


    //EV << "XXXXXXXXXXXXXX Adding repulsive force from node " << addr << endl;

    rp.timestamp = simTime();
    rp.decade_factor = decade_factor;
    rp.weight = weight;
    rp.position = pos;

    volatileRepulsivePointsList[addr] = rp;

}

void FieldForceMobility::setVolatileRepulsiveForce(unsigned int addr, const Coord& pos) {
    setVolatileRepulsiveForce(addr, pos, defaultRepulsiveForceWeight, defaultRepulsiveDecay);
}

} /* namespace inet */
