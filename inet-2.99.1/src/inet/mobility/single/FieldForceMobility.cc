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

    lastChoosenPoint = Coord::NIL;
    nextChoosenPoint_timestamp = simTime();

    idxRPL = 0;
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

        repulsiveForceFromScannedPoints = par("repulsiveForceFromScannedPoints").boolValue();

        towardsNotExploredMapMovement = par("towardsNotExploredMapMovement").boolValue();
        choosenPointTimeValidity = par("choosenPointTimeValidity");
        numberOfChoices = par("numberOfChoices");
        if (numberOfChoices < 1) numberOfChoices = 1;

        randomMovement = par("randomMovement").boolValue();
        weigthRandomMovement = par("weigthRandomMovement").doubleValue();
        stepRandomChangeTime = par("randomStepTimeChange");
        nextRandomChangeTime = simTime() + (stepRandomChangeTime * dblrand());

        debugRandomRepulsivePoints = par("debugRandomRepulsivePoints");

        usv_control = dynamic_cast <USVControl *> (this->getParentModule()->getSubmodule("usv_brain"));

        forced_stop = false;

        nextCalcForce = simTime();

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

            repulsivePointsList[std::make_pair(-1, idxRPL++)] = rp;
        }
    }
}

void FieldForceMobility::move()
{
    if (forced_stop) {
        speed = 0;
        lastSpeed = Coord::ZERO;
    }
    else {

        if (simTime() >= nextCalcForce) {
            updateFieldForce();

            nextCalcForce = simTime() + (0.5 + dblrand());
        }

        if (force != Coord::ZERO) {

            if (force.length() > maxacceleration) {
                force.normalize();
                force *= maxacceleration;


                if (usv_control) {
                    double a, s;
                    usv_control->getAlphaSigmaInPoint(lastPosition, a, s);
                    if (a > 5.0) a = 5.0;

                    //force *= ((6.0 - a) / 6.0);
                    force *= ((a * (0.5 - 1.0)) / 5.0) + 1.0;
                }
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
}

void FieldForceMobility::updateFieldForce(void) {

    //Reset the force
    force = Coord::ZERO;

    //Add node exploring-force ************************************************************
    if (towardsNotExploredMapMovement) {
        force += getExploreForceTowardsNotExploredMap();
    }
    else if (randomMovement) {
        if (nextRandomChangeTime <= simTime()) {
            EV << "RANDOMIZING exploring force" << endl;
            if (lastPosition.x < (constraintAreaMin.x + 10)) {
                do {
                    exploringForce.x = dblrand();
                    exploringForce.y = (dblrand() * 2.0) - 1.0;
                } while(exploringForce.length() < 0.1);     // to avoid math problems
            }
            else if (lastPosition.x > (constraintAreaMax.x - 10)) {
                do {
                    exploringForce.x = dblrand() - 1.0;
                    exploringForce.y = (dblrand() * 2.0) - 1.0;
                } while(exploringForce.length() < 0.1);     // to avoid math problems
            }
            else if (lastPosition.y < (constraintAreaMin.y + 10)) {
                do {
                    exploringForce.x = (dblrand() * 2.0) - 1.0;
                    exploringForce.y = dblrand();
                } while(exploringForce.length() < 0.1);     // to avoid math problems
            }
            else if (lastPosition.y > (constraintAreaMax.y - 10)) {
                do {
                    exploringForce.x = (dblrand() * 2.0) - 1.0;
                    exploringForce.y = dblrand() - 1.0;
                } while(exploringForce.length() < 0.1);     // to avoid math problems
            }
            else {
                if (exploringForce == Coord::ZERO) {
                    do {
                        exploringForce.x = (dblrand() * 2.0) - 1.0;
                        exploringForce.y = (dblrand() * 2.0) - 1.0;
                    } while(exploringForce.length() < 0.1);     // to avoid math problems
                    exploringForce.normalize();
                    exploringForce *= weigthRandomMovement;
                }

                Coord perturbing_force = Coord::ZERO;
                do {
                    perturbing_force.x = (dblrand() * 2.0) - 1.0;
                    perturbing_force.y = (dblrand() * 2.0) - 1.0;
                } while(perturbing_force.length() < 0.1);     // to avoid math problems
                perturbing_force.normalize();
                perturbing_force *= exploringForce.length();

                EV << "Perturbing the old exploring force: " << exploringForce << " by " << perturbing_force <<  endl;

                exploringForce += perturbing_force;
            }


            EV << "First guess for exploring force: " << exploringForce << endl;

            exploringForce.normalize();
            exploringForce *= weigthRandomMovement;

            /*if (usv_control) {
                double a, s;
                usv_control->getAlphaSigmaInPoint(lastPosition, a, s);
                if (a > 5.0) a=5.0;

                exploringForce *= weigthRandomMovement * (a / 5.0);
            } else  {
                exploringForce *= weigthRandomMovement;
            }*/

            speed = 0;

            EV << "NEW exploring force: " << exploringForce << endl;

            nextRandomChangeTime += stepRandomChangeTime + dblrand();
        }
    }
    force += exploringForce;
    //*********************************************************************************************
    //END Add node exploring-force ************************************************************

    if (repulsiveForceFromScannedPoints) {

        //Add volatile repulsive-force ************************************************************
        Coord volRepulsiveSum = Coord::ZERO;

        for (std::map<unsigned int, repulsive_point_t>::iterator it = volatileRepulsivePointsList.begin(); it != volatileRepulsivePointsList.end(); it++) {
            repulsive_point_t *actP = &(it->second);

            Coord actForce = calcRepulsiveForceTimeDecade(actP, defaultVolatileTimeDecay);

            volRepulsiveSum += actForce;
        }

        //erase the useless volatile values
        for (std::map<unsigned int, repulsive_point_t>::iterator it = volatileRepulsivePointsList.begin(); it != volatileRepulsivePointsList.end(); it++) {
            repulsive_point_t *actP = &(it->second);

            Coord actForce = calcRepulsiveForceTimeDecade(actP, defaultVolatileTimeDecay);

            if (actForce.length() < 0.001) {
                volatileRepulsivePointsList.erase(it);

                //restart from the beginning
                it = volatileRepulsivePointsList.begin();
                if (it == volatileRepulsivePointsList.end()) {
                    break;
                }
            }
        }

        EV << "Adding volatile repulsive forces sum: " << volRepulsiveSum << endl;
        force += volRepulsiveSum;
        //*********************************************************************************************
        //END Add volatile repulsive-force ************************************************************


        // Add repulsive forces ************************************************************
        Coord repulsiveSum = Coord::ZERO;

        for (std::map<std::pair<int, unsigned int>, repulsive_point_t>::iterator it = repulsivePointsList.begin(); it != repulsivePointsList.end(); it++) {
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
}


bool compare_lessExplored (const std::pair<Coord, double>& first, const std::pair<Coord, double>& second) {
    return (first.second < second.second);
}

Coord FieldForceMobility::getExploreForceTowardsNotExploredMap(void) {
    Coord ris = Coord::ZERO;

    if (    (lastChoosenPoint == Coord::NIL) ||
            (simTime() > nextChoosenPoint_timestamp) ||
            (lastChoosenPoint.distance(lastPosition) < 5) ){

        std::list<std::pair<Coord, double>> possible_choice;
        int choice_idx;

        for (int i = 0; i < numberOfChoices; i++) {
            Coord random_coord;
            //Coord random_coord_explored_force = Coord::ZERO;
            double random_coord_explored_force_SIZE = 0;
            double max_random_coord_explored_force_SIZE = 0;
            double random_coord_explored_force_COUNT = 0;

            random_coord = Coord(   (dblrand() * (constraintAreaMax.x - constraintAreaMin.x)) + constraintAreaMin.x,
                                    (dblrand() * (constraintAreaMax.y - constraintAreaMin.y)) + constraintAreaMin.y);

            //fprintf(stderr, "Sono %d sono in [%lf:%lf] e voglio andare a [%lf, %lf]\n",
            //        this->getParentModule()->getIndex(), lastPosition.x, lastPosition.y, random_coord.x, random_coord.y); fflush(stderr);

            // calculate the force on the destination point
            for (std::map<std::pair<int, unsigned int>, repulsive_point_t>::iterator itRP = repulsivePointsList.begin(); itRP != repulsivePointsList.end(); itRP++) {
                repulsive_point_t *actP = &(itRP->second);

                Coord actForce = calcRepulsiveForceOnAPoint(random_coord, actP);
                //random_coord_explored_force += actForce;
                random_coord_explored_force_SIZE += actForce.length();
                max_random_coord_explored_force_SIZE += actForce.length();
                random_coord_explored_force_COUNT++;
            }

            //fprintf(stderr, "Forza al punto di destinazione: %lf\n", random_coord_explored_force_SIZE); fflush(stderr);

            /****** TRY ALSO ON THE PATH ************************/
            /*******************************************************************/
            // calculate the force on the path
            double step_size_on_path = 20;
            int n_points_on_path = random_coord.distance(lastPosition) / step_size_on_path;

            Coord step_on_path = random_coord - lastPosition;
            step_on_path.normalize();
            step_on_path *= step_size_on_path;

            //fprintf(stderr, "Faccio %d passi di dimensione %lf con step [%lf:%lf - %lfm]\n",
            //        n_points_on_path, step_size_on_path, step_on_path.x, step_on_path.y, step_on_path.length()); fflush(stderr);

            Coord p_on_path = lastPosition + step_on_path;
            for (int jj = 0; jj < n_points_on_path; jj++){
                // calculate the force on the point on the path
                //Coord force_at_point = Coord::ZERO;
                double force_at_point_SIZE = 0;
                double max_force_at_point_SIZE = 0;

                for (std::map<std::pair<int, unsigned int>, repulsive_point_t>::iterator itRP = repulsivePointsList.begin(); itRP != repulsivePointsList.end(); itRP++) {
                    repulsive_point_t *actP = &(itRP->second);

                    Coord actForce = calcRepulsiveForceOnAPoint(p_on_path, actP);

                    //force_at_point += actForce;
                    //random_coord_explored_force += actForce;
                    force_at_point_SIZE += actForce.length();
                    if (actForce.length() > max_force_at_point_SIZE) max_force_at_point_SIZE = actForce.length();
                }

                //fprintf(stderr, "Forza al punto di [%lf:%lf]: %lf\n",
                //        p_on_path.x, p_on_path.y, force_at_point_SIZE); fflush(stderr);
                //random_coord_explored_force += force_at_point;
                random_coord_explored_force_SIZE += force_at_point_SIZE;
                max_random_coord_explored_force_SIZE += max_force_at_point_SIZE;
                random_coord_explored_force_COUNT++;

                p_on_path += step_on_path;
            }
            //fprintf(stderr, "Forza finale per [%lf, %lf]: %lf\n",
            //        random_coord.x, random_coord.y, random_coord_explored_force_SIZE); fflush(stderr);
            /*******************************************************************/

            //possible_choice.push_back(std::make_pair(random_coord, random_coord_explored_force.length()));
            //possible_choice.push_back(std::make_pair(random_coord, random_coord_explored_force_SIZE/random_coord_explored_force_COUNT));
            possible_choice.push_back(std::make_pair(random_coord, max_random_coord_explored_force_SIZE/random_coord_explored_force_COUNT));
        }

        possible_choice.sort(compare_lessExplored);

        choice_idx = (int) (truncnormal(0.0, MIN(numberOfChoices/5.0, 1)));

        if (choice_idx >= numberOfChoices) choice_idx = 0;

        std::list<std::pair<Coord, double>>::iterator it = possible_choice.begin();
        for (int i = 0; i < choice_idx; i++) {
            it++;
        }
        lastChoosenPoint = it->first;

        //fprintf(stderr, "Choosing index %d with destination point [%lf:%lf]\n\n", choice_idx, lastChoosenPoint.x, lastChoosenPoint.y); fflush(stderr);

        nextChoosenPoint_timestamp = simTime() + truncnormal(choosenPointTimeValidity, choosenPointTimeValidity / 10.0);
    }

    ris = lastChoosenPoint - lastPosition;
    ris.normalize();
    ris *= weigthRandomMovement;

    return ris;
}

Coord FieldForceMobility::calcRepulsiveForceOnAPoint(Coord refPoint, repulsive_point_t *rp) {
    Coord ris = refPoint - rp->position;
    double field;

    //double field = rp->weight * exp(-(rp->decade_factor * lastPosition.distance(rp->position)));
    //double sq_distance = pow(lastPosition.x - rp->position.x, 2.0) + pow(lastPosition.y - rp->position.y, 2.0);
    if (refPoint == rp->position) {
        ris = Coord(dblrand() - 0.5, dblrand() - 0.5);
        field = rp->weight;
    }
    else {
        double sq_distance = refPoint.sqrdist(rp->position);
        field = rp->weight * exp(-(rp->decade_factor * sq_distance));
    }

    ris.normalize();
    ris *= field;

    return ris;
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
    //        << " - time passed " << (simTime() > rp->timestamp) << " [" << (simTime() - rp->timestamp).dbl() << "]"
    //        " - TimeFact: " << timeDecateFac << " - Exp value: " << (simTime() - rp->timestamp).dbl() * timeDecateFac << endl;

    if (simTime() > rp->timestamp) {
    //    EV << "Final Multipler:  " << exp(-((simTime() - rp->timestamp).dbl() * timeDecateFac)) << endl;
        ris *= exp(-( pow((simTime() - rp->timestamp).dbl(), 2) * timeDecateFac));
    }

    //EV << "FINAL volatile force " << ris << endl;

    return ris;
}

void FieldForceMobility::addPersistentRepulsiveForce(unsigned int id, int hostAddr, const Coord& pos, double weight, double decade_factor) {
    repulsive_point_t rp;

    rp.timestamp = simTime();
    rp.decade_factor = decade_factor;
    rp.weight = weight;
    rp.position = pos;

    repulsivePointsList[std::make_pair(hostAddr, id)] = rp;
}

void FieldForceMobility::addPersistentRepulsiveForce(unsigned int id, int hostAddr,  const Coord& pos) {
    addPersistentRepulsiveForce(id, hostAddr, pos, defaultRepulsiveForceWeight, defaultRepulsiveDecay);
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
