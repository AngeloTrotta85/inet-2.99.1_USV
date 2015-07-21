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

#ifndef FIELDFORCEMOBILITY_H_
#define FIELDFORCEMOBILITY_H_

#include "inet/common/INETDefs.h"

#include <list>

#include "inet/mobility/base/LineSegmentsMobilityBase.h"
#include "inet/applications/broadcastapp/USVControl.h"

namespace inet {

class USVControl;

/**
 * @brief Field force based movement model. See NED file for more info.
 *
 * @ingroup mobility
 * @author Angelo Trotta
 */

class INET_API FieldForceMobility : public MovingMobilityBase {
public:
    typedef struct repulsive_point {
        Coord position;
        simtime_t timestamp;
        double weight;
        double decade_factor;
    } repulsive_point_t;

protected:
    double maxspeed;    ///< maximum speed of the host
    double maxacceleration;    ///< acceleration of linear motion

    bool randomMovement;
    double weigthRandomMovement;
    simtime_t nextRandomChangeTime;
    simtime_t stepRandomChangeTime;

    int debugRandomRepulsivePoints;

    double defaultVolatileTimeDecay;
    double defaultRepulsiveForceWeight;
    double defaultRepulsiveDecay;

    double angle;    ///< actual angle-force
    double speed;    ///< actual speed of the host
    Coord force;    ///< actual force of the host

    Coord exploringForce;
    unsigned int idxRPL;
    std::map<unsigned int, repulsive_point_t> repulsivePointsList;
    std::map<unsigned int, repulsive_point_t> volatileRepulsivePointsList;

    USVControl *usv_control;

protected:
  virtual int numInitStages() const override { return NUM_INIT_STAGES; }

  /** @brief Initializes mobility model parameters.*/
  virtual void initialize(int stage) override;

  /** @brief Move the host*/
  virtual void move() override;

public:
  virtual double getMaxSpeed() const override { return maxspeed; }
  FieldForceMobility();

  const Coord& getExploringForce() const { return exploringForce; }

  void setExploringForce(const Coord& exploringForce) { this->exploringForce = exploringForce; }

  void addPersistentRepulsiveForce(unsigned int id, const Coord& pos, double weight, double decade_factor);
  void addPersistentRepulsiveForce(unsigned int id, const Coord& pos);

  void setVolatileRepulsiveForce(unsigned int addr, const Coord& pos, double weight, double decade_factor);
  void setVolatileRepulsiveForce(unsigned int addr, const Coord& pos);

private:
  void updateFieldForce(void);
  Coord calcRepulsiveForce(repulsive_point_t *rp);
  Coord calcRepulsiveForceTimeDecade(repulsive_point_t *rp, double timeDecateFac);
};

} /* namespace inet */

#endif /* FIELDFORCEMOBILITY_H_ */
