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

#include "inet/mobility/base/MovingMobilityBase.h"

namespace inet {

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
    } repulsive_point_t;

protected:
    double maxspeed;    ///< maximum speed of the host
    double maxacceleration;    ///< acceleration of linear motion

    double angle;    ///< actual angle-force
    double speed;    ///< actual speed of the host
    Coord force;    ///< actual force of the host

protected:
  virtual int numInitStages() const override { return NUM_INIT_STAGES; }

  /** @brief Initializes mobility model parameters.*/
  virtual void initialize(int stage) override;

  /** @brief Move the host*/
  virtual void move() override;

public:
  virtual double getMaxSpeed() const override { return maxspeed; }
  FieldForceMobility();

private:
  void updateFieldForce(void);
};

} /* namespace inet */

#endif /* FIELDFORCEMOBILITY_H_ */