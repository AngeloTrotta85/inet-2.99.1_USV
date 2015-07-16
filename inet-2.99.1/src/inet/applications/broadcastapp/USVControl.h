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

#ifndef USVCONTROL_H_
#define USVCONTROL_H_

#include <list>

#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"

#include "inet/applications/broadcastapp/ScannedPointsList_m.h"


namespace inet {

class INET_API USVControl : public cSimpleModule {

public:
    typedef struct PointScan_s {
        Coord pos;
        simtime_t scan_timestamp;
    } PointScan;

protected:

    /** @brief Returns the required number of initialize stages. */
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief This modules should only receive self-messages. */
    virtual void handleMessage(cMessage *msg) override;

public:
    USVControl();

    ScannedPointsList *getPacketToSend(void);

    void addScannedPointsFromOthers(ScannedPointsList *pkt);


private:

    std::list<PointScan> scannedPoints_fromOthers;
    std::list<PointScan> scannedPoints;
    int pktGenerated;

};

} /* namespace inet */

#endif /* USVCONTROL_H_ */
