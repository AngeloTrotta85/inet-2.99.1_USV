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

#include "inet/applications/broadcastapp/USVControl.h"


namespace inet {

Define_Module(USVControl);

USVControl::USVControl() {
    pktGenerated = 0;
}

/** @brief Initializes mobility model parameters. */
void USVControl::initialize(int stage) {

}

/** @brief This modules should only receive self-messages. */
void USVControl::handleMessage(cMessage *msg) {

}

ScannedPointsList *USVControl::getPacketToSend(void) {
    char msgName[32];
    ScannedPointsList *pkt;

    sprintf(msgName, "CooperativeScanningApp-%d", pktGenerated);
    pkt = new ScannedPointsList(msgName);

    // fill the packet with the scanned points
    pkt->setScanPointsArraySize(scannedPoints.size());
    int i = 0;
    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        USVControl::PointScan *ps = &(*it);
        struct ScannedPoint newP;

        newP.position = ps->pos;
        newP.timestamp = ps->scan_timestamp;

        pkt->setScanPoints(i++, newP);
    }

    pkt->setByteLength(pkt->getScanPointsArraySize() * sizeof(struct ScannedPoint));

    if (pkt->getByteLength() > 0) pktGenerated++;

    return pkt;
}

void USVControl::addScannedPointsFromOthers(ScannedPointsList *pkt) {
    for (int i = 0; i < pkt->getScanPointsArraySize(); i++) {
        bool already = false;

        for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
            PointScan *ps = &(*it);
            if (ps->pos == pkt->getScanPoints(i).position) {
                already = true;
                break;
            }
        }

        if (!already){
            scannedPoints_fromOthers.push_back(pkt->getScanPoints(i));
        }
    }
}

} /* namespace inet */
