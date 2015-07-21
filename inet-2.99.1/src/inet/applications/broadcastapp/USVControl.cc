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

#include "inet/environment/common/PhysicalEnvironment.h"

namespace inet {

Define_Module(USVControl);

USVControl::USVControl() {
    pktGenerated = 0;
    scanningID_idx = 0;
}

/** @brief Initializes mobility model parameters. */
void USVControl::initialize(int stage) {
    if (stage == INITSTAGE_LOCAL) {

        pathLossMapAvailable = par("pathLossMapAvailable").boolValue();
        defaultRepulsiveWeigth = par("defaultRepulsiveWeigth");
        desiredWeigthRatio = par("desiredWeigthRatio");

        ffmob = check_and_cast<FieldForceMobility *>(this->getParentModule()->getSubmodule("mobility"));
        pathLossModel = check_and_cast<physicallayer::LogNormalShadowingGrid *>(this->getParentModule()->getParentModule()->getSubmodule("radioMedium")->getSubmodule("pathLoss"));

        signalMapOffset = ffmob->getConstraintAreaMin();

        //signalPropMap.resize((int)(ffmob->getConstraintAreaMax().x - ffmob->getConstraintAreaMin().x));
        signalPropMap.resize((int)(ffmob->getConstraintAreaMax().x));
        for (unsigned int x = 0; x < signalPropMap.size(); x++) {
            //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
            signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y));

            for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {
                signalPropMap[x][y].pathloss_alpha = 2;
                signalPropMap[x][y].lognormal_sigma = 1;
            }
        }

        checkScanTimeStep = par("checkScanTimeStep");
        checkScanTimer = new cMessage("checkScan");
        scheduleAt(simTime() + checkScanTimeStep + dblrand(), checkScanTimer);

        // randomize the color
        r_point_col = rand() % 256;
        g_point_col = rand() % 256;
        b_point_col = rand() % 256;

        WATCH_LIST(scannedPoints_fromOthers);
        WATCH_LIST(scannedPoints);
    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER) {
        if (pathLossMapAvailable) {

            // copy the map from the pathloss simulation module
            for (unsigned int x = 0; x < signalPropMap.size(); x++) {
                for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {

                    pathLossModel->getAlphaSigmaFromAbsCoord(Coord(x, y),
                            signalPropMap[x][y].pathloss_alpha,
                            signalPropMap[x][y].lognormal_sigma);

                }
            }

        }
    }
}

/** @brief This modules should only receive self-messages. */
void USVControl::handleMessage(cMessage *msg) {
    if (msg == checkScanTimer) {
        checkIfScan();

        scheduleAt(simTime() + checkScanTimeStep, msg);
    }
}

void USVControl::checkIfScan(void) {
    double probToScan, maxForce;

    maxForce = -1;

    for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
        PointScan *ps = &(*it);
        double pointForce = calculateForceFromPoint(ps->pos);
        if (pointForce > maxForce) maxForce = pointForce;
    }

    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        PointScan *ps = &(*it);
        double pointForce = calculateForceFromPoint(ps->pos);
        if (pointForce > maxForce) maxForce = pointForce;
    }

    probToScan = 1.0 - (maxForce / defaultRepulsiveWeigth);  // all the force cannot be greater then "defaultRepulsiveWeigth"

    if (dblrand() < probToScan) {

        EV << "Making the scan" << endl;

        // make the scan
        executeScanning();

        // drawGrafically the point
        drawScannedPoint(ffmob->getCurrentPosition());

        // add point to the scanned list
        PointScan newps;
        newps.pos= ffmob->getCurrentPosition();
        newps.scan_timestamp = simTime();
        newps.scanningHostAddr = this->getParentModule()->getIndex();
        newps.scanningID = scanningID_idx++;
        scannedPoints.push_back(newps);

        // at the end update the parameters of the mobility control
        updateMobilityPointsParameters();
    }
}

void USVControl::executeScanning(void) {
    //TODO probabilmente servità uno start_scan e un end_scan
}

double USVControl::calculateUncorrelatedDistance(Coord point) {
    double alpha_loss;
    //double sigma_loss;

    alpha_loss = signalPropMap[point.x][point.y].pathloss_alpha;
    //sigma_loss = signalPropMap[point.x][point.y].lognormal_sigma;

    //TODO faccio semplice, da modificare
    double ris;

    // faccio una proporzione per cui urban=8(alpha>=5), suburban=500(alpha<=2)
    if (alpha_loss < 2) alpha_loss = 2;
    if (alpha_loss > 5) alpha_loss = 5;

    ris = (((alpha_loss - 2) / (5 - 2)) * (8 - 50)) + 50;   //uscito dalla retta passante per 2 punti (2,50)-(5,8)

    return ris;
}

double USVControl::calculateDecayFromWeigthAndChannelLoss(double desiredRatio, double fieldWeigth, Coord point) {
    double desiredDistance, ris;

    // calculate the desired distance
    desiredDistance = calculateUncorrelatedDistance(point);

    ris = log(1/desiredRatio) / (desiredDistance*desiredDistance);

    return ris;
}

double USVControl::calculateForceFromPoint(Coord point) {
    double decayPoint = calculateDecayFromWeigthAndChannelLoss(desiredWeigthRatio, defaultRepulsiveWeigth, point);
    double sq_distance = ffmob->getCurrentPosition().sqrdist(point);

    return (defaultRepulsiveWeigth * exp(-(decayPoint * sq_distance)));
}

void USVControl::updateMobilityPointsParameters(void) {
    for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
        PointScan *ps = &(*it);

        double decade_factor = calculateDecayFromWeigthAndChannelLoss(desiredWeigthRatio, defaultRepulsiveWeigth, ps->pos);

        ffmob->addPersistentRepulsiveForce(ps->scanningID, ps->pos, defaultRepulsiveWeigth, decade_factor);
    }

    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        PointScan *ps = &(*it);

        double decade_factor = calculateDecayFromWeigthAndChannelLoss(desiredWeigthRatio, defaultRepulsiveWeigth, ps->pos);

        ffmob->addPersistentRepulsiveForce(ps->scanningID, ps->pos, defaultRepulsiveWeigth, decade_factor);
    }
}

ScannedPointsList *USVControl::getPacketToSend(void) {
    char msgName[32];
    ScannedPointsList *pkt;

    sprintf(msgName, "CooperativeScanningApp-%d", pktGenerated++);
    pkt = new ScannedPointsList(msgName);

    // fill the packet with the scanned points
    pkt->setScanPointsArraySize(scannedPoints.size());
    int i = 0;
    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        USVControl::PointScan *ps = &(*it);
        struct ScannedPoint newP;

        newP.position = ps->pos;
        newP.timestamp = ps->scan_timestamp;
        newP.scanID = ps->scanningID;

        pkt->setScanPoints(i++, newP);
    }

    pkt->setNodePosition(ffmob->getCurrentPosition());
    pkt->setNodeAddr(this->getParentModule()->getIndex());

    pkt->setByteLength(pkt->getScanPointsArraySize() * sizeof(struct ScannedPoint) + 64);

    return pkt;
}

void USVControl::addScannedPointsFromOthers(ScannedPointsList *pkt) {
    double decayVolatileVal = calculateDecayFromWeigthAndChannelLoss(desiredWeigthRatio, defaultRepulsiveWeigth, pkt->getNodePosition());

    //EV << "Setting force from " << pkt->getNodeAddr() << " with weigth " << defaultRepulsiveWeigth <<
    //        " and decay exp " << decayVolatileVal << endl;

    ffmob->setVolatileRepulsiveForce(pkt->getNodeAddr(), pkt->getNodePosition(), defaultRepulsiveWeigth, decayVolatileVal);


    for (unsigned int i = 0; i < pkt->getScanPointsArraySize(); i++) {
        bool already = false;

        for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
            PointScan *ps = &(*it);

            if ((ps->scanningHostAddr == pkt->getNodeAddr()) && (ps->scanningID == pkt->getScanPoints(i).scanID)){
            //if (ps->pos == pkt->getScanPoints(i).position) {
                already = true;
                break;
            }
        }

        if (!already){
            ScannedPoint *sp = &(pkt->getScanPoints(i));
            PointScan ps;

            ps.pos = sp->position;
            ps.scan_timestamp = sp->timestamp;
            ps.scanningID = sp->scanID;
            ps.scanningHostAddr = pkt->getNodeAddr();

            scannedPoints_fromOthers.push_back(ps);
        }
    }

    updateMobilityPointsParameters();
}

void USVControl::drawScannedPoint(Coord position) {

    if (ev.isGUI()) {
        physicallayer::PhysicalEnvironment *physicalEnvironment = dynamic_cast<physicallayer::PhysicalEnvironment *>(getModuleByPath("environment"));
        if (physicalEnvironment != nullptr) {
            char buffer[100];

            cXMLElement *shadowsXml = new cXMLElement("environment", "", nullptr);

            cXMLElement *newPoint = new cXMLElement("object", "", shadowsXml);

            //set color
            snprintf(buffer, sizeof(buffer), "%d %d %d", r_point_col, g_point_col, b_point_col);
            newPoint->setAttribute("fill-color", buffer);
            newPoint->setAttribute("line-color", buffer);

            //position attribute
            snprintf(buffer, sizeof(buffer), "center %lf %lf %lf", position.x, position.y, 10.0);
            newPoint->setAttribute("position", buffer);

            //shape attribute
            newPoint->setAttribute("shape", "sphere 4");

            //material and fill-color constant
            newPoint->setAttribute("material", "vacuum");

            shadowsXml->appendChild(newPoint);

            physicalEnvironment->addFromXML(shadowsXml);
            physicalEnvironment->updateCanvas();
        }
    }
}

void USVControl::getAlphaSigmaInPoint(Coord point, double &alpha, double &sigma) {
    if ((signalPropMap.size() > point.x) && (signalPropMap[point.x].size() > point.y) ) {
        alpha = signalPropMap[point.x][point.y].pathloss_alpha;
        sigma = signalPropMap[point.x][point.y].lognormal_sigma;
    }
}

} /* namespace inet */
