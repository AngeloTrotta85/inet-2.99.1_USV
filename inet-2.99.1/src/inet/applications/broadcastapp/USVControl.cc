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

#include "inet/physicallayer/common/packetlevel/Radio.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211ScalarTransmitter.h"

namespace inet {

Define_Module(USVControl);

USVControl::~USVControl() {
    cancelAndDelete(checkScanTimer);
    cancelAndDelete(isScanningTimer);
}

USVControl::USVControl() {
    pktGenerated = 0;
    scanningID_idx = 0;

    isScanning = false;
}

/** @brief Initializes mobility model parameters. */
void USVControl::initialize(int stage) {
    if (stage == INITSTAGE_LOCAL) {

        pathLossMapAvailable = par("pathLossMapAvailable").boolValue();
        defaultRepulsiveWeigth = par("defaultRepulsiveWeigth");
        desiredWeigthRatio = par("desiredWeigthRatio");
        sizeOfScenaioReportCells = par("sizeOfScenaioReportCells");
        filename_output_grid = par("outputCellsScanReport").stdstringValue();
        sigmaMultiplierInTxRangeCalculation = par("sigmaMultiplierInTxRangeCalculation");

        //scanPowerThreshold = W(par("scanPowerThreshold").doubleValue());
        scanPowerThreshold = mW(math::dBm2mW(par("scanPowerThreshold")));

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

        isScanningTimer = new cMessage("isScanning");
        scanningTime = par("scanningTime");

        // randomize the color
        r_point_col = rand() % 256;
        g_point_col = rand() % 256;
        b_point_col = rand() % 256;

        // subscribe to the Radio signal
        physicallayer::Radio *radioM = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getSubmodule("wlan",0)->getSubmodule("radio"));
        radioM->subscribe(physicallayer::Radio::maxRSSISignal, this);
        //radioM->subscribe(physicallayer::Radio::minSNIRSignal, this);

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
        bool imScanning = checkIfScan();

        if(!imScanning) {
            scheduleAt(simTime() + checkScanTimeStep, msg);
        }
    }
    else if (msg == isScanningTimer) {
        endScanning();
    }
}

bool USVControl::checkIfScan(void) {
    bool ris = false;
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

    EV << "Checking if needed scan here. Probability to scan is: " << probToScan << endl;

    if (dblrand() < probToScan) {

        ris = true;

        // make the scan
        startScanning();
    }

    return ris;
}

void USVControl::startScanning(void) {
    // start scanning
    EV_DEBUG << "Start scanning the channel" << endl;
    scheduleAt(simTime() + scanningTime, isScanningTimer);


    // stopping the drone during scanning
    ffmob->setForcedStop(true);

    scanningList.clear();

    isScanning = true;

}

bool USVControl::calcIfInRangeTransm(bool scan_result_debug, W resScan_debug) {
    bool ris = false;

    cModule *tower0 = this->getParentModule()->getParentModule()->getSubmodule("tower", 0);

    //EV << "calcIfInRangeTransm - " << tower0->getFullPath() << " - vectnumber: " << tower0->getVectorSize() << endl;

    for (int i = 0; i < tower0->getVectorSize() ; i++) {
        MobilityBase *txMob                                         = check_and_cast<MobilityBase *>(this->getParentModule()->getParentModule()->getSubmodule("tower", i)->getSubmodule("mobility"));
        physicallayer::Radio *receiverRadio                         = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getSubmodule("wlan", 0)->getSubmodule("radio"));
        physicallayer::Radio *radioTX                               = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getParentModule()->getSubmodule("tower", i)->getSubmodule("wlan", 0)->getSubmodule("radio"));
        physicallayer::Ieee80211ScalarTransmitter *radioTransmitter = check_and_cast<physicallayer::Ieee80211ScalarTransmitter *>(radioTX->getSubmodule("transmitter"));

        Coord receiverPos       = ffmob->getCurrentPosition();
        Hz frequency            = radioTransmitter->getCarrierFrequency();
        double p_tx_dBm         = math::mW2dBm(radioTX->getTransmitter()->getMaxPower().get() * 1000);
        double sigma_mult       = sigmaMultiplierInTxRangeCalculation;
        double threshold_dBm    = math::mW2dBm(scanPowerThreshold.get() * 1000);

        m mThDist = pathLossModel->getThresholdDistance(p_tx_dBm, sigma_mult, threshold_dBm, receiverPos, receiverRadio, frequency);

        if (txMob->getCurrentPosition().distance(ffmob->getCurrentPosition()) <= mThDist.get()) {
            ris = true;
            break;
        }
    }

    if (ris != scan_result_debug) {
        for (int i = 0; i < tower0->getVectorSize() ; i++) {
            double p_tx_dBm;
            double sigma_mult;
            double threshold_dBm;
            Coord receiverPos = ffmob->getCurrentPosition();
            MobilityBase *txMob = check_and_cast<MobilityBase *>(this->getParentModule()->getParentModule()->getSubmodule("tower", i)->getSubmodule("mobility"));
            physicallayer::Radio *receiverRadio = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getSubmodule("wlan", 0)->getSubmodule("radio"));
            physicallayer::Radio *radioTX = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getParentModule()->getSubmodule("tower", i)->getSubmodule("wlan", 0)->getSubmodule("radio"));
            physicallayer::Ieee80211ScalarTransmitter *radioTransmitter = check_and_cast<physicallayer::Ieee80211ScalarTransmitter *>(radioTX->getSubmodule("transmitter"));

            Hz frequency = radioTransmitter->getCarrierFrequency();

            std::stringstream printStr;

            printStr << "TRANSMITTING POWER: " << radioTX->getTransmitter()->getMaxPower() << endl;

            printStr << "Complete " << radioTX->getTransmitter()->getCompleteStringRepresentation() << endl;

            p_tx_dBm = math::mW2dBm(radioTX->getTransmitter()->getMaxPower().get() * 1000);
            sigma_mult = sigmaMultiplierInTxRangeCalculation;
            threshold_dBm = math::mW2dBm(scanPowerThreshold.get() * 1000);

            printStr << "Calc TH-Distance:"
                    << " p_tx_dBm: " << p_tx_dBm
                    << "; sigma_mult: " << sigma_mult
                    << "; threshold_dBm: " << threshold_dBm
                    << "; receiverPos: " << receiverPos
                    << "; receiverRadio: " << receiverRadio->getFullPath()
                    << "; frequency: " << frequency
                    << endl;

            m mThDist = pathLossModel->getThresholdDistance(p_tx_dBm, sigma_mult, threshold_dBm, receiverPos, receiverRadio, frequency);

            printStr << "Calc TH-Distance ris: " << mThDist.get() << " meters" << endl;

            printStr << "Calc TH-Distance. Distance from the transmitter: " << txMob->getCurrentPosition().distance(ffmob->getCurrentPosition()) << " meters" << endl;

            printStr << "Calculate says: " << ris << " while the scanning says: " << scan_result_debug << " with pow scanned: " << resScan_debug << "(" << math::mW2dBm(resScan_debug.get()*1000) << "dbm)" << endl;

            fprintf(stderr, "DEBUG!!!\n%s", printStr.str().c_str());
        }
    }

    return ris;
}

void USVControl::endScanning(void) {

    // end scanning
    //EV_DEBUG << "End scanning the channel" << endl;

    ffmob->setForcedStop(false);

    isScanning = false;

    // restart the timer to check scanning
    scheduleAt(simTime() + checkScanTimeStep + (dblrand() / 2.0), checkScanTimer);

    //calculate the result
    W resScan = W(0);
    bool scanResult = false;
    for (std::list<W>::iterator it = scanningList.begin();  it != scanningList.end();  it++) {

        if (((*it) != W(NaN)) && ((*it) > resScan)) {
            resScan = *it;
        }

        EV_DEBUG << "MAX: " << resScan << " - Scanning list: " << *it << endl;
    }
    if (resScan > scanPowerThreshold) {
        scanResult = true;
    }

    bool calculatedRes = calcIfInRangeTransm(scanResult, resScan);

    EV_DEBUG << "End scanning procedure. Max power: " << resScan << " [thr: " << scanPowerThreshold << "] resulting in channel ";
    scanResult ? EV << "busy"<< endl : EV << "free" << endl;

    // drawGrafically the point
    drawScannedPoint(ffmob->getCurrentPosition(), scanResult);

    // add point to the scanned list
    PointScan newps;
    newps.pos= ffmob->getCurrentPosition();
    newps.scan_timestamp = simTime();
    newps.scanningHostAddr = this->getParentModule()->getIndex();

    newps.scanLog.actualResult = scanResult;
    newps.scanLog.powerReceived = resScan;
    newps.scanLog.calculatedResult = calculatedRes;

    newps.scanningID = scanningID_idx++;
    scannedPoints.push_back(newps);

    // at the end update the parameters of the mobility control
    updateMobilityPointsParameters();
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
        PointScan *ps = &(*it);
        struct ScannedPoint newP;

        newP.position = ps->pos;
        newP.timestamp = ps->scan_timestamp;
        newP.scanID = ps->scanningID;
        newP.decisionMade = ps->scanLog.actualResult;
        newP.watt_read = ps->scanLog.powerReceived.get();

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

void USVControl::drawScannedPoint(Coord position, bool isBusy) {

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
            if (isBusy) {
                newPoint->setAttribute("shape", "sphere 4");
            }
            else {
                newPoint->setAttribute("shape", "cuboid 4 4 1");
            }

            //material and fill-color constant
            newPoint->setAttribute("material", "vacuum");

            shadowsXml->appendChild(newPoint);

            physicalEnvironment->addFromXML(shadowsXml);
            physicalEnvironment->updateCanvas();
        }
    }
}

void USVControl::getAlphaSigmaInPoint(Coord point, double &alpha, double &sigma) {
    alpha = 2;
    sigma = 1;
    if ((signalPropMap.size() > point.x) && (signalPropMap[point.x].size() > point.y) ) {
        alpha = signalPropMap[point.x][point.y].pathloss_alpha;
        sigma = signalPropMap[point.x][point.y].lognormal_sigma;
    }
}

void USVControl::receiveSignal(cComponent *source, simsignal_t signalID, double d) {
    if (signalID == physicallayer::Radio::maxRSSISignal) {
        W rssi = W(d);
        EV_DEBUG << this->getFullPath() << " - Signal RSSI caught from " << source->getFullPath() << ". Value: " << rssi << endl;

        if (isScanning) {
            scanningList.push_back(rssi);
        }
    }
}

void USVControl::finish(void) {
    if (this->getParentModule()->getIndex() == 0) {

        std::vector< std::vector< CellScanReport > > gridPointsMatrix;
        std::list<PointScan> fullList;
        //fprintf(stderr, "%s SONO 0\n", this->getFullPath().c_str());fflush(stderr);
        int numberOfNodes = this->getParentModule()->getParentModule()->par("numHosts");

        gridPointsMatrix.resize((ffmob->getConstraintAreaMax().x - ffmob->getConstraintAreaMin().x) / sizeOfScenaioReportCells);
        for (unsigned int x = 0; x < gridPointsMatrix.size(); x++) {

            gridPointsMatrix[x].resize((ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y) / sizeOfScenaioReportCells);

            for (unsigned int y = 0; y < gridPointsMatrix[x].size(); y++) {
                gridPointsMatrix[x][y].scanReport = false;
                gridPointsMatrix[x][y].calculateReport = false;
                //gridPointsMatrix[x][y].listPoints.clear();
            }
        }

        //fprintf(stderr, "La mia mappa è di demensioni %lux%lu\n", gridPointsMatrix.size(), gridPointsMatrix[0].size());fflush(stderr);

        //fprintf(stderr, "%s e ci sono %d host\n", this->getFullPath().c_str(), numberOfNodes);fflush(stderr);

        for (int i = 0; i < numberOfNodes; i++) {
            USVControl *usvNode = check_and_cast<USVControl *>(this->getParentModule()->getParentModule()->getSubmodule("host", i)->getSubmodule("usv_brain"));

            //fprintf(stderr, "Sono %s\n", usvNode->getFullPath().c_str());fflush(stderr);

            for (std::list<PointScan>::iterator it = usvNode->scannedPoints.begin();  it != usvNode->scannedPoints.end();  it++) {
                fullList.push_back(*it);

                int xP = MIN((it->pos.x - ffmob->getConstraintAreaMin().x) / sizeOfScenaioReportCells, gridPointsMatrix.size() - 1);
                int yP = MIN((it->pos.y - ffmob->getConstraintAreaMin().y) / sizeOfScenaioReportCells, gridPointsMatrix[0].size() - 1);

                gridPointsMatrix[xP][yP].listPoints.push_back(*it);
            }
        }

        //make the report
        std::string fn_grid_scan = filename_output_grid + std::string("_scan");
        std::string fn_grid_calc = filename_output_grid + std::string("_calc");
        std::string fn_grid_diff = filename_output_grid + std::string("_diff");

        FILE *f_grid_scan = fopen(fn_grid_scan.c_str(), "w");
        FILE *f_grid_calc = fopen(fn_grid_calc.c_str(), "w");
        FILE *f_grid_diff = fopen(fn_grid_diff.c_str(), "w");


        double cell_free, cell_busy, cell_unknown, tot_cell;
        cell_free = cell_busy = cell_unknown = tot_cell = 0;

        for (unsigned int x = 0; x < gridPointsMatrix.size(); x++) {
            for (unsigned int y = 0; y < gridPointsMatrix[x].size(); y++) {
                tot_cell++;

                for (std::list<PointScan>::iterator it = gridPointsMatrix[x][y].listPoints.begin(); it != gridPointsMatrix[x][y].listPoints.end(); it++) {
                    PointScan *ps = &(*it);
                    if (ps->scanLog.actualResult) {
                        gridPointsMatrix[x][y].scanReport = true;

                        //break;
                    }
                    if (ps->scanLog.calculatedResult) {
                        gridPointsMatrix[x][y].calculateReport = true;

                        //break;
                    }
                }

                if (gridPointsMatrix[x][y].listPoints.size() > 0){
                    if (f_grid_scan) fprintf(f_grid_scan, "%s ", gridPointsMatrix[x][y].scanReport ? "1" : "0");
                    if (f_grid_calc) fprintf(f_grid_calc, "%s ", gridPointsMatrix[x][y].calculateReport ? "1" : "0");
                    //fprintf(stderr, "[%s] ", gridPointsMatrix[x][y].scanReport ? "1" : "0");fflush(stderr);
                    if (gridPointsMatrix[x][y].scanReport) {
                        cell_busy++;
                    } else {
                        cell_free++;
                    }
                }
                else {
                    if (f_grid_scan) fprintf(f_grid_scan, "X ");
                    if (f_grid_calc) fprintf(f_grid_calc, "X ");
                    //fprintf(stderr, "[X] ");fflush(stderr);
                    cell_unknown++;
                }

                if (gridPointsMatrix[x][y].scanReport != gridPointsMatrix[x][y].calculateReport) {
                    if (gridPointsMatrix[x][y].calculateReport) {
                        if (f_grid_diff) fprintf(f_grid_diff, "N ");
                    } else {
                        if (f_grid_diff) fprintf(f_grid_diff, "P ");
                    }
                }
                else {
                    if (f_grid_diff) fprintf(f_grid_diff, "- ");
                }
            }

            if (f_grid_scan) fprintf(f_grid_scan, "\n");
            if (f_grid_calc) fprintf(f_grid_calc, "\n");
            if (f_grid_diff) fprintf(f_grid_diff, "\n");
            //fprintf(stderr, "\n");fflush(stderr);
        }

        if (f_grid_scan) fclose(f_grid_scan);
        if (f_grid_calc) fclose(f_grid_calc);
        if (f_grid_diff) fclose(f_grid_diff);

        recordScalar("freeCells", cell_free);
        recordScalar("unknownCells", cell_unknown);
        recordScalar("busyCells", cell_busy);
        recordScalar("totCells", tot_cell);
        recordScalar("percentageCellScan", (cell_free + cell_busy) / tot_cell);

        // scan percentage
        int n_busy, n_free, n_tot;
        n_busy = n_free = n_tot = 0;

        for (std::list<PointScan>::iterator it = fullList.begin();  it != fullList.end();  it++) {
            //    printf("%d: [%lf:%lf] -> %s\n", it->scanningHostAddr, it->pos.x, it->pos.y, it->scanLog.actualResult ? "busy": "free");
            n_tot++;
            if (it->scanLog.actualResult) {
                n_busy++;
            }
            else {
                n_free++;
            }
        }

        recordScalar("freeScans", n_free);
        recordScalar("busyScans", n_busy);
        recordScalar("totScans", n_tot);
    }
}

} /* namespace inet */
