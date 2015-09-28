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


#define CELL_MIN_SIZE 25                //TODO 25 is a magic number! (to make at least 3 uncorrelated scanning in urban environment)
#define CELL_ALPHA_DIFF_OFFSET 0.3      //TODO 0.5 = magic number (offset in alpha to consider the cell of the same alpha)


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
        radiusApproximatedMap = par("radiusApproximatedMap");
        statisticsTime = par("statisticsTime");
        otherScanToSendProbability = par("otherScanToSendProbability");
        k_over_n = par("k_over_n");

        //scanPowerThreshold = W(par("scanPowerThreshold").doubleValue());
        scanPowerThreshold_dBm = par("scanPowerThreshold");
        scanPowerThreshold = mW(math::dBm2mW(scanPowerThreshold_dBm));

        ffmob = check_and_cast<FieldForceMobility *>(this->getParentModule()->getSubmodule("mobility"));
        pathLossModel = check_and_cast<physicallayer::LogNormalShadowingGrid *>(this->getParentModule()->getParentModule()->getSubmodule("radioMedium")->getSubmodule("pathLoss"));

        signalMapOffset = ffmob->getConstraintAreaMin();

        //signalPropMap.resize((int)(ffmob->getConstraintAreaMax().x - ffmob->getConstraintAreaMin().x));
        signalPropMap.resize((int)(ffmob->getConstraintAreaMax().x));
        //approximatedPropMap.resize((int)(ffmob->getConstraintAreaMax().x));
        for (unsigned int x = 0; x < signalPropMap.size(); x++) {
            //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
            signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y));
            //approximatedPropMap.resize((int)(ffmob->getConstraintAreaMax().y));

            for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {
                signalPropMap[x][y].pathloss_alpha = 2;
                signalPropMap[x][y].lognormal_sigma = 1;
                signalPropMap[x][y].number_of_scans = 0;

                //approximatedPropMap[x][y].resize(0);
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


        free_cells_percentage.setName("Free-cells percentage");
        freeOverScanned_cells_percentage.setName("FreeOverScanned-cells percentage");
        busy_cells_percentage.setName("Busy-cells percentage");
        busyOverScanned_cells_percentage.setName("BusyOverScanned-cells percentage");
        scanned_cells_percentage.setName("Scanned-cells percentage");
        falsePositive_cells_percentage.setName("FalsePositive-cells percentage");
        falseNegative_cells_percentage.setName("FalseNegative-cells percentage");

        WATCH_LIST(scannedPoints_fromOthers);
        WATCH_LIST(scannedPoints);
    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER) {
        if (pathLossMapAvailable) {

            //copy the map from the pathloss simulation module
            for (unsigned int x = 0; x < signalPropMap.size(); x++) {
                for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {

                    pathLossModel->getAlphaSigmaFromAbsCoord(Coord(x, y),
                            signalPropMap[x][y].pathloss_alpha,
                            signalPropMap[x][y].lognormal_sigma);

                    signalPropMap[x][y].number_of_scans = 1;
                }
            }
        }
    }
    else if (stage == INITSTAGE_LAST) {

        // GET tower0 Informations
        cModule *tower0 = this->getParentModule()->getParentModule()->getSubmodule("tower", 0);
        MobilityBase *txMob = check_and_cast<MobilityBase *>(tower0->getSubmodule("mobility"));
        physicallayer::Radio *radioTX = check_and_cast<physicallayer::Radio *>(tower0->getSubmodule("wlan", 0)->getSubmodule("radio"));

        tower0RadioTransmitter = check_and_cast<physicallayer::Ieee80211ScalarTransmitter *>(radioTX->getSubmodule("transmitter"));
        myRadio = check_and_cast<physicallayer::Radio *>(this->getParentModule()->getSubmodule("wlan", 0)->getSubmodule("radio"));

        powerTX = radioTX->getTransmitter()->getMaxPower();
        powerTX_dBm = math::mW2dBm(powerTX.get() * 1000);
        positionTX = txMob->getCurrentPosition();
        pathLossD0 = pathLossModel->getPathLossD0(myRadio, tower0RadioTransmitter->getCarrierFrequency(), 2.0);
        //powerTX = mW(10000);
        //positionTX = Coord(50,50);
        //pathLossD0 = 0;

        if (this->getParentModule()->getIndex() == 0) {
            statisticsTimer = new cMessage("statisticMsg");
            scheduleAt(simTime() + statisticsTime, statisticsTimer);

            double cellMinSize = CELL_MIN_SIZE;

            online_gridReportMatrix.resize((ffmob->getConstraintAreaMax().x - ffmob->getConstraintAreaMin().x - 1) / cellMinSize);     //'-1' is to avoid a cell of size 1
            for (unsigned int x = 0; x < online_gridReportMatrix.size(); x++) {
                online_gridReportMatrix[x].resize((ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y - 1) / cellMinSize);     //'-1' is to avoid a cell of size 1

                for (unsigned int y = 0; y < online_gridReportMatrix[x].size(); y++) {
                    online_gridReportMatrix[x][y] = nullptr;
                }
            }

            for (unsigned int x = 0; x < online_gridReportMatrix.size(); x++) {
                for (unsigned int y = 0; y < online_gridReportMatrix[x].size(); y++) {
                    if (online_gridReportMatrix[x][y] == nullptr) {
                        CellScanReport newCellRep;

                        newCellRep.scanReport = false;
                        newCellRep.calculateReport = false;

                        online_gridReportList.push_back(newCellRep);
                        online_gridReportMatrix[x][y] = &online_gridReportList.back();

                        double alpha, sigma, dist;
                        int xp = (x * cellMinSize) + (cellMinSize/2);
                        int yp = (y * cellMinSize) + (cellMinSize/2);
                        pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp, yp), alpha, sigma);

                        dist = (calculateUncorrelatedDistanceFromAlpha(alpha) * 3.0) + 1.0;

                        //fprintf(stderr, "Distance to make different cells: %lf with alpha: %lf in position [%i %i]\n",
                        //        dist, alpha, xp, yp);

                        for (unsigned int xnext = x; xnext < online_gridReportMatrix.size(); xnext++) {
                            unsigned int ynext = y;
                            for (ynext = y; ynext < online_gridReportMatrix[xnext].size(); ynext++) {
                                if (online_gridReportMatrix[xnext][ynext] == nullptr) {
                                    double alpha_next, sigma_next;
                                    int xp_next = (xnext * cellMinSize) + (cellMinSize/2);
                                    int yp_next = (ynext * cellMinSize) + (cellMinSize/2);
                                    pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp_next, yp_next), alpha_next, sigma_next);

                                    //fprintf(stderr, "Alpha1: %lf; Alpha2: %lf; DistanceX: %i; DistanceY: %i\n",
                                    //        alpha, alpha_next, abs(xp_next - xp), abs(yp_next - yp));

                                    if (    ((abs(xp_next - xp)) < dist) &&
                                            ((abs(yp_next - yp)) < dist) &&
                                            (fabs(alpha_next - alpha) < CELL_ALPHA_DIFF_OFFSET)  ) {
                                        online_gridReportMatrix[xnext][ynext] = online_gridReportMatrix[x][y];
                                    }
                                    else {
                                        break;
                                    }
                                }
                            }

                            if (ynext == y) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
}

/** @brief This modules should only receive self-messages. */
void USVControl::handleMessage(cMessage *msg) {
    if (msg == checkScanTimer) {
        bool imScanning = checkIfScan();
        //bool imScanning = dblrand() < 0.4;

        if(imScanning) {
            // make the scan
            startScanning();
        }
        else {
            scheduleAt(simTime() + checkScanTimeStep, msg);
        }
    }
    else if (msg == isScanningTimer) {
        endScanning();
    }
    else if (msg == statisticsTimer) {
        makeOnlineStats();
        scheduleAt(simTime() + statisticsTime, statisticsTimer);
    }
}

bool USVControl::checkIfScan(void) {
    bool ris = false;
    double probToScan, maxForce;

    maxForce = -1;

    //fprintf(stderr, "Sono %02d - checkIfScan - MYscannedPoints: %02d - OTHERSscannedPoints:%02d\n",
    //        this->getParentModule()->getIndex(), (int) scannedPoints.size(), (int) scannedPoints_fromOthers.size()); fflush(stderr);

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

    addScanOnApproximatedMap(&newps);

    // at the end update the parameters of the mobility control
    updateMobilityPointsParameters();
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

            //fprintf(stderr, "DEBUG!!!\n%s", printStr.str().c_str());
        }
    }

    return ris;
}

double USVControl::calculateUncorrelatedDistanceFromAlpha(double alpha) {

    double alpha_tmp = alpha;
    // faccio una proporzione per cui urban=8(alpha>=5), suburban=500(alpha<=2)
    if (alpha_tmp < 2) alpha_tmp = 2;
    if (alpha_tmp > 5) alpha_tmp = 5;

    //TODO faccio semplice, da modificare
    //return (((alpha_tmp - 2.0) / (5.0 - 2.0)) * (8.0 - 50.0)) + 50.0;   //uscito dalla retta passante per 2 punti (2,50)-(5,8)

    return ((500.0 - 8.0) * exp(-2.0 * (alpha_tmp - 2.0))) + 8;         // curva discendente che passa per (2,50)-(5,8) (5,8)~quasi
}

double USVControl::calculateUncorrelatedDistance(Coord point) {
    double alpha_loss;
    //double sigma_loss;

    alpha_loss = signalPropMap[point.x][point.y].pathloss_alpha;
    //sigma_loss = signalPropMap[point.x][point.y].lognormal_sigma;

    //ris = (((alpha_loss - 2) / (5 - 2)) * (8 - 50)) + 50;   //uscito dalla retta passante per 2 punti (2,50)-(5,8)

    return calculateUncorrelatedDistanceFromAlpha(alpha_loss);
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

        ffmob->addPersistentRepulsiveForce(ps->scanningID, ps->scanningHostAddr, ps->pos, defaultRepulsiveWeigth, decade_factor);
    }

    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        PointScan *ps = &(*it);

        double decade_factor = calculateDecayFromWeigthAndChannelLoss(desiredWeigthRatio, defaultRepulsiveWeigth, ps->pos);

        ffmob->addPersistentRepulsiveForce(ps->scanningID, ps->scanningHostAddr, ps->pos, defaultRepulsiveWeigth, decade_factor);
    }
}

ScannedPointsList *USVControl::getPacketToSend(void) {
    char msgName[32];
    ScannedPointsList *pkt;

    sprintf(msgName, "CooperativeScanningApp-%d", pktGenerated++);
    pkt = new ScannedPointsList(msgName);

    //int n_scanned_points = scannedPoints.size() + scannedPoints_fromOthers.size();

    // fill the packet with the scanned points
    pkt->setScanPointsArraySize(scannedPoints.size());
    int i = 0;
    for (std::list<PointScan>::iterator it = scannedPoints.begin(); it != scannedPoints.end(); it++) {
        PointScan *ps = &(*it);
        struct ScannedPoint newP;

        newP.position = ps->pos;
        newP.timestamp = ps->scan_timestamp;
        newP.scanID = ps->scanningID;
        newP.scannerAddr = this->getParentModule()->getIndex();
        newP.decisionMade = ps->scanLog.actualResult;
        newP.watt_read = ps->scanLog.powerReceived.get();

        pkt->setScanPoints(i++, newP);
    }

    std::list<PointScan *> others_scanP;
    for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
        PointScan *ps = &(*it);
        if (dblrand() <= otherScanToSendProbability) {
            others_scanP.push_back(ps);
        }
    }
    pkt->setScanPoints_fromOthersArraySize(others_scanP.size());
    i = 0;
    for (std::list<PointScan *>::iterator it = others_scanP.begin(); it != others_scanP.end(); it++) {
        PointScan *ps = *it;
        struct ScannedPoint newP;

        newP.position = ps->pos;
        newP.timestamp = ps->scan_timestamp;
        newP.scanID = ps->scanningID;
        newP.scannerAddr = ps->scanningHostAddr;
        newP.decisionMade = ps->scanLog.actualResult;
        newP.watt_read = ps->scanLog.powerReceived.get();

        pkt->setScanPoints_fromOthers(i++, newP);
    }

    pkt->setNodePosition(ffmob->getCurrentPosition());
    pkt->setNodeAddr(this->getParentModule()->getIndex());

    pkt->setByteLength((pkt->getScanPointsArraySize() + pkt->getScanPoints_fromOthersArraySize()) * sizeof(struct ScannedPoint) + 64);

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
            ps.scanLog.actualResult = sp->decisionMade;
            ps.scanLog.powerReceived = W(sp->watt_read);

            scannedPoints_fromOthers.push_back(ps);

            //fprintf(stderr, "[%d] - Adding point-%d from %d\n", this->getParentModule()->getIndex(), ps.scanningID, ps.scanningHostAddr); fflush(stderr);

            addScanOnApproximatedMap(&ps);
        }
    }

    for (unsigned int i = 0; i < pkt->getScanPoints_fromOthersArraySize(); i++) {
        bool already = false;

        if (pkt->getScanPoints_fromOthers(i).scannerAddr == this->getParentModule()->getIndex()) continue;

        for (std::list<PointScan>::iterator it = scannedPoints_fromOthers.begin(); it != scannedPoints_fromOthers.end(); it++) {
            PointScan *ps = &(*it);

            if (    (ps->scanningHostAddr == pkt->getScanPoints_fromOthers(i).scannerAddr) &&
                    (ps->scanningID == pkt->getScanPoints_fromOthers(i).scanID)){
                //if (ps->pos == pkt->getScanPoints(i).position) {
                already = true;
                break;
            }
        }

        if (!already){
            ScannedPoint *sp = &(pkt->getScanPoints_fromOthers(i));
            PointScan ps;

            ps.pos = sp->position;
            ps.scan_timestamp = sp->timestamp;
            ps.scanningID = sp->scanID;
            ps.scanningHostAddr = sp->scannerAddr;
            ps.scanLog.actualResult = sp->decisionMade;
            ps.scanLog.powerReceived = W(sp->watt_read);

            scannedPoints_fromOthers.push_back(ps);

            //fprintf(stderr, "[%d] - Adding point-%d from %d\n", this->getParentModule()->getIndex(), ps.scanningID, ps.scanningHostAddr); fflush(stderr);

            addScanOnApproximatedMap(&ps);
        }
    }

    updateMobilityPointsParameters();
}

void USVControl::addScanOnApproximatedMap(PointScan *ps) {
    if (!pathLossMapAvailable) {
        int minX, maxX, minY, maxY;

        minX = MAX(0, ps->pos.x - radiusApproximatedMap);
        maxX = MIN(ffmob->getConstraintAreaMax().x, ps->pos.x + radiusApproximatedMap);
        minY = MAX(0, ps->pos.y - radiusApproximatedMap);
        maxY = MIN(ffmob->getConstraintAreaMax().y, ps->pos.y + radiusApproximatedMap);

        for (int x = minX; x < maxX; x++) {
            for (int y = minY; y < maxY; y++) {

                if (ps->pos.distance(Coord(x,y)) <= radiusApproximatedMap) {

                    double actPathLossD0;
                    if (signalPropMap[x][y].number_of_scans == 0) {
                        actPathLossD0 = pathLossD0;
                    }
                    else {
                        actPathLossD0 = pathLossModel->getPathLossD0(myRadio, tower0RadioTransmitter->getCarrierFrequency(), signalPropMap[x][y].pathloss_alpha);
                    }

                    double actAlpha = 2.0;  //default but to define later

                    if (ps->scanLog.actualResult) {

                        double prx_dbm = math::mW2dBm(ps->scanLog.powerReceived.get() * 1000);
                        //actAlpha = (ptx_dbm - prx_dbm - pathLossD0) / (10.0 * log10(ps->pos.distance(positionTX)));
                        actAlpha = (powerTX_dBm - prx_dbm - actPathLossD0) / (10.0 * log10(ps->pos.distance(positionTX)));

                    }
                    else {
                        // make a guess of the alpha because there was not received any signal
                        actAlpha = (powerTX_dBm - actPathLossD0 - scanPowerThreshold_dBm) / (10.0 * log10(ps->pos.distance(positionTX)));
                    }

                    //DEBUG!
                    //fprintf(stderr, "Calculating new ALPHA. Ptx=%.2lf - Prx=%.2lf(%lfmW) - d=%.2lf - PLd0=%.2lf --- RES:%.2lf\n",
                    //        ptx_dbm, prx_dbm, ps->scanLog.powerReceived.get()*1000, ps->pos.distance(positionTX), pathLossD0, actAlpha);fflush(stderr);

                    if (signalPropMap[x][y].number_of_scans == 0) {
                        signalPropMap[x][y].number_of_scans = 1;
                        signalPropMap[x][y].pathloss_alpha = actAlpha;
                    }
                    else {

                        double newM = signalPropMap[x][y].pathloss_alpha +
                                ((actAlpha - signalPropMap[x][y].pathloss_alpha) / (((double)(signalPropMap[x][y].number_of_scans)) + 1.0));

                        signalPropMap[x][y].number_of_scans++;
                        signalPropMap[x][y].pathloss_alpha = newM;
                    }
                }
            }
        }

    }
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

void USVControl::makeOnlineStats(void) {
    // here there is only node0
    int numberOfNodes = this->getParentModule()->getParentModule()->par("numHosts");

    double cellMinSize = CELL_MIN_SIZE;

    for (unsigned int x = 0; x < online_gridReportMatrix.size(); x++) {
        for (unsigned int y = 0; y < online_gridReportMatrix[x].size(); y++) {
            online_gridReportMatrix[x][y]->calculateReport = false;
            online_gridReportMatrix[x][y]->scanReport = false;
            if (online_gridReportMatrix[x][y]->listPoints.size() > 0) {
                online_gridReportMatrix[x][y]->listPoints.clear();
            }
        }
    }

    for (int i = 0; i < numberOfNodes; i++) {
        USVControl *usvNode = check_and_cast<USVControl *>(this->getParentModule()->getParentModule()->getSubmodule("host", i)->getSubmodule("usv_brain"));

        //fprintf(stderr, "Sono %s\n", usvNode->getFullPath().c_str());fflush(stderr);
        for (std::list<PointScan>::iterator it = usvNode->scannedPoints.begin();  it != usvNode->scannedPoints.end();  it++) {

            int xP = MIN((it->pos.x - ffmob->getConstraintAreaMin().x) / cellMinSize, online_gridReportMatrix.size() - 1);
            int yP = MIN((it->pos.y - ffmob->getConstraintAreaMin().y) / cellMinSize, online_gridReportMatrix[xP].size() - 1);

            online_gridReportMatrix[xP][yP]->listPoints.push_back(*it);
        }
    }

    double cell_free, cell_busy, cell_unknown, tot_cell, n_falseNeg_alarms, n_falsePos_alarms;
    cell_free = cell_busy = cell_unknown = tot_cell = n_falseNeg_alarms = n_falsePos_alarms = 0;

    for (unsigned int x = 0; x < online_gridReportMatrix.size(); x++) {
        for (unsigned int y = 0; y < online_gridReportMatrix[x].size(); y++) {
            tot_cell++;

            for (std::list<PointScan>::iterator it = online_gridReportMatrix[x][y]->listPoints.begin(); it != online_gridReportMatrix[x][y]->listPoints.end(); it++) {
                PointScan *ps = &(*it);
                if (ps->scanLog.actualResult) {
                    online_gridReportMatrix[x][y]->scanReport = true;
                }
                if (ps->scanLog.calculatedResult) {
                    online_gridReportMatrix[x][y]->calculateReport = true;
                }
            }

            if (online_gridReportMatrix[x][y]->listPoints.size() > 0){
                if (online_gridReportMatrix[x][y]->scanReport) {
                    cell_busy++;
                } else {
                    cell_free++;
                }

                if (online_gridReportMatrix[x][y]->scanReport && (!online_gridReportMatrix[x][y]->calculateReport)) {
                    n_falsePos_alarms++;
                }
                else if ((!online_gridReportMatrix[x][y]->scanReport) && online_gridReportMatrix[x][y]->calculateReport) {
                    n_falseNeg_alarms++;
                }
            }
            else {
                cell_unknown++;
            }
        }
    }

    if ((cell_free + cell_busy) > 0) {
        freeOverScanned_cells_percentage.record(cell_free/(cell_free + cell_busy));
        busyOverScanned_cells_percentage.record(cell_busy/(cell_free + cell_busy));
        falsePositive_cells_percentage.record(n_falsePos_alarms/(cell_free + cell_busy));
        falseNegative_cells_percentage.record(n_falseNeg_alarms/(cell_free + cell_busy));
    }
    free_cells_percentage.record(cell_free/tot_cell);
    busy_cells_percentage.record(cell_busy/tot_cell);
    scanned_cells_percentage.record((cell_free + cell_busy) / tot_cell);
}

double USVControl::calcPointQuality(int numScans, double decorr_distance) {
    double ris = 0;
    double deltad = log(decorr_distance) / 2.0;

    if (numScans > 0) {
        ris = (deltad * log(numScans)) + 1.0;
    }

    return ris;
}

void USVControl::finish(void) {
    if (this->getParentModule()->getIndex() == 0) {

        double cellMinSize = CELL_MIN_SIZE;

        std::list<PointScan> fullList;
        int numberOfNodes = this->getParentModule()->getParentModule()->par("numHosts");

        //copy the map from the pathloss simulation module
        std::vector< std::vector<PointMapSignalCharacteristics> > signalMap;

        signalMap.resize((int)(ffmob->getConstraintAreaMax().x));
        for (unsigned int x = 0; x < signalMap.size(); x++) {
            //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
            signalMap[x].resize((int)(ffmob->getConstraintAreaMax().y));

            for (unsigned int y = 0; y < signalMap[x].size(); y++) {
                pathLossModel->getAlphaSigmaFromAbsCoord(Coord(x, y),
                        signalMap[x][y].pathloss_alpha,
                        signalMap[x][y].lognormal_sigma);
            }
        }

        // create a grid of cell-size 25x25 and a list of "CellScanReport"
        std::vector< std::vector< CellScanReport *> > gridReportMatrix;
        std::list<CellScanReport> gridReportList;

        gridReportMatrix.resize((ffmob->getConstraintAreaMax().x - ffmob->getConstraintAreaMin().x - 1) / cellMinSize);     //'-1' is to avoid a cell of size 1
        for (unsigned int x = 0; x < gridReportMatrix.size(); x++) {
            gridReportMatrix[x].resize((ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y - 1) / cellMinSize);     //'-1' is to avoid a cell of size 1

            for (unsigned int y = 0; y < gridReportMatrix[x].size(); y++) {
                gridReportMatrix[x][y] = nullptr;
            }
        }

        for (unsigned int x = 0; x < gridReportMatrix.size(); x++) {
            for (unsigned int y = 0; y < gridReportMatrix[x].size(); y++) {
                if (gridReportMatrix[x][y] == nullptr) {
                    CellScanReport newCellRep;

                    newCellRep.scanReport = false;
                    newCellRep.calculateReport = false;
                    newCellRep.numOfOccupiedScan = 0;
                    newCellRep.numOfFreeScan = 0;

                    gridReportList.push_back(newCellRep);
                    gridReportMatrix[x][y] = &gridReportList.back();

                    double alpha, sigma, dist;
                    int xp = (x * cellMinSize) + (cellMinSize/2);
                    int yp = (y * cellMinSize) + (cellMinSize/2);
                    pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp, yp), alpha, sigma);

                    dist = (calculateUncorrelatedDistanceFromAlpha(alpha) * 3.0) + 1.0;

                    fprintf(stderr, "Distance to make different cells: %lf with alpha: %lf in position [%i %i]\n",
                            dist, alpha, xp, yp);fflush(stderr);

                    for (unsigned int xnext = x; xnext < gridReportMatrix.size(); xnext++) {
                        double alpha_next, sigma_next;
                        unsigned int ynext = y;

                        if (gridReportMatrix[xnext][ynext] == nullptr) {

                            int xp_next = (xnext * cellMinSize) + (cellMinSize/2);
                            int yp0_next = (y * cellMinSize) + (cellMinSize/2);
                            pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp_next, yp0_next), alpha_next, sigma_next);

                            if (    ((abs(xp_next - xp)) < dist) &&
                                    (fabs(alpha_next - alpha) < CELL_ALPHA_DIFF_OFFSET)  ) {

                                for (ynext = y; ynext < gridReportMatrix[xnext].size(); ynext++) {
                                    if (gridReportMatrix[xnext][ynext] == nullptr) {


                                        int yp_next = (ynext * cellMinSize) + (cellMinSize/2);
                                        pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp_next, yp_next), alpha_next, sigma_next);

                                        fprintf(stderr, "[%u,%u]--[%u,%u] - AlphaORIG: %lf; AlphaCheck: %lf; DistanceX: %i; DistanceY: %i; DistLIMIT: %lf\n",
                                                x, y, xnext, ynext, alpha, alpha_next, abs(xp_next - xp), abs(yp_next - yp), dist); fflush(stderr);
                                        fprintf(stdout, "[%u,%u]--[%u,%u] - AlphaORIG: %lf; AlphaCheck: %lf; DistanceX: %i; DistanceY: %i; DistLIMIT: %lf\n",
                                                x, y, xnext, ynext, alpha, alpha_next, abs(xp_next - xp), abs(yp_next - yp), dist); fflush(stdout);

                                        if (    ((abs(yp_next - yp)) < dist) &&
                                                (fabs(alpha_next - alpha) < CELL_ALPHA_DIFF_OFFSET)  ) {
                                            gridReportMatrix[xnext][ynext] = gridReportMatrix[x][y];
                                        }
                                        else {
                                            break;
                                        }
                                    }
                                    else {
                                        break;
                                    }
                                }
                            } else {
                                //if (ynext == y) {
                                break;
                            }
                        }
                        else {
                            break;
                        }
                    }
                }
            }
        }


        /*for (unsigned int x = 0; x < gridReportMatrix.size(); x++) {
            for (unsigned int y = 0; y < gridReportMatrix[x].size(); y++) {
                fprintf(stderr, "%p ", gridReportMatrix[x][y]);
            }
            fprintf(stderr, "\n");
        }*/

        for (int i = 0; i < numberOfNodes; i++) {
            USVControl *usvNode = check_and_cast<USVControl *>(this->getParentModule()->getParentModule()->getSubmodule("host", i)->getSubmodule("usv_brain"));

            //fprintf(stderr, "Sono %s\n", usvNode->getFullPath().c_str());fflush(stderr);
            for (std::list<PointScan>::iterator it = usvNode->scannedPoints.begin();  it != usvNode->scannedPoints.end();  it++) {
                fullList.push_back(*it);

                int xP = MIN((it->pos.x - ffmob->getConstraintAreaMin().x) / cellMinSize, gridReportMatrix.size() - 1);
                int yP = MIN((it->pos.y - ffmob->getConstraintAreaMin().y) / cellMinSize, gridReportMatrix[xP].size() - 1);

                gridReportMatrix[xP][yP]->listPoints.push_back(*it);
            }
        }

        //make the report
        //std::string fn_grid_alpha = filename_output_grid + std::string("_alpha");
        //std::string fn_grid_gridalpha = filename_output_grid + std::string("_gridalpha");
        std::string fn_grid_scan = filename_output_grid + std::string("_scan");
        std::string fn_grid_nscan = filename_output_grid + std::string("_nscan");
        std::string fn_grid_calc = filename_output_grid + std::string("_calc");
        std::string fn_grid_diff = filename_output_grid + std::string("_diff");
        std::string fn_grid_color = filename_output_grid + std::string("_color");
        std::string fn_grid_quality = filename_output_grid + std::string("_quality");
        std::string fn_grid_falsePositiveList = filename_output_grid + std::string("_FP_list");
        std::string fn_grid_falseNegativeList = filename_output_grid + std::string("_FN_list");
        std::list<std::pair<int, CellScanReport *>> grid_color_list;

        FILE *f_grid_alpha = nullptr; // = fopen(fn_grid_alpha.c_str(), "w");
        FILE *f_grid_gridalpha = nullptr; // = fopen(fn_grid_gridalpha.c_str(), "w");
        FILE *f_grid_scan = fopen(fn_grid_scan.c_str(), "w");
        FILE *f_grid_nscan = fopen(fn_grid_nscan.c_str(), "w");
        FILE *f_grid_calc = fopen(fn_grid_calc.c_str(), "w");
        FILE *f_grid_diff = fopen(fn_grid_diff.c_str(), "w");
        FILE *f_grid_color = fopen(fn_grid_color.c_str(), "w");
        FILE *f_grid_quality = fopen(fn_grid_quality.c_str(), "w");
        FILE *f_grid_falsePositiveList = fopen(fn_grid_falsePositiveList.c_str(), "w");
        FILE *f_grid_falseNegativeList = fopen(fn_grid_falseNegativeList.c_str(), "w");

        double sum_scan_per_cell;
        sum_scan_per_cell = 0;

        double quality_sum = 0;

        double cell_free, cell_busy, cell_unknown, tot_cell, n_falseNeg_alarms, n_falsePos_alarms;
        cell_free = cell_busy = cell_unknown = tot_cell = n_falseNeg_alarms = n_falsePos_alarms = 0;

        for (unsigned int x = 0; x < gridReportMatrix.size(); x++) {
            for (unsigned int y = 0; y < gridReportMatrix[x].size(); y++) {
                int this_cell_scan_free = 0;
                int this_cell_scan_busy = 0;

                tot_cell++;

                double sum_alpha = 0;
                double count_alpha = 0;
                int count_alpha_scan = 0;
                for (int i = 0; i < cellMinSize; i++) {
                    int xi = i + (cellMinSize * x);
                    for (int j = 0; j < cellMinSize; j++) {
                        int yi = j + (cellMinSize * y);
                        sum_alpha += signalPropMap[xi][yi].pathloss_alpha;
                        count_alpha_scan += signalPropMap[xi][yi].number_of_scans;
                        count_alpha++;
                    }
                }
                if (f_grid_gridalpha)  {
                    if (count_alpha_scan > 0)   fprintf(f_grid_gridalpha, "%.01lf ", sum_alpha / count_alpha);
                    else                        fprintf(f_grid_gridalpha, "%.01lf ", 0.0);
                }

                int color_cell = -1;
                int max_color = -1;
                for (std::list<std::pair<int, CellScanReport *>>::iterator it = grid_color_list.begin(); it != grid_color_list.end(); it++) {
                    if (max_color < it->first) max_color = it->first;

                    if (it->second == gridReportMatrix[x][y]) {
                        color_cell = it->first;
                    }
                }

                if (color_cell == (-1)) {
                    color_cell = max_color + 1;
                    grid_color_list.push_back(std::make_pair(color_cell, gridReportMatrix[x][y]));
                }
                if (f_grid_color) fprintf(f_grid_color, "%03d ", color_cell);

                for (std::list<PointScan>::iterator it = gridReportMatrix[x][y]->listPoints.begin(); it != gridReportMatrix[x][y]->listPoints.end(); it++) {
                    PointScan *ps = &(*it);
                    if (ps->scanLog.actualResult) {
                        //gridReportMatrix[x][y]->scanReport = true;
                        gridReportMatrix[x][y]->numOfOccupiedScan++;
                        this_cell_scan_busy++;

                        //break;
                    }
                    else {
                        this_cell_scan_free++;
                        gridReportMatrix[x][y]->numOfFreeScan++;
                    }

                    if (ps->scanLog.calculatedResult) {
                        gridReportMatrix[x][y]->calculateReport = true;

                        //break;
                    }
                }

                if (gridReportMatrix[x][y]->numOfOccupiedScan > 0) {
                    if (k_over_n <= 0) {
                        if (gridReportMatrix[x][y]->numOfFreeScan == 0) {
                            gridReportMatrix[x][y]->scanReport = true;      //n_over_n rule. ALL POSITIVE SCAN
                        }
                    }
                    else {
                        if (gridReportMatrix[x][y]->numOfOccupiedScan >= k_over_n) {
                            gridReportMatrix[x][y]->scanReport = true;      //k_over_n rule. AT LEAST K POSITIVE SCAN
                        }
                    }
                }

                double alpha, sigma, decorr_dist, quality_here;
                int xp = (x * cellMinSize) + (cellMinSize/2);
                int yp = (y * cellMinSize) + (cellMinSize/2);
                pathLossModel->getAlphaSigmaFromAbsCoord(Coord(xp, yp), alpha, sigma);
                decorr_dist = calculateUncorrelatedDistanceFromAlpha(alpha);
                quality_here = calcPointQuality(gridReportMatrix[x][y]->listPoints.size(), decorr_dist);
                quality_sum += quality_here;

                if (f_grid_quality)  fprintf(f_grid_quality, "%lf ", quality_here);

                if (f_grid_nscan) fprintf(f_grid_nscan, "%02d ", ((int) gridReportMatrix[x][y]->listPoints.size()));

                if (gridReportMatrix[x][y]->listPoints.size() > 0){
                    if (f_grid_scan) fprintf(f_grid_scan, "%s ", gridReportMatrix[x][y]->scanReport ? "1" : "0");
                    if (f_grid_calc) fprintf(f_grid_calc, "%s ", gridReportMatrix[x][y]->calculateReport ? "1" : "0");
                    //fprintf(stderr, "[%s] ", gridPointsMatrix[x][y].scanReport ? "1" : "0");fflush(stderr);

                    sum_scan_per_cell += gridReportMatrix[x][y]->listPoints.size();

                    if (gridReportMatrix[x][y]->scanReport) {
                        cell_busy++;
                    } else {
                        cell_free++;
                    }

                    if (gridReportMatrix[x][y]->scanReport && (!gridReportMatrix[x][y]->calculateReport)) {
                        n_falsePos_alarms++;

                        if (f_grid_falsePositiveList) {
                            fprintf(f_grid_falsePositiveList, "%d 0:%d 1:%d\n",
                                    (int)gridReportMatrix[x][y]->listPoints.size(), this_cell_scan_free, this_cell_scan_busy);
                        }
                    }
                    else if ((!gridReportMatrix[x][y]->scanReport) && gridReportMatrix[x][y]->calculateReport) {
                        n_falseNeg_alarms++;

                        if (f_grid_falseNegativeList) {
                            fprintf(f_grid_falseNegativeList, "%d 0:%d 1:%d\n",
                                    (int)gridReportMatrix[x][y]->listPoints.size(), this_cell_scan_free, this_cell_scan_busy);
                        }
                    }
                }
                else {
                    if (f_grid_scan) fprintf(f_grid_scan, "X ");
                    if (f_grid_calc) fprintf(f_grid_calc, "X ");
                    //fprintf(stderr, "[X] ");fflush(stderr);
                    cell_unknown++;
                }

                if (gridReportMatrix[x][y]->scanReport != gridReportMatrix[x][y]->calculateReport) {
                    if (gridReportMatrix[x][y]->calculateReport) {
                        if (f_grid_diff) fprintf(f_grid_diff, "N ");
                    } else {
                        if (f_grid_diff) fprintf(f_grid_diff, "P ");
                    }
                }
                else {
                    if (f_grid_diff) fprintf(f_grid_diff, "- ");
                }
            }

            if (f_grid_gridalpha) fprintf(f_grid_gridalpha, "\n");
            if (f_grid_scan) fprintf(f_grid_scan, "\n");
            if (f_grid_nscan) fprintf(f_grid_nscan, "\n");
            if (f_grid_calc) fprintf(f_grid_calc, "\n");
            if (f_grid_diff) fprintf(f_grid_diff, "\n");
            if (f_grid_quality) fprintf(f_grid_quality, "\n");
            if (f_grid_color) fprintf(f_grid_color, "\n");
            //fprintf(stderr, "\n");fflush(stderr);
        }

        for (unsigned int x = 0; x < signalPropMap.size(); x++) {
            for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {
                if (f_grid_alpha) {
                    if (signalPropMap[x][y].number_of_scans > 0)    fprintf(f_grid_alpha, "%.01lf ", signalPropMap[x][y].pathloss_alpha);
                    else                                            fprintf(f_grid_alpha, "%.01lf ", 0.0);
                }
            }
            if (f_grid_alpha) fprintf(f_grid_alpha, "\n");
        }

        if (f_grid_alpha) fclose(f_grid_alpha);
        if (f_grid_gridalpha) fclose(f_grid_gridalpha);
        if (f_grid_scan) fclose(f_grid_scan);
        if (f_grid_nscan) fclose(f_grid_nscan);
        if (f_grid_calc) fclose(f_grid_calc);
        if (f_grid_diff) fclose(f_grid_diff);
        if (f_grid_color) fclose(f_grid_color);
        if (f_grid_quality) fclose(f_grid_quality);
        if (f_grid_falsePositiveList) fclose(f_grid_falsePositiveList);
        if (f_grid_falseNegativeList) fclose(f_grid_falseNegativeList);

        if ((cell_free + cell_busy) > 0) {
            recordScalar("percentageFreeOverScanned", cell_free/(cell_free + cell_busy));
            recordScalar("percentageBusyOverScanned", cell_busy/(cell_free + cell_busy));
            recordScalar("percentageFalsePositiveOverScanned", n_falsePos_alarms/(cell_free + cell_busy));
            recordScalar("percentageFalseNegativeOverScanned", n_falseNeg_alarms/(cell_free + cell_busy));
            recordScalar("scanPerCellAverageOverScanned", sum_scan_per_cell/(cell_free + cell_busy));
        }
        recordScalar("percentageFree", cell_free/tot_cell);
        recordScalar("percentageBusy", cell_busy/tot_cell);
        recordScalar("percentageFalsePositive", n_falsePos_alarms/tot_cell);
        recordScalar("percentageFalseNegative", n_falseNeg_alarms/tot_cell);
        recordScalar("scanPerCellAverage", sum_scan_per_cell/tot_cell);

        recordScalar("freeCells", cell_free);
        recordScalar("unknownCells", cell_unknown);
        recordScalar("busyCells", cell_busy);
        recordScalar("totCells", tot_cell);
        recordScalar("percentageCellScan", (cell_free + cell_busy) / tot_cell);

        recordScalar("qualityIndexAverage", quality_sum / tot_cell);

        // scan percentage
        int n_busy, n_free, n_tot;
        n_busy = n_free = n_tot = 0;

        for (std::list<PointScan>::iterator it = fullList.begin();  it != fullList.end();  it++) {
            // printf("%d: [%lf:%lf] -> %s\n", it->scanningHostAddr, it->pos.x, it->pos.y, it->scanLog.actualResult ? "busy": "free");
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


        if (true) {
            std::vector< std::vector<PointMapSignalCharacteristics> > signalPropMapALL;

            signalPropMapALL.resize((int)(ffmob->getConstraintAreaMax().x));
            //approximatedPropMap.resize((int)(ffmob->getConstraintAreaMax().x));
            for (unsigned int x = 0; x < signalPropMapALL.size(); x++) {
                //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
                signalPropMapALL[x].resize((int)(ffmob->getConstraintAreaMax().y));
                //approximatedPropMap.resize((int)(ffmob->getConstraintAreaMax().y));

                for (unsigned int y = 0; y < signalPropMapALL[x].size(); y++) {
                    signalPropMapALL[x][y].pathloss_alpha = 0.0;
                    signalPropMapALL[x][y].number_of_scans = 0;
                }
            }

            std::string fn_grid_alpha = filename_output_grid + std::string("_alpha");
            std::string fn_grid_alpha_orig = filename_output_grid + std::string("_alpha_orig");
            std::string fn_grid_alpha_err = filename_output_grid + std::string("_alpha_err");
            FILE *f_grid_alpha = fopen(fn_grid_alpha.c_str(), "w");
            FILE *f_grid_alpha_orig = fopen(fn_grid_alpha_orig.c_str(), "w");
            FILE *f_grid_alpha_err = fopen(fn_grid_alpha_err.c_str(), "w");

            for (int i = 0; i < numberOfNodes; i++) {
                USVControl *usvNode = check_and_cast<USVControl *>(this->getParentModule()->getParentModule()->getSubmodule("host", i)->getSubmodule("usv_brain"));

                for (unsigned int x = 0; x < usvNode->signalPropMap.size(); x++) {
                    for (unsigned int y = 0; y < usvNode->signalPropMap[x].size(); y++) {

                        for (int jj = 0; jj < usvNode->signalPropMap[x][y].number_of_scans; jj++) {

                            if (signalPropMapALL[x][y].number_of_scans == 0) {
                                signalPropMapALL[x][y].pathloss_alpha = usvNode->signalPropMap[x][y].pathloss_alpha;
                            }
                            else {
                                signalPropMapALL[x][y].pathloss_alpha = signalPropMapALL[x][y].pathloss_alpha +
                                        ((usvNode->signalPropMap[x][y].pathloss_alpha - signalPropMapALL[x][y].pathloss_alpha) / (((double)(signalPropMapALL[x][y].number_of_scans)) + 1.0));
                            }

                            signalPropMapALL[x][y].number_of_scans++;
                        }
                    }
                }
            }

            for (unsigned int x = 0; x < signalPropMapALL.size(); x++) {
                for (unsigned int y = 0; y < signalPropMapALL[x].size(); y++) {
                    if (f_grid_alpha) {
                        if (signalPropMapALL[x][y].number_of_scans > 0) fprintf(f_grid_alpha, "%.01lf ", signalPropMapALL[x][y].pathloss_alpha);
                        else                                            fprintf(f_grid_alpha, "%.01lf ", 0.0);
                    }
                }
                if (f_grid_alpha) fprintf(f_grid_alpha, "\n");
            }

            if (f_grid_alpha_orig) {
                for (unsigned int x = 0; x < signalPropMapALL.size(); x++) {
                    for (unsigned int y = 0; y < signalPropMapALL[x].size(); y++) {
                        double r_alpha, r_sigma;
                        pathLossModel->getAlphaSigmaFromAbsCoord(Coord(x, y), r_alpha, r_sigma);

                        fprintf(f_grid_alpha_orig, "%.01lf ", r_alpha);
                    }
                    fprintf(f_grid_alpha_orig, "\n");
                }
            }

            if (f_grid_alpha_err) {
                for (unsigned int x = 0; x < signalPropMapALL.size(); x++) {
                    for (unsigned int y = 0; y < signalPropMapALL[x].size(); y++) {
                        double r_alpha, r_sigma;
                        pathLossModel->getAlphaSigmaFromAbsCoord(Coord(x, y), r_alpha, r_sigma);

                        if (signalPropMapALL[x][y].number_of_scans > 0)
                            fprintf(f_grid_alpha_err, "%.01lf ", fabs(signalPropMapALL[x][y].pathloss_alpha - r_alpha));
                        else
                            fprintf(f_grid_alpha_err, "%.01lf ", 0.0);

                    }
                    fprintf(f_grid_alpha_err, "\n");
                }
            }

            if (f_grid_alpha) fclose(f_grid_alpha);
            if (f_grid_alpha_orig) fclose(f_grid_alpha_orig);
            if (f_grid_alpha_err) fclose(f_grid_alpha_err);
        }
    }

    if (true) {
        //make the report FOR each node
        char buff[8];
        snprintf(buff, sizeof(buff), "%d", this->getParentModule()->getIndex());

        std::string fn_grid_alpha = filename_output_grid + std::string("_alpha-") + std::string(buff);

        FILE *f_grid_alpha = fopen(fn_grid_alpha.c_str(), "w");

        if (f_grid_alpha) {
            for (unsigned int x = 0; x < signalPropMap.size(); x++) {
                for (unsigned int y = 0; y < signalPropMap[x].size(); y++) {

                    if (signalPropMap[x][y].number_of_scans > 0)    fprintf(f_grid_alpha, "%.01lf ", signalPropMap[x][y].pathloss_alpha);
                    else                                            fprintf(f_grid_alpha, "%.01lf ", 0.0);
                }
                fprintf(f_grid_alpha, "\n");
            }
        }

        if (f_grid_alpha) fclose(f_grid_alpha);
    }
}

} /* namespace inet */
