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
#include <vector>

#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"

#include "inet/applications/broadcastapp/ScannedPointsList_m.h"

#include "inet/mobility/single/FieldForceMobility.h"
#include "inet/physicallayer/pathloss/LogNormalShadowingGrid.h"


namespace inet {

class FieldForceMobility;

class INET_API USVControl : public cSimpleModule, public cListener {

public:
    typedef struct ScanResult_s {
        W powerReceived;
        bool actualResult;
    } ScanResult;

    typedef struct PointScan_s {
        int scanningHostAddr;
        unsigned int scanningID;
        Coord pos;
        simtime_t scan_timestamp;
        ScanResult scanLog;
    } PointScan;

    typedef struct PointMapSignalCharacteristics_s {
        double pathloss_alpha;
        double lognormal_sigma;
    } PointMapSignalCharacteristics;

protected:

    /** @brief Returns the required number of initialize stages. */
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief This modules should only receive self-messages. */
    virtual void handleMessage(cMessage *msg) override;

    virtual void finish() override;

    bool checkIfScan(void);

    double calculateUncorrelatedDistance(Coord point);
    double calculateDecayFromWeigthAndChannelLoss(double desiredRatio, double fieldWeigth, Coord point);
    double calculateForceFromPoint(Coord point);

    void updateMobilityPointsParameters(void);

    void startScanning(void);
    void endScanning(void);

    void drawScannedPoint(Coord position, bool isBusy);

    virtual void receiveSignal(cComponent *source, simsignal_t signalID, double d) override;

public:
    USVControl();
    ~USVControl();

    ScannedPointsList *getPacketToSend(void);

    void addScannedPointsFromOthers(ScannedPointsList *pkt);

    void getAlphaSigmaInPoint(Coord point, double &alpha, double &sigma);

protected:
    std::list<PointScan> scannedPoints;

private:

    std::list<PointScan> scannedPoints_fromOthers;

    int pktGenerated;
    unsigned int scanningID_idx;

    bool pathLossMapAvailable;

    double defaultRepulsiveWeigth;
    double desiredWeigthRatio;

    FieldForceMobility *ffmob;
    physicallayer::LogNormalShadowingGrid *pathLossModel;

    cMessage *checkScanTimer;
    simtime_t checkScanTimeStep;

    cMessage *isScanningTimer;
    simtime_t scanningTime;

    std::vector< std::vector<PointMapSignalCharacteristics> > signalPropMap;
    Coord signalMapOffset;

    // scanned point color
    int r_point_col;
    int g_point_col;
    int b_point_col;

    bool isScanning;
    std::list<W> scanningList;
    W scanPowerThreshold;

};

inline std::ostream& operator<<(std::ostream& os, const USVControl::PointScan& ps)
{


   return  os << "Addr:" << ps.scanningHostAddr
            << " - ID:" << ps.scanningID
            << " - Coord:" << ps.pos
            << " - Time:" << ps.scan_timestamp;

    //return stream;
    //return os << "(" << coord.x << ", " << coord.y << ", " << coord.z << ")";
}

} /* namespace inet */

#endif /* USVCONTROL_H_ */
