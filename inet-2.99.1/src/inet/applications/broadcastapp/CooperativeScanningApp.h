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

#ifndef COOPERATIVESCANNINGAPP_H_
#define COOPERATIVESCANNINGAPP_H_

#include <vector>

#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"

#include "inet/applications/broadcastapp/ScannedPointsList_m.h"
#include "inet/applications/broadcastapp/USVControl.h"

namespace inet {

/**
 * CooperativeScanning application. See NED for more info.
 */
class INET_API CooperativeScanningApp : public ApplicationBase
{
  protected:
    enum SelfMsgKinds { START = 1, SEND, STOP };

    // parameters
    std::vector<L3Address> destAddresses;
    int localPort = -1, destPort = -1;
    simtime_t startTime;
    simtime_t stopTime;

    // state
    UDPSocket socket;
    cMessage *selfMsg = nullptr;

    // statistics
    int numSent = 0;
    int numReceived = 0;

    static simsignal_t sentPkSignal;
    static simsignal_t rcvdPkSignal;

  private:
    USVControl *usv;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    // chooses random destination address
    virtual L3Address chooseDestAddr();
    virtual void sendPacket();
    virtual void processPacket(cPacket *msg);
    virtual void setSocketOptions();

    virtual void processStart();
    virtual void processSend();
    virtual void processStop();

    virtual bool handleNodeStart(IDoneCallback *doneCallback) override;
    virtual bool handleNodeShutdown(IDoneCallback *doneCallback) override;
    virtual void handleNodeCrash() override;

    //virtual void fillScannedPointPkt (ScannedPointsList *pkt);

  public:
    CooperativeScanningApp() {}
    ~CooperativeScanningApp();
};

} /* namespace inet */

#endif /* COOPERATIVESCANNINGAPP_H_ */


