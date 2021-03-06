//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "inet/physicallayer/base/packetlevel/ReceiverBase.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"
#include "inet/physicallayer/contract/packetlevel/IRadioSignal.h"
#include "inet/physicallayer/common/packetlevel/BandListening.h"

namespace inet {

namespace physicallayer {

bool ReceiverBase::computeIsReceptionPossible(const ITransmission *transmission) const
{
    return true;
}

bool ReceiverBase::computeIsReceptionAttempted(const IListening *listening, const IReception *reception, const IInterference *interference) const
{
    if (!computeIsReceptionPossible(listening, reception))
        return false;
    else if (simTime() == reception->getStartTime())
        // TODO: isn't there a better way for this optimization? see also in RadioMedium::isReceptionAttempted
        return !reception->getReceiver()->getReceptionInProgress();
    else {
        const IRadio *radio = reception->getReceiver();
        const IRadioMedium *radioMedium = radio->getMedium();
        const std::vector<const IReception *> *interferingReceptions = interference->getInterferingReceptions();
        for (auto interferingReception : *interferingReceptions) {
            
            bool isPrecedingReception = interferingReception->getStartTime() < reception->getStartTime() ||
                (interferingReception->getStartTime() == reception->getStartTime() &&
                 interferingReception->getTransmission()->getId() < reception->getTransmission()->getId());
            if (isPrecedingReception) {
                const ITransmission *interferingTransmission = interferingReception->getTransmission();
                if (interferingReception->getStartTime() <= simTime()) {
                    if (radio->getReceptionInProgress() == interferingTransmission)
                        return false;
                }
                else if (radioMedium->isReceptionAttempted(radio, interferingTransmission))
                    return false;
            }
        }
        return true;
    }
}

W ReceiverBase::computeMinReceivedPower(const IListening *listening, const IReception *reception, const ITransmission *transmission) const {
    W minReceptionPower = W(NaN);
    const INarrowbandSignal *narrowbandSignalAnalogModel = dynamic_cast<const INarrowbandSignal *>(reception->getAnalogModel());
    const BandListening *bandListening = dynamic_cast<const BandListening *>(listening);

    if (narrowbandSignalAnalogModel && bandListening) {
        if ((bandListening->getCarrierFrequency() == narrowbandSignalAnalogModel->getCarrierFrequency()) &&
                (bandListening->getBandwidth() == narrowbandSignalAnalogModel->getBandwidth())) {
            minReceptionPower = narrowbandSignalAnalogModel->computeMinPower(reception->getStartTime(), reception->getEndTime());
        }
    }

    //if ((narrowbandSignalAnalogModel) && (computeIsReceptionPossible(transmission)))
       // minReceptionPower = narrowbandSignalAnalogModel->computeMinPower(reception->getStartTime(), reception->getEndTime());

    //EV << "ReceiverBase::computeMinReceivedPower received MIN: " << minReceptionPower << " - sensitivity: " << this->getMinReceptionPower() << endl;

    return minReceptionPower;
}

W ReceiverBase::computeMaxReceivedPower(const IListening *listening, const IReception *reception, const ITransmission *transmission) const {
    W maxReceptionPower = W(NaN);

    const INarrowbandSignal *narrowbandSignalAnalogModel = dynamic_cast<const INarrowbandSignal *>(reception->getAnalogModel());
    const BandListening *bandListening = dynamic_cast<const BandListening *>(listening);

    //EV << "ReceiverBase::computeMaxReceivedPower - analogModel:" << reception->getAnalogModel()->getDetailStringRepresentation()
    //            << " bandListening:" << listening->getDetailStringRepresentation() << endl;
    //EV << "ReceiverBase::computeMaxReceivedPower - analogModel:" << narrowbandSignalAnalogModel
    //        << " bandListening:" << bandListening << endl;
    //EV << "ReceiverBase::computeMaxReceivedPower1 - bandList::getCarrierFrequency:" << bandListening->getCarrierFrequency()
    //        << " analModel::carrFreq: " << narrowbandSignalAnalogModel->getCarrierFrequency()
    //            << " - bandList::getBandwidth:" << bandListening->getBandwidth()
    //            << " analModel::getBandwidth:" << narrowbandSignalAnalogModel->getBandwidth() << endl;

    if (narrowbandSignalAnalogModel && bandListening) {
        if ((bandListening->getCarrierFrequency() == narrowbandSignalAnalogModel->getCarrierFrequency()) &&
                (bandListening->getBandwidth() == narrowbandSignalAnalogModel->getBandwidth())) {

            maxReceptionPower = narrowbandSignalAnalogModel->computeMaxPower(reception->getStartTime(), reception->getEndTime());
            //EV << "maxReceptionPower: " << maxReceptionPower << endl;
        }
    }

    //if ((narrowbandSignalAnalogModel) && (computeIsReceptionPossible(transmission)))
    //    maxReceptionPower = narrowbandSignalAnalogModel->computeMaxPower(reception->getStartTime(), reception->getEndTime());

    //EV << "ReceiverBase::computeMaxReceivedPower received MAX: " << maxReceptionPower << " - sensitivity: " << this->getMinReceptionPower() << endl;
    return maxReceptionPower;
}

} // namespace physicallayer

} // namespace inet

