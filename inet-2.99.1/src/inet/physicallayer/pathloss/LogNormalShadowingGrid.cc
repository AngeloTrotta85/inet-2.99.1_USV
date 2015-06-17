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

#include "inet/physicallayer/pathloss/LogNormalShadowingGrid.h"

namespace inet {
namespace physicallayer {

Define_Module(LogNormalShadowingGrid);

LogNormalShadowingGrid::LogNormalShadowingGrid() :
        LogNormalShadowing()
{
}

void LogNormalShadowingGrid::initialize(int stage)
{
    LogNormalShadowing::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {

        if (!readChannelGridFile())
            throw cRuntimeError("LogNormalShadowingGrid: error in reading the shadowing file");
    }
}

std::ostream& LogNormalShadowingGrid::printToStream(std::ostream& stream, int level) const
{
    stream << "LogNormalShadowingGrid";
    if (level >= PRINT_LEVEL_TRACE)
        stream << ", alpha = " << alpha
               << ", systemLoss = " << systemLoss
               << ", sigma = " << sigma;
    return stream;
}

double LogNormalShadowingGrid::computePathLoss(mps propagationSpeed, Hz frequency, m distance) const
{
    m d0 = m(1.0);
    // reference path loss
    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, alpha, systemLoss);
    double PL_d0_db = 10.0 * log10(1 / freeSpacePathLoss);
    // path loss at distance d + normal distribution with sigma standard deviation
    double PL_db = PL_d0_db + 10 * alpha * log10(unit(distance / d0).get()) + normal(0.0, sigma);
    return math::dB2fraction(-PL_db);
}

/**
 * Function to read the specified posture specification input file and load the shadow grid.
 * The shadows-parameters are organized as a tree
 */
bool LogNormalShadowingGrid::readChannelGridFile() {
    cXMLElement *xmlShadow = par("shadowFile").xmlValue();
    if (xmlShadow == nullptr)
        return false;

    // read the specification of every posture from file and make a list of postures
    cXMLElementList shadowCell;

    shadowCell = xmlShadow->getChildrenByTagName("shadowCell");

    // find the number of defined postures
    int numCell = shadowCell.size();
    if (numRoot == 0)
        throw cRuntimeError("Error defining the shadow-map grid");

    EV_DEBUG << "Reading XML shadow file. TAG shadowCell found: " << numCell << endl;




    return true;
}

} /* namespace physicallayer */
} /* namespace inet */
