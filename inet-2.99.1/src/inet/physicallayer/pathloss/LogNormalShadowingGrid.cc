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
    if (numCell == 0)
        throw cRuntimeError("Error defining the shadow-map grid");

    EV_DEBUG << "Reading XML shadow file. TAG shadowCell found: " << numCell << endl;

    //set starting default value for the whole scenario
    grid_map.exponent = 2;
    grid_map.stddev = grid_map.variance = 0;
    grid_map.children.clear();
    grid_map.card = GridCardinality::OVERALL;

    loadXMLtree(&shadowCell, &grid_map);

    /*do {
        cXMLElementList::const_iterator cell_part;
        for (cell_part = shadowCell.begin(); cell_part != shadowCell.end(); cell_part++) {

        }
    } while (!shadowCell.empty());*/

    printMap(&grid_map, 0);

    return true;
}

void LogNormalShadowingGrid::loadXMLtree(cXMLElementList *xml, Cell_t *grid) {

    if (xml->size() == 0) {
        return;
    }
    else {
        cXMLElementList::const_iterator cell_part;
        for (cell_part = xml->begin(); cell_part != xml->end(); cell_part++) {
            Cell_t new_grid;
            cXMLElement *act_xml = *cell_part;

            //default values
            new_grid.exponent = 2;
            new_grid.stddev = grid_map.variance = 0;
            new_grid.children.clear();
            new_grid.card = GridCardinality::OVERALL;


            // read the attributes
            const char *str = act_xml->getAttribute("cardinality");
            if (str != NULL) {
                if (strcmp(str, "NE") == 0) {
                    new_grid.card = GridCardinality::NE;
                }
                else if (strcmp(str, "NW") == 0) {
                    new_grid.card = GridCardinality::NW;
                }
                else if (strcmp(str, "SE") == 0) {
                    new_grid.card = GridCardinality::SE;
                }
                else if (strcmp(str, "SW") == 0) {
                    new_grid.card = GridCardinality::SW;
                }
            }
            str = act_xml->getAttribute("exponent");
            if (str != NULL) {
                new_grid.exponent = strtod(str, nullptr);
            }
            str = act_xml->getAttribute("variance");
            if (str != NULL) {
                new_grid.variance = strtod(str, nullptr);
                new_grid.stddev = sqrt(new_grid.variance);
            }

            // read recoursively
            cXMLElementList shadowCell;
            shadowCell = act_xml->getChildrenByTagName("shadowCell");

            loadXMLtree(&shadowCell, &new_grid);

            grid->children.push_back(new_grid);
        }
    }

}

void LogNormalShadowingGrid::printMap(Cell_t *map, int ntab) {
    for (int i = 0; i < ntab; i++) {
        EV << "\t\t";
    }
    switch (map->card) {
    case GridCardinality::OVERALL:
        EV << "OVERALL";
        break;
    case GridCardinality::NE:
        EV << "NE";
        break;
    case GridCardinality::NW:
        EV << "NW";
        break;
    case GridCardinality::SE:
        EV << "SE";
        break;
    case GridCardinality::SW:
        EV << "SW";
        break;
    }
    //EV << "CARD: " << map->card << "; EXP: " << map->exponent << "VAR: " << map->variance << "STDDEV: " << map->stddev << endl;
    EV << "; EXP: " << map->exponent << "; VAR: " << map->variance << "; STDDEV: " << map->stddev << endl;

    for (std::list<struct Cell>::iterator it = map->children.begin(); it != map->children.end(); it++) {
        Cell_t *new_map = &(*it);
        printMap(new_map, ntab+1);
    }
}

} /* namespace physicallayer */
} /* namespace inet */
