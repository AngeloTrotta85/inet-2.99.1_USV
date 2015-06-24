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

#include "inet/environment/common/PhysicalEnvironment.h"

//#include "inet/common/ModuleAccess.h"
//#include "inet/physicallayer/common/packetlevel/RadioMedium.h"

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

        scenarioCoordMin.x = par("constraintAreaMinX");
        scenarioCoordMin.y = par("constraintAreaMinY");
        scenarioCoordMin.z = par("constraintAreaMinZ");
        scenarioCoordMax.x = par("constraintAreaMaxX");
        scenarioCoordMax.y = par("constraintAreaMaxY");
        scenarioCoordMax.z = par("constraintAreaMaxZ");

        max_RGB_R = par("maxRGBdraw_r");
        max_RGB_G = par("maxRGBdraw_g");
        max_RGB_B = par("maxRGBdraw_b");
        min_alpha_val = par("minDrawAlpha");
        max_alpha_val = par("maxDrawAlpha");
        color2dark = par("color2dark").boolValue();

        if (!readChannelGridFile())
            throw cRuntimeError("LogNormalShadowingGrid: error in reading the shadowing file");
    }
    else if (stage == INITSTAGE_PHYSICAL_ENVIRONMENT) {
        PhysicalEnvironment *physicalEnvironment = dynamic_cast<PhysicalEnvironment *>(getModuleByPath("environment"));
        if (physicalEnvironment != nullptr) {
            cXMLElement *shadowsXml = new cXMLElement("environment", "", nullptr);
            addXMLchild_object(shadowsXml, &grid_map, scenarioCoordMin, scenarioCoordMax);
            physicalEnvironment->addFromXML(shadowsXml);
        }
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

double LogNormalShadowingGrid::computePathLoss_parametric(mps propagationSpeed, Hz frequency, m distance, double alpha_par, double sigma_par) const
{
    EV_DEBUG << "Using for log-normal path loss"
            << ": alpha = " << alpha_par
            << ", sigma = " << sigma_par
            << ", propagationSpeed = " << propagationSpeed
            << ", frequency = " << frequency
            << ", distance = " << distance
            << endl;

    //throw cRuntimeError("LogNormalShadowingGrid::computePathLoss OK");
    //exit(0);

    m d0 = m(1.0);
    // reference path loss
    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, alpha_par, systemLoss);
    double PL_d0_db = 10.0 * log10(1 / freeSpacePathLoss);
    // path loss at distance d + normal distribution with sigma standard deviation
    double PL_db = PL_d0_db + 10 * alpha_par * log10(unit(distance / d0).get()) + normal(0.0, sigma_par);
    return math::dB2fraction(-PL_db);
}

double LogNormalShadowingGrid::computePathLoss(mps propagationSpeed, Hz frequency, m distance) const
{
    return computePathLoss_parametric(propagationSpeed, frequency, distance, alpha, sigma);
    /*m d0 = m(1.0);
    // reference path loss
    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, alpha, systemLoss);
    double PL_d0_db = 10.0 * log10(1 / freeSpacePathLoss);
    // path loss at distance d + normal distribution with sigma standard deviation
    double PL_db = PL_d0_db + 10 * alpha * log10(unit(distance / d0).get()) + normal(0.0, sigma);
    return math::dB2fraction(-PL_db);*/
}

double LogNormalShadowingGrid::computePathLossExt(mps propagationSpeed, Hz frequency, m distance, Coord transmitter, Coord receiver) const
{
    EV_DEBUG << "Start computing log-normal path loss from TX at " << transmitter << " to RX at = " << receiver << endl;

    return computePathLossGrid(propagationSpeed, frequency, distance, transmitter, receiver);
}

void LogNormalShadowingGrid::getAlphaSigmaFromCoord(const Cell_t *grid, Coord min, Coord max, Coord point, double &alpha_p, double &sigma_p) const
{

    //EV << "Checking point " << point << " in range [" << min << " - " << max << "]" << endl;

    if (    (point.x < min.x) ||
            (point.y < min.y) ||
            (point.x > max.x) ||
            (point.y > max.y)
    ){
        //alpha_p = alpha;
        //sigma_p = sigma;
        //return;
    }
    else {

        alpha_p = grid->exponent;
        sigma_p = grid->stddev;

        //EV << "Is inside alpha: " << alpha_p << " - sigma: " << sigma_p << endl;

        for (std::list<struct Cell>::const_iterator it = grid->children.begin(); it != grid->children.end(); it++) {
            const Cell_t *new_map = &(*it);
            Coord new_min, new_max;

            new_min = min;
            new_max = max;

            switch (new_map->card) {
            default:
            case GridCardinality::OVERALL:
                // get default (the whole)
                break;

            case GridCardinality::NE:
                new_min.x = (min.x + max.x) / 2.0;
                new_max.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::NW:
                new_max.x = (min.x + max.x) / 2.0;
                new_max.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::SE:
                new_min.x = (min.x + max.x) / 2.0;
                new_min.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::SW:
                new_max.x = (min.x + max.x) / 2.0;
                new_min.y = (min.y + max.y) / 2.0;
                break;
            }

            getAlphaSigmaFromCoord(new_map, new_min, new_max, point, alpha_p, sigma_p);
        }
    }

    //EV << "ALPHA: " << alpha_p << " - SIGMA: " << sigma_p << endl;
}

double LogNormalShadowingGrid::computePathLossGrid(mps propagationSpeed, Hz frequency, m distance, Coord transmitter, Coord receiver) const
{
    double grid_alphaMax, grid_sigmaMax;
    double grid_alphaTx, grid_sigmaTx;
    double grid_alphaRx, grid_sigmaRx;

    grid_alphaTx = grid_alphaRx = alpha;
    grid_sigmaTx = grid_sigmaRx = sigma;

    //EV_DEBUG << "Scenario: = " << scenarioCoordMin << " " << scenarioCoordMax << endl;
    //EV_DEBUG << "TX at = " << transmitter << endl;
    //EV_DEBUG << "RX at = " << receiver << endl;

    getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, transmitter, grid_alphaTx, grid_sigmaTx);
    getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, receiver, grid_alphaRx, grid_sigmaRx);

    //EV_DEBUG << "TX: alpha = " << grid_alphaTx << " and sigma = " << grid_sigmaTx << endl;
    //EV_DEBUG << "RX: alpha = " << grid_alphaRx << " and sigma = " << grid_sigmaRx << endl;

    // Choose the one with the maximum value of alpha
    if (grid_alphaRx >= grid_alphaTx) {
        grid_alphaMax = grid_alphaRx;
        grid_sigmaMax = grid_sigmaRx;
    }
    else {
        grid_alphaMax = grid_alphaTx;
        grid_sigmaMax = grid_sigmaTx;
    }

    return computePathLoss_parametric(propagationSpeed, frequency, distance, grid_alphaMax, grid_sigmaMax);
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
    initCellGrid(&grid_map);

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
            initCellGrid(&new_grid);

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

void LogNormalShadowingGrid::initCellGrid(Cell_t *cell) {
    cell->exponent = alpha;
    cell->stddev = sigma;
    cell->variance = sigma*sigma;
    cell->children.clear();
    cell->card = GridCardinality::OVERALL;
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

void LogNormalShadowingGrid::setXMLattr(cXMLElement *el, Coord posObj, double dimObj, double opacity) {
    char buffer[64];

    if (opacity > 1) opacity = 1;
    if (opacity < 0) opacity = 0;

    //fill-color e line-color attribute
#if OMNETPP_CANVAS_VERSION >= 0x20140908
    snprintf(buffer, sizeof(buffer), "%d %d %d", max_RGB_R, max_RGB_G, max_RGB_B);
    el->setAttribute("fill-color", buffer);
#else
    opacity = 1.0 - opacity;
    if (color2dark) {
        snprintf(buffer, sizeof(buffer), "%d %d %d",
                (int)(((double) max_RGB_R) * opacity),
                (int)(((double) max_RGB_G) * opacity),
                (int)(((double) max_RGB_B) * opacity));
        el->setAttribute("fill-color", buffer);

        snprintf(buffer, sizeof(buffer), "%d %d %d", max_RGB_R, max_RGB_G, max_RGB_B);
        el->setAttribute("line-color", buffer);
    }
    else {
        snprintf(buffer, sizeof(buffer), "%d %d %d",
                ((int)(((double) (255 - max_RGB_R)) * opacity)) + max_RGB_R,
                ((int)(((double) (255 - max_RGB_G)) * opacity)) + max_RGB_G,
                ((int)(((double) (255 - max_RGB_B)) * opacity)) + max_RGB_B);
        el->setAttribute("fill-color", buffer);

        snprintf(buffer, sizeof(buffer), "255 255 255");
        el->setAttribute("line-color", buffer);
    }

    opacity = 1;
#endif

    //position attribute
    snprintf(buffer, sizeof(buffer), "min %lf %lf %lf", posObj.x, posObj.y, posObj.z);
    el->setAttribute("position", buffer);

    //shape attribute
    snprintf(buffer, sizeof(buffer), "cuboid %lf %lf %lf", dimObj, dimObj, dimObj);
    el->setAttribute("shape", buffer);

    //material and fill-color constant
    el->setAttribute("material", "vacuum");

    //opacity
#if OMNETPP_CANVAS_VERSION >= 0x20140908
    snprintf(buffer, sizeof(buffer), "%lf", opacity);
    el->setAttribute("opacity", buffer);
#endif
}

#define NE_MASK 0x01
#define NW_MASK 0x02
#define SE_MASK 0x04
#define SW_MASK 0x08
#define OA_MASK 0x10

void LogNormalShadowingGrid::addXMLchild_object(cXMLElement *parent, Cell_t *map, Coord min, Coord max) {

    double myExponent = map->exponent;
    if(myExponent < min_alpha_val) myExponent = min_alpha_val;
    if(myExponent > max_alpha_val) myExponent = max_alpha_val;

    double myOpacity = (myExponent - min_alpha_val) / (max_alpha_val - min_alpha_val);

    //EV << "addXMLchild_object. [" << min << " " << max << "]" << endl;

    //if ((map->card == GridCardinality::OVERALL) || (map->children.size() == 0)) {
    if (map->children.size() == 0) {

        //EV << "Appending new child. Min: " << min << " - Dim: " << max.x - min.x << endl;

        cXMLElement *newGrid = new cXMLElement("object", "", parent);
        setXMLattr(newGrid, min, max.x - min.x, myOpacity);
        //EV << newGrid->detailedInfo() << endl;

        parent->appendChild(newGrid);
    }
    else {
        // check if there are all the children: NE, NW, SE, SW
        unsigned int test_child = 0;

        for (std::list<struct Cell>::iterator it = map->children.begin(); it != map->children.end(); it++) {
            Cell_t *new_map = &(*it);
            Coord new_min, new_max;

            new_min = min;
            new_max = max;

            switch (new_map->card) {
            case GridCardinality::NE:
                test_child |= NE_MASK;
                new_min.x = (min.x + max.x) / 2.0;
                new_max.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::NW:
                test_child |= NW_MASK;
                new_max.x = (min.x + max.x) / 2.0;
                new_max.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::SE:
                test_child |= SE_MASK;
                new_min.x = (min.x + max.x) / 2.0;
                new_min.y = (min.y + max.y) / 2.0;
                break;

            case GridCardinality::SW:
                test_child |= SW_MASK;
                new_max.x = (min.x + max.x) / 2.0;
                new_min.y = (min.y + max.y) / 2.0;
                break;

            default:
                test_child |= NE_MASK;
                test_child |= NW_MASK;
                test_child |= SE_MASK;
                test_child |= SW_MASK;
                break;
            }

            addXMLchild_object(parent, new_map, new_min, new_max);
        }

        if ((test_child & NE_MASK) == 0) {
            Coord new_min, new_max;

            new_min = min;
            new_max = max;
            new_min.x = (min.x + max.x) / 2.0;
            new_max.y = (min.y + max.y) / 2.0;

            cXMLElement *newGrid = new cXMLElement("object", "", parent);
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            parent->appendChild(newGrid);
        }

        if ((test_child & NW_MASK) == 0) {
            Coord new_min, new_max;

            new_min = min;
            new_max = max;
            new_max.x = (min.x + max.x) / 2.0;
            new_max.y = (min.y + max.y) / 2.0;

            cXMLElement *newGrid = new cXMLElement("object", "", parent);
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            parent->appendChild(newGrid);
        }

        if ((test_child & SE_MASK) == 0) {
            Coord new_min, new_max;

            new_min = min;
            new_max = max;
            new_min.x = (min.x + max.x) / 2.0;
            new_min.y = (min.y + max.y) / 2.0;

            cXMLElement *newGrid = new cXMLElement("object", "", parent);
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            parent->appendChild(newGrid);
        }

        if ((test_child & SW_MASK) == 0) {
            Coord new_min, new_max;

            new_min = min;
            new_max = max;
            new_max.x = (min.x + max.x) / 2.0;
            new_min.y = (min.y + max.y) / 2.0;

            cXMLElement *newGrid = new cXMLElement("object", "", parent);
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            parent->appendChild(newGrid);
        }
    }
}

} /* namespace physicallayer */
} /* namespace inet */
