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

/* Misc defines */
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif  /* MAX */

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif  /* MIN */

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
        alpha_rand = &par("pathLossAlpha");
        sigma_rand = &par("pathLossSigma");

        randomShadowMap = par("randomShadowMap").boolValue();
        gridCellRandomSize = par("gridCellRandomSize");

        max_RGB_R = par("maxRGBdraw_r");
        max_RGB_G = par("maxRGBdraw_g");
        max_RGB_B = par("maxRGBdraw_b");
        min_alpha_val = par("minDrawAlpha");
        max_alpha_val = par("maxDrawAlpha");
        color2dark = par("color2dark").boolValue();

        if (randomShadowMap) {
            if (!makeRandomShadowMap())
                throw cRuntimeError("LogNormalShadowingGrid: error creating the random shadow map");
        }
        else {
            if (!readChannelGridFile())
                throw cRuntimeError("LogNormalShadowingGrid: error in reading the shadowing file");
        }
    }
    else if (stage == INITSTAGE_PHYSICAL_ENVIRONMENT) {

        if (randomShadowMap) {
            if (ev.isGUI()) {
                PhysicalEnvironment *physicalEnvironment = dynamic_cast<PhysicalEnvironment *>(getModuleByPath("environment"));
                if (physicalEnvironment != nullptr) {
                    cXMLElement *shadowsXml = new cXMLElement("environment", "", nullptr);
                    addXMLchild_randomMap(shadowsXml);
                    physicalEnvironment->addFromXML(shadowsXml);
                }
            }
        }
        else {
            if (ev.isGUI()) {
                PhysicalEnvironment *physicalEnvironment = dynamic_cast<PhysicalEnvironment *>(getModuleByPath("environment"));
                if (physicalEnvironment != nullptr) {
                    cXMLElement *shadowsXml = new cXMLElement("environment", "", nullptr);
                    addXMLchild_object(shadowsXml, &grid_map, scenarioCoordMin, scenarioCoordMax);
                    physicalEnvironment->addFromXML(shadowsXml);
                }
            }

            fastSignalMap.resize((int)(scenarioCoordMax.x) + 1);
            for (unsigned int x = 0; x < fastSignalMap.size(); x++) {
                //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
                fastSignalMap[x].resize((int)(scenarioCoordMax.y) + 1);

                for (unsigned int y = 0; y < fastSignalMap[x].size(); y++) {
                    fastSignalMap[x][y].exponent = 2;
                    fastSignalMap[x][y].stddev = 1;
                }
            }
            // copy the map from the pathloss simulation module
            for (unsigned int x = 0; x < fastSignalMap.size(); x++) {
                for (unsigned int y = 0; y < fastSignalMap[x].size(); y++) {

                    getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, Coord(x, y), fastSignalMap[x][y].exponent, fastSignalMap[x][y].stddev);
                    //getAlphaSigmaFromAbsCoord(Coord(x, y),
                    //        fastSignalMap[x][y].exponent,
                    //        fastSignalMap[x][y].stddev);

                }
            }
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
    //throw cRuntimeError("LogNormalShadowingGrid::computePathLoss OK");
    //exit(0);

    m d0 = m(1.0);
    // reference path loss
    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, alpha_par, systemLoss);
    double PL_d0_db = 10.0 * log10(1 / freeSpacePathLoss);
    // path loss at distance d + normal distribution with sigma standard deviation
    double PL_db = PL_d0_db + 10 * alpha_par * log10(unit(distance / d0).get()) + normal(0.0, sigma_par);
    double PL_ris = math::dB2fraction(-PL_db);

    EV_DEBUG << "Using for log-normal path loss"
                << ": alpha = " << alpha_par
                << ", sigma = " << sigma_par
                << ", propagationSpeed = " << propagationSpeed
                << ", PathLoss(D0) = " << PL_d0_db
                << ", frequency = " << frequency
                << ", distance = " << distance
                << ". Total path loss: " << PL_db << "dB"
                << endl;

    return PL_ris;
}

double LogNormalShadowingGrid::getPathLossD0(IRadio *receiverRadio, Hz frequency, double alpha_p) {

    m d0 = m(1.0);
    const IRadioMedium *radioMedium = receiverRadio->getMedium();
    mps propagationSpeed = radioMedium->getPropagation()->getPropagationSpeed();

    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, alpha_p, systemLoss);

    return 10.0 * log10(1 / freeSpacePathLoss);
}

m LogNormalShadowingGrid::getThresholdDistance(double p_tx_dBm, double sigma_mult, double threshold_dBm, Coord receiverPos,
        IRadio *receiverRadio, Hz frequency) {
    double grid_alphaRx, grid_sigmaRx;

    grid_alphaRx = alpha;
    grid_sigmaRx = sigma;
    //getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, receiverPos, grid_alphaRx, grid_sigmaRx);
    getAlphaSigmaFromAbsCoord(receiverPos, grid_alphaRx, grid_sigmaRx);

    m d0 = m(1.0);
    const IRadioMedium *radioMedium = receiverRadio->getMedium();
    mps propagationSpeed = radioMedium->getPropagation()->getPropagationSpeed();

    double freeSpacePathLoss = computeFreeSpacePathLoss(propagationSpeed / frequency, d0, grid_alphaRx, systemLoss);
    double PL_d0_db = 10.0 * log10(1 / freeSpacePathLoss);

    m d = d0 * pow(10, (p_tx_dBm - PL_d0_db + (sigma_mult * grid_sigmaRx) - threshold_dBm) / (10 * grid_alphaRx));

    //EV << "alpha " << grid_alphaRx
    //        << " - sigma " << grid_sigmaRx
    //        << " - propagationSpeed " << propagationSpeed
    //        << " - freeSpacePathLoss " << freeSpacePathLoss
    //        << " - PL_d0_db " << PL_d0_db
    //        << " - d0 " << d0
    //        << " - ris " << d
    //        << endl;

    return d;
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

void LogNormalShadowingGrid::getAlphaSigmaFromAbsCoord(Coord point, double &alpha_p, double &sigma_p) const {
    //getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, point, alpha_p, sigma_p);
    alpha_p = fastSignalMap[point.x][point.y].exponent;
    sigma_p = fastSignalMap[point.x][point.y].stddev;
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

    // check if the fast map is present
    if (fastSignalMap.size() > 0) {
        getAlphaSigmaFromAbsCoord(transmitter, grid_alphaTx, grid_sigmaTx);
        getAlphaSigmaFromAbsCoord(receiver, grid_alphaRx, grid_sigmaRx);
    }
    else {
        getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, transmitter, grid_alphaTx, grid_sigmaTx);
        getAlphaSigmaFromCoord(&grid_map, scenarioCoordMin, scenarioCoordMax, receiver, grid_alphaRx, grid_sigmaRx);
    }

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

    // TODO trick to enhance 2.4 communications
    if (frequency > Hz(2300000000)) {
        grid_alphaMax = grid_alphaMax / 2.0;
        if (grid_alphaMax < 2) grid_alphaMax = 2.0;
    }

    return computePathLoss_parametric(propagationSpeed, frequency, distance, grid_alphaMax, grid_sigmaMax);
}

bool LogNormalShadowingGrid::makeRandomShadowMap(void) {

    if (gridCellRandomSize <= 0) return false;

    fastSignalMap.resize((int)(scenarioCoordMax.x) + 1);
    for (unsigned int x = 0; x < fastSignalMap.size(); x++) {
        //signalPropMap[x].resize((int)(ffmob->getConstraintAreaMax().y - ffmob->getConstraintAreaMin().y));
        fastSignalMap[x].resize((int)(scenarioCoordMax.y) + 1);

        for (unsigned int y = 0; y < fastSignalMap[x].size(); y++) {
            fastSignalMap[x][y].exponent = 2;
            fastSignalMap[x][y].stddev = 1;
        }
    }
    // copy the map from the pathloss simulation module
    int xSteps = ((scenarioCoordMax.x - scenarioCoordMin.x) / gridCellRandomSize) + 1;
    int ySteps = ((scenarioCoordMax.y - scenarioCoordMin.y) / gridCellRandomSize) + 1;
    for (int x = 0; x < xSteps; x++) {
        for (int y = 0; y < ySteps; y++) {
            int xcmin, xcmax, ycmin, ycmax;
            double alpha_r, sigma_r;
            //cPar *alpha_r = nullptr;
            //cPar *sigma_r = nullptr;

            xcmin = (x * gridCellRandomSize) + scenarioCoordMin.x;
            ycmin = (y * gridCellRandomSize) + scenarioCoordMin.y;

            xcmax = MIN((int)(scenarioCoordMax.x), xcmin + gridCellRandomSize);
            ycmax = MIN((int)(scenarioCoordMax.y), ycmin + gridCellRandomSize);

            alpha_r = alpha_rand->doubleValue();
            sigma_r = sigma_rand->doubleValue();

            //fprintf(stderr, "Assigning to [%d %d] alpha:%lf, sigma:%lf\n", xcmin, ycmin, alpha_r, sigma_r); fflush(stderr);

            for (int xc = xcmin; xc < xcmax; xc++) {
                for (int yc = ycmin; yc < ycmax; yc++) {
                    fastSignalMap[xc][yc].exponent = alpha_r;
                    fastSignalMap[xc][yc].stddev = sigma_r;
                }
            }
        }
    }

    return true;
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

void LogNormalShadowingGrid::setXMLattr(cXMLElement *el, Coord posObj, double dimObjX, double dimObjY, double opacity) {
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
    snprintf(buffer, sizeof(buffer), "min %lf %lf %lf", posObj.x, posObj.y, 0.0);
    el->setAttribute("position", buffer);

    //shape attribute
    snprintf(buffer, sizeof(buffer), "cuboid %lf %lf %lf", dimObjX, dimObjY, 1.0);
    el->setAttribute("shape", buffer);

    //material and fill-color constant
    el->setAttribute("material", "vacuum");

    //opacity
#if OMNETPP_CANVAS_VERSION >= 0x20140908
    snprintf(buffer, sizeof(buffer), "%lf", opacity);
    el->setAttribute("opacity", buffer);
#endif
}

void LogNormalShadowingGrid::addXMLchild_randomMap(cXMLElement *parent) {
    if (gridCellRandomSize <= 0) return;

    // copy the map from the pathloss simulation module
    int xSteps = ((scenarioCoordMax.x - scenarioCoordMin.x) / gridCellRandomSize) + 1;
    int ySteps = ((scenarioCoordMax.y - scenarioCoordMin.y) / gridCellRandomSize) + 1;
    for (int x = 0; x < xSteps; x++) {
        for (int y = 0; y < ySteps; y++) {
            int xcmin, xcmax, ycmin, ycmax;
            double alpha_r, myOpacity;

            xcmin = (x * gridCellRandomSize) + scenarioCoordMin.x;
            ycmin = (y * gridCellRandomSize) + scenarioCoordMin.y;

            xcmax = MIN(xcmin + gridCellRandomSize, scenarioCoordMax.x);
            ycmax = MIN(ycmin + gridCellRandomSize, scenarioCoordMax.y);

            //if ((xcmax >= scenarioCoordMax.x) || (ycmax >= scenarioCoordMax.y)) continue;

            //fprintf(stderr, "xcmin %d, xcmax %d, ycmin %d, ycmax %d\n", xcmin, xcmax, ycmin, ycmax); fflush(stderr);

            alpha_r = fastSignalMap[(xcmin + xcmax)/2][(ycmin + ycmax) / 2].exponent;
            //alpha_r = intrand(7);

            //fprintf(stderr, "alpha_r %lf\n", alpha_r); fflush(stderr);

            if(alpha_r < min_alpha_val) alpha_r = min_alpha_val;
            if(alpha_r > max_alpha_val) alpha_r = max_alpha_val;

            myOpacity = (alpha_r - min_alpha_val) / (max_alpha_val - min_alpha_val);

            cXMLElement *newGrid = new cXMLElement("object", "", parent);
            setXMLattr(newGrid, Coord(xcmin, ycmin), xcmax - xcmin, ycmax - ycmin, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            //fprintf(stderr, "Adding element of opacity: %lf\n\n", myOpacity); fflush(stderr);

            parent->appendChild(newGrid);
        }
    }
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
        setXMLattr(newGrid, min, max.x - min.x, max.y - min.y, myOpacity);
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
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, new_max.y - new_min.y, myOpacity);
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
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, new_max.y - new_min.y, myOpacity);
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
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, new_max.y - new_min.y, myOpacity);
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
            setXMLattr(newGrid, new_min, new_max.x - new_min.x, new_max.y - new_min.y, myOpacity);
            //EV << newGrid->detailedInfo() << endl;

            parent->appendChild(newGrid);
        }
    }
}

} /* namespace physicallayer */
} /* namespace inet */
