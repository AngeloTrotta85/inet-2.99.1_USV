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

#ifndef __INET_LOGNORMALSHADOWINGGRID_H
#define __INET_LOGNORMALSHADOWINGGRID_H

#include <list>

#include "inet/common/geometry/common/Coord.h"

#include "inet/physicallayer/pathloss/LogNormalShadowing.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/physicallayer/contract/packetlevel/IRadioMedium.h"

namespace inet {

namespace physicallayer {

/**
 * This class implements the log normal shadowing model.
 */
class INET_API LogNormalShadowingGrid : public LogNormalShadowing
{
  public:
    enum class GridCardinality {
        OVERALL,
        NW,
        NE,
        SW,
        SE
    };

    typedef struct Cell {
        double exponent;
        double stddev;
        double variance;
        GridCardinality card;

        std::list<struct Cell> children;
    } Cell_t;

    typedef struct PointChar {
        double exponent;
        double stddev;
    } PointChar_t;

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Reading the input xml file with the channel characteristics */
    bool readChannelGridFile();

    /** @brief fill recoursively the grid-struct from the xml file */
    void loadXMLtree(cXMLElementList *xml, Cell_t *grid);

  public:
    LogNormalShadowingGrid();
    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
    virtual double computePathLoss(mps propagationSpeed, Hz frequency, m distance) const override;
    virtual double computePathLossExt(mps propagationSpeed, Hz frequency, m distance, Coord transmitter, Coord receiver) const override;

    m getThresholdDistance(double p_tx_dBm, double sigma_mult, double threshold_dBm, Coord receiverPos, IRadio *receiverRadio, Hz frequency);

    void getAlphaSigmaFromAbsCoord(Coord point, double &alpha_p, double &sigma_p) const;

  private:
    void setXMLattr(cXMLElement *el, Coord posObj, double dimObj, double opacity);
    void addXMLchild_object(cXMLElement *parent, Cell_t *map, Coord min, Coord max);
    void printMap(Cell_t *map, int ntab);
    void initCellGrid(Cell_t *cell);

    double computePathLossGrid(mps propagationSpeed, Hz frequency, m distance, Coord transmitter, Coord receiver) const;
    double computePathLoss_parametric(mps propagationSpeed, Hz frequency, m distance, double alpha_par, double sigma_par) const;

    void getAlphaSigmaFromCoord(const Cell_t *grid, Coord min, Coord max, Coord point, double &alpha_p, double &sigma_p) const;


  private:
    Cell_t grid_map;
    Coord scenarioCoordMin;
    Coord scenarioCoordMax;

    std::vector< std::vector<PointChar_t> > fastSignalMap;

    //draw option
    int max_RGB_R;
    int max_RGB_G;
    int max_RGB_B;
    double min_alpha_val;
    double max_alpha_val;
    bool color2dark;
};

} // namespace physicallayer

} // namespace inet


#endif /* __INET_LOGNORMALSHADOWINGGRID_H */
