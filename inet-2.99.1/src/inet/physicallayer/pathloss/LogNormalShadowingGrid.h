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

#include "inet/physicallayer/pathloss/LogNormalShadowing.h"

namespace inet {

namespace physicallayer {

/**
 * This class implements the log normal shadowing model.
 */
class INET_API LogNormalShadowingGrid : public LogNormalShadowing
{
  public:
    enum class GridCardinality {OVERALL, NW, NE, SW, SE};

    typedef struct Cell {
        double exponent;
        double stddev;
        double variance;
        GridCardinality card;

        std::list<struct Cell> children;
    } Cell_t;

  protected:
    virtual void initialize(int stage) override;

    /** @brief Reading the input xml file with the channel characteristics */
    bool readChannelGridFile();

    /** @brief fill recoursively the grid-struct from the xml file */
    void loadXMLtree(cXMLElementList *xml, Cell_t *grid);

  public:
    LogNormalShadowingGrid();
    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
    virtual double computePathLoss(mps propagationSpeed, Hz frequency, m distance) const override;

  private:
    void printMap(Cell_t *map, int ntab);

  private:
    Cell_t grid_map;
};

} // namespace physicallayer

} // namespace inet


#endif /* __INET_LOGNORMALSHADOWINGGRID_H */
