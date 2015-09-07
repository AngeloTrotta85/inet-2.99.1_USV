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

package inet.physicallayer.pathloss;

module LogNormalShadowingGrid extends LogNormalShadowing
{
    parameters:
        xml shadowFile;        // The input file that specify the characteristics of the log-normal path loss shadowing for each grid-cell
        xml testShadowDraw = default(xml("<environment/>"));
        
        bool randomShadowMap = default(false);	// if true the xml file is not read and the shadow map is computed randomly
        volatile double pathLossAlpha;					// decay exponential factor (could be a random value)
        volatile double pathLossSigma;					// sigma parameter fo the log-normal shadowing
        double gridCellRandomSize @unit(m) = default(50m);	// size of the cells to randomize
        
        int maxRGBdraw_r = default(203);		// Specify the red (RGB) value for the shadow drawing
        int maxRGBdraw_g = default(65);			// Specify the green (RGB) value for the shadow drawing
        int maxRGBdraw_b = default(84);			// Specify the blue (RGB) value for the shadow drawing
        double minDrawAlpha = default(2);		// Specify the min alpha val for the shadow drawing 
        double maxDrawAlpha = default(6);		// Specify the max alpha val for the shadow drawing 
        bool color2dark = default(true);		// If true the color goes from "color" to black, if false the color goes from white to "color"
        
        double constraintAreaMinX @unit(m) = default(-1m/0); // min x position of the constraint area, unconstrained by default (negative infinity)
        double constraintAreaMinY @unit(m) = default(-1m/0); // min y position of the constraint area, unconstrained by default (negative infinity)
        double constraintAreaMinZ @unit(m) = default(-1m/0); // min z position of the constraint area, unconstrained by default (negative infinity)
        double constraintAreaMaxX @unit(m) = default(1m/0); // max x position of the constraint area, unconstrained by default (positive infinity)
        double constraintAreaMaxY @unit(m) = default(1m/0); // max y position of the constraint area, unconstrained by default (positive infinity)
        double constraintAreaMaxZ @unit(m) = default(1m/0); // max z position of the constraint area, unconstrained by default (positive infinity)
        
        @class(LogNormalShadowingGrid);
}
